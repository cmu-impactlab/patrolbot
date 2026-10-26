"""Bridge-owned origin guard and asynchronous AMCL recovery coordinator."""

import math
import socket
import threading
import time

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from .epoch_recovery import EpochRecovery
from .map_runtime import MapRuntimeMixin
from .odom_epoch import OdomEpochGuard
from .pose_innovation import innovation, OdomHistory, propagate, valid_planar_covariance


class EpochRuntime(MapRuntimeMixin):
    def __init__(self, node):
        self.node = node
        self.lock = threading.RLock()
        self.guard = OdomEpochGuard()
        self.recovery = EpochRecovery()
        self._odom_history = OdomHistory()
        self._pose_anchor = None
        self._localization_fault = ''
        self._token = None
        self.first_tf_ns = None
        self._tf_generation = 0
        self._tf_ready_at = None
        self._tf_ready_since_ns = None
        self._tf_ready_token = None
        self._tf_probe = None
        self._tf_probe_started = None
        self._tf_probe_generation = None
        self._tf_probe_token = None
        self._tf_last_query = float('-inf')
        self._tf_readiness = node.create_client(Trigger, '/amcl/initial_pose_tf_ready')
        self._localization_active = node.create_client(
            Trigger, '/lifecycle_manager_localization/is_active'
        )
        self._phase = 'wait-localization-manager'
        self._future = None
        self._future_kind = None
        self._deadline = None
        self._failed = False
        self._navigation_ready = False
        self.undock_hold = False
        # Tuning activation hold. Owned by the tuning coordinator and surfaced
        # here so it rides the *existing* admission boundaries: a pending or
        # unknown activation job blocks autonomy (commands_allowed) and manual
        # DRIVE, including the manual deadman bypass.
        self.tuning_hold = False
        self._tuning_hold_sources = {}
        self._tuning_hold_reason = ''
        self._lidar_reseed = False
        self._lidar_cancel = [
            node.create_client(CancelGoal, '/' + name + '/_action/cancel_goal')
            for name in ('navigate_to_pose', 'navigate_through_poses')
        ]
        self._navigation = node.create_client(
            ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes'
        )
        self._accepted_seed_stamp = None
        self._pending_pose = None
        self._pending_pose_received_at = None
        self._parameters = node.create_client(
            SetParametersAtomically, '/amcl/set_parameters_atomically'
        )
        self._get = node.create_client(GetState, '/amcl/get_state')
        self._change = node.create_client(ChangeState, '/amcl/change_state')
        self._seed_sub = node.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.seed, 10
        )
        self._ack_sub = node.create_subscription(
            PoseWithCovarianceStamped, '/amcl/accepted_initial_pose', self.seed_accepted, 10
        )
        self._pose_sub = node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose, 10
        )
        self.init_maps()
        self._timer = node.create_timer(0.05, self.tick)

    def begin_session(self):
        with self.lock:
            session = self.guard.begin_session()
            self.withdraw()
            return session

    def disconnect(self, session):
        with self.lock:
            self.guard.disconnect(session)
            if session == self.guard.session:
                self.withdraw()

    def set_tuning_hold(self, held, reason=''):
        self._set_tuning_hold_source('job', held, reason)

    def set_tuning_policy_hold(self, held, reason=''):
        self._set_tuning_hold_source('policy', held, reason)

    def _set_tuning_hold_source(self, source, held, reason=''):
        """
        Set the tuning hold on the existing admission boundary.

        Set by the tuning coordinator before any service is stopped and held
        until the job reaches a terminal state. A pending, running or unknown
        job keeps this True across a restart (see ``recover``), blocking both
        autonomy and manual DRIVE.
        """
        with self.lock:
            if held:
                self._tuning_hold_sources[source] = str(reason or '')
            else:
                self._tuning_hold_sources.pop(source, None)
            self.tuning_hold = bool(self._tuning_hold_sources)
            self._tuning_hold_reason = '; '.join(self._tuning_hold_sources.values())
            publisher = getattr(self.node, '_tuning_hold_pub', None)
            if publisher is not None:
                from std_msgs.msg import Bool
                publisher.publish(Bool(data=self.tuning_hold))
            if held:
                self._pending_pose = None
                self._accepted_seed_stamp = None
                self._navigation_ready = False
                self.node.get_logger().warning(
                    'Tuning activation hold engaged: ' + self._tuning_hold_reason)

    def withdraw(self, reset_odometry=True):
        turn = getattr(self.node, '_undock_turn', None)
        if turn is not None:
            turn.revoke('odometry transport withdrawn')
        self.first_tf_ns = None
        self._tf_generation += 1
        self._tf_ready_at = None
        self._tf_ready_since_ns = None
        if reset_odometry:
            with self.node._pose_lock:
                self.node._pose_received = False
                self.node._pose_at = 0.0
                self.node._undock_scan_at = 0.0

    def announce(self, session, epoch):
        with self.lock:
            decision = self.guard.announce(session, epoch)
            if not decision.accepted:
                self.invalidate_transport(decision.reason)
                return
            if decision.reset_required:
                self.node._odom_validator.reset_epoch()
                with self.node._pose_lock:
                    self.node._odom_covariance.reset()
                self.withdraw()
            self.recovery.observe_origin(epoch)

    def invalidate_transport(self, reason):
        self.guard.disconnect(self.guard.session)
        self.withdraw()
        self.node.get_logger().error('Odometry origin invalid: ' + reason)
        # Existing SBC client-loss handling stops/disables the base and cancels
        # its undock operation. Do not allow a bad-origin stream to keep that
        # operation alive merely by continuing to send auxiliary telemetry.
        with self.node.sock_lock:
            if self.node.sock is not None:
                try:
                    self.node.sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass

    @property
    def streams_allowed(self):
        return self.guard.ready and self.recovery.stage in (
            EpochRecovery.WAIT_SEED,
            EpochRecovery.WAIT_POSE,
            EpochRecovery.READY,
        )

    @property
    def commands_allowed(self):
        with self.lock:
            if self.undock_hold:
                return False
            if self.tuning_hold:
                return False
            if getattr(self, '_tuning_nav_pending', False):
                return False
            if not self.node._lidar_ready():
                return False
            if self.map_hold:
                return False
            if not (
                self.guard.ready
                and not self.recovery.required
                and self._navigation_ready
                and self.first_tf_ns is not None
            ):
                return False
                # A valid origin does not make old odometry current. Apply the same
                # receipt-age limit as TF, even if AUX traffic keeps TCP alive.
            with self.node._pose_lock:
                age = time.monotonic() - self.node._pose_at
                return (
                    self.node._pose_received
                    and math.isfinite(age)
                    and 0.0 <= age <= self.node._odom_tf_timeout
                )

    @property
    def seed_tf_ready(self):
        if (
            not self.streams_allowed
            or self.first_tf_ns is None
            or self._tf_ready_at is None
            or self._tf_ready_token != self.recovery.token
        ):
            return False
        age = time.monotonic() - self._tf_ready_at
        return 0.0 <= age <= self.node._odom_tf_timeout

    def _poll_tf_readiness(self):
        # Read-only requests may be discarded on timeout. Unlike lifecycle
        # transitions, an old response cannot change the server's state.
        now = time.monotonic()
        if self._tf_probe is not None:
            age = now - self._tf_probe_started
            if self._tf_probe.done():
                valid = (
                    self._tf_probe_generation == self._tf_generation
                    and self._tf_probe_token == self.recovery.token
                    and 0.0 <= age <= self.node._odom_tf_timeout
                )
                try:
                    result = self._tf_probe.result()
                    valid = valid and result.success is True
                except Exception:
                    valid = False
                if valid:
                    if not self.seed_tf_ready:
                        self._tf_ready_since_ns = self.node.get_clock().now().nanoseconds
                    self._tf_ready_at = self._tf_probe_started
                    self._tf_ready_token = self.recovery.token
                else:
                    self._tf_ready_at = None
                    self._tf_ready_since_ns = None
                self._tf_probe = None
            elif not 0.0 <= age <= self.node._odom_tf_timeout:
                self._tf_readiness.remove_pending_request(self._tf_probe)
                self._tf_probe.cancel()
                self._tf_probe = None
                self._tf_ready_at = None
                self._tf_ready_since_ns = None
        if not self.streams_allowed or self.first_tf_ns is None:
            self._tf_ready_at = None
            self._tf_ready_since_ns = None
            return
        if (
            self._tf_probe is None
            and now - self._tf_last_query >= 0.1
            and self._tf_readiness.service_is_ready()
        ):
            self._tf_last_query = now
            self._tf_probe_started = now
            self._tf_probe_generation = self._tf_generation
            self._tf_probe_token = self.recovery.token
            try:
                self._tf_probe = self._tf_readiness.call_async(Trigger.Request())
            except Exception:
                self._tf_ready_at = None
                self._tf_ready_since_ns = None

    @property
    def status_stage(self):
        if self._localization_fault:
            return 'failed:localization-innovation'
        if self._map_error:
            return 'failed:map-state'
        if self.maps and self.maps.hold:
            return 'map:' + self.maps.stage
        if self.recovery.stage == EpochRecovery.WAIT_SEED and self.first_tf_ns is None:
            return 'wait-first-odometry'
        if self.recovery.stage == EpochRecovery.WAIT_SEED and not self.seed_tf_ready:
            return 'wait-amcl-tf'
        if self._failed:
            return 'failed:' + self._phase
        if self.recovery.stage == EpochRecovery.READY and not self._navigation_ready:
            return self._phase if self._phase.endswith('navigation') else 'wait-navigation'
        if self.recovery.stage == EpochRecovery.WAIT_SEED and self.navigation_idle_problem():
            return self.navigation_idle_problem()
        return self.recovery.stage

    def frame_allowed(self, session, line):
        decision = self.guard.check_frame(session, line)
        if not decision.accepted:
            if decision.reason == 'frame-epoch-mismatch':
                self.invalidate_transport(decision.reason)
            return False
        return self.streams_allowed

    @staticmethod
    def stamp_ns(message):
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def seed(self, message):
        with self.lock:
            if self.tuning_hold:
                return
            if self.undock_hold:
                self.node._undock_turn.revoke('initial pose requested during undock')
                return
            if not self.node._lidar_ready():
                return
            if self._lidar_reseed and (
                self._navigation_status
                or self._navigation_idle_at is None
                or not 0 <= time.monotonic() - self._navigation_idle_at <= 0.5
            ):
                return
            if (
                self.maps
                and self.maps.hold
                and (
                    self.maps.stage != 'wait-localization'
                    or self.stamp_ns(message) != self.maps.seed_stamp
                )
            ):
                return
            pose = message.pose.pose
            q = pose.orientation
            values = (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                q.x,
                q.y,
                q.z,
                q.w,
                *message.pose.covariance,
            )
            if (
                not self.seed_tf_ready
                or self._tf_ready_since_ns is None
                or self.stamp_ns(message) < self._tf_ready_since_ns
                or self.stamp_ns(message) < self.first_tf_ns
                or message.header.frame_id != 'map'
                or not all(math.isfinite(v) for v in values)
                or sum(v * v for v in (q.x, q.y, q.z, q.w)) == 0
                or any(message.pose.covariance[i] <= 0 for i in (0, 7, 35))
            ):
                return
            if self.recovery.operator_seed(
                self.recovery.token,
                self.stamp_ns(message),
                self.node.get_clock().now().nanoseconds,
            ):
                self._pending_pose = None
                self._navigation_ready = False
                self._lidar_reseed = False
                self._pose_anchor = None
                self._localization_fault = ''

    def require_lidar_seed(self):
        """Power changes invalidate localization, while odometry keeps flowing."""
        with self.lock:
            self.recovery.require_seed(self.node.get_clock().now().nanoseconds)
            self._navigation_ready = False
            self._accepted_seed_stamp = None
            self._pending_pose = None
            self._pending_pose_received_at = None
            self._lidar_reseed = True
            self.cancel_lidar_navigation()
            # Keep the reset token current: this barrier needs a fresh operator
            # seed, not an odometry/AMCL origin reset or a new cached goal.
            self._token = self.recovery.token

    def cancel_lidar_navigation(self):
        # A delayed cancellation must never cancel goals issued after recovery.
        request = CancelGoal.Request()
        request.goal_info.stamp = self.node.get_clock().now().to_msg()
        for client in self._lidar_cancel:
            if client.service_is_ready():
                client.call_async(request)

    def pose(self, message):
        with self.lock:
            if self.undock_hold:
                return
            if (
                not self.node._lidar_ready()
                or not self.streams_allowed
                or message.header.frame_id != 'map'
            ):
                return
            p = message.pose.pose
            if not all(
                math.isfinite(v)
                for v in (
                    p.position.x,
                    p.position.y,
                    p.position.z,
                    p.orientation.x,
                    p.orientation.y,
                    p.orientation.z,
                    p.orientation.w,
                    *message.pose.covariance,
                )
            ):
                if self._pose_anchor is not None:
                    self._reject_localization('non-finite AMCL pose or covariance')
                return
            q = p.orientation
            if abs(sum(v * v for v in (q.x, q.y, q.z, q.w)) - 1.0) > 1e-3:
                if self._pose_anchor is not None:
                    self._reject_localization('invalid AMCL quaternion')
                return
            if self._pose_anchor is not None and self.recovery.stage == EpochRecovery.READY:
                anchor, stamp, origin, start = self._pose_anchor
                if self.stamp_ns(message) <= stamp:
                    return
                try:
                    if not valid_planar_covariance(message.pose.covariance) or any(
                        message.pose.covariance[i] > 0.25 for i in (0, 7, 35)
                    ):
                        raise ValueError('invalid localization covariance')
                    if (
                        not 0
                        <= (self.node.get_clock().now().nanoseconds - self.stamp_ns(message)) / 1e9
                        <= 1.0
                    ):
                        raise ValueError('localization scan is stale or future dated')
                    if origin != self.guard.epoch:
                        raise ValueError('odometry origin mismatch')
                    end = self._odom_history.at(self.stamp_ns(message), self.guard.epoch)
                    predicted = propagate(anchor, start, end)
                    with self.node._pose_lock:
                        stationary = abs(self.node._vx) <= 0.005 and abs(self.node._vth) <= 0.01
                    problem = innovation(
                        predicted,
                        self.planar_pose(message),
                        0.1 if stationary else 0.2,
                        math.radians(3 if stationary else 5),
                    )
                except ValueError as exc:
                    problem = str(exc)
                if problem:
                    self._reject_localization(problem)
                    return
                if not stationary:
                    self._pose_anchor = (
                        self.planar_pose(message),
                        self.stamp_ns(message),
                        self.guard.epoch,
                        end,
                    )
            self._pending_pose = message
            self._pending_pose_received_at = time.monotonic()
            self._accept_pending_pose()

    def _reject_localization(self, problem):
        self._localization_fault = 'LOCALIZATION_DEGRADED: ' + problem
        self._navigation_ready = False
        self.recovery.require_seed(self.node.get_clock().now().nanoseconds)
        self._token = self.recovery.token
        self.cancel_lidar_navigation()
        self.node.cmd_vel_callback(Twist())
        if self.maps and self.maps.hold:
            self.maps.fail(self._localization_fault)
        self.node.get_logger().error(self._localization_fault)

    @staticmethod
    def planar_pose(message):
        p = message.pose.pose
        q = p.orientation
        return (
            p.position.x,
            p.position.y,
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
        )

    def seed_accepted(self, message):
        with self.lock:
            if self.undock_hold:
                return
            if not self.node._lidar_ready():
                return
            stamp = self.stamp_ns(message)
            if (
                message.header.frame_id != 'map'
                or not self.streams_allowed
                or self.recovery._reset_ns is None
                or stamp <= self.recovery._reset_ns
            ):
                return
            self._accepted_seed_stamp = stamp
            self._accept_pending_pose()

    def _accept_pending_pose(self):
        if self._pending_pose is None or self._accepted_seed_stamp != self.recovery._seed_ns:
            return
        self.recovery.amcl_pose(
            self.recovery.token,
            self.stamp_ns(self._pending_pose),
            self.node.get_clock().now().nanoseconds,
        )

    def _request(self, phase):
        self._future_kind = (
            'read-only' if phase in ('wait-localization-manager', 'inspect') else 'mutation'
        )
        if phase == 'wait-localization-manager':
            if not self._localization_active.service_is_ready():
                return
            self._future = self._localization_active.call_async(Trigger.Request())
        elif phase == 'inspect':
            if not self._get.service_is_ready():
                return
            self._future = self._get.call_async(GetState.Request())
        elif phase in ('pause-navigation', 'resume-navigation'):
            if not self._navigation.service_is_ready():
                return
            request = ManageLifecycleNodes.Request()
            request.command = request.PAUSE if phase == 'pause-navigation' else request.RESUME
            self._future = self._navigation.call_async(request)
        elif phase == 'reset-parameters':
            if not self._parameters.service_is_ready():
                return
            request = SetParametersAtomically.Request()
            request.parameters = [
                Parameter('set_initial_pose', value=False).to_parameter_msg(),
                Parameter('always_reset_initial_pose', value=True).to_parameter_msg(),
            ]
            self._future = self._parameters.call_async(request)
        else:
            if not self._change.service_is_ready():
                return
            request = ChangeState.Request()
            request.transition.id = {'deactivate': 4, 'cleanup': 2, 'configure': 1, 'activate': 3}[
                phase
            ]
            self._future = self._change.call_async(request)
        self._deadline = time.monotonic() + (2.0 if phase == 'wait-localization-manager' else 10.0)

    def tick(self):
        with self.lock:
            if self.tuning_hold:
                self.poll_navigation_idle()
                return
            self.recovery.observe_clock(self.node.get_clock().now().nanoseconds)
            token = self.recovery.token
            if self.undock_hold:
                return
            self._poll_tf_readiness()
            self.poll_navigation_idle()
            self.tick_tuning_map_gate()
            if self.tick_map():
                return
            self.tick_startup_map()
            if self._future is not None:
                if not self._future.done():
                    if (
                        self._phase == 'wait-localization-manager'
                        and self._deadline is not None
                        and time.monotonic() > self._deadline
                    ):
                        # Read-only readiness can be retried; lifecycle mutations cannot.
                        self._localization_active.remove_pending_request(self._future)
                        self._future.cancel()
                        self._future = None
                        self._future_kind = None
                        return
                    if (
                        self._deadline is not None
                        and time.monotonic() > self._deadline
                        and not self._failed
                    ):
                        self._failed = True
                        self.node.get_logger().error(
                            'AMCL epoch reset timed out; recovery remains held.'
                        )
                        # A timeout is not proof the server stopped executing. Do
                        # not start overlapping transitions or cancel-and-retry.
                    return
                future, self._future = self._future, None
                self._future_kind = None
                if token != self._token:
                    self._phase = 'wait-localization-manager'
                    self._failed = False
                else:
                    try:
                        result = future.result()
                        if self._phase == 'wait-localization-manager':
                            if result.success:
                                self._phase = 'inspect'
                        elif self._phase == 'inspect':
                            self._phase = {1: 'reset-parameters', 2: 'cleanup', 3: 'deactivate'}[
                                result.current_state.id
                            ]
                        elif self._phase == 'reset-parameters':
                            if not result.result.successful:
                                raise RuntimeError('AMCL reset parameters refused')
                            self._phase = 'configure'
                        elif not result.success:
                            raise RuntimeError('lifecycle transition refused')
                        elif self._phase == 'pause-navigation':
                            self._phase = 'resume-navigation'
                        elif self._phase == 'resume-navigation':
                            self._anchor_localization()
                            self._navigation_ready = True
                            self._phase = 'navigation-ready'
                            return
                        elif self._phase == 'activate':
                            self.recovery.reset_complete(
                                token, self.node.get_clock().now().nanoseconds
                            )
                            self._failed = False
                            return
                        else:
                            self._phase = {
                                'deactivate': 'cleanup',
                                'cleanup': 'reset-parameters',
                                'configure': 'activate',
                            }[self._phase]
                    except Exception as exc:
                        self._failed = True
                        self.node.get_logger().error(
                            f'AMCL epoch reset failed; recovery held: {exc}'
                        )
            if not self.guard.ready or token is None:
                return
            if token != self._token:
                self._token = token
                self._phase = 'wait-localization-manager'
                self._failed = False
                self._navigation_ready = False
                self._accepted_seed_stamp = None
                self._pending_pose = None
                self.withdraw()
            if self.recovery.stage == EpochRecovery.WAIT_RESET and not self._failed:
                self._request(self._phase)
            elif (
                self.recovery.stage == EpochRecovery.READY
                and not self._navigation_ready
                and not self._failed
            ):
                if self._phase not in ('pause-navigation', 'resume-navigation'):
                    self._phase = 'pause-navigation'
                self._request(self._phase)
