"""Bridge-owned origin guard and asynchronous AMCL recovery coordinator."""

import math
import socket
import threading
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from .epoch_recovery import EpochRecovery
from .odom_epoch import OdomEpochGuard


class EpochRuntime:
    def __init__(self, node):
        self.node = node
        self.lock = threading.RLock()
        self.guard = OdomEpochGuard()
        self.recovery = EpochRecovery()
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
            Trigger, '/lifecycle_manager_localization/is_active')
        self._phase = 'wait-localization-manager'
        self._future = None
        self._deadline = None
        self._failed = False
        self._navigation_ready = False
        self._navigation = node.create_client(
            ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        self._accepted_seed_stamp = None
        self._pending_pose = None
        self._parameters = node.create_client(
            SetParametersAtomically, '/amcl/set_parameters_atomically')
        self._get = node.create_client(GetState, '/amcl/get_state')
        self._change = node.create_client(ChangeState, '/amcl/change_state')
        self._seed_sub = node.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.seed, 10)
        self._ack_sub = node.create_subscription(
            PoseWithCovarianceStamped, '/amcl/accepted_initial_pose', self.seed_accepted, 10)
        self._pose_sub = node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose, 10)
        self._timer = node.create_timer(.05, self.tick)

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

    def withdraw(self):
        self.first_tf_ns = None
        self._tf_generation += 1
        self._tf_ready_at = None
        self._tf_ready_since_ns = None
        with self.node._pose_lock:
            self.node._pose_received = False
            self.node._pose_at = 0.0

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
            EpochRecovery.WAIT_SEED, EpochRecovery.WAIT_POSE,
            EpochRecovery.READY)

    @property
    def commands_allowed(self):
        with self.lock:
            if not (self.guard.ready and not self.recovery.required
                    and self._navigation_ready and self.first_tf_ns is not None):
                return False
            # A valid origin does not make old odometry current. Apply the same
            # receipt-age limit as TF, even if AUX traffic keeps TCP alive.
            with self.node._pose_lock:
                age = time.monotonic() - self.node._pose_at
                return (self.node._pose_received and math.isfinite(age)
                        and 0.0 <= age <= self.node._odom_tf_timeout)

    @property
    def seed_tf_ready(self):
        if (not self.streams_allowed or self.first_tf_ns is None or
                self._tf_ready_at is None or self._tf_ready_token != self.recovery.token):
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
                valid = (self._tf_probe_generation == self._tf_generation and
                         self._tf_probe_token == self.recovery.token and
                         0.0 <= age <= self.node._odom_tf_timeout)
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
        if (not self.streams_allowed or self.first_tf_ns is None):
            self._tf_ready_at = None
            self._tf_ready_since_ns = None
            return
        if (self._tf_probe is None and now - self._tf_last_query >= .1 and
                self._tf_readiness.service_is_ready()):
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
        if (self.recovery.stage == EpochRecovery.WAIT_SEED and
                self.first_tf_ns is None):
            return 'wait-first-odometry'
        if self.recovery.stage == EpochRecovery.WAIT_SEED and not self.seed_tf_ready:
            return 'wait-amcl-tf'
        if self._failed:
            return 'failed:' + self._phase
        if self.recovery.stage == EpochRecovery.READY and not self._navigation_ready:
            return self._phase if self._phase.endswith('navigation') else 'wait-navigation'
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
            pose = message.pose.pose
            q = pose.orientation
            values = (pose.position.x, pose.position.y, pose.position.z,
                      q.x, q.y, q.z, q.w, *message.pose.covariance)
            if (not self.seed_tf_ready or self._tf_ready_since_ns is None
                    or self.stamp_ns(message) < self._tf_ready_since_ns
                    or self.stamp_ns(message) < self.first_tf_ns
                    or message.header.frame_id != 'map'
                    or not all(math.isfinite(v) for v in values)
                    or sum(v * v for v in (q.x, q.y, q.z, q.w)) == 0
                    or any(message.pose.covariance[i] <= 0 for i in (0, 7, 35))):
                return
            if self.recovery.operator_seed(
                    self.recovery.token, self.stamp_ns(message),
                    self.node.get_clock().now().nanoseconds):
                self._pending_pose = None
                self._navigation_ready = False

    def pose(self, message):
        with self.lock:
            if not self.streams_allowed or message.header.frame_id != 'map':
                return
            p = message.pose.pose
            if not all(math.isfinite(v) for v in (
                    p.position.x, p.position.y, p.position.z,
                    p.orientation.x, p.orientation.y, p.orientation.z,
                    p.orientation.w, *message.pose.covariance)):
                return
            self._pending_pose = message
            self._accept_pending_pose()

    def seed_accepted(self, message):
        with self.lock:
            stamp = self.stamp_ns(message)
            if (message.header.frame_id != 'map' or not self.streams_allowed
                    or self.recovery._reset_ns is None
                    or stamp <= self.recovery._reset_ns):
                return
            self._accepted_seed_stamp = stamp
            self._accept_pending_pose()

    def _accept_pending_pose(self):
        if (self._pending_pose is None or
                self._accepted_seed_stamp != self.recovery._seed_ns):
            return
        self.recovery.amcl_pose(
            self.recovery.token, self.stamp_ns(self._pending_pose),
            self.node.get_clock().now().nanoseconds)


    def _request(self, phase):
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
            request.command = (request.PAUSE if phase == 'pause-navigation'
                               else request.RESUME)
            self._future = self._navigation.call_async(request)
        elif phase == 'reset-parameters':
            if not self._parameters.service_is_ready():
                return
            request = SetParametersAtomically.Request()
            request.parameters = [
                Parameter('set_initial_pose', value=False).to_parameter_msg(),
                Parameter('always_reset_initial_pose', value=True).to_parameter_msg()]
            self._future = self._parameters.call_async(request)
        else:
            if not self._change.service_is_ready():
                return
            request = ChangeState.Request()
            request.transition.id = {
                'deactivate': 4, 'cleanup': 2,
                'configure': 1, 'activate': 3}[phase]
            self._future = self._change.call_async(request)
        self._deadline = (None if phase == 'wait-localization-manager' else
                          time.monotonic() + 10.0)

    def tick(self):
        with self.lock:
            self.recovery.observe_clock(self.node.get_clock().now().nanoseconds)
            token = self.recovery.token
            self._poll_tf_readiness()
            if self._future is not None:
                if not self._future.done():
                    if (self._deadline is not None and
                            time.monotonic() > self._deadline and not self._failed):
                        self._failed = True
                        self.node.get_logger().error(
                            'AMCL epoch reset timed out; recovery remains held.')
                    # A timeout is not proof the server stopped executing. Do
                    # not start overlapping transitions or cancel-and-retry.
                    return
                future, self._future = self._future, None
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
                            self._phase = {
                                1: 'reset-parameters', 2: 'cleanup',
                                3: 'deactivate'}[result.current_state.id]
                        elif self._phase == 'reset-parameters':
                            if not result.result.successful:
                                raise RuntimeError('AMCL reset parameters refused')
                            self._phase = 'configure'
                        elif not result.success:
                            raise RuntimeError('lifecycle transition refused')
                        elif self._phase == 'pause-navigation':
                            self._phase = 'resume-navigation'
                        elif self._phase == 'resume-navigation':
                            self._navigation_ready = True
                            self._phase = 'navigation-ready'
                            return
                        elif self._phase == 'activate':
                            self.recovery.reset_complete(
                                token, self.node.get_clock().now().nanoseconds)
                            self._failed = False
                            return
                        else:
                            self._phase = {
                                'deactivate': 'cleanup', 'cleanup': 'reset-parameters',
                                'configure': 'activate'}[self._phase]
                    except Exception as exc:
                        self._failed = True
                        self.node.get_logger().error(
                            f'AMCL epoch reset failed; recovery held: {exc}')
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
            elif (self.recovery.stage == EpochRecovery.READY and
                    not self._navigation_ready and not self._failed):
                if self._phase not in ('pause-navigation', 'resume-navigation'):
                    self._phase = 'pause-navigation'
                self._request(self._phase)
