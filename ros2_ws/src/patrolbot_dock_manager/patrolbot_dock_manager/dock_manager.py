"""Fail-closed, cancelable PatrolBot guarded-undock action server."""

from functools import partial
import math
import struct
import threading
import time
from uuid import uuid4

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import BackUp
from nav_msgs.msg import Odometry
from patrolbot_dock_manager.contracts import (
    dock_pose_initialization_failures,
    precheck_failures,
    reverse_failures,
    SafetySnapshot,
    transition_allowed,
    validate_goal,
)
from patrolbot_interfaces.action import Undock
from patrolbot_interfaces.msg import BaseState, DockReadiness
from patrolbot_interfaces.srv import ChargeRelease, InitializeDockPose
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Empty


class DockManager(Node):
    """Own hardware release orchestration and guarded Nav2 reverse motion."""

    def __init__(self, parameter_overrides=None):
        super().__init__(
            'patrolbot_dock_manager', parameter_overrides=parameter_overrides)
        self._callbacks = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._goal_reserved = False

        self._base_state = None
        self._base_state_at = 0.0
        self._sonar_at = 0.0
        self._rear_clear = False
        self._odom = None
        self._odom_at = 0.0
        self._localization = None
        self._localization_at = 0.0
        self._state_name = 'IDLE'
        self._state_started = time.monotonic()
        self._action_started = self._state_started
        self._backup_feedback_distance = 0.0
        self._path_checked_at = 0.0
        self._path_valid = False
        self._path_reason = 'not checked'
        self._other_action_active = {}
        self._owned_backup_goal_id = None
        self._active_correlation_id = ''
        self._last_localization_update_request_at = 0.0

        self.declare_parameter('dock_pose_configured', False)
        self.declare_parameter('dock_id', 'main_charger')
        self.declare_parameter('dock_pose_x', 0.0)
        self.declare_parameter('dock_pose_y', 0.0)
        self.declare_parameter('dock_pose_yaw', 0.0)
        self.declare_parameter('dock_position_tolerance', 0.25)
        self.declare_parameter('dock_yaw_tolerance', 0.35)
        self.declare_parameter('max_position_variance', 0.25)
        self.declare_parameter('max_yaw_variance', 0.25)
        self.declare_parameter('allow_front_bumper_at_dock', False)
        self.declare_parameter('sensor_timeout', 0.5)
        self.declare_parameter('operator_motor_window', 10.0)
        self.declare_parameter('precheck_timeout', 5.0)
        self.declare_parameter('backup_timeout', 10.0)
        self.declare_parameter('dock_pose_initialization_timeout', 5.0)
        self.declare_parameter('initial_pose_xy_variance', 0.04)
        self.declare_parameter('initial_pose_yaw_variance', 0.0685)

        self.create_subscription(
            BaseState, '/patrolbot/base_state', self._base_state_callback, 10,
            callback_group=self._callbacks)
        self.create_subscription(
            PointCloud2, '/sonar', self._sonar_callback,
            qos_profile_sensor_data, callback_group=self._callbacks)
        self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10,
            callback_group=self._callbacks)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self._localization_callback, 10, callback_group=self._callbacks)
        self._status_subscriptions = []
        for action_name in (
                'navigate_to_pose', 'navigate_through_poses', 'follow_waypoints',
                'follow_path', 'spin', 'wait', 'drive_on_heading',
                'assisted_teleop', 'backup'):
            self._other_action_active[action_name] = False
            self._status_subscriptions.append(self.create_subscription(
                GoalStatusArray, f'/{action_name}/_action/status',
                partial(self._action_status_callback, action_name),
                qos_profile_action_status_default,
                callback_group=self._callbacks))

        self._release_client = self.create_client(
            ChargeRelease, '/patrolbot/charge_release',
            callback_group=self._callbacks)
        self._localization_update_client = self.create_client(
            Empty, '/request_nomotion_update',
            callback_group=self._callbacks)
        self._backup_client = ActionClient(
            self, BackUp, '/backup', callback_group=self._callbacks)
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._readiness_pub = self.create_publisher(
            DockReadiness, '/patrolbot/dock_readiness', 10)
        self._initialize_pose_service = self.create_service(
            InitializeDockPose, '/patrolbot/initialize_dock_pose',
            self._initialize_dock_pose_callback,
            callback_group=self._callbacks)
        self._undock_server = ActionServer(
            self, Undock, '/patrolbot/undock',
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=self._callbacks)
        self.create_timer(
            0.5, self._readiness_timer, callback_group=self._callbacks)
        self.get_logger().info(
            'Guarded undock manager ready. Motion remains fail-closed until '
            'bumper mapping and dock pose are explicitly configured.')

    def _configured_dock_pose(self):
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = float(
            self.get_parameter('dock_pose_x').value)
        pose.pose.pose.position.y = float(
            self.get_parameter('dock_pose_y').value)
        yaw = float(self.get_parameter('dock_pose_yaw').value)
        pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose.pose.covariance[0] = float(
            self.get_parameter('initial_pose_xy_variance').value)
        pose.pose.covariance[7] = float(
            self.get_parameter('initial_pose_xy_variance').value)
        pose.pose.covariance[35] = float(
            self.get_parameter('initial_pose_yaw_variance').value)
        return pose

    def _initialize_dock_pose_callback(self, request, response):
        """Publish a configured dock pose only after zero-motion dock checks."""
        started = time.monotonic()
        response.dock_id = request.dock_id
        if not request.operator_authorized:
            response.code = InitializeDockPose.Response.REJECTED
            response.message = 'explicit operator authorization is required'
            return self._finish_initialization_response(response, started)
        configured_id = str(self.get_parameter('dock_id').value)
        if request.dock_id != configured_id:
            response.code = InitializeDockPose.Response.REJECTED
            response.message = f'unknown dock_id: {request.dock_id}'
            return self._finish_initialization_response(response, started)
        if not bool(self.get_parameter('dock_pose_configured').value):
            response.code = InitializeDockPose.Response.NOT_CONFIGURED
            response.message = 'dock pose is not commissioned'
            return self._finish_initialization_response(response, started)
        with self._goal_lock:
            if self._goal_reserved:
                response.code = InitializeDockPose.Response.PRECONDITION_FAILED
                response.message = 'an undock goal is active'
                return self._finish_initialization_response(response, started)
        failures = dock_pose_initialization_failures(self._snapshot())
        if failures:
            response.code = InitializeDockPose.Response.PRECONDITION_FAILED
            response.message = '; '.join(failures)
            return self._finish_initialization_response(response, started)

        requested_pose = self._configured_dock_pose()
        response.requested_pose = requested_pose
        published_at = time.monotonic()
        self._initial_pose_pub.publish(requested_pose)
        self.get_logger().info(
            f'operator-authorized dock pose published for {configured_id}; '
            'waiting for AMCL convergence')
        deadline = published_at + float(
            self.get_parameter('dock_pose_initialization_timeout').value)
        while time.monotonic() < deadline:
            with self._lock:
                pose = self._localization
                pose_at = self._localization_at
            valid, at_dock = self._localization_health(pose)
            if pose is not None and pose_at > published_at and valid and at_dock:
                response.success = True
                response.code = InitializeDockPose.Response.OK
                response.message = 'AMCL converged at configured dock pose'
                response.confirmed_pose = pose
                return self._finish_initialization_response(response, started)
            time.sleep(0.05)
        response.code = InitializeDockPose.Response.TIMEOUT
        response.message = 'AMCL did not converge at the dock before timeout'
        return self._finish_initialization_response(response, started)

    @staticmethod
    def _finish_initialization_response(response, started):
        response.elapsed_seconds = float(time.monotonic() - started)
        return response

    def _readiness_timer(self):
        snapshot = self._snapshot()
        now = time.monotonic()
        with self._lock:
            base_age = now - self._base_state_at if self._base_state_at else 1e6
            sonar_age = now - self._sonar_at if self._sonar_at else 1e6
            odom_age = now - self._odom_at if self._odom_at else 1e6
            pose_age = (
                now - self._localization_at if self._localization_at else 1e6)
            state_name = self._state_name
        failures = precheck_failures(
            snapshot,
            bool(self.get_parameter('allow_front_bumper_at_dock').value))
        message = DockReadiness()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.ready_for_undock = not failures
        for field in (
                'link_connected', 'hardware_state_valid', 'charge_state_valid',
                'motors_enabled', 'estop_pressed', 'bumpers_valid',
                'sonar_fresh', 'rear_clear', 'odom_fresh', 'stationary',
                'localization_fresh', 'localization_valid', 'dock_pose_valid',
                'command_path_valid', 'navigation_idle'):
            setattr(message, field, getattr(snapshot, field))
        message.session_generation = snapshot.session_generation
        message.charge_state = snapshot.charge_state
        message.fault_flags = snapshot.fault_flags
        message.base_state_age = float(base_age)
        message.sonar_age = float(sonar_age)
        message.odom_age = float(odom_age)
        message.localization_age = float(pose_age)
        message.manager_state = state_name
        message.blocking_reason = '; '.join(failures) if failures else 'ready'
        self._readiness_pub.publish(message)

    def _base_state_callback(self, message):
        with self._lock:
            self._base_state = message
            self._base_state_at = time.monotonic()

    def _sonar_callback(self, message):
        try:
            rear_clear = not self._cloud_has_rear_obstacle(message)
        except (IndexError, KeyError, struct.error, ValueError):
            rear_clear = False
        with self._lock:
            # An empty but well-formed, freshly received cloud is clear. Stale
            # detection is based on arrival age, never point count (amendment A-5).
            self._rear_clear = rear_clear
            self._sonar_at = time.monotonic()

    @staticmethod
    def _cloud_has_rear_obstacle(message):
        fields = {field.name: field for field in message.fields}
        x_field = fields['x']
        y_field = fields['y']
        if (x_field.datatype != PointField.FLOAT32 or
                y_field.datatype != PointField.FLOAT32):
            raise ValueError('sonar x/y must be FLOAT32')
        if message.point_step <= max(x_field.offset, y_field.offset) + 3:
            raise ValueError('invalid point_step')
        endian = '>' if message.is_bigendian else '<'
        for row in range(message.height):
            row_start = row * message.row_step
            for column in range(message.width):
                start = row_start + column * message.point_step
                x = struct.unpack_from(
                    endian + 'f', message.data, start + x_field.offset)[0]
                y = struct.unpack_from(
                    endian + 'f', message.data, start + y_field.offset)[0]
                if not (math.isfinite(x) and math.isfinite(y)):
                    raise ValueError('non-finite sonar point')
                if -0.60 <= x <= -0.15 and abs(y) <= 0.35:
                    return True
        return False

    def _odom_callback(self, message):
        with self._lock:
            self._odom = message
            self._odom_at = time.monotonic()

    def _localization_callback(self, message):
        with self._lock:
            self._localization = message
            self._localization_at = time.monotonic()

    def _action_status_callback(self, action_name, message):
        with self._lock:
            owned_backup = self._owned_backup_goal_id
        active = False
        for status in message.status_list:
            if status.status not in (
                    GoalStatus.STATUS_ACCEPTED,
                    GoalStatus.STATUS_EXECUTING,
                    GoalStatus.STATUS_CANCELING):
                continue
            goal_id = bytes(status.goal_info.goal_id.uuid)
            if action_name != 'backup' or goal_id != owned_backup:
                active = True
                break
        with self._lock:
            self._other_action_active[action_name] = active

    def _goal_callback(self, request):
        if not request.operator_authorized:
            self.get_logger().error('Undock goal rejected: operator authorization absent.')
            return GoalResponse.REJECT
        if request.dock_id != self.get_parameter('dock_id').value:
            self.get_logger().error(
                f'Undock goal rejected: unknown dock ID {request.dock_id!r}.')
            return GoalResponse.REJECT
        failures = validate_goal(
            request.reverse_distance, request.reverse_speed,
            request.validation_mode)
        if failures:
            self.get_logger().error(
                f'Undock goal rejected: {"; ".join(failures)}')
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._goal_reserved:
                self.get_logger().error('Undock goal rejected: another goal is active.')
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def _transition(self, state, correlation_id):
        if not transition_allowed(self._state_name, state):
            raise RuntimeError(
                f'invalid undock transition {self._state_name}->{state}')
        self._state_name = state
        self._state_started = time.monotonic()
        self.get_logger().info(f'undock={correlation_id} state={state}')

    def _execute_callback(self, goal_handle):
        correlation_id = str(uuid4())
        self._active_correlation_id = correlation_id
        self._action_started = time.monotonic()
        initial_charge = -1
        try:
            self._transition('PRECHECK', correlation_id)
            snapshot = self._snapshot()
            initial_charge = snapshot.charge_state
            precheck_ok, reason = self._wait_for_precheck(
                goal_handle, correlation_id)
            if not precheck_ok:
                return self._abort(
                    goal_handle, Undock.Result.PRECHECK_FAILED, reason,
                    initial_charge, manual_recovery=False)
            expected_session = self._snapshot().session_generation

            self._transition('REQUEST_DOCK_RELEASE', correlation_id)
            release = ChargeRelease.Request()
            release.request_id = correlation_id
            if not self._release_client.wait_for_service(timeout_sec=1.0):
                return self._abort(
                    goal_handle, Undock.Result.RELEASE_FAILED,
                    'charge-release service unavailable', initial_charge,
                    manual_recovery=False)
            release_future = self._release_client.call_async(release)
            self._transition('WAIT_FOR_CHARGE_RELEASE', correlation_id)
            if not self._wait_future(
                    release_future, goal_handle, 7.0, correlation_id):
                return self._abort(
                    goal_handle, Undock.Result.RELEASE_FAILED,
                    'charge-release timeout or cancellation', initial_charge,
                    manual_recovery=True)
            release_response = release_future.result()
            if release_response is None or not release_response.success:
                message = (
                    release_response.message if release_response is not None
                    else 'charge-release returned no result')
                return self._abort(
                    goal_handle, Undock.Result.RELEASE_FAILED, message,
                    initial_charge, manual_recovery=True)
            if release_response.session_generation != expected_session:
                return self._abort(
                    goal_handle, Undock.Result.SESSION_CHANGED,
                    'hardware session changed during release', initial_charge,
                    manual_recovery=True)

            # Amendment A-1: E-stop and motor verification share one 10-second
            # operator window. The operator may use physical PWR ON or invoke the
            # separate /patrolbot/motor_enable service deliberately. This action
            # never enables motors itself.
            self._transition('VERIFY_ESTOP_AND_MOTOR', correlation_id)
            window = float(self.get_parameter('operator_motor_window').value)
            if not self._wait_for_operator_motor_state(
                    goal_handle, expected_session, window, correlation_id):
                return self._abort(
                    goal_handle, Undock.Result.MOTOR_STATE_FAILED,
                    'E-stop did not clear and motors did not enable in operator window',
                    initial_charge, manual_recovery=True)

            self._transition('VERIFY_REAR_SONAR', correlation_id)
            allow_front = bool(
                self.get_parameter('allow_front_bumper_at_dock').value)
            failures = reverse_failures(
                self._snapshot(), expected_session,
                allow_front_bumper_while_reversing=allow_front)
            if failures:
                return self._abort(
                    goal_handle, Undock.Result.SENSOR_FAILED,
                    '; '.join(failures), initial_charge,
                    manual_recovery=True)

            self._transition('VERIFY_LOCALIZATION', correlation_id)
            if not self._wait_for_localization(
                    goal_handle, 1.0, correlation_id):
                return self._abort(
                    goal_handle, Undock.Result.LOCALIZATION_FAILED,
                    'localization invalid immediately before reverse',
                    initial_charge, manual_recovery=True)

            self._transition('VERIFY_COMMAND_PATH', correlation_id)
            self._refresh_command_path(force=True)
            if not self._snapshot().command_path_valid:
                return self._abort(
                    goal_handle, Undock.Result.COMMAND_PATH_FAILED,
                    self._path_reason, initial_charge, manual_recovery=True)

            if goal_handle.is_cancel_requested:
                return self._canceled(goal_handle, initial_charge)
            self._transition('GUARDED_REVERSE', correlation_id)
            motion_ok, motion_reason, traveled = self._run_backup(
                goal_handle, expected_session,
                goal_handle.request.reverse_distance,
                goal_handle.request.reverse_speed,
                correlation_id)
            if not motion_ok:
                code = (Undock.Result.CANCELED if goal_handle.is_cancel_requested
                        else Undock.Result.MOTION_FAILED)
                return self._abort(
                    goal_handle, code, motion_reason, initial_charge,
                    manual_recovery=True, distance=traveled)

            self._transition('CLEAR_OF_DOCK', correlation_id)
            if not self._wait_clear_of_dock(
                    goal_handle, 2.0, correlation_id):
                return self._abort(
                    goal_handle, Undock.Result.SAFETY_FAULT,
                    'robot did not become stationary with charge and all '
                    'bumper contacts clear after BackUp',
                    initial_charge, manual_recovery=True, distance=traveled)
            self._transition('STOPPED', correlation_id)
            self._transition('COMPLETE', correlation_id)
            result = self._result(
                True, Undock.Result.OK, 'guarded undock complete',
                initial_charge, False, traveled)
            goal_handle.succeed()
            return result
        except Exception as exc:  # Fail closed; action never publishes velocity.
            self.get_logger().exception(
                f'undock={correlation_id} unhandled failure: {exc}')
            return self._abort(
                goal_handle, Undock.Result.SAFETY_FAULT,
                f'unhandled fail-closed error: {exc}', initial_charge,
                manual_recovery=True)
        finally:
            with self._goal_lock:
                self._goal_reserved = False
            with self._lock:
                self._owned_backup_goal_id = None
            self._active_correlation_id = ''
            if self._state_name in ('COMPLETE', 'ABORTED', 'FAULT'):
                self._state_name = 'IDLE'

    def _wait_for_precheck(self, goal_handle, correlation_id):
        deadline = time.monotonic() + float(
            self.get_parameter('precheck_timeout').value)
        stable_since = None
        last_failures = ['precheck not evaluated']
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False, 'goal canceled during precheck'
            self._request_localization_update()
            failures = precheck_failures(
                self._snapshot(), bool(
                    self.get_parameter('allow_front_bumper_at_dock').value))
            if failures:
                stable_since = None
                last_failures = failures
            elif stable_since is None:
                stable_since = time.monotonic()
            elif time.monotonic() - stable_since >= 1.0:
                return True, ''
            self._publish_feedback(goal_handle, deadline, correlation_id)
            time.sleep(0.05)
        return False, '; '.join(last_failures)

    def _wait_for_operator_motor_state(
            self, goal_handle, expected_session, timeout, correlation_id):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            self._request_localization_update()
            state = self._snapshot()
            if (state.session_generation != expected_session or
                    not state.link_connected or not state.hardware_state_valid):
                return False
            if (state.charge_state_valid and state.charge_state == 0 and
                    not state.estop_pressed and state.motors_enabled):
                return True
            self._publish_feedback(goal_handle, deadline, correlation_id)
            time.sleep(0.05)
        return False

    def _wait_for_localization(
            self, goal_handle, timeout, correlation_id):
        """Force a stationary AMCL update and require a fresh valid estimate."""
        deadline = time.monotonic() + timeout
        self._request_localization_update(force=True)
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            self._request_localization_update()
            state = self._snapshot()
            if state.localization_fresh and state.localization_valid:
                return True
            self._publish_feedback(goal_handle, deadline, correlation_id)
            time.sleep(0.05)
        return False

    def _request_localization_update(self, force=False):
        """Ask AMCL to update from the next scan even without odometry motion."""
        now = time.monotonic()
        if (not force and
                now - self._last_localization_update_request_at < 0.20):
            return False
        if not self._localization_update_client.service_is_ready():
            return False
        self._localization_update_client.call_async(Empty.Request())
        self._last_localization_update_request_at = now
        return True

    def _run_backup(
            self, goal_handle, expected_session, distance, speed,
            correlation_id):
        if not self._backup_client.wait_for_server(timeout_sec=2.0):
            return False, 'Nav2 BackUp action unavailable', 0.0
        backup = BackUp.Goal()
        # Nav2 specifies positive target.x and positive speed for BackUp; the
        # behavior itself produces negative linear velocity.
        backup.target.x = float(distance)
        backup.speed = float(speed)
        backup.time_allowance.sec = int(
            self.get_parameter('backup_timeout').value)
        backup.time_allowance.nanosec = 0
        self._backup_feedback_distance = 0.0
        send_future = self._backup_client.send_goal_async(
            backup, feedback_callback=self._backup_feedback_callback)
        if not self._wait_future(
                send_future, goal_handle, 2.0, correlation_id):
            return False, 'BackUp goal acceptance timeout', 0.0
        nav_goal = send_future.result()
        if nav_goal is None or not nav_goal.accepted:
            return False, 'BackUp goal rejected', 0.0
        with self._lock:
            self._owned_backup_goal_id = bytes(nav_goal.goal_id.uuid)
            # A status sample may have arrived between goal acceptance and the
            # client receiving its ID. The pre-send gate proved BackUp idle;
            # subsequent status samples still detect any foreign goal.
            self._other_action_active['backup'] = False
        result_future = nav_goal.get_result_async()
        deadline = time.monotonic() + float(
            self.get_parameter('backup_timeout').value)
        reason = ''
        while not result_future.done() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                reason = 'goal canceled during reverse'
                break
            self._request_localization_update()
            failures = reverse_failures(
                self._snapshot(), expected_session,
                allow_front_bumper_while_reversing=bool(
                    self.get_parameter(
                        'allow_front_bumper_at_dock').value))
            if failures:
                reason = '; '.join(failures)
                break
            self._publish_feedback(goal_handle, deadline, correlation_id)
            time.sleep(0.05)
        if reason or not result_future.done():
            if not reason:
                reason = 'BackUp hard timeout'
            cancel_future = nav_goal.cancel_goal_async()
            cancel_deadline = time.monotonic() + 1.0
            while not cancel_future.done() and time.monotonic() < cancel_deadline:
                time.sleep(0.02)
            if not self._wait_for_odom_zero(1.5):
                reason += '; zero velocity after cancellation was not confirmed'
            with self._lock:
                self._owned_backup_goal_id = None
            return False, reason, self._backup_feedback_distance
        wrapped = result_future.result()
        with self._lock:
            self._owned_backup_goal_id = None
        if (wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED or
                wrapped.result.error_code != BackUp.Result.NONE):
            message = 'BackUp failed'
            if wrapped is not None:
                message = wrapped.result.error_msg or message
            return False, message, self._backup_feedback_distance
        return True, '', self._backup_feedback_distance

    def _backup_feedback_callback(self, feedback):
        self._backup_feedback_distance = float(
            feedback.feedback.distance_traveled)

    def _wait_future(
            self, future, goal_handle, timeout, correlation_id):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            self._publish_feedback(goal_handle, deadline, correlation_id)
            time.sleep(0.05)
        return future.done()

    def _wait_clear_of_dock(
            self, goal_handle, timeout, correlation_id):
        deadline = time.monotonic() + timeout
        stable_since = None
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            state = self._snapshot()
            clear_and_stopped = (
                state.odom_fresh and state.stationary and
                state.charge_state_valid and state.charge_state == 0 and
                state.bumpers_valid and not state.front_bumper_pressed and
                not state.rear_bumper_pressed)
            if clear_and_stopped:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= 0.5:
                    return True
            else:
                stable_since = None
            self._publish_feedback(goal_handle, deadline, correlation_id)
            time.sleep(0.05)
        return False

    def _wait_for_odom_zero(self, timeout):
        deadline = time.monotonic() + timeout
        stable_since = None
        while time.monotonic() < deadline:
            state = self._snapshot()
            if state.odom_fresh and state.stationary:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= 0.25:
                    return True
            else:
                stable_since = None
            time.sleep(0.02)
        return False

    def _snapshot(self):
        now = time.monotonic()
        timeout = float(self.get_parameter('sensor_timeout').value)
        self._refresh_command_path()
        with self._lock:
            base = self._base_state
            base_age = now - self._base_state_at if self._base_state_at else math.inf
            sonar_age = now - self._sonar_at if self._sonar_at else math.inf
            odom = self._odom
            odom_age = now - self._odom_at if self._odom_at else math.inf
            pose = self._localization
            pose_age = (
                now - self._localization_at if self._localization_at else math.inf)
            rear_clear = self._rear_clear
            navigation_idle = not any(self._other_action_active.values())

        localization_valid, dock_pose_valid = self._localization_health(pose)
        stationary = False
        if odom is not None:
            stationary = (
                abs(odom.twist.twist.linear.x) <= 0.005 and
                abs(odom.twist.twist.angular.z) <= 0.01)
        return SafetySnapshot(
            link_connected=bool(base and base.link_connected),
            hardware_state_valid=bool(
                base and base.hardware_state_valid and base_age <= timeout),
            session_generation=base.session_generation if base else 0,
            charge_state_valid=bool(base and base.charge_state_valid),
            charge_state=base.charge_state if base else -1,
            motors_enabled=bool(base and base.motors_enabled),
            estop_pressed=True if base is None else base.estop_pressed,
            fault_flags=base.fault_flags if base else 0,
            bumpers_valid=bool(base and base.bumpers_valid),
            front_bumper_pressed=bool(base and base.front_bumper_pressed),
            rear_bumper_pressed=bool(base and base.rear_bumper_pressed),
            sonar_fresh=sonar_age <= timeout,
            rear_clear=rear_clear,
            odom_fresh=odom_age <= timeout,
            stationary=stationary,
            localization_fresh=pose_age <= timeout,
            localization_valid=localization_valid and pose_age <= timeout,
            dock_pose_valid=dock_pose_valid and pose_age <= timeout,
            command_path_valid=self._path_valid,
            navigation_idle=navigation_idle,
        )

    def _localization_health(self, pose):
        if pose is None:
            return False, False
        covariance = pose.pose.covariance
        values = (
            pose.pose.pose.position.x, pose.pose.pose.position.y,
            pose.pose.pose.orientation.z, pose.pose.pose.orientation.w,
            covariance[0], covariance[7], covariance[35])
        if not all(math.isfinite(value) for value in values):
            return False, False
        valid = (
            covariance[0] <= self.get_parameter('max_position_variance').value and
            covariance[7] <= self.get_parameter('max_position_variance').value and
            covariance[35] <= self.get_parameter('max_yaw_variance').value)
        if not bool(self.get_parameter('dock_pose_configured').value):
            return valid, False
        x = pose.pose.pose.position.x
        y = pose.pose.pose.position.y
        yaw = self._yaw(pose.pose.pose.orientation)
        dx = x - float(self.get_parameter('dock_pose_x').value)
        dy = y - float(self.get_parameter('dock_pose_y').value)
        target_yaw = float(self.get_parameter('dock_pose_yaw').value)
        yaw_error = math.atan2(
            math.sin(yaw - target_yaw), math.cos(yaw - target_yaw))
        at_dock = (
            math.hypot(dx, dy) <=
            float(self.get_parameter('dock_position_tolerance').value) and
            abs(yaw_error) <=
            float(self.get_parameter('dock_yaw_tolerance').value))
        return valid, valid and at_dock

    @staticmethod
    def _yaw(quaternion):
        siny = 2.0 * (
            quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy = 1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny, cosy)

    def _refresh_command_path(self, force=False):
        now = time.monotonic()
        if not force and now - self._path_checked_at < 0.5:
            return
        self._path_checked_at = now
        failures = []
        expected_path = (
            ('/input/nav_controller', {'controller_server'},
             {'nav_command_mux'}),
            ('/input/nav_behavior', {'behavior_server'},
             {'nav_command_mux'}),
            ('/input/docking', set(), {'nav_command_mux'}),
            ('/cmd_vel_nav', {'nav_command_mux'},
             {'velocity_smoother'}),
            ('/cmd_vel_smoothed', {'velocity_smoother'},
             {'collision_monitor'}),
            ('/input/navi', {'collision_monitor'}, {'cmd_vel_mux'}),
            ('/cmd_vel_out', {'cmd_vel_mux'}, {'teleop_velocity_smoother'}),
            ('/cmd_vel', {'teleop_velocity_smoother'}, {'patrolbot_bridge'}),
        )
        for topic, expected_publishers, required_subscribers in expected_path:
            publishers = {
                info.node_name
                for info in self.get_publishers_info_by_topic(topic)
            }
            subscribers = {
                info.node_name
                for info in self.get_subscriptions_info_by_topic(topic)
            }
            if publishers != expected_publishers:
                failures.append(
                    f'{topic} publishers={sorted(publishers)}, expected='
                    f'{sorted(expected_publishers)}')
            missing = required_subscribers - subscribers
            if missing:
                failures.append(
                    f'{topic} missing subscribers={sorted(missing)}')
        nodes = {name for name, _namespace in self.get_node_names_and_namespaces()}
        for required in (
                'nav_command_mux', 'collision_monitor', 'cmd_vel_mux',
                'teleop_velocity_smoother', 'patrolbot_safety_watchdog'):
            if required not in nodes:
                failures.append(f'required node absent: {required}')
        if 'docking_server' in nodes:
            failures.append('unsafe docking_server is present')
        self._path_valid = not failures
        self._path_reason = '; '.join(failures) if failures else 'valid'

    def _publish_feedback(self, goal_handle, deadline, _correlation_id):
        now = time.monotonic()
        snapshot = self._snapshot()
        with self._lock:
            sonar_age = now - self._sonar_at if self._sonar_at else 1e6
            base_age = now - self._base_state_at if self._base_state_at else 1e6
            pose_age = (
                now - self._localization_at if self._localization_at else 1e6)
        feedback = Undock.Feedback()
        feedback.state = self._state_name
        feedback.elapsed_seconds = float(now - self._action_started)
        feedback.state_timeout_remaining = float(max(0.0, deadline - now))
        feedback.sonar_age = float(sonar_age)
        feedback.base_state_age = float(base_age)
        feedback.localization_age = float(pose_age)
        feedback.rear_clear = snapshot.rear_clear and snapshot.sonar_fresh
        feedback.localization_valid = snapshot.localization_valid
        feedback.charge_state = snapshot.charge_state
        feedback.motors_enabled = snapshot.motors_enabled
        feedback.estop_pressed = snapshot.estop_pressed
        feedback.session_generation = snapshot.session_generation
        goal_handle.publish_feedback(feedback)

    def _result(
            self, success, code, message, initial_charge,
            manual_recovery, distance=0.0):
        snapshot = self._snapshot()
        result = Undock.Result()
        result.success = success
        result.code = code
        result.message = message
        result.final_state = self._state_name
        result.distance_traveled = float(distance)
        result.elapsed_seconds = float(time.monotonic() - self._action_started)
        result.initial_charge_state = int(initial_charge)
        result.final_charge_state = int(snapshot.charge_state)
        result.final_motors_enabled = snapshot.motors_enabled
        result.final_estop_pressed = snapshot.estop_pressed
        result.manual_recovery_required = manual_recovery
        with self._lock:
            if self._localization is not None:
                result.final_pose = self._localization
        return result

    def _abort(
            self, goal_handle, code, message, initial_charge,
            manual_recovery, distance=0.0):
        if goal_handle.is_cancel_requested or code == Undock.Result.CANCELED:
            return self._canceled(goal_handle, initial_charge, distance)
        self._transition(
            'FAULT' if code in (
                Undock.Result.SAFETY_FAULT, Undock.Result.SESSION_CHANGED)
            else 'ABORTED', self._active_correlation_id or 'terminal')
        goal_handle.abort()
        return self._result(
            False, code, message, initial_charge, manual_recovery, distance)

    def _canceled(self, goal_handle, initial_charge, distance=0.0):
        self._transition(
            'ABORTED', self._active_correlation_id or 'terminal')
        goal_handle.canceled()
        return self._result(
            False, Undock.Result.CANCELED, 'undock canceled',
            initial_charge, True, distance)

    def destroy_node(self):
        self._undock_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DockManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
