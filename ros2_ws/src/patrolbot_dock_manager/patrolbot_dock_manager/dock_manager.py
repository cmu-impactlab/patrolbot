"""
Fail-closed, cancelable PatrolBot undock orchestrator.

Contact separation is not performed here. The SBC owns charger release, motor
enable, reverse velocity, encoder displacement, charge-state recovery, stall
detection and rear sonar/bumper safety as one exclusive, stateful operation.
This node validates the Pi-side preconditions, requests that operation, relays
its progress, and resumes Nav2 once the SBC reports the robot physically clear.

It publishes no Twist and no longer uses Nav2 BackUp to escape the charging
contacts. The 180 degree turn afterwards is still a Nav2 Spin, so it keeps full
costmap and collision-monitor awareness; it runs only after the SBC has handed
motion authority back with the motors already live.
"""

from functools import partial
import math
import struct
import threading
import time
from uuid import uuid4

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import Spin
from nav_msgs.msg import Odometry
from patrolbot_dock_manager.epoch_policy import epoch_hardware_ready, lidar_ready
from patrolbot_dock_manager.contracts import (
    automatic_docking_allowed,
    DOCK_DOCKED_CONFIRMED,
    dock_pose_initialization_failures,
    DOCK_UNKNOWN,
    localization_position_limit,
    odometry_values_finite,
    planar_covariance_usable,
    precheck_failures,
    resume_failures,
    SafetySnapshot,
    transition_allowed,
    validate_goal,
)
from patrolbot_interfaces.action import HardwareUndock, Undock
from patrolbot_interfaces.msg import BaseState, DockReadiness
from patrolbot_interfaces.srv import InitializeDockPose
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Empty


# What counts as "not moving" when reading the /odom twist.
STATIONARY_LINEAR_LIMIT = 0.005
STATIONARY_ANGULAR_LIMIT = 0.01

# FAULTFLAGS bits 7 and 8, encoder_degraded_warning and encoder_failure (see
# patrolbot_bridge.safety_telemetry). Any parked-robot allowance is only as
# trustworthy as the odometry underneath it, so a wheel that is not being
# measured properly must void it instead of supplying convenient zeros.
ENCODER_FAULT_MASK = (1 << 7) | (1 << 8)


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
        # Monotonic time the robot was last observed moving. Zero means it has
        # not been seen to move in this process's lifetime.
        self._last_motion_at = 0.0
        self._localization = None
        self._localization_at = 0.0
        self._state_name = 'IDLE'
        self._state_started = time.monotonic()
        self._action_started = self._state_started
        self._hardware_distance = 0.0
        self._hardware_phase = ''
        self._hardware_release_attempts = 0
        self._redock_inhibit_remaining = 0.0
        self._path_checked_at = 0.0
        self._path_valid = False
        self._path_reason = 'not checked'
        self._other_action_active = {}
        self._owned_spin_goal_id = None
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
        # A frozen filter remains a refusal unless an on-site operator makes
        # the exceptional decision to accept its finite covariance values.
        self.declare_parameter('allow_degenerate_localization', False)
        # Seated on the charger, AMCL's position variance along the approach
        # axis never comes down: the dock fills the laser's near field and the
        # scan cannot constrain that one direction. Measured 0.57-0.63 over 70
        # forced no-motion updates while yy was 0.001 and yaw 0.0003, with the
        # pose itself accurate to ~6 cm against the surveyed dock pose. This
        # separate allowance applies ONLY while the SBC's dock observer reports
        # mated plates and the pose already agrees with the surveyed dock pose;
        # `max_position_variance` still guards everything else, because it also
        # gates general navigation.
        self.declare_parameter('docked_max_position_variance', 1.0)
        self.declare_parameter('max_yaw_variance', 0.25)
        self.declare_parameter('sensor_timeout', 0.5)
        # Every operator-triggered localization phase is bounded independently.
        # Only calls that actually reach AMCL consume this budget; an absent
        # service still gets the full phase timeout to appear before the action
        # fails closed.
        self.declare_parameter('localization_update_max_attempts', 5)
        self.declare_parameter('precheck_timeout', 5.0)
        # Requested of the SBC, which independently rejects anything outside
        # its own hard caps rather than clamping it.
        self.declare_parameter('undock_clearance_distance', 0.53)
        self.declare_parameter('undock_reverse_speed', 0.10)
        self.declare_parameter('undock_max_duration', 45.0)
        self.declare_parameter('undock_turn_yaw', math.pi)
        self.declare_parameter('spin_timeout', 20.0)
        self.declare_parameter('post_undock_localization_timeout', 5.0)
        self.declare_parameter('handoff_timeout', 4.0)
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
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose',
            self._initial_pose_callback, 10, callback_group=self._callbacks)
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

        self._localization_update_client = self.create_client(
            Empty, '/request_nomotion_update',
            callback_group=self._callbacks)
        # The one motion request this node makes for contact separation. There
        # is no BackUp client any more: a generic Nav2 behavior must never be
        # what pulls the robot off the charging contacts.
        self._hardware_undock_client = ActionClient(
            self, HardwareUndock, '/patrolbot/hardware_undock',
            callback_group=self._callbacks)
        self._spin_client = ActionClient(
            self, Spin, '/spin', callback_group=self._callbacks)
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
        if (self._base_state is None or getattr(self._base_state, 'map_id', '') != 'cmuq-floor2'):
            response.code = InitializeDockPose.Response.PRECONDITION_FAILED
            response.message = 'dock pose is valid only on confirmed Floor 2'
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
        failures = dock_pose_initialization_failures(
            self._snapshot(for_pose_initialization=True))
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
            valid, at_dock = self._localization_health(
                pose, docked=self._dock_contact_confirmed())
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
        failures = precheck_failures(snapshot)
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
        if snapshot.degenerate_localization_override_engaged:
            message.blocking_reason += (
                '; safety gate override: degenerate localization covariance '
                'accepted')
        # Undock readiness and docking inhibition are different facts with
        # different causes. After a successful undock the robot is not ready to
        # undock (it is already clear) and not allowed to dock (the inhibition
        # is running); collapsing those into one field hid both reasons.
        message.dock_state = snapshot.dock_state
        message.physically_clear = snapshot.physically_clear
        message.automatic_docking_allowed = automatic_docking_allowed(snapshot)
        message.redock_inhibited = snapshot.redock_inhibited
        message.redock_inhibit_remaining = self._redock_inhibit_remaining
        message.undock_profile_commissioned = (
            snapshot.undock_profile_commissioned)
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
        orientation = message.pose.pose.orientation
        if not odometry_values_finite(
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                (orientation.x, orientation.y, orientation.z, orientation.w),
                message.twist.twist.linear.x,
                message.twist.twist.angular.z):
            rejected_at = time.monotonic()
            with self._lock:
                self._odom = None
                self._odom_at = 0.0
                # Invalid odometry cannot prove the robot remained parked.
                self._last_motion_at = rejected_at
            self.get_logger().error(
                'Rejected non-finite /odom; freshness and parked-pose '
                'allowances invalidated.',
                throttle_duration_sec=5.0)
            return
        with self._lock:
            self._odom = message
            self._odom_at = time.monotonic()
            # Remember WHEN the robot was last seen moving, not merely whether
            # the newest sample reads zero. A pose may only be trusted past its
            # freshness timeout if nothing has moved since it was published,
            # and the latest twist alone cannot establish that: a robot that
            # drove away while AMCL was silent and then stopped reports zero
            # velocity too, and would otherwise revalidate the pose it held
            # before it moved.
            if (abs(message.twist.twist.linear.x) > STATIONARY_LINEAR_LIMIT or
                    abs(message.twist.twist.angular.z) >
                    STATIONARY_ANGULAR_LIMIT):
                self._last_motion_at = self._odom_at

    def _localization_callback(self, message):
        with self._lock:
            self._localization = message
            self._localization_at = time.monotonic()

    def _initial_pose_callback(self, _message):
        """Give each newly received pose seed at most one forced AMCL update."""
        self._request_localization_update(force=True)

    def _action_status_callback(self, action_name, message):
        with self._lock:
            owned_goal = {
                'spin': self._owned_spin_goal_id,
            }.get(action_name)
        active = False
        for status in message.status_list:
            if status.status not in (
                    GoalStatus.STATUS_ACCEPTED,
                    GoalStatus.STATUS_EXECUTING,
                    GoalStatus.STATUS_CANCELING):
                continue
            goal_id = bytes(status.goal_info.goal_id.uuid)
            if owned_goal is None or goal_id != owned_goal:
                active = True
                break
        with self._lock:
            self._other_action_active[action_name] = active

    def _goal_callback(self, request):
        if not request.operator_authorized:
            self.get_logger().error('Undock goal rejected: operator authorization absent.')
            return GoalResponse.REJECT
        state = self._snapshot(for_undock=True)
        if not (state.hardware_state_valid and state.charge_state_valid
                and state.charge_state > 0 and state.dock_state_valid
                and state.dock_state == DOCK_DOCKED_CONFIRMED):
            self.get_logger().error('Undock goal rejected: fresh charging required.')
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
        self._hardware_distance = 0.0
        self._hardware_phase = ''
        self._hardware_release_attempts = 0
        initial_charge = -1
        try:
            self._transition('PRECHECK', correlation_id)
            snapshot = self._snapshot()
            initial_charge = snapshot.charge_state
            precheck_ok, reason = self._wait_for_precheck(
                goal_handle, correlation_id,
                bool(goal_handle.request.validation_mode))
            if not precheck_ok:
                return self._abort(
                    goal_handle, Undock.Result.PRECHECK_FAILED, reason,
                    initial_charge, manual_recovery=False)
            expected_session = self._snapshot().session_generation

            # AMCL can age out while the robot sits still, and the SBC will move
            # it in a moment, so refresh the estimate before anything moves.
            self._transition('VERIFY_LOCALIZATION', correlation_id)
            if not self._wait_for_localization(
                    goal_handle, 2.0, correlation_id):
                return self._abort(
                    goal_handle, Undock.Result.LOCALIZATION_FAILED,
                    'localization invalid immediately before undocking',
                    initial_charge, manual_recovery=False)

            self._transition('VERIFY_COMMAND_PATH', correlation_id)
            self._refresh_command_path(force=True)
            if not self._snapshot().command_path_valid:
                return self._abort(
                    goal_handle, Undock.Result.COMMAND_PATH_FAILED,
                    self._path_reason, initial_charge, manual_recovery=False)

            if goal_handle.is_cancel_requested:
                return self._canceled(goal_handle, initial_charge)

            # One atomic hardware operation replaces the release, the motor
            # enable, the guarded reverse, the charge-latch clear and the
            # clearance wait that used to be five separate Pi-driven steps.
            self._transition('HARDWARE_UNDOCK', correlation_id)
            hardware_ok, hardware_reason, hardware_result = (
                self._run_hardware_undock(goal_handle, correlation_id))
            hardware_completed_at = time.monotonic()
            traveled = self._hardware_distance
            if not hardware_ok:
                code = self._undock_result_code(
                    goal_handle, hardware_result, hardware_reason)
                return self._abort(
                    goal_handle, code, hardware_reason, initial_charge,
                    manual_recovery=True, distance=traveled)

            # Localization is restored BEFORE the handoff gate, because the
            # gate checks it: the robot has just moved half a metre, so the
            # pre-undock AMCL sample is necessarily stale by now.
            self._transition('RESTORE_LOCALIZATION', correlation_id)
            if not self._wait_for_localization(
                    goal_handle,
                    float(self.get_parameter(
                        'post_undock_localization_timeout').value),
                    correlation_id, newer_than=hardware_completed_at):
                return self._abort(
                    goal_handle, Undock.Result.LOCALIZATION_FAILED,
                    'localization did not recover after the hardware undock',
                    initial_charge, manual_recovery=True, distance=traveled)

            # The SBC hands off stopped, physically clear, with the motors still
            # live, and its terminal frame says so before the telemetry does:
            # BaseState publishes at 10 Hz and its dock section carries the
            # SBC's ~4 Hz auxiliary rate. Sampling once here read dock_state as
            # DEPARTING and aborted a completed undock, so wait for the
            # published state to catch up instead of taking one sample.
            handoff_ok, handoff_reason = self._wait_for_handoff(
                goal_handle, expected_session,
                float(self.get_parameter('handoff_timeout').value),
                correlation_id)
            if not handoff_ok:
                return self._abort(
                    goal_handle, Undock.Result.SAFETY_FAULT, handoff_reason,
                    initial_charge, manual_recovery=True, distance=traveled)

            # Collision-aware turn, on the full safety chain. It is a Nav2 Spin
            # and not an SBC manoeuvre precisely because the SBC cannot see the
            # costmap or the laser.
            self._transition('TURN_AWAY_FROM_DOCK', correlation_id)
            turn_ok, turn_reason = self._run_spin(
                goal_handle, expected_session,
                float(self.get_parameter('undock_turn_yaw').value),
                correlation_id)
            if not turn_ok:
                code = (Undock.Result.CANCELED if goal_handle.is_cancel_requested
                        else Undock.Result.MOTION_FAILED)
                return self._abort(
                    goal_handle, code, turn_reason, initial_charge,
                    manual_recovery=True, distance=traveled)

            self._transition('STOPPED', correlation_id)
            self._transition('COMPLETE', correlation_id)
            result = self._result(
                True, Undock.Result.OK, 'undock complete', initial_charge,
                False, traveled)
            goal_handle.succeed()
            return result
        except Exception as exc:  # Fail closed; action never publishes velocity.
            self.get_logger().error(
                f'undock={correlation_id} unhandled fail-closed error: {exc}')
            return self._abort(
                goal_handle, Undock.Result.SAFETY_FAULT,
                f'unhandled fail-closed error: {exc}', initial_charge,
                manual_recovery=True)
        finally:
            with self._goal_lock:
                self._goal_reserved = False
            with self._lock:
                self._owned_spin_goal_id = None
            self._active_correlation_id = ''
            if self._state_name in ('COMPLETE', 'ABORTED', 'FAULT'):
                self._state_name = 'IDLE'

    @staticmethod
    def _undock_result_code(goal_handle, hardware_result, hardware_reason=''):
        """Map an SBC failure onto the manager-facing result code."""
        if goal_handle.is_cancel_requested:
            return Undock.Result.CANCELED
        if hardware_result is None:
            # Only an explicit refusal proves this request never started.
            # Missing acceptance or terminal results remain unknown outcomes.
            if hardware_reason == 'hardware undock goal rejected':
                return Undock.Result.REJECTED
            return Undock.Result.MOTION_FAILED
        code = hardware_result.code
        mapping = {
            HardwareUndock.Result.CANCELED: Undock.Result.CANCELED,
            HardwareUndock.Result.TIMEOUT: Undock.Result.TIMEOUT,
            HardwareUndock.Result.SESSION_CHANGED: Undock.Result.SESSION_CHANGED,
            HardwareUndock.Result.RELEASE_FAILED: Undock.Result.RELEASE_FAILED,
            HardwareUndock.Result.MOTOR_STATE_FAILED:
                Undock.Result.MOTOR_STATE_FAILED,
            HardwareUndock.Result.SENSOR_BLOCKED: Undock.Result.SENSOR_FAILED,
            HardwareUndock.Result.SAFETY_FAULT: Undock.Result.SAFETY_FAULT,
            HardwareUndock.Result.MOTION_FAILED: Undock.Result.MOTION_FAILED,
            HardwareUndock.Result.PRECONDITION_FAILED:
                Undock.Result.PRECHECK_FAILED,
            HardwareUndock.Result.HARDWARE_PROFILE_UNCOMMISSIONED:
                Undock.Result.PRECHECK_FAILED,
            HardwareUndock.Result.BUSY: Undock.Result.PRECHECK_FAILED,
            HardwareUndock.Result.INVALID_REQUEST: Undock.Result.REJECTED,
            HardwareUndock.Result.LINK_UNAVAILABLE: Undock.Result.SESSION_CHANGED,
        }
        return mapping.get(code, Undock.Result.MOTION_FAILED)

    def _run_hardware_undock(self, goal_handle, correlation_id):
        """
        Request the SBC operation and relay its progress.

        Returns (ok, reason, result). The Pi does not monitor the reverse: the
        SBC evaluates rear sonar, bumpers, stall, displacement and charge state
        every 50 ms with direct control of the wheels, and reports one terminal
        result.
        """
        if not self._hardware_undock_client.wait_for_server(timeout_sec=3.0):
            return False, 'hardware undock action unavailable', None
        goal = HardwareUndock.Goal()
        goal.clearance_distance = float(
            self.get_parameter('undock_clearance_distance').value)
        goal.reverse_speed = float(
            self.get_parameter('undock_reverse_speed').value)
        goal.maximum_duration = float(
            self.get_parameter('undock_max_duration').value)
        goal.validation_mode = bool(goal_handle.request.validation_mode)

        send_future = self._hardware_undock_client.send_goal_async(
            goal, feedback_callback=self._hardware_feedback_callback)
        if not self._wait_future(send_future, goal_handle, 5.0, correlation_id):
            return False, 'hardware undock goal acceptance timeout', None
        hardware_goal = send_future.result()
        if hardware_goal is None:
            return False, 'hardware undock goal acceptance unknown', None
        if not hardware_goal.accepted:
            return False, 'hardware undock goal rejected', None

        result_future = hardware_goal.get_result_async()
        deadline = time.monotonic() + goal.maximum_duration + 30.0
        cancel_sent = False
        while not result_future.done() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested and not cancel_sent:
                hardware_goal.cancel_goal_async()
                cancel_sent = True
            self._publish_feedback(goal_handle, deadline, correlation_id)
            time.sleep(0.05)
        if not result_future.done():
            hardware_goal.cancel_goal_async()
            return False, 'hardware undock produced no result', None

        wrapped = result_future.result()
        if wrapped is None:
            return False, 'hardware undock returned no result', None
        result = wrapped.result
        self._hardware_distance = float(result.distance_traveled)
        self._hardware_release_attempts = int(result.release_attempts)
        if not result.success:
            return False, result.message or 'hardware undock failed', result
        if not result.physically_clear:
            return (
                False,
                'hardware undock reported success without physical clearance',
                result)
        self.get_logger().info(
            f'undock={correlation_id} hardware undock complete: '
            f'{result.distance_traveled:.3f} m, '
            f'{result.release_attempts} release attempt(s), '
            f'dock_state={result.final_dock_state}, '
            f'motors_enabled={result.motors_enabled}')
        return True, '', result

    def _wait_for_handoff(
            self, goal_handle, expected_session, timeout, correlation_id):
        """
        Wait for the published state to agree that Nav2 may take control.

        Returns (ok, reason). The SBC's terminal frame is authoritative and has
        already been checked; this confirms the same thing through the
        independent telemetry path before anything commands velocity.
        """
        deadline = time.monotonic() + timeout
        attempts = 0
        attempt_limit = self._localization_update_attempt_limit()
        last_failures = ['handoff not evaluated']
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False, 'goal canceled during handoff'
            state = self._snapshot(for_undock=True)
            failures = resume_failures(state, expected_session)
            if not failures:
                return True, ''
            # Pose sync stays here even though admission no longer gates on
            # localization: Nav2 must not take control without a fresh pose.
            # Only the admission precheck was authorized to drop it.
            localization_satisfied = bool(
                state.localization_fresh and state.localization_valid)
            if not localization_satisfied and attempts < attempt_limit:
                if self._request_localization_update(force=attempts == 0):
                    attempts += 1
            last_failures = failures
            self._publish_feedback(goal_handle, deadline, correlation_id)
            time.sleep(0.05)
        return False, '; '.join(last_failures)

    def _hardware_feedback_callback(self, feedback):
        message = feedback.feedback
        with self._lock:
            self._hardware_distance = float(message.distance_traveled)
            self._hardware_phase = message.phase_name
            self._hardware_release_attempts = int(message.release_attempts)

    def _wait_for_precheck(
            self, goal_handle, correlation_id, validation_mode=False):
        deadline = time.monotonic() + float(
            self.get_parameter('precheck_timeout').value)
        stable_since = None
        last_failures = ['precheck not evaluated']
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False, 'goal canceled during precheck'
            state = self._snapshot(for_undock=True)
            failures = precheck_failures(state, validation_mode)
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

    def _wait_for_localization(
            self, goal_handle, timeout, correlation_id, newer_than=0.0):
        """Bound AMCL requests and require a fresh valid pose for this phase."""
        deadline = time.monotonic() + timeout
        attempts = 0
        attempt_limit = self._localization_update_attempt_limit()
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            state = self._snapshot()
            with self._lock:
                localization_at = self._localization_at
            if (localization_at > newer_than and
                    state.localization_fresh and state.localization_valid):
                return True
            if attempts < attempt_limit:
                if self._request_localization_update(force=attempts == 0):
                    attempts += 1
            self._publish_feedback(goal_handle, deadline, correlation_id)
            time.sleep(0.05)
        return False

    def _localization_update_attempt_limit(self):
        return max(0, int(
            self.get_parameter('localization_update_max_attempts').value))

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

    def _run_spin(
            self, goal_handle, expected_session, target_yaw, correlation_id):
        """
        Turn away from the dock through Nav2, on the full safety chain.

        Deliberately not an SBC manoeuvre: the SBC is blind to the costmap, the
        laser and the collision monitor, and a 180 degree in-place turn is a
        navigation concern rather than a hardware-contact one. By this point the
        SBC has handed motion authority back with the motors live and the dock
        observer in CLEAR_CONFIRMED, so an ordinary DRIVE is permitted again
        without any charge-state exception.
        """
        if not self._spin_client.wait_for_server(timeout_sec=2.0):
            return False, 'Nav2 Spin action unavailable'
        spin = Spin.Goal()
        spin.target_yaw = target_yaw
        timeout = float(self.get_parameter('spin_timeout').value)
        spin.time_allowance.sec = int(timeout)
        spin.time_allowance.nanosec = 0
        send_future = self._spin_client.send_goal_async(spin)
        if not self._wait_future(
                send_future, goal_handle, 2.0, correlation_id):
            return False, 'Spin goal acceptance timeout'
        nav_goal = send_future.result()
        if nav_goal is None or not nav_goal.accepted:
            return False, 'Spin goal rejected'
        with self._lock:
            self._owned_spin_goal_id = bytes(nav_goal.goal_id.uuid)
            self._other_action_active['spin'] = False
        result_future = nav_goal.get_result_async()
        deadline = time.monotonic() + timeout
        reason = ''
        while not result_future.done() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                reason = 'goal canceled during turn'
                break
            failures = resume_failures(self._snapshot(), expected_session)
            if failures:
                reason = '; '.join(failures)
                break
            self._publish_feedback(goal_handle, deadline, correlation_id)
            time.sleep(0.05)
        if reason or not result_future.done():
            if not reason:
                reason = 'Spin hard timeout'
            cancel_future = nav_goal.cancel_goal_async()
            cancel_deadline = time.monotonic() + 1.0
            while not cancel_future.done() and time.monotonic() < cancel_deadline:
                time.sleep(0.02)
            if not self._wait_for_odom_zero(1.5):
                reason += '; zero velocity after Spin cancellation not confirmed'
            with self._lock:
                self._owned_spin_goal_id = None
            return False, reason
        wrapped = result_future.result()
        with self._lock:
            self._owned_spin_goal_id = None
        if (wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED or
                wrapped.result.error_code != Spin.Result.NONE):
            message = 'Spin failed'
            if wrapped is not None:
                message = wrapped.result.error_msg or message
            return False, message
        if not self._wait_for_odom_zero(1.5):
            return False, 'zero velocity after Spin was not confirmed'
        return True, ''

    def _wait_future(
            self, future, goal_handle, timeout, correlation_id):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            self._publish_feedback(goal_handle, deadline, correlation_id)
            time.sleep(0.05)
        return future.done()

    def _wait_for_odom_zero(self, timeout, stable_duration=0.25):
        deadline = time.monotonic() + timeout
        stable_since = None
        while time.monotonic() < deadline:
            state = self._snapshot()
            if state.odom_fresh and state.stationary:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= stable_duration:
                    return True
            else:
                stable_since = None
            time.sleep(0.02)
        return False

    @staticmethod
    def _parked_since_the_pose(
            stationary, odom_age, timeout, last_motion_at, localization_at,
            base, fault_flags):
        # Elapsed time is only evidence that a pose has gone stale if the robot
        # could have moved since it was published. AMCL publishes /amcl_pose
        # only after update_min_d/update_min_a of motion, so a parked robot
        # stops producing pose messages entirely and the age climbs without
        # bound -- 102 s measured live on 2026-08-18 against a 0.5 s timeout.
        # The robot knew exactly where it was; the message had gone stale, not
        # the estimate, and navigation goals were refused over it.
        #
        # Every conjunct below is load-bearing:
        #
        #   stationary          the newest sample shows no motion;
        #   odom_age            ...and odometry is actually still arriving,
        #                       because a dead link leaves the last message
        #                       reading zero velocity while the robot drives
        #                       away;
        #   _last_motion_at     ...and nothing has moved SINCE the pose was
        #                       published. Without this, a robot that drove off
        #                       while AMCL was silent and then stopped would
        #                       revalidate the pose it held before it moved --
        #                       the predicate would prove only "velocity is
        #                       zero right now", never "this pose still
        #                       describes the robot";
        #   encoder faults      ...and the wheels are actually being measured.
        #                       This allowance is only as good as the odometry
        #                       it trusts, so a degraded or failed encoder must
        #                       void it rather than silently supply the zeros
        #                       that satisfy it.
        return (
            stationary and
            odom_age <= timeout and
            last_motion_at <= localization_at and
            base is not None and
            not fault_flags & ENCODER_FAULT_MASK)

    def _snapshot(self, *, for_pose_initialization=False, for_undock=False):
        now = time.monotonic()
        timeout = float(self.get_parameter('sensor_timeout').value)
        self._refresh_command_path()
        with self._lock:
            base = self._base_state
            base_age = now - self._base_state_at if self._base_state_at else math.inf
            sonar_age = now - self._sonar_at if self._sonar_at else math.inf
            odom = self._odom
            odom_age = now - self._odom_at if self._odom_at else math.inf
            last_motion_at = self._last_motion_at
            localization_at = self._localization_at
            pose = self._localization
            pose_age = (
                now - self._localization_at if self._localization_at else math.inf)
            rear_clear = self._rear_clear
            navigation_idle = not any(self._other_action_active.values())
            self._redock_inhibit_remaining = float(
                base.redock_inhibit_remaining if base else 0.0)

        localization_valid, dock_pose_valid = self._localization_health(
            pose, docked=self._dock_contact_confirmed())
        degenerate_override_engaged = (
            self._degenerate_localization_override_engaged(pose))
        stationary = False
        if odom is not None:
            stationary = (
                abs(odom.twist.twist.linear.x) <= STATIONARY_LINEAR_LIMIT and
                abs(odom.twist.twist.angular.z) <= STATIONARY_ANGULAR_LIMIT)
        fault_flags = base.fault_flags if base else 0
        parked_since_the_pose = self._parked_since_the_pose(
            stationary, odom_age, timeout, last_motion_at, localization_at,
            base, fault_flags)
        base_stamp_age = ((self.get_clock().now().nanoseconds -
                           base.header.stamp.sec * 1_000_000_000 -
                           base.header.stamp.nanosec) / 1e9 if base else math.inf)
        scans_ready = lidar_ready(base, receipt_age=base_age, stamp_age=base_stamp_age)
        epoch_ready = bool(
            base and getattr(base, 'odom_epoch_valid', False)
            and not getattr(base, 'localization_recovery_required', True))
        hardware_epoch_ready = epoch_hardware_ready(
            origin_valid=getattr(base, 'odom_epoch_valid', False),
            recovery_required=getattr(base, 'localization_recovery_required', True),
            recovery_stage=getattr(base, 'localization_recovery_stage', ''),
            for_pose_initialization=for_pose_initialization,
            for_undock=for_undock)
        seed_stamp = getattr(base, 'localization_seed_stamp_ns', 0) if base else 0
        pose_stamp = (pose.header.stamp.sec * 1_000_000_000 +
                      pose.header.stamp.nanosec) if pose else 0
        pose_still_describes_the_robot = (
            epoch_ready and seed_stamp > 0 and pose_stamp > seed_stamp
            and (pose_age <= timeout or parked_since_the_pose))
        return SafetySnapshot(
            link_connected=bool(base and base.link_connected),
            hardware_state_valid=bool(
                base and base.hardware_state_valid and base_age <= timeout
                and hardware_epoch_ready and scans_ready),
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
            # Raw, deliberately. The VERIFY/RESTORE localization flow steps
            # treat it as proof that a pose arrived recently, and
            # _wait_for_localization's default newer_than=0.0 would otherwise
            # accept an arbitrarily old pose as authorization for the 0.53 m
            # hardware undock. The parked-robot allowance is a usability
            # relaxation for consumers that are not about to move the robot,
            # and must not reach this. Undock admission itself no longer
            # consults localization (authorized 2026-10-16).
            localization_fresh=pose_age <= timeout,
            localization_valid=(
                localization_valid and pose_still_describes_the_robot),
            dock_pose_valid=dock_pose_valid and pose_still_describes_the_robot,
            degenerate_localization_override_engaged=(
                degenerate_override_engaged),
            command_path_valid=self._path_valid,
            navigation_idle=navigation_idle,
            # Mirrored from the SBC. Absent or stale telemetry reads as UNKNOWN
            # and as inhibited, so every one of these fails closed.
            dock_state=base.dock_state if base else DOCK_UNKNOWN,
            dock_state_valid=bool(
                base and base.dock_state_valid and base_age <= timeout),
            undock_active=bool(base and base.undock_active),
            physically_clear=bool(
                base and base.dock_state_valid and
                base.dock_state == 'CLEAR_CONFIRMED'),
            redock_inhibited=True if base is None else base.redock_inhibited,
            undock_profile_commissioned=bool(
                base and base.undock_profile_commissioned),
        )

    def _dock_contact_confirmed(self):
        """
        Report whether the SBC's own dock observer sees mated plates.

        Fail-closed on purpose: missing or stale `BaseState` reads as not
        docked, so the docked variance allowance below can never be granted on
        the strength of telemetry the Pi can no longer see.
        """
        now = time.monotonic()
        timeout = float(self.get_parameter('sensor_timeout').value)
        with self._lock:
            base = self._base_state
            base_age = now - self._base_state_at if self._base_state_at else math.inf
        return bool(
            base is not None and base.dock_state_valid and
            base.dock_state == DOCK_DOCKED_CONFIRMED and base_age <= timeout)

    def _localization_health(self, pose, docked=False):
        if pose is None:
            return False, False
        covariance = pose.pose.covariance
        values = (
            pose.pose.pose.position.x, pose.pose.pose.position.y,
            pose.pose.pose.orientation.z, pose.pose.pose.orientation.w,
            covariance[0], covariance[7], covariance[35])
        if not all(math.isfinite(value) for value in values):
            return False, False
        # Before asking whether the filter is confident enough, ask whether it
        # is reporting a covariance at all. The limit test below is an upper
        # bound only, so a degenerate diagonal passes it the most easily of
        # all. Observed live on the docked robot with cov[0] negative and the
        # gate reporting ready_for_undock: true.
        degenerate = not planar_covariance_usable(
            covariance[0], covariance[7], covariance[35])
        override_engaged = (
            self._degenerate_localization_override_engaged(pose))
        if degenerate and not override_engaged:
            return False, False
        if override_engaged:
            self.get_logger().warning(
                'SAFETY GATE OVERRIDE: accepting a degenerate AMCL planar '
                'covariance',
                throttle_duration_sec=10.0)
        configured = (bool(self.get_parameter('dock_pose_configured').value)
                      and self._base_state is not None
                      and getattr(self._base_state, 'map_id', '') == 'cmuq-floor2')
        at_dock = False
        if configured:
            yaw = self._yaw(pose.pose.pose.orientation)
            dx = pose.pose.pose.position.x - float(
                self.get_parameter('dock_pose_x').value)
            dy = pose.pose.pose.position.y - float(
                self.get_parameter('dock_pose_y').value)
            target_yaw = float(self.get_parameter('dock_pose_yaw').value)
            yaw_error = math.atan2(
                math.sin(yaw - target_yaw), math.cos(yaw - target_yaw))
            at_dock = bool(
                math.hypot(dx, dy) <=
                float(self.get_parameter('dock_position_tolerance').value) and
                abs(yaw_error) <=
                float(self.get_parameter('dock_yaw_tolerance').value))
        position_limit = localization_position_limit(
            self.get_parameter('max_position_variance').value,
            self.get_parameter('docked_max_position_variance').value,
            docked, at_dock)
        # bool() here is load-bearing, not decoration. `covariance` is
        # numpy-backed, so each comparison yields numpy.bool_ and `and` returns
        # the last operand — leaving `valid` a numpy.bool_. Assigning that to a
        # ROS bool field aborts the whole process: rosidl's C converter asserts
        # PyBool_Check, numpy.bool_ fails it (it is not a subclass of bool),
        # and a failed assertion is SIGABRT. That killed this node on every
        # DockReadiness publish and mid-undock on the first feedback — but only
        # once AMCL had a pose, since this returns early on `pose is None`.
        valid = bool(
            covariance[0] <= position_limit and
            covariance[7] <= position_limit and
            covariance[35] <= self.get_parameter('max_yaw_variance').value)
        if not configured:
            return valid, False
        return valid, bool(valid and at_dock)

    def _degenerate_localization_override_engaged(self, pose):
        if (pose is None or not self.get_parameter(
                'allow_degenerate_localization').value):
            return False
        covariance = pose.pose.covariance
        values = (
            pose.pose.pose.position.x, pose.pose.pose.position.y,
            pose.pose.pose.orientation.z, pose.pose.pose.orientation.w,
            covariance[0], covariance[7], covariance[35])
        # A non-finite sample never reaches the overridable gate, so describing
        # the override as engaged for it would make readiness telemetry lie.
        return bool(
            all(math.isfinite(value) for value in values) and
            not planar_covariance_usable(
                covariance[0], covariance[7], covariance[35]))

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
        with self._lock:
            feedback.hardware_phase = self._hardware_phase
            feedback.distance_traveled = float(self._hardware_distance)
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
        result.release_attempts = int(self._hardware_release_attempts)
        result.physically_clear = snapshot.physically_clear
        result.final_dock_state = snapshot.dock_state
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
