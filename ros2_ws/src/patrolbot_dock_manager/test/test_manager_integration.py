"""
Isolated ROS integration tests for the undock orchestrator.

The fake world provides a HardwareUndock server standing in for the SBC. The
manager under test never commands motion for contact separation, so what these
tests exercise is the orchestration: preconditions, delegation, handoff
validation, the collision-aware turn, and fail-closed behaviour when the
hardware operation fails or the goal is cancelled.
"""

import os
import threading
import time

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import Spin
from nav_msgs.msg import Odometry
from patrolbot_dock_manager.dock_manager import (
    DockManager,
    STATIONARY_ANGULAR_LIMIT,
    STATIONARY_LINEAR_LIMIT,
)
from patrolbot_interfaces.action import HardwareUndock, Undock
from patrolbot_interfaces.msg import BaseState, DockReadiness
from patrolbot_interfaces.srv import InitializeDockPose
import pytest
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Empty


pytestmark = pytest.mark.skipif(
    os.environ.get('PATROLBOT_RUN_INTEGRATION') != '1',
    reason='set PATROLBOT_RUN_INTEGRATION=1 in an isolated ROS domain')


class PathNode(Node):
    def __init__(self, name, publish_topics=(), subscribe_topics=()):
        super().__init__(name)
        self._path_publishers = [
            self.create_publisher(Twist, topic, 10) for topic in publish_topics]
        self._path_subscriptions = [
            self.create_subscription(Twist, topic, lambda _message: None, 10)
            for topic in subscribe_topics]


class PassiveGoal:
    """Minimal action handle for directly exercising bounded wait phases."""

    is_cancel_requested = False

    @staticmethod
    def publish_feedback(_feedback):
        pass


class FakeWorld(Node):
    """Stands in for the SBC-backed bridge and for Nav2's behavior server."""

    def __init__(self, localization_needs_a_request=False):
        super().__init__('behavior_server')
        self.callbacks = ReentrantCallbackGroup()
        # Dock observer, exactly as the SBC would publish it.
        self.dock_state = 'DOCKED_CONFIRMED'
        self.undock_active = False
        self.motors_enabled = False
        self.charge_state = 2
        self.redock_inhibited = False
        self.profile_commissioned = True
        self.sonar_enabled = True
        # Scripted hardware behaviour.
        self.undock_calls = 0
        self.undock_should_succeed = True
        self.undock_failure_code = HardwareUndock.Result.SENSOR_BLOCKED
        self.undock_failure_message = 'rear path blocked'
        self.undock_steps = 6
        self.undock_reports_clear = True
        self.undock_leaves_motors_enabled = True
        self.handoff_telemetry_delay = 0.0
        self.pause_localization_during_undock = False
        self.localization_restore_delay = 0.0
        self._dock_state_clear_at = 0.0
        self._localization_resume_at = 0.0
        self.epoch_valid = True
        self.recovery_required = False
        self.seed_stamp_ns = 1
        self.recovery_stage = 'ready'
        self.lidar_state = 'READY'
        self.lidar_scan_age = 0.0
        self.localization_enabled = True
        # Odometry is its own stream, stoppable independently of /amcl_pose.
        # A gate that infers "parked" must not be satisfiable by telemetry that
        # has simply stopped arriving, and that is only testable if the two can
        # be silenced separately.
        self.odom_enabled = True
        self.odom_linear_x = 0.0
        self.odom_angular_z = 0.0
        # FAULTFLAGS as the SBC reports them. Bits 7 and 8 are
        # encoder_degraded_warning and encoder_failure.
        self.fault_flags = 0
        self.hardware_completed_at = 0.0
        self.localization_resumed_at = 0.0
        self.spin_called_at = 0.0
        self.spin_calls = 0
        # Stands in for AMCL's no-motion update service. A real stationary AMCL
        # publishes nothing until asked, which is why the manager asks.
        self.nomotion_update_calls = 0
        # Off by default so the existing tests keep their free-running pose
        # stream. Switched on, the fake behaves the way AMCL actually does: a
        # stationary robot produces a pose ONLY in response to
        # /request_nomotion_update, and a moving one produces them from its own
        # motion. Without this the manager's refresh policy is untestable --
        # poses arrive on a timer regardless of whether anyone asked, so
        # "stopped asking" and "still asking" look identical from the outside.
        self.localization_needs_a_request = localization_needs_a_request
        self._nomotion_requested = False
        # Scripted AMCL output. The defaults are a tight fix at the configured
        # dock pose; tests move them to exercise the docked variance allowance.
        self.position_variance = 0.01
        self.cross_variance = 0.01
        self.yaw_variance = 0.01
        self.pose_x = 0.0
        self.pose_y = 0.0

        self.behavior_pub = self.create_publisher(
            Twist, '/input/nav_behavior', 10)
        self.base_pub = self.create_publisher(
            BaseState, '/patrolbot/base_state', 10)
        self.sonar_pub = self.create_publisher(PointCloud2, '/sonar', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.undock_server = ActionServer(
            self, HardwareUndock, '/patrolbot/hardware_undock',
            execute_callback=self._hardware_undock,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self.callbacks)
        self.spin_server = ActionServer(
            self, Spin, '/spin', execute_callback=self._spin,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self.callbacks)
        self.nomotion_service = self.create_service(
            Empty, '/request_nomotion_update', self._nomotion_update,
            callback_group=self.callbacks)
        self.timer = self.create_timer(
            0.04, self._publish_state, callback_group=self.callbacks)

    def _nomotion_update(self, _request, response):
        self.nomotion_update_calls += 1
        self._nomotion_requested = True
        return response

    def _hardware_undock(self, goal_handle):
        self.undock_calls += 1
        self.dock_state = 'DEPARTING'
        self.undock_active = True
        self.motors_enabled = True
        if self.pause_localization_during_undock:
            self.localization_enabled = False
        result = HardwareUndock.Result()
        feedback = HardwareUndock.Feedback()
        for step in range(self.undock_steps):
            if goal_handle.is_cancel_requested:
                self._settle(success=False)
                result.success = False
                result.code = HardwareUndock.Result.CANCELED
                result.message = 'canceled'
                result.distance_traveled = 0.01 * (step + 1)
                result.motors_enabled = False
                result.physically_clear = False
                result.final_dock_state = 'FAULT_STOPPED'
                goal_handle.canceled()
                return result
            feedback.phase = 4
            feedback.phase_name = 'BACKING_OUT'
            feedback.distance_traveled = 0.01 * (step + 1)
            feedback.charge_state = 3
            feedback.motors_enabled = True
            feedback.minimum_rear_range = 99.0
            feedback.release_attempts = 0
            feedback.dock_state = 'DEPARTING'
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)

        success = self.undock_should_succeed
        self._settle(success=success)
        result.success = success
        result.code = (
            HardwareUndock.Result.OK if success else self.undock_failure_code)
        result.message = 'undocked' if success else self.undock_failure_message
        result.distance_traveled = 0.53 if success else 0.12
        result.release_attempts = 1
        result.final_charge_state = 3
        # Success hands off with the motors live; failure leaves them down.
        result.motors_enabled = bool(
            success and self.undock_leaves_motors_enabled)
        result.physically_clear = bool(success and self.undock_reports_clear)
        result.final_dock_state = (
            'CLEAR_CONFIRMED' if success else 'FAULT_STOPPED')
        self.hardware_completed_at = time.monotonic()
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _settle(self, success):
        self.undock_active = False
        if success:
            if self.handoff_telemetry_delay is None:
                self._dock_state_clear_at = 0.0
            elif self.handoff_telemetry_delay > 0.0:
                self._dock_state_clear_at = (
                    time.monotonic() + self.handoff_telemetry_delay)
            else:
                self.dock_state = 'CLEAR_CONFIRMED'
            self.motors_enabled = bool(self.undock_leaves_motors_enabled)
            self.redock_inhibited = True
            if self.pause_localization_during_undock:
                if self.localization_restore_delay is None:
                    self._localization_resume_at = 0.0
                else:
                    self._localization_resume_at = (
                        time.monotonic() + self.localization_restore_delay)
            # The base re-latches a stale charge state after the robot is
            # physically clear. It must not block anything any more.
            self.charge_state = 3
        else:
            self.dock_state = 'FAULT_STOPPED'
            self.motors_enabled = False

    def _spin(self, goal_handle):
        self.spin_calls += 1
        self.spin_called_at = time.monotonic()
        time.sleep(0.10)
        result = Spin.Result()
        result.error_code = Spin.Result.NONE
        goal_handle.succeed()
        return result

    def _publish_state(self):
        monotonic_now = time.monotonic()
        if (self._dock_state_clear_at and
                monotonic_now >= self._dock_state_clear_at):
            self.dock_state = 'CLEAR_CONFIRMED'
            self._dock_state_clear_at = 0.0
        if (self._localization_resume_at and
                monotonic_now >= self._localization_resume_at):
            self.localization_enabled = True
            self.localization_resumed_at = monotonic_now
            self._localization_resume_at = 0.0
        now = self.get_clock().now().to_msg()
        state = BaseState()
        state.map_id = "cmuq-floor2"
        state.map_revision = "test-floor2"
        state.header.stamp = now
        state.header.frame_id = 'base_link'
        state.session_generation = 7
        state.link_connected = True
        state.telemetry_age = 0.0
        state.hardware_state_valid = True
        state.lidar_state = self.lidar_state
        state.lidar_scan_age = self.lidar_scan_age
        state.lidar_power_supported = False
        state.odom_epoch_valid = self.epoch_valid
        state.localization_recovery_required = self.recovery_required
        state.localization_seed_stamp_ns = self.seed_stamp_ns
        state.localization_recovery_stage = self.recovery_stage
        state.charge_state = self.charge_state
        state.charge_state_valid = True
        state.motors_enabled = self.motors_enabled
        state.estop_pressed = False
        state.fault_flags = self.fault_flags
        state.bumpers_valid = True
        state.front_bumper_pressed = self.dock_state == 'DOCKED_CONFIRMED'
        state.rear_bumper_pressed = False
        state.dock_state = self.dock_state
        state.dock_state_valid = True
        state.undock_active = self.undock_active
        state.redock_inhibited = self.redock_inhibited
        state.redock_inhibit_remaining = 60.0 if self.redock_inhibited else 0.0
        state.undock_profile_commissioned = self.profile_commissioned
        state.minimum_rear_range = 99.0
        state.rear_sonar_usable = True
        self.base_pub.publish(state)

        if self.sonar_enabled:
            cloud = PointCloud2()
            cloud.header.stamp = now
            cloud.header.frame_id = 'base_link'
            cloud.height = 1
            cloud.width = 0
            cloud.fields = [
                PointField(
                    name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(
                    name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(
                    name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            cloud.point_step = 12
            cloud.row_step = 0
            cloud.is_dense = True
            self.sonar_pub.publish(cloud)

        if self.odom_enabled:
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.twist.twist.linear.x = self.odom_linear_x
            odom.twist.twist.angular.z = self.odom_angular_z
            self.odom_pub.publish(odom)

        publish_pose = self.localization_enabled
        if publish_pose and self.localization_needs_a_request:
            moving = (
                abs(self.odom_linear_x) > STATIONARY_LINEAR_LIMIT or
                abs(self.odom_angular_z) > STATIONARY_ANGULAR_LIMIT)
            if not moving:
                # One pose per request, then silence again -- update_min_d and
                # update_min_a keep a real stationary AMCL quiet otherwise.
                publish_pose = self._nomotion_requested
                self._nomotion_requested = False

        if publish_pose:
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'map'
            pose.pose.pose.position.x = self.pose_x
            pose.pose.pose.position.y = self.pose_y
            pose.pose.pose.orientation.w = 1.0
            pose.pose.covariance[0] = self.position_variance
            pose.pose.covariance[7] = self.cross_variance
            pose.pose.covariance[35] = self.yaw_variance
            self.pose_pub.publish(pose)

    def destroy_node(self):
        self.undock_server.destroy()
        self.spin_server.destroy()
        return super().destroy_node()


def wait_until(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def _build_harness(
        extra_parameter_overrides=(), localization_needs_a_request=False,
        wait_for_initial_localization=True):
    rclpy.init()
    parameter_overrides = [
        Parameter('dock_pose_configured', value=True),
        Parameter('dock_pose_x', value=0.0),
        Parameter('dock_pose_y', value=0.0),
        Parameter('dock_pose_yaw', value=0.0),
        Parameter('sensor_timeout', value=0.25),
        Parameter('precheck_timeout', value=3.0),
        Parameter('undock_clearance_distance', value=0.53),
        Parameter('undock_reverse_speed', value=0.10),
        Parameter('undock_max_duration', value=10.0),
        Parameter('spin_timeout', value=5.0),
        Parameter('post_undock_localization_timeout', value=0.75),
        Parameter('handoff_timeout', value=0.75),
        Parameter('dock_pose_initialization_timeout', value=1.0),
    ]
    parameter_overrides.extend(extra_parameter_overrides)
    manager = DockManager(parameter_overrides=parameter_overrides)
    world = FakeWorld(
        localization_needs_a_request=localization_needs_a_request)
    nodes = [
        world,
        PathNode('controller_server', publish_topics=['/input/nav_controller']),
        PathNode(
            'nav_command_mux', publish_topics=['/cmd_vel_nav'],
            subscribe_topics=[
                '/input/nav_controller', '/input/nav_behavior',
                '/input/docking']),
        PathNode(
            'velocity_smoother', publish_topics=['/cmd_vel_smoothed'],
            subscribe_topics=['/cmd_vel_nav']),
        PathNode(
            'collision_monitor', publish_topics=['/input/navi'],
            subscribe_topics=['/cmd_vel_smoothed']),
        PathNode(
            'cmd_vel_mux', publish_topics=['/cmd_vel_out'],
            subscribe_topics=['/input/navi']),
        PathNode(
            'teleop_velocity_smoother', publish_topics=['/cmd_vel'],
            subscribe_topics=['/cmd_vel_out']),
        PathNode('patrolbot_bridge', subscribe_topics=['/cmd_vel']),
        PathNode('patrolbot_safety_watchdog'),
        Node('dock_manager_integration_client'),
    ]
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(manager)
    for node in nodes:
        executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    if wait_for_initial_localization:
        assert wait_until(lambda: manager._snapshot().dock_pose_valid)
    else:
        assert wait_until(
            lambda: manager._snapshot().dock_state == 'DOCKED_CONFIRMED')
    assert wait_until(lambda: manager._snapshot().command_path_valid)
    yield manager, world, nodes[-1]
    # Stop callback workers and the spin thread before destroying ROS handles.
    assert executor.shutdown(timeout_sec=3.0), 'executor failed to drain callbacks'
    spin_thread.join(timeout=2.0)
    assert not spin_thread.is_alive(), 'executor spin thread survived shutdown'
    executor.remove_node(manager)
    manager.destroy_node()
    for node in nodes:
        executor.remove_node(node)
        node.destroy_node()
    rclpy.try_shutdown()


@pytest.fixture
def harness():
    """Run the manager against the shared default fake world."""
    yield from _build_harness()


@pytest.fixture
def no_pose_harness():
    """Start with an AMCL that stays silent until an explicit request."""
    yield from _build_harness(
        localization_needs_a_request=True,
        wait_for_initial_localization=False)


@pytest.fixture
def harness_with_degenerate_override():
    """Run the same fake world with the supervised override enabled."""
    yield from _build_harness([
        Parameter('allow_degenerate_localization', value=True),
    ])


def wait_future(future, timeout=6.0):
    assert wait_until(future.done, timeout)
    return future.result()


def send_undock(client_node, validation_mode=True):
    client = ActionClient(client_node, Undock, '/patrolbot/undock')
    assert client.wait_for_server(timeout_sec=1.0)
    goal = Undock.Goal()
    goal.dock_id = 'main_charger'
    goal.reverse_distance = 0.10
    goal.reverse_speed = 0.05
    goal.validation_mode = validation_mode
    goal.operator_authorized = True
    return wait_future(client.send_goal_async(goal))


def test_operator_dock_pose_initialization_and_hardware_undock(harness):
    _manager, world, client_node = harness
    requests_before_initialization = world.nomotion_update_calls
    initialize = client_node.create_client(
        InitializeDockPose, '/patrolbot/initialize_dock_pose')
    assert initialize.wait_for_service(timeout_sec=1.0)
    request = InitializeDockPose.Request()
    request.dock_id = 'main_charger'
    request.operator_authorized = True
    response = wait_future(initialize.call_async(request), timeout=3.0)
    assert response.success
    assert response.code == InitializeDockPose.Response.OK
    assert wait_until(
        lambda: world.nomotion_update_calls > requests_before_initialization)
    requests_after_initialization = world.nomotion_update_calls
    time.sleep(0.4)
    assert requests_after_initialization == requests_before_initialization + 1
    assert world.nomotion_update_calls == requests_after_initialization, (
        'the guarded initializer duplicated its /initialpose-triggered update')

    goal_handle = send_undock(client_node)
    assert goal_handle.accepted
    wrapped = wait_future(goal_handle.get_result_async(), timeout=15.0)
    assert wrapped.result.success
    assert wrapped.result.code == Undock.Result.OK
    # One hardware operation, not a chain of release/enable/backup calls.
    assert world.undock_calls == 1
    assert wrapped.result.distance_traveled == pytest.approx(0.53)
    assert wrapped.result.physically_clear
    assert wrapped.result.final_dock_state == 'CLEAR_CONFIRMED'
    # The turn stays in Nav2 so it keeps costmap awareness.
    assert world.spin_calls == 1
    # A stale charge state after clearance must not have blocked anything.
    assert world.charge_state == 3


def test_waits_for_post_result_localization_and_delayed_handoff(harness):
    """A terminal success may arrive before either Pi telemetry path catches up."""
    _manager, world, client_node = harness
    world.handoff_telemetry_delay = 0.20
    world.pause_localization_during_undock = True
    world.localization_restore_delay = 0.25

    goal_handle = send_undock(client_node)
    wrapped = wait_future(goal_handle.get_result_async(), timeout=15.0)

    assert wrapped.result.success
    assert world.localization_resumed_at > world.hardware_completed_at
    assert world.spin_called_at >= world.localization_resumed_at
    assert world.spin_calls == 1


def test_handoff_telemetry_timeout_stays_stopped(harness):
    """The terminal result cannot bypass an independent telemetry timeout."""
    _manager, world, client_node = harness
    world.handoff_telemetry_delay = None

    goal_handle = send_undock(client_node)
    wrapped = wait_future(goal_handle.get_result_async(), timeout=15.0)

    assert not wrapped.result.success
    assert wrapped.result.code == Undock.Result.SAFETY_FAULT
    assert 'physical clearance' in wrapped.result.message
    assert world.spin_calls == 0


def test_post_result_localization_timeout_stays_stopped(harness):
    """A pre-undock AMCL sample is not accepted after physical separation."""
    _manager, world, client_node = harness
    world.pause_localization_during_undock = True
    world.localization_restore_delay = None

    goal_handle = send_undock(client_node)
    wrapped = wait_future(goal_handle.get_result_async(), timeout=15.0)

    assert not wrapped.result.success
    assert wrapped.result.code == Undock.Result.LOCALIZATION_FAILED
    assert world.spin_calls == 0


def test_hardware_failure_is_reported_without_a_nav2_turn(harness):
    _manager, world, client_node = harness
    world.undock_should_succeed = False
    goal_handle = send_undock(client_node)
    assert goal_handle.accepted
    wrapped = wait_future(goal_handle.get_result_async(), timeout=15.0)
    assert not wrapped.result.success
    assert wrapped.result.code == Undock.Result.SENSOR_FAILED
    assert 'rear' in wrapped.result.message
    # Nothing may move after a failed separation.
    assert world.spin_calls == 0
    assert wrapped.result.manual_recovery_required


def test_success_without_physical_clearance_is_refused(harness):
    """A success flag alone is not proof; displacement evidence is required."""
    _manager, world, client_node = harness
    world.undock_reports_clear = False
    goal_handle = send_undock(client_node)
    wrapped = wait_future(goal_handle.get_result_async(), timeout=15.0)
    assert not wrapped.result.success
    assert world.spin_calls == 0


def test_handoff_with_disabled_motors_is_refused(harness):
    """Nav2 cannot drive a robot whose motors the handoff left down."""
    _manager, world, client_node = harness
    world.undock_leaves_motors_enabled = False
    goal_handle = send_undock(client_node)
    wrapped = wait_future(goal_handle.get_result_async(), timeout=15.0)
    assert not wrapped.result.success
    assert wrapped.result.code == Undock.Result.SAFETY_FAULT
    assert 'motors' in wrapped.result.message
    assert world.spin_calls == 0


def test_uncommissioned_profile_blocks_a_production_undock(harness):
    _manager, world, client_node = harness
    world.profile_commissioned = False
    goal_handle = send_undock(client_node, validation_mode=False)
    assert goal_handle.accepted
    wrapped = wait_future(goal_handle.get_result_async(), timeout=15.0)
    assert not wrapped.result.success
    assert wrapped.result.code == Undock.Result.PRECHECK_FAILED
    assert 'commissioned' in wrapped.result.message
    # Refused before the hardware was ever asked.
    assert world.undock_calls == 0


def test_undock_is_refused_unless_the_sbc_reports_the_robot_docked(harness):
    # Refused at goal admission (fresh charging required), so the hardware
    # is never asked: the goal is rejected, not failed after a precheck.
    _manager, world, client_node = harness
    world.dock_state = 'CLEAR_CONFIRMED'
    goal_handle = send_undock(client_node)
    assert not goal_handle.accepted
    assert world.undock_calls == 0


def test_cancel_is_forwarded_to_the_hardware_operation(harness):
    _manager, world, client_node = harness
    world.undock_steps = 60
    goal_handle = send_undock(client_node)
    assert goal_handle.accepted
    assert wait_until(lambda: world.undock_calls == 1, timeout=5.0)
    time.sleep(0.2)
    goal_handle.cancel_goal_async()
    wrapped = wait_future(goal_handle.get_result_async(), timeout=15.0)
    assert not wrapped.result.success
    assert wrapped.result.code == Undock.Result.CANCELED
    assert world.spin_calls == 0


def test_readiness_separates_undock_readiness_from_docking_inhibition(harness):
    manager, world, _client_node = harness
    assert wait_until(
        lambda: manager._snapshot().dock_state == 'DOCKED_CONFIRMED')
    snapshot = manager._snapshot()
    assert precheck_ready(manager)
    assert not snapshot.redock_inhibited

    world.dock_state = 'CLEAR_CONFIRMED'
    world.redock_inhibited = True
    assert wait_until(lambda: manager._snapshot().redock_inhibited)
    # Not ready to undock (already clear) and not allowed to dock (inhibited):
    # two different reasons, two different fields.
    assert not precheck_ready(manager)
    assert manager._snapshot().redock_inhibited


def test_docked_state_never_triggers_background_localization_requests(
        no_pose_harness):
    """No telemetry condition may autonomously restart AMCL resampling."""
    manager, world, _client_node = no_pose_harness
    assert manager._localization is None
    time.sleep(0.4)
    assert world.nomotion_update_calls == 0

    world.odom_enabled = False
    time.sleep(0.4)
    assert world.nomotion_update_calls == 0

    world.odom_enabled = True
    world.odom_linear_x = 0.25
    time.sleep(0.4)
    assert world.nomotion_update_calls == 0

    world.odom_linear_x = 0.0
    world.fault_flags = 1 << 8
    time.sleep(0.4)
    assert world.nomotion_update_calls == 0


@pytest.mark.parametrize(
    ('phase', 'expect_ok', 'expect_updates'),
    # Authorized 2026-10-16: admission no longer consults AMCL, so the
    # precheck passes on a silent filter without requesting updates. The
    # localization step and the handoff pose sync still do.
    (('precheck', True, 0),
     ('localization', False, 5),
     ('handoff', False, 5)))
def test_explicit_phases_request_updates_only_where_a_pose_is_needed(
        no_pose_harness, phase, expect_ok, expect_updates):
    manager, world, _client_node = no_pose_harness
    world.localization_enabled = False
    assert wait_until(manager._localization_update_client.service_is_ready)
    goal = PassiveGoal()

    if phase == 'precheck':
        ok, _reason = manager._wait_for_precheck(goal, 'test')
    elif phase == 'localization':
        ok = manager._wait_for_localization(goal, 1.4, 'test')
    else:
        ok, _reason = manager._wait_for_handoff(goal, 7, 1.4, 'test')

    assert ok == expect_ok
    assert world.nomotion_update_calls == expect_updates


def test_explicit_precheck_never_requests_a_pose_update(
        no_pose_harness):
    # Admission does not consult AMCL, so the precheck must not poke it:
    # no requests even while the filter stays silent, and success follows
    # from E-stop plus charging contact alone.
    manager, world, _client_node = no_pose_harness
    assert wait_until(manager._localization_update_client.service_is_ready)

    ok, reason = manager._wait_for_precheck(PassiveGoal(), 'test')

    assert ok, reason
    assert world.nomotion_update_calls == 0


def test_one_initialpose_event_dispatches_at_most_one_update(no_pose_harness):
    manager, world, client_node = no_pose_harness
    world.localization_enabled = False
    publisher = client_node.create_publisher(
        PoseWithCovarianceStamped, '/initialpose', 10)
    assert wait_until(lambda: publisher.get_subscription_count() >= 1)
    publisher.publish(PoseWithCovarianceStamped())

    assert wait_until(lambda: world.nomotion_update_calls == 1)
    time.sleep(0.5)
    assert world.nomotion_update_calls == 1
    assert manager._localization is None


def test_spin_never_forces_a_no_motion_update(harness):
    manager, world, _client_node = harness
    world.dock_state = 'CLEAR_CONFIRMED'
    world.motors_enabled = True
    assert wait_until(
        lambda: manager._snapshot().dock_state == 'CLEAR_CONFIRMED')
    requests_before_spin = world.nomotion_update_calls

    ok, reason = manager._run_spin(PassiveGoal(), 7, 0.25, 'test')

    assert ok, reason
    assert world.nomotion_update_calls == requests_before_spin


def test_parked_freshness_neither_grants_nor_blocks_motion_precheck(harness):
    # Authorized 2026-10-16: admission is E-stop plus charging contact and
    # no longer consults localization, so a stale-but-valid pose neither
    # grants nor blocks the motion precheck. Freshness stays enforced by
    # the VERIFY/RESTORE localization flow steps, not by admission.
    manager, world, client_node = harness
    seen = []
    client_node.create_subscription(
        DockReadiness, '/patrolbot/dock_readiness',
        lambda message: seen.append(message), 10)
    world.localization_enabled = False
    timeout = manager.get_parameter('sensor_timeout').value

    assert wait_until(
        lambda: bool(seen) and seen[-1].localization_age > timeout,
        timeout=2.0)
    assert seen[-1].localization_valid
    assert not seen[-1].localization_fresh
    assert seen[-1].ready_for_undock
    assert precheck_ready(manager)
    # ...and losing charging contact blocks it again regardless of pose.
    world.charge_state = 0
    assert wait_until(lambda: not precheck_ready(manager))


def test_docked_variance_allowance_needs_the_pose_to_match_the_dock(harness):
    """The wider docked limit is evidence-gated, not a blanket relaxation."""
    manager, world, _client_node = harness
    assert wait_until(
        lambda: manager._snapshot().dock_state == 'DOCKED_CONFIRMED')
    # The measured docked approach-axis variance: refused by the general limit,
    # admitted only because the plates are mated and the pose is at the dock.
    world.position_variance = 0.63
    assert wait_until(lambda: manager._snapshot().localization_valid)

    # Same variance, but the estimate has drifted well outside the dock's
    # capture tolerance: the allowance must not apply any more.
    world.pose_x = 5.0
    assert wait_until(lambda: not manager._snapshot().localization_valid)


def test_a_frozen_filter_is_refused_however_small_its_variance(harness):
    """
    The live 2026-08-11 defect, end to end through the published gate.

    A negative variance is far *below* every limit, so the upper-bound test
    admitted it and `ready_for_undock` went true on a filter that had stopped
    updating. The robot would have been cleared to reverse off the charger.
    """
    manager, world, _client_node = harness
    assert wait_until(
        lambda: manager._snapshot().dock_state == 'DOCKED_CONFIRMED')
    assert wait_until(lambda: manager._snapshot().localization_valid)

    world.position_variance = -7.861393e-10
    world.cross_variance = 1.860600e-07
    world.yaw_variance = 4.660659e-07
    assert wait_until(lambda: not manager._snapshot().localization_valid)
    # Authorized 2026-10-16: admission no longer consults the filter, so
    # the degenerate estimate does not block the precheck. Pose gating
    # moved to the VERIFY/RESTORE localization flow steps, not admission.
    assert precheck_ready(manager)

    # And it recovers when AMCL does, rather than latching off.
    world.position_variance = 0.01
    world.cross_variance = 0.01
    world.yaw_variance = 0.01
    assert wait_until(lambda: manager._snapshot().localization_valid)


def precheck_ready(manager):
    from patrolbot_dock_manager.contracts import precheck_failures
    return precheck_failures(manager._snapshot()) == []


# The live degenerate diagonal, captured from the docked robot 2026-08-14 and
# already pinned by test_a_frozen_filter_is_refused_however_small_its_variance.
DEGENERATE = (-7.861393e-10, 1.860600e-07, 4.660659e-07)


def _drive_degenerate(world):
    """Put the fake AMCL into the exact live degenerate state."""
    world.position_variance, world.cross_variance, world.yaw_variance = (
        DEGENERATE)


def test_the_degenerate_covariance_override_is_off_unless_asked_for(harness):
    """
    The override must never be the default.

    It admits a filter that has stopped updating, so the safe state has to be
    the one you get by not knowing the parameter exists. A default flip here
    would silently undo the whole guard on every robot that never sets it.
    """
    manager, world, _client_node = harness
    assert wait_until(
        lambda: manager._snapshot().dock_state == 'DOCKED_CONFIRMED')
    assert wait_until(lambda: manager._snapshot().localization_valid)

    _drive_degenerate(world)
    assert wait_until(lambda: not manager._snapshot().localization_valid)
    assert not precheck_ready(manager)


def test_the_override_admits_a_degenerate_covariance_when_enabled(
        harness_with_degenerate_override):
    """
    With the override on, the live degenerate diagonal no longer blocks undock.

    The operator is on site and wants to move the robot off the charger while
    AMCL is in its recurring degenerate state. This does not fix the filter and
    is not meant to: it converts a hard refusal into a supervised decision.
    """
    manager, world, _client_node = harness_with_degenerate_override
    assert wait_until(
        lambda: manager._snapshot().dock_state == 'DOCKED_CONFIRMED')
    assert wait_until(lambda: manager._snapshot().localization_valid)

    _drive_degenerate(world)
    # The fake publishes at 25 Hz, and the manager starts from a *healthy*
    # pose. Asserting validity the instant after driving the world degenerate
    # would be satisfied by the pre-degenerate sample and prove nothing, so
    # wait out the propagation and then require validity to HOLD. The
    # companion default-off test drives the same world state and shows the
    # snapshot going invalid well inside this window.
    time.sleep(0.6)
    for _ in range(10):
        assert manager._snapshot().localization_valid, (
            'override did not admit the degenerate covariance')
        time.sleep(0.05)
    assert precheck_ready(manager)


def test_the_override_still_refuses_a_covariance_that_is_not_a_number(
        harness_with_degenerate_override):
    """
    Overriding "degenerate" must not also override "garbage".

    A negative variance is a real number the rest of the gate can still reason
    about; NaN and inf are not. They poison every comparison downstream —
    every `<` against a limit is False for NaN, so a non-finite pose would sail
    through the very checks that are supposed to bound it. This boundary stays
    fail-closed no matter what the override says.
    """
    manager, world, _client_node = harness_with_degenerate_override
    assert wait_until(
        lambda: manager._snapshot().dock_state == 'DOCKED_CONFIRMED')
    assert wait_until(lambda: manager._snapshot().localization_valid)

    for bad in (float('nan'), float('inf'), float('-inf')):
        world.position_variance = bad
        world.cross_variance = 0.01
        world.yaw_variance = 0.01
        assert wait_until(
            lambda: not manager._snapshot().localization_valid), (
            f'a {bad} variance was admitted by the override')
        world.position_variance = 0.01
        assert wait_until(lambda: manager._snapshot().localization_valid)


def test_an_engaged_override_is_visible_in_the_published_readiness(
        harness_with_degenerate_override):
    """
    The operator has to be able to see that the gate was overridden.

    `ready_for_undock: true` means something different when it was reached by
    suppressing a guard, and the difference must not be invisible to whoever
    reads the dashboard. This is reported in `blocking_reason`, which is a
    string that already exists — adding a message field would change the
    DockReadiness interface hash and detach the dashboard (PB-AUDIT-025).
    """
    manager, world, client_node = harness_with_degenerate_override
    assert wait_until(
        lambda: manager._snapshot().dock_state == 'DOCKED_CONFIRMED')

    seen = []
    client_node.create_subscription(
        DockReadiness, '/patrolbot/dock_readiness',
        lambda msg: seen.append(msg), 10)

    _drive_degenerate(world)
    assert wait_until(
        lambda: bool(seen) and 'override' in seen[-1].blocking_reason.lower()), (
        'an engaged override left no trace in the published readiness')


def test_a_parked_robot_keeps_a_usable_pose_past_the_freshness_timeout(harness):
    """
    Elapsed time is only evidence of staleness if the robot could have moved.

    AMCL publishes /amcl_pose only after update_min_d/update_min_a of motion,
    so a stationary robot stops producing pose messages entirely and the age
    climbs without bound -- 102 s was observed live on 2026-08-18. The gate
    then declared localization invalid and the dashboard refused to accept a
    navigation goal, reporting that the robot did not know where it was.

    It knew exactly where it was. Nothing had moved, so the last pose still
    described the robot perfectly. What had gone stale was the message, not
    the estimate.
    """
    manager, world, client_node = harness
    assert wait_until(lambda: manager._snapshot().localization_valid)
    timeout = manager.get_parameter('sensor_timeout').value

    # The measured age lives on the published message, not on the snapshot.
    seen = []
    client_node.create_subscription(
        DockReadiness, '/patrolbot/dock_readiness',
        lambda msg: seen.append(msg), 10)

    # AMCL goes quiet, exactly as it does when the robot is not moving.
    world.localization_enabled = False
    time.sleep(timeout * 3)

    assert manager._snapshot().stationary, (
        'precondition: the fake robot is not moving')
    assert wait_until(
        lambda: bool(seen) and seen[-1].localization_age > timeout), (
        'precondition: the pose must actually be older than the timeout')
    assert seen[-1].localization_valid, (
        'a parked robot was declared unlocalized purely because AMCL had '
        'nothing new to say')


def test_a_moving_robot_still_needs_a_fresh_pose(harness):
    """
    The allowance is for parked robots only, and must not leak to moving ones.

    Once the wheels are turning the pose really is going stale, at a rate the
    gate cannot bound. This is the case the 0.5 s timeout exists for, and it
    has to keep working.
    """
    manager, world, _client_node = harness
    assert wait_until(lambda: manager._snapshot().localization_valid)
    timeout = manager.get_parameter('sensor_timeout').value

    world.localization_enabled = False
    world.odom_linear_x = 0.20
    time.sleep(timeout * 3)

    snapshot = manager._snapshot()
    assert not snapshot.stationary, 'precondition: the fake robot is moving'
    assert not snapshot.localization_valid, (
        'a moving robot kept a stale pose')


def test_dead_odometry_cannot_make_a_robot_look_parked(harness):
    """
    The whole allowance rests on knowing the robot is stationary.

    "Stationary" is read from /odom, so odometry that has simply stopped
    arriving must never be mistaken for odometry reporting zero velocity. A
    robot driving away behind a dead telemetry link is precisely the case that
    would otherwise hold a stale pose valid forever -- the failure mode this
    gate exists to prevent, reached through the fix for a different one.
    """
    manager, world, _client_node = harness
    assert wait_until(lambda: manager._snapshot().localization_valid)
    timeout = manager.get_parameter('sensor_timeout').value

    # Both streams stop: no new pose, and no odometry to prove it is parked.
    world.localization_enabled = False
    world.odom_enabled = False
    time.sleep(timeout * 3)

    snapshot = manager._snapshot()
    assert not snapshot.odom_fresh, 'precondition: odometry is stale'
    assert not snapshot.localization_valid, (
        'a stale pose was held valid on the strength of stale odometry')


def test_the_published_age_still_reports_the_real_number(harness):
    """
    The allowance must not disguise how old the pose actually is.

    localization_age is what an operator reads to judge the situation, so it
    stays the raw measurement even when the gate has decided the pose is still
    usable. Hiding the age would trade one misleading readout for another.
    """
    manager, world, client_node = harness
    assert wait_until(lambda: manager._snapshot().localization_valid)
    timeout = manager.get_parameter('sensor_timeout').value

    seen = []
    client_node.create_subscription(
        DockReadiness, '/patrolbot/dock_readiness',
        lambda msg: seen.append(msg), 10)

    world.localization_enabled = False
    time.sleep(timeout * 3)

    assert wait_until(lambda: bool(seen) and seen[-1].localization_age > timeout)
    assert seen[-1].localization_valid, 'precondition: still usable'


def test_movement_after_a_pose_does_not_revalidate_it_once_the_robot_stops(
        harness):
    """
    Zero velocity now is not evidence that an old pose still fits.

    The first version of this allowance tested only the newest odometry sample.
    That proves "the robot is not moving at this instant" and nothing more: a
    robot that drove away while AMCL was silent and then parked reports zero
    velocity too, and its pre-departure pose would spring back to valid the
    moment the wheels stopped. The pose has to be backed by an unbroken absence
    of motion since it was published, not by a single quiet sample.
    """
    manager, world, _client_node = harness
    assert wait_until(lambda: manager._snapshot().localization_valid)
    timeout = manager.get_parameter('sensor_timeout').value

    # AMCL goes silent, then the robot drives somewhere.
    world.localization_enabled = False
    world.odom_linear_x = 0.25
    time.sleep(timeout * 3)
    assert not manager._snapshot().localization_valid, (
        'precondition: a moving robot with a stale pose is already invalid')

    # It parks. The newest sample now reads zero, but the pose predates the
    # drive and no longer describes where the robot is.
    world.odom_linear_x = 0.0
    time.sleep(timeout * 3)

    assert manager._snapshot().stationary, 'precondition: it really did stop'
    assert not manager._snapshot().localization_valid, (
        'a pose from before the robot drove away was revalidated by it '
        'stopping')


def test_localization_fresh_stays_a_statement_about_age(harness):
    """
    Physical-motion consumers gate on this field, so it must not be relaxed.

    precheck_failures and _wait_for_localization read localization_fresh as
    proof that a pose actually arrived recently, and _wait_for_localization's
    default newer_than=0.0 accepts any previously received pose once the
    booleans go true. Folding the parked-robot allowance into this field would
    therefore authorise the 0.53 m hardware undock on an arbitrarily old
    estimate -- turning a dashboard usability fix into motion authorisation.
    """
    manager, world, _client_node = harness
    assert wait_until(lambda: manager._snapshot().localization_valid)
    timeout = manager.get_parameter('sensor_timeout').value

    world.localization_enabled = False
    time.sleep(timeout * 3)

    snapshot = manager._snapshot()
    assert snapshot.localization_valid, (
        'precondition: the parked allowance is in effect')
    assert not snapshot.localization_fresh, (
        'precondition: the pose message went stale while parked')
    # Authorized 2026-10-16: admission no longer reads localization_fresh,
    # so a stale pose neither grants nor blocks the precheck. Freshness
    # stays enforced by the VERIFY/RESTORE localization flow steps.
    assert precheck_ready(manager)


def test_the_undock_precheck_ignores_pose_freshness_by_authorization(harness):
    """
    Authorized 2026-10-16: admission is E-stop plus charging contact.

    The parked allowance still governs what localization_valid means, but
    a stale pose no longer blocks the undock precheck. Freshness stays
    enforced by the VERIFY/RESTORE localization flow steps, not admission.
    """
    manager, world, _client_node = harness
    assert wait_until(
        lambda: manager._snapshot().dock_state == 'DOCKED_CONFIRMED')
    assert wait_until(lambda: precheck_ready(manager))
    timeout = manager.get_parameter('sensor_timeout').value

    world.localization_enabled = False
    time.sleep(timeout * 3)

    assert manager._snapshot().localization_valid, (
        'precondition: the parked allowance is in effect')
    assert precheck_ready(manager)


def test_an_encoder_fault_voids_the_parked_allowance(harness):
    """
    The allowance is only as trustworthy as the odometry underneath it.

    "Stationary" is a reading off the wheels. A degraded or failed encoder is
    the case where that reading is exactly the thing that cannot be trusted,
    and it reports zeros that satisfy the test perfectly. FAULTFLAGS bits 7 and
    8 say so explicitly, so the allowance must stand down rather than accept
    convenient numbers from a sensor the base has already disowned.
    """
    manager, world, _client_node = harness
    assert wait_until(lambda: manager._snapshot().localization_valid)
    timeout = manager.get_parameter('sensor_timeout').value

    world.localization_enabled = False
    time.sleep(timeout * 3)
    assert manager._snapshot().localization_valid, (
        'precondition: the parked allowance is in effect')

    world.fault_flags = 1 << 8      # encoder_failure
    assert wait_until(lambda: not manager._snapshot().localization_valid), (
        'a failed encoder still propped up the parked allowance')

    world.fault_flags = 1 << 7      # encoder_degraded_warning
    time.sleep(timeout * 2)
    assert not manager._snapshot().localization_valid, (
        'a degraded encoder still propped up the parked allowance')


def test_epoch_recovery_blocks_readiness_despite_fresh_stationary_inputs(harness):
    manager, world, _ = harness
    assert wait_until(lambda: manager._snapshot().localization_valid)
    world.recovery_required = True
    assert wait_until(lambda: not manager._snapshot().hardware_state_valid)
    assert not manager._snapshot().localization_valid
    assert not precheck_ready(manager)


def test_new_epoch_seed_barrier_rejects_cached_parked_amcl(harness):
    manager, world, _ = harness
    assert wait_until(lambda: manager._snapshot().localization_valid)
    world.localization_enabled = False
    time.sleep(.15)
    world.seed_stamp_ns = world.get_clock().now().nanoseconds
    assert wait_until(lambda: not manager._snapshot().localization_valid)
    assert manager._snapshot().hardware_state_valid
    assert not manager._snapshot().dock_pose_valid
    world.localization_enabled = True
    assert wait_until(lambda: manager._snapshot().localization_valid)


@pytest.mark.parametrize('stage', ['wait-operator-seed', 'wait-amcl-pose'])
def test_operator_seed_can_recover_epoch_without_releasing_motion(harness, stage):
    manager, world, client_node = harness
    world.recovery_required = True
    world.recovery_stage = stage
    assert wait_until(lambda: not manager._snapshot().hardware_state_valid)
    assert manager._snapshot(for_pose_initialization=True).hardware_state_valid
    assert not precheck_ready(manager)
    initialize = client_node.create_client(
        InitializeDockPose, '/patrolbot/initialize_dock_pose')
    assert initialize.wait_for_service(timeout_sec=1.0)
    request = InitializeDockPose.Request()
    request.dock_id = 'main_charger'
    request.operator_authorized = True
    response = wait_future(initialize.call_async(request), timeout=3.0)
    assert response.success
    assert not manager._snapshot().hardware_state_valid
    assert not manager._snapshot().localization_valid
    assert world.undock_calls == 0


@pytest.mark.parametrize('stage', [
    'wait-origin', 'wait-reset', 'wait-first-odometry', 'wait-amcl-tf', 'failed:activate'])
def test_operator_seed_cannot_bypass_unfinished_origin_reset(harness, stage):
    manager, world, _ = harness
    world.recovery_required = True
    world.recovery_stage = stage
    assert wait_until(lambda: not manager._snapshot().hardware_state_valid)
    assert not manager._snapshot(for_pose_initialization=True).hardware_state_valid


def test_operator_seed_cannot_bypass_invalid_origin(harness):
    manager, world, _ = harness
    world.recovery_required = True
    world.recovery_stage = 'wait-operator-seed'
    world.epoch_valid = False
    assert wait_until(lambda: not manager._snapshot().hardware_state_valid)
    assert not manager._snapshot(for_pose_initialization=True).hardware_state_valid



@pytest.mark.parametrize('lidar_state,scan_age', [
    ('OFF', -1.), ('STARTING', -1.), ('UNKNOWN', -1.), ('FAULT', -1.),
    ('READY', .6), ('READY', float('nan')),
])
def test_lidar_blocks_typed_pose_recovery_and_undock(harness, lidar_state, scan_age):
    manager, world, client_node = harness
    world.recovery_required = True
    world.recovery_stage = 'wait-operator-seed'
    world.lidar_state, world.lidar_scan_age = lidar_state, scan_age
    assert wait_until(lambda: not manager._snapshot(for_pose_initialization=True).hardware_state_valid)
    assert not manager._snapshot().hardware_state_valid
    initialize = client_node.create_client(
        InitializeDockPose, '/patrolbot/initialize_dock_pose')
    assert initialize.wait_for_service(timeout_sec=1.0)
    response = wait_future(initialize.call_async(InitializeDockPose.Request(
        dock_id='main_charger', operator_authorized=True)), timeout=3.0)
    assert not response.success
    assert response.code == InitializeDockPose.Response.PRECONDITION_FAILED
    assert world.nomotion_update_calls == 0
    assert world.undock_calls == 0


def test_localization_override_cannot_bypass_missing_lidar(harness_with_degenerate_override):
    manager, world, _ = harness_with_degenerate_override
    world.position_variance = world.cross_variance = world.yaw_variance = 0.0
    world.lidar_state, world.lidar_scan_age = 'OFF', -1.
    assert wait_until(lambda: not manager._snapshot().hardware_state_valid)
    assert not precheck_ready(manager)
    assert world.undock_calls == 0
