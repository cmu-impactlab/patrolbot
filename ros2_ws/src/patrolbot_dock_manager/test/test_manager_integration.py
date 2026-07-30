"""Isolated ROS integration tests for the undock orchestrator.

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
from patrolbot_dock_manager.dock_manager import DockManager
from patrolbot_interfaces.action import HardwareUndock, Undock
from patrolbot_interfaces.msg import BaseState
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


class FakeWorld(Node):
    """Stands in for the SBC-backed bridge and for Nav2's behavior server."""

    def __init__(self):
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
        self.localization_enabled = True
        self.hardware_completed_at = 0.0
        self.localization_resumed_at = 0.0
        self.spin_called_at = 0.0
        self.spin_calls = 0
        # Stands in for AMCL's no-motion update service. A real stationary AMCL
        # publishes nothing until asked, which is why the manager asks.
        self.nomotion_update_calls = 0
        # Scripted AMCL output. The defaults are a tight fix at the configured
        # dock pose; tests move them to exercise the docked variance allowance.
        self.position_variance = 0.01
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
        state.header.stamp = now
        state.header.frame_id = 'base_link'
        state.session_generation = 7
        state.link_connected = True
        state.telemetry_age = 0.0
        state.hardware_state_valid = True
        state.charge_state = self.charge_state
        state.charge_state_valid = True
        state.motors_enabled = self.motors_enabled
        state.estop_pressed = False
        state.fault_flags = 0
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

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        self.odom_pub.publish(odom)

        if self.localization_enabled:
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'map'
            pose.pose.pose.position.x = self.pose_x
            pose.pose.pose.position.y = self.pose_y
            pose.pose.pose.orientation.w = 1.0
            pose.pose.covariance[0] = self.position_variance
            pose.pose.covariance[7] = 0.01
            pose.pose.covariance[35] = 0.01
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


@pytest.fixture
def harness():
    rclpy.init()
    manager = DockManager(parameter_overrides=[
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
    ])
    world = FakeWorld()
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
    assert wait_until(lambda: manager._snapshot().dock_pose_valid)
    assert wait_until(lambda: manager._snapshot().command_path_valid)
    yield manager, world, nodes[-1]
    executor.shutdown(timeout_sec=3.0)
    manager.destroy_node()
    for node in nodes:
        node.destroy_node()
    spin_thread.join(timeout=2.0)
    rclpy.shutdown()


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
    initialize = client_node.create_client(
        InitializeDockPose, '/patrolbot/initialize_dock_pose')
    assert initialize.wait_for_service(timeout_sec=1.0)
    request = InitializeDockPose.Request()
    request.dock_id = 'main_charger'
    request.operator_authorized = True
    response = wait_future(initialize.call_async(request), timeout=3.0)
    assert response.success
    assert response.code == InitializeDockPose.Response.OK

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
    _manager, world, client_node = harness
    world.dock_state = 'CLEAR_CONFIRMED'
    goal_handle = send_undock(client_node)
    wrapped = wait_future(goal_handle.get_result_async(), timeout=15.0)
    assert not wrapped.result.success
    assert wrapped.result.code == Undock.Result.PRECHECK_FAILED
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


def test_docked_localization_is_refreshed_while_idle(harness):
    """PB-AUDIT-026: a stationary AMCL must still be asked, or readiness lies.

    Without this the docked pose is permanently older than `sensor_timeout` and
    idle readiness reports "localization invalid" — the blocker the operator saw
    on the dashboard — even though the precheck's own forced refresh clears it.
    """
    manager, world, _client_node = harness
    assert wait_until(
        lambda: manager._snapshot().dock_state == 'DOCKED_CONFIRMED')
    baseline = world.nomotion_update_calls
    assert wait_until(lambda: world.nomotion_update_calls > baseline + 2)
    assert manager._snapshot().localization_fresh


def test_localization_is_not_refreshed_once_clear_of_the_dock(harness):
    """Away from the dock, motion is what updates AMCL. Do not poll it."""
    manager, world, _client_node = harness
    assert wait_until(lambda: world.nomotion_update_calls > 0)
    world.dock_state = 'CLEAR_CONFIRMED'
    assert wait_until(
        lambda: manager._snapshot().dock_state == 'CLEAR_CONFIRMED')
    # Let any request already in flight land before sampling the counter.
    time.sleep(0.5)
    settled = world.nomotion_update_calls
    time.sleep(1.0)
    assert world.nomotion_update_calls == settled


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


def precheck_ready(manager):
    from patrolbot_dock_manager.contracts import precheck_failures
    return precheck_failures(manager._snapshot()) == []
