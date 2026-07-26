"""Isolated ROS integration tests for the guarded-undock manager."""

import os
import threading
import time

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import BackUp, Spin
from nav_msgs.msg import Odometry
from patrolbot_dock_manager.dock_manager import DockManager
from patrolbot_interfaces.action import Undock
from patrolbot_interfaces.msg import BaseState
from patrolbot_interfaces.srv import (
    ChargeRelease,
    InitializeDockPose,
    MotorEnable,
)
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
    def __init__(self):
        super().__init__('behavior_server')
        self.callbacks = ReentrantCallbackGroup()
        self.released = False
        self.release_calls = 0
        self.charge_latched = True
        self.clear_of_dock = False
        self.motors_enabled = False
        self.sonar_enabled = True
        self.drop_sonar_on_backup = False
        self.behavior_pub = self.create_publisher(
            Twist, '/input/nav_behavior', 10)
        self.base_pub = self.create_publisher(
            BaseState, '/patrolbot/base_state', 10)
        self.sonar_pub = self.create_publisher(PointCloud2, '/sonar', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.release_service = self.create_service(
            ChargeRelease, '/patrolbot/charge_release', self._release,
            callback_group=self.callbacks)
        self.enable_service = self.create_service(
            MotorEnable, '/patrolbot/motor_enable', self._enable,
            callback_group=self.callbacks)
        self.backup_server = ActionServer(
            self, BackUp, '/backup', execute_callback=self._backup,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self.callbacks)
        self.spin_server = ActionServer(
            self, Spin, '/spin', execute_callback=self._spin,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self.callbacks)
        self.timer = self.create_timer(
            0.04, self._publish_state, callback_group=self.callbacks)

    def _release(self, request, response):
        self.release_calls += 1
        self.released = True
        # Model the real zero-motion sequence: release returns with motors off.
        self.motors_enabled = False
        response.request_id = request.request_id
        response.accepted = True
        if self.release_calls == 3:
            # The live base reported residual rotational motion on the first
            # final CHARGE 0 even after Nav2 had completed Spin. The manager
            # must settle and retry only this exact fail-closed response.
            response.success = False
            response.code = ChargeRelease.Response.HARDWARE_REJECTED
            response.message = 'motion detected during release'
            response.charge_state = 3
            response.motors_enabled = False
            response.estop_pressed = False
            response.session_generation = 7
            return response
        self.charge_latched = False
        response.success = True
        response.code = ChargeRelease.Response.OK
        response.message = 'released by isolated fake'
        response.charge_state = 0
        response.motors_enabled = False
        response.estop_pressed = False
        response.session_generation = 7
        return response

    def _enable(self, request, response):
        self.motors_enabled = True
        # The measured base remains at charge 0 through the second enable
        # confirmation, then re-latches after Spin starts.
        self.charge_latched = False
        response.request_id = request.request_id
        response.accepted = True
        response.success = True
        response.code = MotorEnable.Response.OK
        response.message = 'enabled by isolated fake'
        response.charge_state = 0
        response.motors_enabled = True
        response.estop_pressed = False
        response.session_generation = 7
        return response

    def _backup(self, goal_handle):
        if self.drop_sonar_on_backup:
            self.sonar_enabled = False
        feedback = BackUp.Feedback()
        for step in range(10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = BackUp.Result()
                result.error_code = BackUp.Result.UNKNOWN
                result.error_msg = 'canceled by manager safety gate'
                return result
            feedback.distance_traveled = 0.01 * (step + 1)
            goal_handle.publish_feedback(feedback)
            # Keep charge physically reasserted beyond 0.05 m. The scoped
            # departure authorization must remain valid until the contacts
            # actually clear, rather than ending at an arbitrary distance.
            if step == 7:
                self.clear_of_dock = True
            time.sleep(0.05)
        result = BackUp.Result()
        result.error_code = BackUp.Result.NONE
        if self.drop_sonar_on_backup:
            goal_handle.abort()
            result.error_msg = 'test should have canceled before completion'
        else:
            goal_handle.succeed()
        return result

    def _spin(self, goal_handle):
        self.charge_latched = True
        time.sleep(0.10)
        result = Spin.Result()
        result.error_code = Spin.Result.NONE
        goal_handle.succeed()
        return result

    def _publish_state(self):
        now = self.get_clock().now().to_msg()
        state = BaseState()
        state.header.stamp = now
        state.header.frame_id = 'base_link'
        state.session_generation = 7
        state.link_connected = True
        state.telemetry_age = 0.0
        state.hardware_state_valid = True
        state.charge_state = 2
        if self.released:
            state.charge_state = 3 if self.charge_latched else 0
        state.charge_state_valid = True
        state.motors_enabled = self.motors_enabled
        state.estop_pressed = not self.released
        state.fault_flags = 0
        state.bumpers_valid = True
        state.front_bumper_pressed = not self.clear_of_dock
        state.rear_bumper_pressed = False
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

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'map'
        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance[0] = 0.01
        pose.pose.covariance[7] = 0.01
        pose.pose.covariance[35] = 0.01
        self.pose_pub.publish(pose)

    def destroy_node(self):
        self.backup_server.destroy()
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
        Parameter('allow_front_bumper_at_dock', value=True),
        Parameter('sensor_timeout', value=0.25),
        Parameter('precheck_timeout', value=3.0),
        Parameter('operator_motor_window', value=1.0),
        Parameter('backup_timeout', value=2.0),
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


def test_operator_dock_pose_initialization_and_guarded_undock(harness):
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

    # Reproduce the live manual-charge entry state after pose initialization.
    world.motors_enabled = True

    client = ActionClient(client_node, Undock, '/patrolbot/undock')
    assert client.wait_for_server(timeout_sec=1.0)
    goal = Undock.Goal()
    goal.dock_id = 'main_charger'
    goal.reverse_distance = 0.10
    goal.reverse_speed = 0.05
    goal.validation_mode = True
    goal.operator_authorized = True
    goal_handle = wait_future(client.send_goal_async(goal))
    assert goal_handle.accepted
    wrapped = wait_future(goal_handle.get_result_async(), timeout=8.0)
    assert wrapped.result.success
    assert wrapped.result.code == Undock.Result.OK
    assert world.released
    assert world.release_calls == 4
    assert not world.charge_latched
    assert not world.motors_enabled


def test_stale_rear_sonar_cancels_reverse(harness):
    _manager, world, client_node = harness
    world.motors_enabled = True
    world.drop_sonar_on_backup = True
    client = ActionClient(client_node, Undock, '/patrolbot/undock')
    assert client.wait_for_server(timeout_sec=1.0)
    goal = Undock.Goal()
    goal.dock_id = 'main_charger'
    goal.reverse_distance = 0.10
    goal.reverse_speed = 0.05
    goal.validation_mode = True
    goal.operator_authorized = True
    goal_handle = wait_future(client.send_goal_async(goal))
    assert goal_handle.accepted
    wrapped = wait_future(goal_handle.get_result_async(), timeout=8.0)
    assert not wrapped.result.success
    assert wrapped.result.code == Undock.Result.MOTION_FAILED
    assert 'sonar' in wrapped.result.message
