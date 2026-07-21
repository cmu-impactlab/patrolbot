"""Exercise the installed Jazzy Collision Monitor with production parameters."""

import math
import os
from pathlib import Path
import subprocess
import threading
import time

from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PARAMS_FILE = PACKAGE_ROOT / 'config' / 'nav2_params.yaml'
pytestmark = pytest.mark.skipif(
    os.environ.get('PATROLBOT_RUN_INTEGRATION') != '1' or
    int(os.environ.get('ROS_DOMAIN_ID', '0')) < 90,
    reason='requires PATROLBOT_RUN_INTEGRATION=1 and isolated ROS_DOMAIN_ID >= 90')


def wait_until(predicate, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class CollisionHarness(Node):
    def __init__(self):
        super().__init__('collision_monitor_integration_client')
        self.scan_pub = self.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self.sonar_pub = self.create_publisher(
            PointCloud2, '/sonar', qos_profile_sensor_data)
        self.command_pub = self.create_publisher(Twist, '/cmd_vel_smoothed', 10)
        self.outputs = []
        self.output_sub = self.create_subscription(
            Twist, '/input/navi', self.outputs.append, 10)

    def scan(self, obstacle=False):
        message = LaserScan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.angle_min = -math.pi / 2.0
        message.angle_max = math.pi / 2.0
        message.angle_increment = math.pi / 20.0
        message.range_min = 0.25
        message.range_max = 8.0
        message.ranges = [0.25 if obstacle else 1.0] * 21
        self.scan_pub.publish(message)

    def sonar(self, obstacle=False):
        message = PointCloud2()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.height = 1
        message.width = 1 if obstacle else 0
        message.fields = [
            PointField(
                name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.point_step = 12
        message.row_step = message.width * message.point_step
        message.is_dense = True
        if obstacle:
            import struct
            message.data = struct.pack('<fff', -0.30, 0.0, 0.0)
        self.sonar_pub.publish(message)

    def command(self, linear, angular=0.0, scan_obstacle=False,
                sonar_obstacle=False, publish_sonar=True):
        for _unused in range(4):
            self.scan(scan_obstacle)
            if publish_sonar:
                self.sonar(sonar_obstacle)
            time.sleep(0.03)
        count = len(self.outputs)
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.command_pub.publish(message)
        assert wait_until(lambda: len(self.outputs) > count, timeout=1.0)
        return self.outputs[-1]


@pytest.fixture(scope='module')
def monitor():
    process = subprocess.Popen(
        [
            '/opt/ros/jazzy/lib/nav2_collision_monitor/collision_monitor',
            '--ros-args', '--params-file', str(PARAMS_FILE),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    rclpy.init()
    node = CollisionHarness()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    change_state = node.create_client(
        ChangeState, '/collision_monitor/change_state')
    assert change_state.wait_for_service(timeout_sec=5.0)
    for transition_id in (
            Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = change_state.call_async(request)
        assert wait_until(future.done, timeout=5.0)
        assert future.result().success
    assert wait_until(lambda: node.command_pub.get_subscription_count() == 1)
    yield node
    executor.shutdown(timeout_sec=2.0)
    node.destroy_node()
    spin_thread.join(timeout=2.0)
    rclpy.shutdown()
    process.terminate()
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2.0)


def assert_velocity(message, linear, angular=0.0):
    assert message.linear.x == pytest.approx(linear, abs=1e-6)
    assert message.angular.z == pytest.approx(angular, abs=1e-6)


def test_front_zone_blocks_forward_but_not_reverse(monitor):
    assert_velocity(monitor.command(0.05, scan_obstacle=True), 0.0)
    assert_velocity(monitor.command(-0.05, scan_obstacle=True), -0.05)


def test_rear_zone_blocks_reverse_but_not_forward(monitor):
    assert_velocity(
        monitor.command(-0.05, sonar_obstacle=True), 0.0)
    assert_velocity(
        monitor.command(0.05, sonar_obstacle=True), 0.05)


def test_stale_rear_source_stops_reverse(monitor):
    monitor.scan()
    monitor.sonar()
    time.sleep(0.6)
    output = monitor.command(-0.05, publish_sonar=False)
    assert_velocity(output, 0.0)


@pytest.mark.parametrize(
    ('linear', 'angular'),
    [
        (-0.26, -1.0), (-0.01, 1.0), (0.0, -1.0),
        (0.01, 1.0), (0.26, -1.0), (-0.05, 0.4), (0.05, -0.4),
    ],
)
def test_clear_velocity_boundaries_and_arcs_are_not_dropped(
        monitor, linear, angular):
    assert_velocity(monitor.command(linear, angular), linear, angular)
