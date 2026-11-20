"""Exercise the installed Jazzy Collision Monitor with production parameters."""

import json
import math
import os
from pathlib import Path
import subprocess
import sys
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


# Shared source-only test support; never installed with production nodes.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'patrolbot_bridge' / 'test'))
from native_fixture import NativeFixture, fixture_log_path

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
        self.lock = threading.RLock()
        self.outputs = []
        self.output_sources = []
        self.ack_source = 0
        self.evidence = []
        self.sequence = 0
        self.probes = set()
        self.sensor_ticks = 0
        self.world = (False, False, True)
        self.output_sub = self.create_subscription(
            Twist, '/input/navi', self.receive, 10)
        self.sensor_timer = self.create_timer(0.03, self.sensors)

    def record(self, event, **values):
        with self.lock:
            self.evidence.append(dict(event=event, monotonic=time.monotonic(),
                                      ros_ns=self.get_clock().now().nanoseconds,
                                      world=self.world, **values))

    def receive(self, message, info):
        with self.lock:
            self.outputs.append(message)
            self.output_sources.append(info.source_timestamp)
            self.record('output', source_ns=info.source_timestamp, velocity=[
                message.linear.x, message.linear.y, message.linear.z,
                message.angular.x, message.angular.y, message.angular.z])

    def graph(self):
        return dict(command=self.command_pub.get_subscription_count(),
                    scan=self.scan_pub.get_subscription_count(),
                    sonar=self.sonar_pub.get_subscription_count(),
                    output=self.output_sub.get_publisher_count())

    def set_world(self, scan_obstacle=False, sonar_obstacle=False,
                  publish_sonar=True):
        with self.lock:
            self.world = (scan_obstacle, sonar_obstacle, publish_sonar)
            self.record('world')
            return self.sensor_ticks

    def sensors(self):
        with self.lock:
            scan_obstacle, sonar_obstacle, publish_sonar = self.world
            self.scan(scan_obstacle)
            if publish_sonar:
                self.sonar(sonar_obstacle)
            self.sensor_ticks += 1
            self.record('sensors', tick=self.sensor_ticks)

    def publish_command(self, linear, angular, phase):
        with self.lock:
            self.sequence += 1
            count = len(self.outputs)
            self.record('command', identity=self.sequence, phase=phase,
                        linear=linear, angular=angular, graph=self.graph(),
                        output_barrier=count)
            message = Twist()
            message.linear.x = linear
            message.angular.z = angular
            self.command_pub.publish(message)
            return count

    def clear_precondition(self, timeout=4.0):
        self.set_world()
        self.record('precondition-start', graph=self.graph())
        deadline = time.monotonic() + timeout
        assert wait_until(lambda: all(self.graph().values()), timeout=timeout), (
            'Collision Monitor graph not ready', self.graph())
        # Twist has no command ID: a unique clear velocity acknowledges the last
        # probe on the ordered command/output streams before any fault is applied.
        while time.monotonic() < deadline:
            angular = 0.123 + (self.sequence + 1) * 0.000001
            self.probes.add((0.037, angular))
            count = self.publish_command(0.037, angular, 'precondition')
            if wait_until(lambda: any(
                    (m.linear.x, m.angular.z) == (0.037, angular)
                    for m in self.outputs[count:]),
                    timeout=min(0.1, max(0, deadline - time.monotonic()))):
                self.record('precondition-ready', graph=self.graph())
                with self.lock:
                    self.ack_source = max(
                        source for m, source in zip(
                            self.outputs[count:], self.output_sources[count:])
                        if (m.linear.x, m.angular.z) == (0.037, angular))
                assert self.ack_source > 0, 'DDS output source timestamp unavailable'
                return
        raise AssertionError('Collision Monitor clear nonzero precondition timed out')

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
        self.clear_precondition()
        tick = self.set_world(scan_obstacle, sonar_obstacle, publish_sonar)
        assert wait_until(lambda: self.sensor_ticks >= tick + 4, timeout=1.0)
        if not publish_sonar:
            # Disable the timer's sonar before aging it; scan continues throughout.
            started = time.monotonic()
            assert wait_until(lambda: time.monotonic() - started >= 0.6, timeout=1.0)
        count = self.publish_command(linear, angular, 'scenario')
        def response():
            with self.lock:
                return next((m for m, source in zip(
                    self.outputs[count:], self.output_sources[count:])
                    if source > self.ack_source and
                    (m.linear.x, m.angular.z) not in self.probes), None)
        assert wait_until(lambda: response() is not None, timeout=1.0), (
            'Collision Monitor scenario output timed out', self.graph())
        return response()


@pytest.fixture(scope='module')
def monitor(tmp_path_factory):
    log_path = fixture_log_path(
        'collision_monitor.log', tmp_path_factory.mktemp('collision-monitor'))
    with log_path.open('w') as log, NativeFixture(rclpy) as lifetime:
        process = lifetime.process(subprocess.Popen(
            [
                '/opt/ros/jazzy/lib/nav2_collision_monitor/collision_monitor',
                '--ros-args', '--params-file', str(PARAMS_FILE),
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
        ))
        lifetime.init()
        node = lifetime.node(CollisionHarness())
        node.process = process
        node.log_path = log_path

        def save_evidence():
            with node.lock:
                log_path.with_name('collision_monitor_fixture.json').write_text(
                    json.dumps(node.evidence, indent=2) + '\n')

        lifetime.cleanups.append(save_evidence)
        try:
            lifetime.start(MultiThreadedExecutor(num_threads=3))
            change_state = node.create_client(
                ChangeState, '/collision_monitor/change_state')
            assert change_state.wait_for_service(timeout_sec=5.0)
            for transition_id in (
                    Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                request = ChangeState.Request()
                request.transition.id = transition_id
                future = change_state.call_async(request)
                assert wait_until(future.done, timeout=5.0)
                node.record('lifecycle', transition=transition_id,
                            success=future.result().success, graph=node.graph())
                assert future.result().success
            node.clear_precondition()
            yield node
        finally:
            node.record('fixture-end', graph=node.graph(), child_status=process.poll())


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
