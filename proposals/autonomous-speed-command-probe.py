"""Controller-output study. Run only in the documented loopback-only container."""

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from xml.etree import ElementTree

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import StaticTransformBroadcaster
import yaml


ROOT = Path(__file__).resolve().parents[1]
INSTALL = Path('/opt/scoped-native/install')
NODE = 'isolated_rpp_command_probe'
COMMAND = '/isolated_rpp_probe_cmd_vel'
ODOM = '/isolated_rpp_probe_odom'
SPEED_LIMIT = '/isolated_rpp_probe_speed_limit'
SPEEDS = ('FollowPath.desired_linear_vel', 'FollowPath.rotate_to_heading_angular_vel')
sys.path.insert(0, str(ROOT / 'ros2_ws/src/patrolbot_navigation/scripts'))

from autonomous_speed_output import (  # noqa: E402
    ActiveOutputFence,
    CommandEpoch,
    ControllerSample,
    guard_autonomous_speed_output,
)


def probe(node):
    def wait(future, label):
        rclpy.spin_until_future_complete(node, future, timeout_sec=4)
        assert future.done() and future.result() is not None, label
        return future.result()

    def call(service, name, request):
        client = node.create_client(service, f'/controller_server/{name}')
        try:
            assert client.wait_for_service(timeout_sec=8), name
            return wait(client.call_async(request), name)
        finally:
            node.destroy_client(client)

    def transition(transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        assert call(ChangeState, 'change_state', request).success, transition_id

    def read(expected):
        state = call(GetState, 'get_state', GetState.Request()).current_state
        assert state.id == State.PRIMARY_STATE_ACTIVE, state
        request = GetParameters.Request(names=list(SPEEDS))
        values = call(GetParameters, 'get_parameters', request).values
        assert [value.type for value in values] == [ParameterType.PARAMETER_DOUBLE] * 2
        observed = tuple(value.double_value for value in values)
        assert observed == expected, (observed, expected)
        return observed

    def set_pair(linear, angular):
        request = SetParametersAtomically.Request(parameters=[
            Parameter(name, value=value).to_parameter_msg()
            for name, value in zip(SPEEDS, (linear, angular))
        ])
        result = call(SetParametersAtomically, 'set_parameters_atomically', request).result
        assert result.successful, result.reason

    def spin_for(predicate, seconds, label):
        deadline = time.monotonic() + seconds
        while not predicate():
            assert time.monotonic() < deadline, label
            rclpy.spin_once(node, timeout_sec=0.05)

    samples = []

    def observe(twist):
        vx, vy, vz = twist.linear.x, twist.linear.y, twist.linear.z
        wx, wy, wz = twist.angular.x, twist.angular.y, twist.angular.z
        assert all(math.isfinite(v) for v in (vx, vy, vz, wx, wy, wz)), twist
        assert math.hypot(vx, vy, vz) <= 0.26 and math.hypot(wx, wy, wz) <= 0.70, twist
        assert max(abs(v) for v in (vy, vz, wx, wy, wz)) < 1e-7, twist
        samples.append((vx, wz, time.monotonic()))

    node.create_subscription(Twist, COMMAND, observe, 10)
    speed_pub = node.create_publisher(SpeedLimit, SPEED_LIMIT, 10)
    odom_pub = node.create_publisher(Odometry, ODOM, 10)
    static_tf = StaticTransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.stamp = node.get_clock().now().to_msg()
    tf.header.frame_id, tf.child_frame_id = 'odom', 'base_link'
    tf.transform.rotation.w = 1.0
    static_tf.sendTransform(tf)

    def publish_fixed_odom():
        msg = Odometry()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose.orientation.w = 1.0
        odom_pub.publish(msg)

    node.create_timer(0.05, publish_fixed_odom)
    transition(Transition.TRANSITION_CONFIGURE)
    transition(Transition.TRANSITION_ACTIVATE)

    def fixture_ready():
        publishers = node.get_publishers_info_by_topic(COMMAND)
        subscribers = node.get_subscriptions_info_by_topic(COMMAND)
        return ([info.node_name for info in publishers] == ['controller_server']
                and [info.node_name for info in subscribers] == [NODE]
                and speed_pub.get_subscription_count() == 1
                and odom_pub.get_subscription_count() == 1)

    spin_for(fixture_ready, 4, 'expected one controller publisher and only the probe subscriber')
    client = ActionClient(node, FollowPath, '/follow_path')
    assert client.wait_for_server(timeout_sec=4), 'FollowPath action unavailable'
    goal = FollowPath.Goal(controller_id='FollowPath', goal_checker_id='general_goal_checker',
                           progress_checker_id='progress_checker')
    goal.path.header.frame_id = 'odom'
    goal.path.header.stamp = node.get_clock().now().to_msg()
    for i in range(13):
        pose = PoseStamped(header=goal.path.header)
        pose.pose.position.x = i / 10.0
        pose.pose.orientation.w = 1.0
        goal.path.poses.append(pose)

    baseline = (0.22, 0.70)
    updated = (0.20, 0.60)
    read(baseline)
    goal_handle = wait(client.send_goal_async(goal), 'send synthetic FollowPath')
    assert goal_handle.accepted, 'synthetic FollowPath rejected'
    result = goal_handle.get_result_async()

    def measure(label, expected_output, parameters, start_index):
        def settled():
            assert not result.done(), (label, result.result())
            recent = samples[start_index:]
            return len(recent) >= 3 and all(
                abs(x - expected_output) < 1e-6 for x, _, _ in recent[-3:])

        spin_for(settled, 3, (label, expected_output, samples))
        observed = samples[-3:]
        assert fixture_ready(), 'unexpected controller-output endpoint'
        assert read(parameters) == parameters
        # Replay each captured sample through the pure model at its simulated
        # receipt instant. This publishes no guarded velocity to any graph.
        fence = ActiveOutputFence(CommandEpoch(), parameters)
        replay = [guard_autonomous_speed_output(
            ControllerSample(fence.epoch, x, z, received), fence,
            now_monotonic_s=received) for x, z, received in observed]
        assert all(abs(x - min(expected_output, parameters[0])) < 1e-6
                   and abs(z) < 1e-7 for x, z in replay), (label, replay)
        print(json.dumps({'phase': label, 'parameters': dict(zip(SPEEDS, parameters)),
                          'commands_linear_x_angular_z': [(x, z) for x, z, _ in observed],
                          'offline_guard_replay_linear_x_angular_z': replay}))

    def speed_limit(value, *, percentage):
        msg = SpeedLimit(percentage=percentage, speed_limit=value)
        msg.header.stamp = node.get_clock().now().to_msg()
        speed_pub.publish(msg)

    try:
        measure('initial', 0.22, baseline, 0)
        set_pair(*updated)
        read(updated)
        measure('after atomic set and readback', 0.20, updated, len(samples))
        speed_limit(75.0, percentage=True)
        measure('route-style SpeedLimit 75%', 0.15, updated, len(samples))
        speed_limit(0.24, percentage=False)
        measure('absolute SpeedLimit 0.24', 0.24, updated, len(samples))
        speed_limit(0.0, percentage=True)  # Nav2 no-limit sentinel; restore the base target.
        measure('SpeedLimit cleared', 0.20, updated, len(samples))
    finally:
        if not result.done():
            cancel = wait(goal_handle.cancel_goal_async(), 'cancel synthetic FollowPath')
            assert cancel.goals_canceling, cancel
    assert wait(result, 'synthetic FollowPath terminal result').status == GoalStatus.STATUS_CANCELED
    transition(Transition.TRANSITION_DEACTIVATE)
    transition(Transition.TRANSITION_CLEANUP)


def main():
    if (os.geteuid() == 0 or os.environ.get('PATROLBOT_ISOLATED_TEST') != '1'
            or os.environ.get('ROS_DISTRO') != 'jazzy'
            or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'
            or os.environ.get('ROS_STATIC_PEERS', '')
            or {p.name for p in Path('/sys/class/net').iterdir()} != {'lo'}
            or not __debug__):
        raise SystemExit('requires nonroot, Jazzy and isolated loopback-only Docker')
    for package in ('nav2_controller', 'nav2_regulated_pure_pursuit_controller', 'nav2_msgs'):
        prefix = Path(get_package_prefix(package))
        manifest = Path(get_package_share_directory(package)) / 'package.xml'
        version = ElementTree.parse(manifest).findtext('version')
        assert prefix.is_relative_to(INSTALL) and version == '1.3.12', (prefix, version)
        print(f'{package}: {prefix}, version {version}')
    route_src = Path('/opt/scoped-native/src/nav2/nav2_route')
    for source_file in route_src.rglob('adjust_speed_limit.cpp'):
        for line_no, line in enumerate(source_file.read_text().splitlines(), 1):
            if 'percentage' in line or '->publish(' in line or '"speed_limit"' in line:
                print(f'{source_file.relative_to(route_src)}:{line_no}: {line.strip()}')

    source = ROOT / 'ros2_ws/src/patrolbot_navigation/config/nav2_params.yaml'
    controller = yaml.safe_load(source.read_text())['controller_server']
    params = controller['ros__parameters']
    params.update(odom_topic=ODOM, speed_limit_topic=SPEED_LIMIT, enable_stamped_cmd_vel=False)
    params['FollowPath']['rotate_to_heading_angular_vel'] = 0.70
    params['progress_checker']['movement_time_allowance'] = 20.0
    config = {'controller_server': controller,
              'local_costmap': {'local_costmap': {'ros__parameters': {
                  'global_frame': 'odom', 'robot_base_frame': 'base_link', 'rolling_window': True,
                  'width': 3, 'height': 3, 'resolution': 0.05, 'robot_radius': 0.22,
                  'plugins': ['inflation_layer'],
                  'inflation_layer': {'plugin': 'nav2_costmap_2d::InflationLayer'},
              }}}}
    executable = Path(get_package_prefix('nav2_controller')) / 'lib/nav2_controller/controller_server'
    rclpy.init()
    node = Node(NODE)
    try:
        with tempfile.TemporaryDirectory() as tmp:
            params_file = Path(tmp) / 'controller.yaml'
            params_file.write_text(yaml.safe_dump(config))
            with (Path(tmp) / 'controller.log').open('w+') as log:
                proc = subprocess.Popen([
                    str(executable), '--ros-args', '--params-file', str(params_file),
                    '-r', f'cmd_vel:={COMMAND}',
                ], stdout=log, stderr=subprocess.STDOUT)
                try:
                    probe(node)
                finally:
                    if proc.poll() is None:
                        proc.send_signal(signal.SIGINT)
                    try:
                        proc.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=3)
                        raise
                    finally:
                        log.seek(0)
                        print(log.read())
                assert proc.returncode == 0, proc.returncode
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print('PASS: one straight FollowPath controller-output probe')


if __name__ == '__main__':
    main()
