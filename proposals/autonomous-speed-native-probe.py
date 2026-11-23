"""Study-only Jazzy probe. No goals, velocity publishers in the probe, or robot peers."""

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
from xml.etree import ElementTree

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import StaticTransformBroadcaster
import yaml


ROOT = Path(__file__).resolve().parents[1]
SOURCE = Path('/opt/scoped-native/src/nav2')
INSTALL = Path('/opt/scoped-native/install')
SPEEDS = ('FollowPath.desired_linear_vel', 'FollowPath.rotate_to_heading_angular_vel')


def source_excerpt(file, needles, margin=3):
    lines = (SOURCE / file).read_text().splitlines()
    selected = {j for i, line in enumerate(lines) if any(n in line for n in needles)
                for j in range(max(0, i - margin), min(len(lines), i + margin + 1))}
    print(f'\n{file}')
    for j in sorted(selected):
        print(f'{j + 1}: {lines[j]}')


def source_evidence():
    patch = '0018-tuning-navigation-admission.patch'
    print('cached build patch matches study:', (Path('/opt/scoped-patches') / patch).read_bytes()
          == (ROOT / 'docker/nav2-patches' / patch).read_bytes())
    source_excerpt('nav2_regulated_pure_pursuit_controller/src/parameter_handler.cpp',
                   ('add_on_set_parameters_callback', 'desired_linear_vel =',
                    'rotate_to_heading_angular_vel =', 'lock('))
    source_excerpt('nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp',
                   ('param_handler_->getMutex', 'linear_vel = params_->desired_linear_vel',
                    'sign * params_->rotate_to_heading_angular_vel', 'base_desired_linear_vel'))
    source_excerpt('nav2_controller/src/controller_server.cpp',
                   ('dynamicParametersCallback', 'find(\'.\')', 'try_lock()', '"follow_path"',
                    'create_subscription<nav2_msgs::msg::SpeedLimit>'))
    source_excerpt('nav2_core/include/nav2_core/behavior_tree_navigator.hpp',
                   ('tryStartNavigating(getName())', 'stopNavigating(getName())', 'onPreempt'))
    source_excerpt('nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp',
                   ('on_goal_received_callback_(', 'on_preempt_callback_('))
    source_excerpt('nav2_util/include/nav2_util/simple_action_server.hpp',
                   ('GoalResponse::ACCEPT', 'pending_handle_ = handle', 'execution_callback_();'))
    source_excerpt('nav2_waypoint_follower/src/waypoint_follower.cpp',
                   ('"follow_waypoints"', '"navigate_to_pose"', 'goal_handle->accepted',
                    'result.code =='), margin=2)


def probe(node):
    def call(service, name, request):
        client = node.create_client(service, f'/controller_server/{name}')
        try:
            assert client.wait_for_service(timeout_sec=12), name
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=8)
            assert future.done() and future.result() is not None, name
            return future.result()
        finally:
            node.destroy_client(client)

    def transition(transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        assert call(ChangeState, 'change_state', request).success, transition_id

    def read(expected, label):
        state = call(GetState, 'get_state', GetState.Request()).current_state
        assert state.id == State.PRIMARY_STATE_ACTIVE, state
        request = GetParameters.Request(names=list(SPEEDS))
        values = call(GetParameters, 'get_parameters', request).values
        assert [value.type for value in values] == [ParameterType.PARAMETER_DOUBLE] * 2, values
        observed = tuple(value.double_value for value in values)
        assert observed == expected, (label, observed, expected)
        print(json.dumps({'sample': label, 'state': state.label, 'values': dict(zip(SPEEDS, observed))}))

    def set_pair(values, *, success):
        request = SetParametersAtomically.Request(parameters=[
            Parameter(name, value=value).to_parameter_msg() for name, value in zip(SPEEDS, values)
        ])
        result = call(SetParametersAtomically, 'set_parameters_atomically', request).result
        print(json.dumps({'request': values, 'success': result.successful, 'reason': result.reason}))
        assert result.successful == success, result

    transition(Transition.TRANSITION_CONFIGURE)
    transition(Transition.TRANSITION_ACTIVATE)
    read((0.22, 0.70), 'initial')
    set_pair((0.20, 0.60), success=True)
    read((0.20, 0.60), 'changed')
    set_pair((0.19, 'invalid type'), success=False)
    read((0.20, 0.60), 'after rejected transaction')
    set_pair((0.22, 0.70), success=True)
    read((0.22, 0.70), 'restored')
    transition(Transition.TRANSITION_DEACTIVATE)
    transition(Transition.TRANSITION_CLEANUP)


def main():
    if (os.geteuid() == 0 or os.environ.get('PATROLBOT_ISOLATED_TEST') != '1'
            or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'
            or os.environ.get('ROS_STATIC_PEERS', '')
            or {p.name for p in Path('/sys/class/net').iterdir()} != {'lo'}
            or not __debug__):
        raise SystemExit('requires the non-root, network-isolated study container without -O')
    for package in ('nav2_controller', 'nav2_regulated_pure_pursuit_controller'):
        prefix = Path(get_package_prefix(package))
        version = ElementTree.parse(Path(get_package_share_directory(package)) / 'package.xml')
        version = version.findtext('version')
        print(f'{package}: {prefix}, version {version}')
        assert prefix.is_relative_to(INSTALL) and version == '1.3.12', (prefix, version)
    source_evidence()
    config = yaml.safe_load((ROOT / 'ros2_ws/src/patrolbot_navigation/config/nav2_params.yaml')
                            .read_text())
    config = {'controller_server': config['controller_server'],
              'local_costmap': {'local_costmap': {'ros__parameters': {
                  'global_frame': 'odom', 'robot_base_frame': 'base_link', 'rolling_window': True,
                  'width': 3, 'height': 3, 'resolution': 0.05, 'robot_radius': 0.22,
                  'plugins': ['inflation_layer'],
                  'inflation_layer': {'plugin': 'nav2_costmap_2d::InflationLayer'},
              }}}}
    config['controller_server']['ros__parameters']['FollowPath']['rotate_to_heading_angular_vel'] = 0.70
    rclpy.init()
    node = Node('isolated_speed_probe')
    broadcaster = StaticTransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.stamp = node.get_clock().now().to_msg()
    tf.header.frame_id, tf.child_frame_id = 'odom', 'base_link'
    tf.transform.rotation.w = 1.0
    broadcaster.sendTransform(tf)
    try:
        with tempfile.TemporaryDirectory() as tmp:
            params = Path(tmp) / 'controller.yaml'
            params.write_text(yaml.safe_dump(config))
            executable = Path(get_package_prefix('nav2_controller')) / 'lib/nav2_controller/controller_server'
            with (Path(tmp) / 'controller.log').open('w+') as log:
                proc = subprocess.Popen([
                    str(executable), '--ros-args', '--params-file', str(params),
                    '-r', 'cmd_vel:=isolated_probe_unused_cmd_vel',
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
    print('PASS: one active-controller set/readback probe')


if __name__ == '__main__':
    main()
