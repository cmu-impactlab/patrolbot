"""Shutdown during missing-TF activation; isolated container only.

PATROLBOT_STARTUP_MANAGER=1 routes startup through LifecycleManager. Otherwise
exercise direct activation. Both paths must exit cleanly without a forced kill.
No TF, pose, goal, velocity or hardware commands are published.
"""
import json, os, signal, subprocess, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from composition_interfaces.srv import LoadNode
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_msgs.srv import ManageLifecycleNodes
assert os.environ.get('PATROLBOT_ISOLATED_TEST') == '1'
assert {p.name for p in Path('/sys/class/net').iterdir()} == {'lo'}
rclpy.init()
node = Node('startup_cancel_probe')
managed = os.environ.get('PATROLBOT_STARTUP_MANAGER') == '1'
manager = node.create_client(ManageLifecycleNodes, '/startup_manager/manage_nodes')
log = Path('/tmp/startup-cancel.log')
def call(client, request):
    assert client.wait_for_service(timeout_sec=10), client.srv_name
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20)
    assert future.done(), client.srv_name
    return future.result()
with log.open('w') as out:
    proc = subprocess.Popen(['/opt/ros/jazzy/lib/rclcpp_components/component_container_isolated', '--ros-args', '-r', '__node:=startup_cancel_container'], stdout=out, stderr=subprocess.STDOUT)
    forced = False
    try:
        loader = node.create_client(LoadNode, '/startup_cancel_container/_container/load_node')
        request = LoadNode.Request()
        request.package_name = 'nav2_controller'
        request.plugin_name = 'nav2_controller::ControllerServer'
        request.node_name = 'controller_server'
        request.parameters = [Parameter('controller_plugins', value=['FollowPath']).to_parameter_msg(), Parameter('FollowPath.plugin', value='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController').to_parameter_msg()]
        assert call(loader, request).success
        if managed:
            request = LoadNode.Request()
            request.package_name = 'nav2_lifecycle_manager'
            request.plugin_name = 'nav2_lifecycle_manager::LifecycleManager'
            request.node_name = 'startup_manager'
            request.parameters = [Parameter('node_names', value=['controller_server']).to_parameter_msg()]
            assert call(loader, request).success
            assert manager.wait_for_service(timeout_sec=10)
            request = ManageLifecycleNodes.Request()
            request.command = request.STARTUP
            activation = manager.call_async(request)
        else:
            change = node.create_client(ChangeState, '/controller_server/change_state')
            request = ChangeState.Request()
            request.transition.id = Transition.TRANSITION_CONFIGURE
            assert call(change, request).success
            request = ChangeState.Request()
            request.transition.id = Transition.TRANSITION_ACTIVATE
            activation = change.call_async(request)
        deadline = time.monotonic() + 10
        while 'Timed out waiting for transform' not in log.read_text() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        assert 'Timed out waiting for transform' in log.read_text(), 'did not reach TF wait'
        assert not activation.done(), 'activation unexpectedly completed without TF'
        start = time.monotonic()
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            forced = True
            proc.kill()
            proc.wait()
        print(json.dumps({'event': 'shutdown_during_tf_wait', 'managed': managed, 'exit': proc.returncode, 'forced': forced, 'elapsed': time.monotonic()-start}), flush=True)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
        node.destroy_node()
        rclpy.try_shutdown()
        print(log.read_text(), flush=True)
assert not forced and proc.returncode == 0, 'shutdown during missing-TF activation failed'
