"""Isolated configured-component teardown regression (no robot devices).

Run inside the deployed ROS image with --network none --ipc private and
PATROLBOT_ISOLATED_TEST=1. PATROLBOT_TEST_CLEANUP=1 inserts explicit cleanup
before shutdown. A crash, lifecycle failure, or forced kill fails the test.
"""
import json, os, signal, subprocess, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from composition_interfaces.srv import LoadNode
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from rclpy.parameter import Parameter
assert os.environ.get('PATROLBOT_ISOLATED_TEST') == '1'
assert {p.name for p in Path('/sys/class/net').iterdir()} == {'lo'}
rclpy.init()
node = Node('isolated_component_probe')
def call(client, request):
    assert client.wait_for_service(timeout_sec=12), client.srv_name
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15)
    assert future.done(), client.srv_name
    return future.result()
results=[]
for case_index, (package, plugin, original_name) in enumerate([('nav2_bt_navigator','nav2_bt_navigator::BtNavigator','bt_navigator'),('nav2_bt_navigator','nav2_bt_navigator::BtNavigator','bt_navigator'),('nav2_waypoint_follower','nav2_waypoint_follower::WaypointFollower','waypoint_follower'),('nav2_route','nav2_route::RouteServer','route_server'),('nav2_controller','nav2_controller::ControllerServer','controller_server')]):
  for configure in (False,True):
    name=f'{original_name}_{case_index}_{int(configure)}'
    container=f'probe_container_{case_index}_{int(configure)}'
    label=name
    path=Path('/tmp/'+label+'.log')
    with path.open('w') as log:
      proc=subprocess.Popen(['/opt/ros/jazzy/lib/rclcpp_components/component_container_isolated','--ros-args','-r','__node:='+container],stdout=log,stderr=subprocess.STDOUT)
      failure=None
      try:
        c=node.create_client(LoadNode,'/'+container+'/_container/load_node')
        req=LoadNode.Request();req.package_name=package;req.plugin_name=plugin;req.node_name=name
        if package == 'nav2_controller':
          # Synthetic TF only inside the required network-isolated container.
          broadcaster = StaticTransformBroadcaster(node)
          tf = TransformStamped()
          tf.header.frame_id = 'map'; tf.child_frame_id = 'base_link'
          tf.header.stamp = node.get_clock().now().to_msg()
          tf.transform.rotation.w = 1.0
          broadcaster.sendTransform(tf)
          req.parameters = [
            Parameter('controller_plugins', value=['FollowPath']).to_parameter_msg(),
            Parameter('FollowPath.plugin', value='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController').to_parameter_msg(),
          ]
        response=call(c,req);assert response.success,response.error_message
        node.destroy_client(c)
        if configure:
          c=node.create_client(ChangeState,'/'+name+'/change_state')
          transitions = [Transition.TRANSITION_CONFIGURE]
          if os.environ.get('PATROLBOT_TEST_CLEANUP') == '1':
            transitions += [Transition.TRANSITION_CLEANUP, Transition.TRANSITION_UNCONFIGURED_SHUTDOWN]
          else:
            transitions += [Transition.TRANSITION_INACTIVE_SHUTDOWN]
          for transition in transitions:
            req=ChangeState.Request();req.transition.id=transition
            assert call(c,req).success,(name,transition)
          node.destroy_client(c)
      except Exception as exc: failure=repr(exc)
      finally:
        if proc.poll() is None:proc.send_signal(signal.SIGINT)
        try:code=proc.wait(timeout=15)
        except subprocess.TimeoutExpired:proc.kill();code=proc.wait();failure=str(failure)+' forced kill'
    output=path.read_text()
    result={'case':label,'exit':code,'failure':failure,'segfault':code == -signal.SIGSEGV or 'SIGSEGV' in output or 'Segmentation' in output}
    results.append(result);print(json.dumps(result),flush=True);print(output,flush=True)
node.destroy_node();rclpy.try_shutdown()
print(json.dumps({'results':results}),flush=True)

assert all(r["exit"] == 0 and r["failure"] is None and not r["segfault"] for r in results), results
