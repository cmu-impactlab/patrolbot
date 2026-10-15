"""Installed navigator's current-UUID path evidence across identical-goal preemption.

Synthetic action servers and a fixed TF; this verifies ownership, not robot motion.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from functools import partial
from types import SimpleNamespace as NS

assert os.environ.get('PATROLBOT_ISOLATED_TEST') == '1'
assert {p.name for p in Path('/sys/class/net').iterdir()} == {'lo'}
assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 90
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import (NavigateToPose, NavigateThroughPoses, ComputePathToPose,
    ComputePathThroughPoses, FollowPath, BackUp, Spin, Wait)
from nav2_msgs.srv import ClearEntireCostmap
from patrolbot_interfaces.msg import NavigationPath
from patrolbot_interfaces.srv import GetNavigationPath
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import StaticTransformBroadcaster
from patrolbot_web_bridge.commands import CommandExecutor
from navigation_localization_fixture import localization_fixture

parser = argparse.ArgumentParser()
parser.add_argument('--tree', choices=('to-pose', 'through-poses'), required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
started = time.monotonic()
deadline = started + 120.
def alarm(_signal, _frame):
    raise TimeoutError('120-second case budget exceeded')
signal.signal(signal.SIGALRM, alarm)
signal.alarm(120)
def until(predicate):
    while time.monotonic() < deadline:
        if capture.get("executor_failure"):
            raise RuntimeError(capture["executor_failure"])
        if predicate(): return
        time.sleep(.01)
    raise TimeoutError('120-second case budget exceeded')
def wait(future):
    until(future.done)
    return future.result()

capture = dict(tree=args.tree, physical_acceptance=False, paths=[], feedback=[],
               plan_calls=0, follow_calls=0, web_messages=[], unrelated_paths_published=0, passed=False)
binary = Path(get_package_prefix('nav2_bt_navigator'))/'lib/nav2_bt_navigator/bt_navigator'
assert str(binary).startswith('/opt/patrolbot_nav2/install/')
process = executor = node = None
release_old_plan = threading.Event()
finish_follow = threading.Event()
try:
    rclpy.init()
    node = rclpy.create_node('isolated_path_provenance')
    group = ReentrantCallbackGroup()
    transform = TransformStamped()
    transform.header.frame_id = 'map'; transform.child_frame_id = 'base_link'
    transform.header.stamp = node.get_clock().now().to_msg(); transform.transform.rotation.w = 1.
    broadcaster = StaticTransformBroadcaster(node); broadcaster.sendTransform(transform)
    localization_state, timer, localization_publishers = localization_fixture(node)
    class WebSocketCapture:
        def send(self, kind, data):
            capture['web_messages'].append(dict(kind=kind, data=data))
            if kind == 'telemetry.path':
                capture['paths'].append(dict(uuid=web._active['goal_id'].hex(),
                    points=len(data['points']), plan_calls=capture['plan_calls'],
                    time=time.monotonic()-started))
    web = CommandExecutor.__new__(CommandExecutor)
    web._node = NS(cfg={'path_max_points': 200})
    web._ws = WebSocketCapture(); web._active = None
    web._navigation_context = None; web._last_progress_mono = 0.
    web.update_navigation_context(dict(map_id='fixture', map_revision='v1',
        map_activated_stamp_ns=1, session_generation=1, odom_epoch_valid=True,
        localization_recovery_required=False, localization_seed_stamp_ns=1))
    web._GetNavigationPath = GetNavigationPath
    web._path_future = None; web._last_navigation_path = None
    endpoint = '/patrolbot/navigate_to_pose_path' if args.tree == 'to-pose' else '/patrolbot/navigate_through_poses_path'
    web._path_client = node.create_client(GetNavigationPath, endpoint, callback_group=group)
    path_timer = node.create_timer(.5, web._poll_navigation_path, callback_group=group)
    unrelated_pub = node.create_publisher(NavigationPath, '/patrolbot/navigation_path', 10)
    unrelated_plan_pub = node.create_publisher(NavPath, '/plan', 10)
    def execute(kind, action, handle):
        result = action.Result()
        if kind == 'plan':
            capture['plan_calls'] += 1
            if capture['plan_calls'] <= 2:
                until(release_old_plan.is_set)
            first = PoseStamped(); first.header.frame_id = 'map'; first.pose.orientation.w = 1.
            target = PoseStamped(); target.header.frame_id = 'map'; target.pose.position.x = 1.
            target.pose.orientation.w = 1.
            result.path = NavPath(); result.path.header.frame_id = 'map'
            result.path.header.stamp = node.get_clock().now().to_msg()
            result.path.poses = [first, target]
        elif kind == 'follow':
            capture['follow_calls'] += 1
            until(lambda: finish_follow.is_set() or handle.is_cancel_requested)
            if handle.is_cancel_requested:
                handle.canceled(); return result
        handle.succeed(); return result
    servers = [ActionServer(node, action, name, execute_callback=partial(execute,kind,action),
                            callback_group=group)
        for action,name,kind in [(ComputePathToPose,'compute_path_to_pose','plan'),
            (ComputePathThroughPoses,'compute_path_through_poses','plan'),
            (FollowPath,'follow_path','follow'), (BackUp,'backup','recovery'),
            (Spin,'spin','recovery'), (Wait,'wait','recovery')]]
    services = [node.create_service(ClearEntireCostmap, name, lambda request,response:response,
                                    callback_group=group)
        for name in ('local_costmap/clear_entirely_local_costmap',
                     'global_costmap/clear_entirely_global_costmap')]
    executor = MultiThreadedExecutor(num_threads=6); executor.add_node(node)
    def spin():
        try:
            executor.spin()
        except Exception as error:
            capture['executor_failure'] = repr(error)
    thread = threading.Thread(target=spin, daemon=True); thread.start()
    share = Path(get_package_share_directory('patrolbot_navigation'))
    trees = share/'config/behavior_trees'
    process = subprocess.Popen([str(binary),'--ros-args','--params-file',str(share/'config/nav2_params.yaml'),
        '-p','default_nav_to_pose_bt_xml:='+str(trees/'navigate_recovery_always_try.xml'),
        '-p','default_nav_through_poses_bt_xml:='+str(trees/'navigate_through_poses_recovery_always_try.xml')])
    change = node.create_client(ChangeState, '/bt_navigator/change_state')
    assert change.wait_for_service(timeout_sec=15.)
    for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
        request = ChangeState.Request(); request.transition.id = transition
        assert wait(change.call_async(request)).success
    action = NavigateToPose if args.tree == 'to-pose' else NavigateThroughPoses
    client = ActionClient(node, action, 'navigate_to_pose' if args.tree == 'to-pose' else 'navigate_through_poses')
    assert client.wait_for_server(timeout_sec=10.)
    goal = action.Goal(); pose = PoseStamped(); pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg(); pose.pose.position.x = 1.; pose.pose.orientation.w = 1.
    if args.tree == 'to-pose': goal.pose = pose
    elif hasattr(goal.poses,'goals'): goal.poses.header.frame_id = 'map'; goal.poses.goals = [pose]
    else: goal.poses = [pose]
    def feedback(message):
        if web._active is not None:
            web._on_feedback(web._active['command_id'], message)
        capture['feedback'].append(dict(uuid=bytes(message.goal_id.uuid).hex(),
                                         distance=float(message.feedback.distance_remaining)))
    old = wait(client.send_goal_async(goal, feedback_callback=feedback)); assert old.accepted
    old_uuid = bytes(old.goal_id.uuid).hex()
    web._active = dict(command_id='old', goal_id=bytes(old.goal_id.uuid), path_valid=False,
        path_blocked=False, navigation_context=web._navigation_context)
    until(lambda: capture['plan_calls'] == 1 and any(p['uuid'] == old_uuid for p in capture['paths']))
    assert all(p['points'] == 0 for p in capture['paths'] if p['uuid'] == old_uuid)
    assert any(f['uuid'] == old_uuid and f['distance'] == 0. for f in capture['feedback'])
    new = wait(client.send_goal_async(goal, feedback_callback=feedback)); assert new.accepted
    new_uuid = bytes(new.goal_id.uuid).hex(); assert new_uuid != old_uuid
    web._active = dict(command_id='new', goal_id=bytes(new.goal_id.uuid), path_valid=False,
        path_blocked=False, navigation_context=web._navigation_context)
    web._last_progress_mono = 0.
    until(lambda: any(p['uuid'] == new_uuid for p in capture['paths']))
    assert all(p['points'] == 0 for p in capture['paths'] if p['uuid'] == new_uuid)
    forged = NavigationPath(); forged.goal_id = new.goal_id
    forged.path.header.frame_id = 'map'; forged.path.header.stamp = node.get_clock().now().to_msg()
    forged.path.poses = [pose, pose]
    unrelated_pub.publish(forged); unrelated_plan_pub.publish(forged.path)
    capture['unrelated_paths_published'] = 2
    time.sleep(.6)
    assert not web._active['path_valid'], 'another publisher authorized the current path'
    until(lambda: any(m['kind'] == 'command.progress' and m['data']['command_id'] == 'new'
                     and m['data']['stage'] == 'planning' and m['data']['distance_remaining'] is None
                     for m in capture['web_messages']))
    release_old_plan.set()
    until(lambda: any(p['uuid'] == new_uuid and p['points'] > 0 for p in capture['paths']))
    current = [p for p in capture['paths'] if p['uuid'] == new_uuid and p['points'] > 0]
    assert all(p['plan_calls'] >= 2 for p in current), 'old same-destination path was attributed to new action'
    until(lambda: any(f['uuid'] == new_uuid and f['distance'] > 0. for f in capture['feedback']))
    until(lambda: any(m['kind'] == 'telemetry.path' and m['data']['command_id'] == 'new'
                     and m['data']['path_valid'] and len(m['data']['points']) == 2
                     for m in capture['web_messages']))
    until(lambda: any(m['kind'] == 'command.progress' and m['data']['command_id'] == 'new'
                     and m['data']['path_valid'] and m['data']['distance_remaining'] > 0.
                     for m in capture['web_messages']))
    finish_follow.set()
    capture['status'] = wait(new.get_result_async()).status
    assert capture['status'] == 4
    web._active = None
    path_clients = [node.create_client(GetNavigationPath, name, callback_group=group) for name in (
        '/patrolbot/navigate_to_pose_path', '/patrolbot/navigate_through_poses_path')]
    request_path = GetNavigationPath.Request(); request_path.goal_id = new.goal_id
    for path_client in path_clients:
        assert path_client.wait_for_service(timeout_sec=2.)
    request = ChangeState.Request(); request.transition.id = Transition.TRANSITION_DEACTIVATE
    assert wait(change.call_async(request)).success
    for path_client in path_clients:
        response = wait(path_client.call_async(request_path))
        assert not response.active and not response.path.poses, 'inactive navigator exposed a path'
    pending_queries = [path_client.call_async(request_path) for path_client in path_clients]
    request.transition.id = Transition.TRANSITION_CLEANUP
    assert wait(change.call_async(request)).success
    until(lambda: all(not c.service_is_ready() for c in path_clients))
    assert process.poll() is None, 'path query crashed navigator during cleanup'
    state = node.create_client(GetState, '/bt_navigator/get_state')
    assert wait(state.call_async(GetState.Request())).current_state.id == 1
    for path_client, future in zip(path_clients, pending_queries):
        if future.done():
            assert not future.result().active
        else:
            path_client.remove_pending_request(future)
    # A second configuration must recreate both services without old snapshots.
    for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
        request.transition.id = transition
        assert wait(change.call_async(request)).success
    for path_client in path_clients:
        assert path_client.wait_for_service(timeout_sec=2.)
        response = wait(path_client.call_async(request_path))
        assert not response.active and not response.path.poses, 'old action revived after reconfigure'
    capture.update(old_uuid=old_uuid, new_uuid=new_uuid,
                   lifecycle_queries=['inactive', 'cleanup', 'reconfigure'], passed=True)
except Exception as error:
    capture['failure'] = str(error)
    raise
finally:
    release_old_plan.set(); finish_follow.set(); signal.alarm(0)
    if process is not None:
        process.terminate()
        try: process.wait(timeout=5.)
        except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=5.)
        capture['navigator_exit_code'] = process.returncode
        assert process.returncode == 0, f'navigator did not shut down cleanly: {process.returncode}'
    if executor is not None: executor.shutdown(timeout_sec=2.)
    if node is not None: node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
    capture['elapsed_s'] = time.monotonic()-started
    args.output.write_text(json.dumps(capture, indent=2)+'\n')
    print(json.dumps(capture), flush=True)
assert capture['passed'], capture.get('failure','incomplete path evidence')
