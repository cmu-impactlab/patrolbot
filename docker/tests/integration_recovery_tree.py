"""Real installed BT navigator; synthetic typed action responses, not robot dynamics.

Run each mode/tree separately in a network-none container with candidate overlay.
No production retries/timeouts or XML are overridden. Synthetic behavior actions
return immediately; this is full tree orchestration acceptance only.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from functools import partial

assert os.environ.get('PATROLBOT_ISOLATED_TEST') == '1'
assert {p.name for p in Path('/sys/class/net').iterdir()} == {'lo'}
assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 90
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Path as NavPath
from nav2_msgs.action import (NavigateToPose, NavigateThroughPoses, ComputePathToPose,
    ComputePathThroughPoses, FollowPath, BackUp, Spin, Wait)
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import StaticTransformBroadcaster
from recovery_tree_capture import validate
from navigation_localization_fixture import localization_fixture

parser = argparse.ArgumentParser()
parser.add_argument('--tree', choices=('to-pose','through-poses'), required=True)
parser.add_argument('--mode', choices=('exhaust','fault-clears','planner-exhaust'), required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
started = time.monotonic()
deadline = started + 120.
def alarm(_signal, _frame):
    raise TimeoutError('original 120-second case budget exceeded')
signal.signal(signal.SIGALRM, alarm)
signal.alarm(120)
def until(predicate):
    while time.monotonic() < deadline:
        if predicate(): return
        time.sleep(.01)
    raise TimeoutError('original 120-second case budget exceeded')
def wait(future):
    until(future.done)
    return future.result()

share = Path(get_package_share_directory('patrolbot_navigation'))
trees = share / 'config/behavior_trees'
selected = trees / ('navigate_recovery_always_try.xml' if args.tree == 'to-pose' else 'navigate_through_poses_recovery_always_try.xml')
capture = dict(mode=args.mode, tree=args.tree, physical_acceptance=False,
    acceptance_mode='production-tree-injected-actions', natural_frozen_exhaustion_passed=False,
    tree_sha256=hashlib.sha256(selected.read_bytes()).hexdigest(),
    follow_calls=0, plan_calls=0, recovery_actions=[], events=[], passed=False)
binary = Path(get_package_prefix('nav2_bt_navigator')) / 'lib/nav2_bt_navigator/bt_navigator'
assert str(binary).startswith('/opt/patrolbot_nav2/install/'), 'patched production navigator overlay unavailable'
capture.update(navigator_binary=str(binary), navigator_binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
process = executor = node = None
try:
    rclpy.init()
    node = rclpy.create_node('isolated_recovery_actions')
    group = ReentrantCallbackGroup()
    transform = TransformStamped()
    transform.header.frame_id = 'map'; transform.child_frame_id = 'base_link'
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.transform.rotation.w = 1.
    broadcaster = StaticTransformBroadcaster(node); broadcaster.sendTransform(transform)
    localization = localization_fixture(node)
    def execute(kind, action, handle):
        capture['events'].append(kind)
        result = action.Result()
        if kind == 'follow':
            capture['follow_calls'] += 1
            if args.mode == 'exhaust' or capture['follow_calls'] <= 8:
                result.error_code = result.FAILED_TO_MAKE_PROGRESS
                handle.abort(); return result
        elif kind == 'plan':
            capture['plan_calls'] += 1
            if args.mode == 'planner-exhaust':
                # A route failure permits one contextual clear, then stops without motion.
                result.error_code = result.NO_VALID_PATH
                handle.abort(); return result
            pose = PoseStamped(); pose.header.frame_id = 'map'; pose.pose.orientation.w = 1.
            target = PoseStamped(); target.header.frame_id = 'map'; target.pose.position.x = 1.; target.pose.orientation.w = 1.
            result.path = NavPath(); result.path.header.frame_id = 'map'; result.path.poses = [pose,target]
        else:
            capture['recovery_actions'].append(kind)
        handle.succeed(); return result
    servers = [ActionServer(node, action, name, execute_callback=partial(execute,kind,action), callback_group=group)
        for action,name,kind in [(ComputePathToPose,'compute_path_to_pose','plan'),
            (ComputePathThroughPoses,'compute_path_through_poses','plan'),
            (FollowPath,'follow_path','follow'), (BackUp,'backup','backup'),
            (Spin,'spin','spin'), (Wait,'wait','wait')]]
    def clear(kind, request, response):
        capture['events'].append(kind)
        # Outer clearing is local THEN global; contextual clearing affects only one costmap.
        if capture['events'][-2:] == ['clear-local', 'clear-global']:
            capture['recovery_actions'].append('clear')
        return response
    services = [node.create_service(ClearEntireCostmap, name, partial(clear,kind), callback_group=group)
        for name,kind in [('local_costmap/clear_entirely_local_costmap','clear-local'),
                          ('global_costmap/clear_entirely_global_costmap','clear-global')]]
    executor = MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
    process = subprocess.Popen([str(binary),'--ros-args',
        '--params-file',str(share/'config/nav2_params.yaml'),
        '-p','default_nav_to_pose_bt_xml:='+str(trees/'navigate_recovery_always_try.xml'),
        '-p','default_nav_through_poses_bt_xml:='+str(trees/'navigate_through_poses_recovery_always_try.xml')])
    change = node.create_client(ChangeState, '/bt_navigator/change_state')
    assert change.wait_for_service(timeout_sec=15.), 'navigator lifecycle unavailable'
    for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
        request = ChangeState.Request(); request.transition.id = transition
        assert wait(change.call_async(request)).success, 'navigator lifecycle transition failed'
    action = NavigateToPose if args.tree == 'to-pose' else NavigateThroughPoses
    client = ActionClient(node, action, 'navigate_to_pose' if args.tree == 'to-pose' else 'navigate_through_poses')
    assert client.wait_for_server(timeout_sec=10.)
    goal = action.Goal(); pose = PoseStamped(); pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg(); pose.pose.position.x = 1.; pose.pose.orientation.w = 1.
    if args.tree == 'to-pose': goal.pose = pose
    elif hasattr(goal.poses,'goals'): goal.poses.header.frame_id = 'map'; goal.poses.goals = [pose]
    else: goal.poses = [pose]
    handle = wait(client.send_goal_async(goal)); assert handle.accepted
    capture['status'] = wait(handle.get_result_async()).status
    capture['elapsed_s'] = time.monotonic()-started
    outer_clears = capture['recovery_actions'].count('clear')
    capture['contextual_clear_calls'] = capture['events'].count('clear-local') - outer_clears
    capture['planner_contextual_clear_calls'] = capture['events'].count('clear-global') - outer_clears
except Exception as error:
    capture['failure'] = str(error)
    raise
finally:
    signal.alarm(0)
    if process is not None:
        process.terminate()
        try: process.wait(timeout=5.)
        except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=5.)
        capture['navigator_exit_code'] = process.returncode
    if executor is not None: executor.shutdown(timeout_sec=2.)
    if node is not None: node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
    capture['elapsed_s'] = time.monotonic()-started
    if 'status' in capture:
        try:
            validate(capture)
            capture['passed'] = True
        except AssertionError as error:
            capture.update(passed=False, failure=str(error))
    args.output.write_text(json.dumps(capture, indent=2)+'\n')
    print(json.dumps({'event':'production_recovery_tree',**capture}),flush=True)
assert capture['passed'], capture.get('failure', 'recovery capture incomplete')
