"""Installed BT goal guard with synthetic inputs/actions, entirely network isolated."""
import argparse
from functools import partial
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

assert os.environ.get('PATROLBOT_ISOLATED_TEST') == '1'
assert {p.name for p in Path('/sys/class/net').iterdir()} == {'lo'}
assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 90
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Path as NavPath
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses, ComputePathToPose, ComputePathThroughPoses, FollowPath, BackUp, Spin, Wait
from nav2_msgs.srv import ClearEntireCostmap
from patrolbot_interfaces.srv import GetNavigationPath
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Empty
from tf2_ros import StaticTransformBroadcaster
from navigation_localization_fixture import localization_fixture

parser = argparse.ArgumentParser()
parser.add_argument('--tree', choices=('to-pose', 'through-poses'), required=True)
parser.add_argument('--mode', choices=('refresh-success', 'blocked-persistent', 'stale-and-blocked',
    'costmap-timeout', 'service-unavailable', 'stale-amcl', 'invalid-amcl', 'context-change',
    'cancel', 'replacement', 'replacement-planning', 'replacement-following', 'failed-replan', 'controller-recovery', 'stale-odom',
    'unaccepted-amcl', 'invalid-quaternion', 'invalid-covariance', 'blocked-service-unavailable',
    'retained-amcl', 'retained-stale', 'invalid-odom', 'invalid-odom-frame', 'following-ack-lag'), required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
start = time.monotonic(); deadline = start + 60.
capture = dict(tree=args.tree, mode=args.mode, physical_acceptance=False,
    fixture_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    localization_fixture_sha256=hashlib.sha256(Path(__file__).with_name('navigation_localization_fixture.py').read_bytes()).hexdigest(), plan_calls=0,
    follow_calls=0, refresh_calls=0, clear_calls=0, recovery_actions=[], statuses=[], feedback=[], velocities=[], passed=False)
def alarm(*_): raise TimeoutError('60-second case budget exceeded')
signal.signal(signal.SIGALRM, alarm); signal.alarm(60)
def until(predicate):
    while time.monotonic() < deadline:
        if capture.get('executor_failure'): raise RuntimeError(capture['executor_failure'])
        if predicate(): return
        time.sleep(.01)
    raise TimeoutError('case deadline exceeded')
def wait(future):
    until(future.done); return future.result()
node = executor = process = None
try:
    rclpy.init(); node = rclpy.create_node('isolated_stationary_goal')
    group = ReentrantCallbackGroup()
    state, timer, publishers = localization_fixture(node)
    def velocity(message):
        twist = message.twist if hasattr(message, 'twist') else message
        values = [getattr(vector, axis) for vector in (twist.linear, twist.angular) for axis in ('x', 'y', 'z')]
        capture['velocities'].append(values)
        assert all(v == 0. for v in values), values
    velocity_subs = [node.create_subscription(kind, topic, velocity, 10) for kind, topic in (
        (Twist, '/cmd_vel'), (TwistStamped, '/cmd_vel_nav'), (TwistStamped, '/cmd_vel_smoothed'))]
    needs_stale = args.mode in ('stale-and-blocked', 'stale-amcl', 'service-unavailable', 'invalid-amcl',
        'context-change', 'cancel', 'replacement', 'stale-odom', 'retained-stale', 'invalid-odom', 'invalid-odom-frame') or (args.tree == 'through-poses' and args.mode == 'refresh-success')
    if needs_stale: state['pose_age'] = 31.
    if args.mode == 'invalid-amcl': state['invalid_pose'] = True
    if args.mode == 'invalid-quaternion': state['bad_quaternion'] = True
    if args.mode == 'invalid-covariance': state['bad_covariance'] = True
    if args.mode == 'unaccepted-amcl': state['accepted'] = False
    if args.mode == 'stale-odom': state['odom_enabled'] = False
    if args.mode == 'invalid-odom': state['invalid_odom'] = True
    if args.mode == 'invalid-odom-frame': state['bad_odom_frame'] = True
    if args.mode in ('retained-amcl', 'retained-stale'): state['update_pose'] = False
    if args.mode == 'following-ack-lag': state['ack_lag'] = .15
    tf = TransformStamped(); tf.header.frame_id = 'map'; tf.child_frame_id = 'base_link'
    tf.header.stamp = node.get_clock().now().to_msg(); tf.transform.rotation.w = 1.
    broadcaster = StaticTransformBroadcaster(node); broadcaster.sendTransform(tf)
    def refresh(request, response):
        capture['refresh_calls'] += 1
        capture['refresh_time'] = time.monotonic() - start
        if args.mode == 'replacement':
            time.sleep(.6)  # A late service reply must not mutate the replacement goal.
            capture['late_refresh_reply_time'] = time.monotonic() - start
        if args.mode not in ('stale-amcl', 'cancel', 'context-change', 'replacement'):
            state['pose_age'] = 0.
            state['update_pose'] = True
        return response
    refresh_service = None if args.mode in ('service-unavailable', 'blocked-service-unavailable') else node.create_service(
        Empty, '/request_nomotion_update', refresh, callback_group=group)
    def execute(kind, action, handle):
        result = action.Result()
        if kind == 'plan':
            capture['plan_calls'] += 1
            count = capture['plan_calls']
            if args.mode == 'replacement-planning' and count == 1:
                time.sleep(.6)
                if handle.is_cancel_requested: handle.canceled(); return result
            blocked = args.mode in ('blocked-persistent', 'stale-and-blocked', 'blocked-service-unavailable') or (
                args.mode == 'refresh-success' and args.tree == 'to-pose' and count == 1)
            if blocked or args.mode == 'costmap-timeout' or (args.mode == 'failed-replan' and count > 1):
                result.error_code = result.START_OCCUPIED if blocked else (
                    result.TIMEOUT if args.mode == 'costmap-timeout' else result.NO_VALID_PATH)
                result.error_msg = f'fixture planning error {result.error_code}'
                handle.abort(); return result
            first = PoseStamped(); first.header.frame_id = 'map'; first.pose.orientation.w = 1.
            target = PoseStamped(); target.header.frame_id = 'map'; target.pose.orientation.w = 1.; target.pose.position.x = 1.
            result.path = NavPath(); result.path.header.frame_id = 'map'
            result.path.header.stamp = node.get_clock().now().to_msg(); result.path.poses = [first, target]
        elif kind == 'follow':
            capture['follow_calls'] += 1
            if args.mode == 'controller-recovery' and capture['follow_calls'] <= 2:
                result.error_code = result.FAILED_TO_MAKE_PROGRESS; handle.abort(); return result
            if args.mode == 'failed-replan' or (args.mode == 'replacement-following' and capture['follow_calls'] == 1):
                until(lambda: handle.is_cancel_requested)
                handle.canceled(); return result
            # Permit a service snapshot before fast synthetic completion.
            time.sleep(.25)
        else:
            capture['recovery_actions'].append(kind)
        handle.succeed(); return result
    servers = [ActionServer(node, action, name, execute_callback=partial(execute, kind, action), callback_group=group, cancel_callback=lambda _: CancelResponse.ACCEPT)
        for action, name, kind in ((ComputePathToPose, 'compute_path_to_pose', 'plan'),
            (ComputePathThroughPoses, 'compute_path_through_poses', 'plan'), (FollowPath, 'follow_path', 'follow'),
            (BackUp, 'backup', 'backup'), (Spin, 'spin', 'spin'), (Wait, 'wait', 'wait'))]
    def clear(request, response): capture['clear_calls'] += 1; return response
    services = [node.create_service(ClearEntireCostmap, name, clear, callback_group=group)
        for name in ('local_costmap/clear_entirely_local_costmap', 'global_costmap/clear_entirely_global_costmap')]
    executor = MultiThreadedExecutor(num_threads=8); executor.add_node(node)
    def spin():
        try: executor.spin()
        except Exception as error: capture['executor_failure'] = repr(error)
    thread = threading.Thread(target=spin, daemon=True); thread.start()
    share = Path(get_package_share_directory('patrolbot_navigation')); trees = share/'config/behavior_trees'
    binary = Path(get_package_prefix('nav2_bt_navigator'))/'lib/nav2_bt_navigator/bt_navigator'
    assert str(binary).startswith('/opt/patrolbot_nav2/install/')
    capture['binary_sha256'] = hashlib.sha256(binary.read_bytes()).hexdigest()
    guard_binary = Path(get_package_prefix('nav2_behavior_tree'))/'lib/libnav2_patrolbot_localization_guard_bt_node.so'
    capture['guard_binary_sha256'] = hashlib.sha256(guard_binary.read_bytes()).hexdigest()
    tree = trees/('navigate_recovery_always_try.xml' if args.tree == 'to-pose' else 'navigate_through_poses_recovery_always_try.xml')
    capture['tree_sha256'] = hashlib.sha256(tree.read_bytes()).hexdigest()
    process = subprocess.Popen([str(binary), '--ros-args', '--params-file', str(share/'config/nav2_params.yaml'),
        '-p', 'default_nav_to_pose_bt_xml:='+str(trees/'navigate_recovery_always_try.xml'),
        '-p', 'default_nav_through_poses_bt_xml:='+str(trees/'navigate_through_poses_recovery_always_try.xml')])
    change = node.create_client(ChangeState, '/bt_navigator/change_state'); assert change.wait_for_service(timeout_sec=15.)
    for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
        request = ChangeState.Request(); request.transition.id = transition; assert wait(change.call_async(request)).success
    action = NavigateToPose if args.tree == 'to-pose' else NavigateThroughPoses
    endpoint = 'navigate_to_pose' if args.tree == 'to-pose' else 'navigate_through_poses'
    client = ActionClient(node, action, endpoint); assert client.wait_for_server(timeout_sec=10.)
    path_client = node.create_client(GetNavigationPath, '/patrolbot/'+endpoint+'_path', callback_group=group)
    assert path_client.wait_for_service(timeout_sec=5.)
    goal = action.Goal(); pose = PoseStamped(); pose.header.frame_id = 'map'; pose.pose.orientation.w = 1.; pose.pose.position.x = 1.
    if args.tree == 'to-pose': goal.pose = pose
    elif hasattr(goal.poses, 'goals'): goal.poses.header.frame_id = 'map'; goal.poses.goals = [pose]
    else: goal.poses = [pose]
    def feedback(msg): capture['feedback'].append(float(msg.feedback.distance_remaining))
    handle = wait(client.send_goal_async(goal, feedback_callback=feedback)); assert handle.accepted
    request_path = GetNavigationPath.Request(); request_path.goal_id = handle.goal_id
    query_started = [False]
    def query():
        if query_started[0]: return
        query_started[0] = True
        future = path_client.call_async(request_path)
        def done(f):
            query_started[0] = False
            response = f.result()
            capture['statuses'].append(dict(stage=response.planning_stage, code=response.planning_error_code,
                message=response.planning_error_message, points=len(response.path.poses), active=response.active,
                time=time.monotonic()-start))
        future.add_done_callback(done)
    query_timer = node.create_timer(.05, query, callback_group=group)
    result_future = handle.get_result_async()
    if args.mode in ('cancel', 'context-change', 'replacement', 'replacement-planning', 'replacement-following'):
        if args.mode == 'replacement-planning': until(lambda: capture['plan_calls'] == 1)
        elif args.mode == 'replacement-following': until(lambda: capture['follow_calls'] == 1)
        else: until(lambda: capture['refresh_calls'] == 1)
        if args.mode == 'context-change': state['generation'] += 1
        elif args.mode == 'cancel': wait(handle.cancel_goal_async())
        else:
            state['pose_age'] = 0.
            until(lambda: node.get_clock().now().nanoseconds-state['pose_stamp_ns'] < 1_000_000_000)
            replacement = wait(client.send_goal_async(goal)); assert replacement.accepted
            replacement_result = wait(replacement.get_result_async()); assert replacement_result.status == 4
            capture['replacement_uuid'] = bytes(replacement.goal_id.uuid).hex()
    result = wait(result_future)
    capture.update(status=result.status, error_code=result.result.error_code, error_message=result.result.error_msg)
    success = args.mode in ('refresh-success', 'controller-recovery', 'retained-amcl', 'retained-stale', 'following-ack-lag')
    assert result.status == (4 if success else 5 if args.mode == 'cancel' else 6), capture
    expected_refresh = 0 if args.mode in ('costmap-timeout', 'controller-recovery', 'invalid-amcl', 'invalid-quaternion', 'invalid-covariance', 'unaccepted-amcl', 'failed-replan', 'stale-odom', 'service-unavailable', 'blocked-service-unavailable', 'replacement-planning', 'replacement-following', 'retained-amcl', 'invalid-odom', 'invalid-odom-frame', 'following-ack-lag') or (
        args.mode == 'blocked-persistent' and args.tree == 'through-poses') else 1
    assert capture['refresh_calls'] == expected_refresh, capture
    if args.mode == 'following-ack-lag':
        assert capture['plan_calls'] == capture['follow_calls'] == 1, capture
        after_path = [s for s in capture['statuses'] if s['stage'] == 'following']
        assert after_path and all(s['points'] == 2 for s in after_path), capture
    if args.mode == 'controller-recovery': assert capture['recovery_actions'] == ['backup'], capture
    else: assert not capture['recovery_actions'], capture
    if args.mode == 'costmap-timeout': assert capture['plan_calls'] == 2 and capture['clear_calls'] == 1, capture
    if args.mode == 'blocked-persistent':
        assert capture['plan_calls'] == (2 if args.tree == 'to-pose' else 1), capture
        assert result.result.error_code == (205 if args.tree == 'to-pose' else 305), capture
    if args.mode == 'stale-and-blocked': assert capture['plan_calls'] == 1, capture
    if args.mode == 'blocked-service-unavailable':
        assert 'fixture planning error' in result.result.error_msg, capture
        assert result.result.error_code == (205 if args.tree == 'to-pose' else 305), capture
    if args.mode == 'failed-replan':
        assert capture['plan_calls'] == 3 and capture['follow_calls'] == 1, capture
        assert any(s['points'] > 0 for s in capture['statuses']), capture
        assert result.result.error_code == (208 if args.tree == 'to-pose' else 308), capture
    if not success and args.mode not in ('replacement', 'replacement-planning', 'replacement-following', 'failed-replan'): assert capture['follow_calls'] == 0, capture
    if expected_refresh:
        refreshing = [s for s in capture['statuses'] if s['stage'] == 'refreshing']
        assert refreshing and all(s['points'] == 0 for s in refreshing), capture
        assert capture['refresh_time'] - refreshing[0]['time'] >= .4, capture
    response = wait(path_client.call_async(request_path))
    assert not response.active and not response.path.poses and response.planning_stage == '', response
    capture['passed'] = True
except Exception as error:
    capture['failure'] = str(error); raise
finally:
    signal.alarm(0)
    if process:
        process.terminate()
        try: process.wait(timeout=5.)
        except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=5.)
        capture['navigator_exit_code'] = process.returncode
    if executor: executor.shutdown(timeout_sec=2.)
    if node: node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
    capture['elapsed_s'] = time.monotonic()-start
    args.output.write_text(json.dumps(capture, indent=2)+'\n'); print(json.dumps(capture), flush=True)
assert capture['navigator_exit_code'] == 0, capture
