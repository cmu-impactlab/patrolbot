"""Real Nav2 with a raycast feedback plant. Requires network-none isolation.

Run each scenario in a fresh container; synthetic results are software evidence.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from uuid import uuid4

assert os.environ.get('PATROLBOT_ISOLATED_TEST') == '1'
assert {p.name for p in Path('/sys/class/net').iterdir()} == {'lo'}
sys.path.insert(0, '/validation/docker/tests')
sys.path.insert(0, '/validation/ros2_ws/src/patrolbot_bridge/test')
from closed_loop_fixture import ClosedLoopFakeSbc
from navigation_capture import validate_outcome, stall_recovery_evidence, outer_recovery_stages
from test_fake_sbc_integration import wait_until as fixture_wait_until
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from rosidl_runtime_py.convert import message_to_ordereddict
from nav2_msgs.action import ComputePathToPose, NavigateToPose, NavigateThroughPoses
from nav2_msgs.srv import ManageLifecycleNodes
from patrolbot_interfaces.action import LocalizeOnMap
from patrolbot_bridge.bridge_node import PatrolBotBridge
from patrolbot_bridge.map_catalog import MapCatalog
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

parser = argparse.ArgumentParser()
parser.add_argument('--scenario', choices=('reach', 'cancel', 'progress-failure', 'bounded-stall-cancel', 'blocked-goal', 'stale-scan', 'lidar-off', 'origin-change', 'false-amcl-jump'), default='reach')
parser.add_argument('--tree', choices=('to-pose', 'through-poses'), default='to-pose')
parser.add_argument('--map', default='cmuq-floor1')
parser.add_argument('--output', type=Path, default=Path('/tmp/navigation-capture.json'))
args = parser.parse_args()
launch = base_launch = bridge = fake = executor = node = log = None
shutdown = {}
log_path = args.output.with_suffix('.log')
capture = {'scenario': args.scenario, 'tree': args.tree, 'physical_acceptance': False}
deadline = time.monotonic() + 120
localization_pairs = []
navigation_started_ns = None


def wait_until(predicate, timeout=10):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise AssertionError('120-second case budget exhausted; recovery incomplete')
    return fixture_wait_until(predicate, timeout=min(timeout, remaining))


def wait(future, seconds=10):
    assert wait_until(future.done, timeout=seconds), 'ROS future timed out'
    return future.result()


def pose(x, y, yaw=0.):
    message = PoseStamped()
    message.header.frame_id = 'map'
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.orientation.z = math.sin(yaw / 2)
    message.pose.orientation.w = math.cos(yaw / 2)
    return message


def send(client, goal):
    assert client.wait_for_server(timeout_sec=15), 'action server unavailable'
    handle = wait(client.send_goal_async(goal))
    assert handle.accepted, 'action rejected'
    return handle


def stopped():
    samples = fake.snapshot()['samples']
    return len(samples) >= 5 and all(
        abs(s['velocity'][0]) < .001 and abs(s['velocity'][1]) < .001
        for s in samples[-5:])


def readiness():
    epoch = bridge._epoch
    return {'stage': epoch.status_stage, 'phase': epoch._phase,
            'map_step': epoch._map_step, 'map_error': epoch._map_error,
            'idle_service_ready': epoch._navigation_idle.service_is_ready(),
            'idle_at': epoch._navigation_idle_at,
            'commands_allowed': epoch.commands_allowed}


try:
    rclpy.init()
    node = Node('isolated_navigation_closed_loop')
    catalog = MapCatalog('/opt/patrolbot_ws/install/patrolbot_navigation/share/patrolbot_navigation/maps')
    entry = catalog.maps[args.map]
    # Find a 2.4 m square of canonical known-free raster, providing clearance
    # for the footprint, inflation, rotations and the .6 m test goal.
    from geometry import MapRaster
    raster = MapRaster(entry['path'], args.map, entry['revision'])
    margin = math.ceil(1.2 / raster.resolution)
    start = None
    for gy in range(margin, raster.height - margin, 5):
        for gx in range(margin, raster.width - margin, 5):
            if all(raster.grid_cell(cx, cy) == 'free'
                   for cy in range(gy-margin, gy+margin+1)
                   for cx in range(gx-margin, gx+margin+1)):
                yaw = raster.origin[2]
                lx, ly = (gx+.5)*raster.resolution, (gy+.5)*raster.resolution
                start = (raster.origin[0]+math.cos(yaw)*lx-math.sin(yaw)*ly,
                         raster.origin[1]+math.sin(yaw)*lx+math.cos(yaw)*ly, yaw)
                break
        if start:
            break
    assert start, 'canonical map lacks required known-free scenario clearance'
    fake = ClosedLoopFakeSbc(entry['path'], args.map, entry['revision'], start, beams=181)
    capture['laser_beams'] = fake.beams
    bridge = PatrolBotBridge(parameter_overrides=[
        Parameter('server_ip', value='127.0.0.1'),
        Parameter('server_port', value=fake.port), Parameter('reconnect_delay', value=.1)])
    shutdown = {name: node.create_client(ManageLifecycleNodes, f'/lifecycle_manager_{name}/manage_nodes')
                for name in ('navigation', 'localization')}
    state = node.create_client(GetState, '/bt_navigator/get_state')
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    executor.add_node(bridge)
    threading.Thread(target=executor.spin, daemon=True).start()
    def observe_amcl(message):
        stamp_ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        if navigation_started_ns is None or stamp_ns < navigation_started_ns:
            return
        now_ns = node.get_clock().now().nanoseconds
        captured_at = time.monotonic() - (now_ns-stamp_ns)/1e9
        plant = fake.snapshot()
        samples = plant['samples']
        if not samples:
            return
        nearest = min(samples, key=lambda sample: abs(sample['time']-captured_at))
        p = message.pose.pose
        yaw = math.atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),
                         1-2*(p.orientation.y**2+p.orientation.z**2))
        truth = nearest['world_pose']
        scan = min(plant['scan_samples'], key=lambda sample: abs(sample['acquired_at']-captured_at))
        localization_pairs.append({'stamp_ns': stamp_ns, 'sample_time': nearest['time'],
            'time_error_s': abs(nearest['time']-captured_at),
            'amcl': [p.position.x,p.position.y,yaw], 'truth': truth,
            'position_error_m': math.hypot(p.position.x-truth[0],p.position.y-truth[1]),
            'yaw_error_rad': abs(math.remainder(yaw-truth[2],2*math.pi)),
            'scan_truth': scan['world_pose'], 'scan_time_error_s': abs(scan['acquired_at']-captured_at),
            'scan_yaw_error_rad': abs(math.remainder(yaw-scan['world_pose'][2],2*math.pi))})
    amcl_sub = node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                        observe_amcl, qos_profile_sensor_data)
    capture['localization_pairs'] = localization_pairs
    capture['timed_sensors'] = []
    def observe_sensor(topic, message):
        if navigation_started_ns is not None:
            capture['timed_sensors'].append({'topic': topic, 'received_monotonic': time.monotonic(),
                'message': message_to_ordereddict(message)})
    sensor_subscriptions = [node.create_subscription(kind, topic,
        lambda message, topic=topic: observe_sensor(topic, message), qos_profile_sensor_data)
        for topic, kind in [('/scan', LaserScan), ('/odom', Odometry), ('/tf', TFMessage)]]
    log = log_path.open('w')
    # Production arbiter and its managed smoother complete input/navi -> /cmd_vel.
    base_launch = subprocess.Popen(
        ['ros2', 'launch', 'patrolbot-launch', 'bringup.xml'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    velocity_stages = {}
    stage_subscriptions = []
    for topic in ('/input/nav_controller', '/input/nav_behavior', '/cmd_vel_nav',
                  '/cmd_vel_smoothed', '/input/navi', '/input/safety_controller',
                  '/cmd_vel_out', '/cmd_vel'):
        velocity_stages[topic] = []
        def observe(message, topic=topic):
            velocity_stages[topic].append([time.monotonic(), message.linear.x, message.angular.z])
        stage_subscriptions.append(node.create_subscription(Twist, topic, observe, 10))
    capture['velocity_stages'] = velocity_stages
    launch = subprocess.Popen(['ros2', 'launch', 'patrolbot_navigation', 'bringup.launch.py'],
                              stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    def report():
        while not bridge._stop_event.wait(10):
            print(json.dumps({'event': 'startup_or_runtime', **readiness()}), flush=True)
    threading.Thread(target=report, daemon=True).start()
    assert wait_until(lambda: bridge._epoch._navigation_idle_at is not None and
                      bridge._epoch._future is None, timeout=90), readiness()
    assert state.wait_for_service(timeout_sec=5)
    capture['startup_bt_state'] = wait(state.call_async(GetState.Request())).current_state.id
    assert capture['startup_bt_state'] == 2, 'Nav2 must configure inactive before coordinator activation'
    assert not bridge._epoch.commands_allowed
    localize = ActionClient(node, LocalizeOnMap, '/patrolbot/localize_on_map')
    goal = LocalizeOnMap.Goal()
    goal.request_id = str(uuid4())
    goal.map_id = args.map
    goal.map_revision = entry['revision']
    goal.operator_authorized = True
    goal.measured_pose = True
    goal.pose.pose = pose(*start).pose
    goal.pose.covariance[0] = goal.pose.covariance[7] = .0009
    goal.pose.covariance[35] = .00030461742
    stages = []
    assert localize.wait_for_server(timeout_sec=15)
    handle = wait(localize.send_goal_async(goal, feedback_callback=lambda msg: stages.append(msg.feedback.stage)))
    assert handle.accepted, readiness()
    response = wait(handle.get_result_async(), 120).result
    assert response.localized and bridge._epoch.commands_allowed, (response, readiness())
    capture['localization_stages'] = stages
    target = (start[0] + .6*math.cos(start[2]), start[1] + .6*math.sin(start[2]), start[2])
    planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    request = ComputePathToPose.Goal()
    request.goal = pose(*target)
    request.planner_id = 'GridBased'
    planned = wait(send(planner, request).get_result_async(), 30)
    assert planned.status == GoalStatus.STATUS_SUCCEEDED and len(planned.result.path.poses) > 2
    capture['planned_path_points'] = len(planned.result.path.poses)
    if args.scenario == 'blocked-goal':
        # Exercise real planner/behavior errors directly; dashboard bounds admission is separate.
        x, y = (raster.width + 1) * raster.resolution, (raster.height + 1) * raster.resolution
        yaw = raster.origin[2]
        target = (raster.origin[0] + math.cos(yaw)*x - math.sin(yaw)*y,
                  raster.origin[1] + math.sin(yaw)*x + math.cos(yaw)*y, yaw)
        assert raster.cell(*target[:2]) == 'out_of_map'
        capture['acceptance_mode'] = 'actual-nav2-outside-map-full-recovery'
        capture['natural_frozen_exhaustion_passed'] = False
        capture['actual_recovery_stages'] = []
        recovery_seen = set()
        recovery_subscriptions = []
        for behavior in ('backup', 'spin', 'wait'):
            def observe_recovery(message, behavior=behavior):
                for status in message.status_list:
                    key = (behavior, bytes(status.goal_info.goal_id.uuid))
                    if status.status == GoalStatus.STATUS_SUCCEEDED and key not in recovery_seen:
                        recovery_seen.add(key)
                        capture['actual_recovery_stages'].append(behavior)
            recovery_subscriptions.append(node.create_subscription(GoalStatusArray,
                '/'+behavior+'/_action/status', observe_recovery, 10))
    if args.scenario in ('progress-failure', 'bounded-stall-cancel'):
        fake.motion_frozen = True
        capture['frozen_start_pose'] = fake.snapshot()['world_pose']
    if args.tree == 'through-poses':
        navigator = ActionClient(node, NavigateThroughPoses, '/navigate_through_poses')
        request = NavigateThroughPoses.Goal()
        # Jazzy uses nav_msgs/Goals; older Nav2 uses PoseStamped[].
        if hasattr(request.poses, 'goals'):
            request.poses.header.frame_id = 'map'
            request.poses.goals = [pose(*target)]
        else:
            request.poses = [pose(*target)]
    else:
        navigator = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        request = NavigateToPose.Goal()
        request.pose = pose(*target)
    navigation_started_ns = node.get_clock().now().nanoseconds
    navigation_log_offset = log_path.stat().st_size
    handle = send(navigator, request)
    result = handle.get_result_async()
    if args.scenario == 'bounded-stall-cancel':
        def observed_stall_retry():
            with log_path.open() as stream:
                stream.seek(navigation_log_offset)
                capture['stall_recovery_log'] = stall_recovery_evidence(stream.read())
            return bool(capture['stall_recovery_log']) or result.done()
        assert wait_until(observed_stall_retry, timeout=45), 'no logged progress failure and contextual retry'
        capture['no_success_before_cancel'] = not result.done() or result.result().status != GoalStatus.STATUS_SUCCEEDED
        assert capture['no_success_before_cancel'], 'frozen navigation succeeded'
        assert capture['stall_recovery_log'], 'navigation terminated before progress failure and retry'
        if not result.done():
            canceled = wait(handle.cancel_goal_async(), 5)
            assert canceled.goals_canceling or result.done(), 'stall cancellation rejected'
        wait(result, 5)
    if args.scenario in ('cancel', 'stale-scan', 'lidar-off', 'origin-change', 'false-amcl-jump'):
        assert wait_until(lambda: fake.snapshot()['distance_m'] > .05, timeout=30), 'controller never moved plant'
        if args.scenario != 'cancel':
            if args.scenario == 'stale-scan':
                fake.scan_delay = 2.
            elif args.scenario == 'lidar-off':
                fake.lidar_state = 'OFF'
                fake.lidar_scan_age_ms = -1
            elif args.scenario == 'false-amcl-jump':
                # Isolated fault injection at the existing subscriber boundary;
                # do not publish a second AMCL authority on the ROS graph.
                jump = PoseWithCovarianceStamped()
                jump.header.frame_id = 'map'
                jump.header.stamp = rclpy.time.Time(
                    nanoseconds=bridge._epoch._odom_history.samples[-1][0]).to_msg()
                jump.pose.pose = pose(start[0]+5., start[1]+5., start[2]).pose
                jump.pose.covariance[0] = jump.pose.covariance[7] = .0009
                jump.pose.covariance[35] = .00030461742
                injected_at = time.monotonic()
                bridge._epoch.pose(jump)
                withdrawn = wait_until(lambda: not bridge._epoch.commands_allowed, timeout=.6)
                capture['readiness_withdrawn'] = withdrawn
                capture['hold_latency_s'] = time.monotonic()-injected_at
                assert withdrawn, 'false AMCL jump retained navigation readiness'
                assert 'position_error_m=' in bridge._epoch._localization_fault, bridge._epoch._localization_fault
            else:
                previous_epoch = bridge._epoch.guard.epoch
                fake.odom_epoch = str(uuid4())
                fake.drop_connection()
                assert wait_until(lambda: bridge._epoch.guard.epoch != previous_epoch, timeout=10)
            assert wait_until(lambda: not bridge._lidar_ready() or not bridge._epoch.commands_allowed, timeout=10)
            assert wait_until(stopped, timeout=5)
            before = len(fake.received)
            command = Twist()
            command.linear.x = .1
            bridge.cmd_vel_callback(command)
            time.sleep(.2)
            capture['blocked_nonzero'] = not any(
                line.startswith('DRIVE:') and any(abs(float(v)) > .001 for v in line.split(':')[1:])
                for _, line in fake.received[before:])
        if not result.done():
            cancel_response = wait(handle.cancel_goal_async())
            assert cancel_response.goals_canceling or result.done(), 'goal cancellation rejected'
    completed = wait(result, 120)
    capture['status'] = completed.status
    capture['navigate_error_code'] = completed.result.error_code
    capture['navigate_error_message'] = getattr(completed.result, 'error_msg', '')
    terminal_snapshot = fake.snapshot()['commands']
    terminal_index = len(terminal_snapshot)
    if args.scenario == 'blocked-goal':
        # Wait stops before its result; idle output can retain zero without publishing again.
        capture['terminal_latched_drive'] = next((line for _, line in reversed(terminal_snapshot)
            if line.startswith('DRIVE:')), None)
    assert deadline - time.monotonic() >= .6, 'case budget cannot cover terminal output observation'
    terminal_observation_start = time.monotonic()
    time.sleep(.6)
    capture['terminal_observation_s'] = time.monotonic() - terminal_observation_start
    capture['terminal_commands'] = fake.snapshot()['commands'][terminal_index:]
    assert wait_until(stopped, timeout=5), 'navigation left feedback plant moving'
    snapshot = fake.snapshot()
    capture.update(snapshot)
    capture['stopped'] = stopped()
    capture['drive_nonzero'] = any(line.startswith('DRIVE:') and
        any(abs(float(v)) > .001 for v in line.split(':')[1:]) for _, line in snapshot['commands'])
    capture['position_error_m'] = math.hypot(snapshot['world_pose'][0]-target[0], snapshot['world_pose'][1]-target[1])
    capture['yaw_error_rad'] = abs(math.remainder(snapshot['world_pose'][2]-target[2], 2*math.pi))
    if args.scenario == 'reach':
        assert len(localization_pairs) >= 2, 'insufficient moving AMCL truth observations'
        assert all(pair['time_error_s'] <= .10 for pair in localization_pairs), 'AMCL truth timing unmatched'
        assert all(pair['position_error_m'] <= .10 and pair['yaw_error_rad'] <= math.radians(3)
                   for pair in localization_pairs), 'moving AMCL alignment exceeds .10m/3deg'
    capture['case_elapsed_s'] = 120 - (deadline - time.monotonic())
    if args.scenario == 'bounded-stall-cancel':
        capture['acceptance_mode'] = 'bounded-stall-cancel'
        capture['autonomous_recovery_exhaustion_passed'] = False
    if args.scenario == 'blocked-goal':
        capture['terminal_feedback_fresh'] = bool(snapshot['samples'] and
            snapshot['samples'][-1]['time'] >= terminal_observation_start)
        capture['actual_backup_motion'] = any(sample['velocity'][0] < -.001 for sample in snapshot['samples'])
        capture['actual_spin_motion'] = any(abs(sample['velocity'][1]) > .001 for sample in snapshot['samples'])
    else:
        validate_outcome(capture)
        capture['passed'] = True
except Exception as error:
    capture['passed'] = False
    capture['failure'] = str(error)
    capture['case_elapsed_s'] = 120 - (deadline - time.monotonic())
    raise
finally:
    original_error = sys.exc_info()[0] is not None
    cleanup_errors = []

    def cleanup(label, operation):
        try:
            return operation()
        except Exception as error:
            cleanup_errors.append(f'{label}: {type(error).__name__}: {error}')

    if bridge:
        value = cleanup('readiness snapshot', readiness)
        if value is not None:
            capture['final_readiness'] = value
    if fake:
        value = cleanup('plant snapshot', fake.snapshot)
        if value is not None:
            capture['plant'] = value
    if node and executor:
        def stop_manager(client):
            if client.wait_for_service(timeout_sec=3):
                request = ManageLifecycleNodes.Request()
                request.command = request.SHUTDOWN
                future = client.call_async(request)
                assert fixture_wait_until(future.done, timeout=15), 'manager shutdown timed out'
                assert future.result().success, 'manager shutdown rejected'
        for name, client in shutdown.items():
            cleanup(f'{name} manager shutdown', lambda client=client: stop_manager(client))
        cleanup('executor shutdown', lambda: executor.shutdown(timeout_sec=5))
    if bridge:
        cleanup('bridge shutdown', bridge.destroy_node)
    if fake:
        cleanup('fake shutdown', fake.close)
    if node:
        cleanup('node shutdown', node.destroy_node)
    if rclpy.ok():
        cleanup('ROS shutdown', rclpy.try_shutdown)
    for process in (launch, base_launch):
        if process and process.poll() is None:
            def stop_launch(process=process):
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                    raise AssertionError('ROS launch teardown required forced termination')
            cleanup('launch shutdown', stop_launch)
    if log:
        cleanup('log close', log.close)
    if capture.get('scenario') == 'blocked-goal' and log_path.exists():
        teardown_log = log_path.read_text()
        exits = [line for line in teardown_log.splitlines() if 'process has died' in line]
        capture['teardown_errors'] = exits
        capture['complete_teardown_passed'] = not exits
        for line in exits:
            # Report the existing base double-shutdown separately; all other child failures fail.
            if not ('[lifecycle_mgr.py-' in line and 'exit code 1,' in line and
                    'rcl_shutdown already called on the given context' in teardown_log):
                cleanup_errors.append('unexpected launched process failure: ' + line)
        capture['total_elapsed_s'] = 120 - (deadline - time.monotonic())
        if capture['total_elapsed_s'] > 120:
            cleanup_errors.append('120-second case budget exceeded including teardown')
        if not original_error:
            # ROS launch can flush the final recovery event only when its producers exit.
            with log_path.open() as stream:
                stream.seek(navigation_log_offset)
                recovery_log = stream.read()
            capture['planner_failure_observed'] = ('outside' in recovery_log.lower() and
                '[compute_path' in recovery_log and 'Aborting handle' in recovery_log)
            capture['outer_recovery_stages'] = outer_recovery_stages(recovery_log)
            capture['full_outer_clears'] = capture['outer_recovery_stages'].count('clear')
            capture['contextual_global_clears'] = recovery_log.count(
                'Received request to clear entirely the global_costmap') - capture['full_outer_clears']
            cleanup('recovery evidence', lambda: validate_outcome(capture))
            capture['passed'] = not cleanup_errors
    if cleanup_errors:
        capture['cleanup_errors'] = cleanup_errors
        capture['passed'] = False
    try:
        args.output.write_text(json.dumps(capture, indent=2) + '\n')
    except OSError as error:
        cleanup_errors.append(f'evidence write: {error}')
        print(json.dumps(capture), flush=True)
    if cleanup_errors and not original_error:
        raise AssertionError('; '.join(cleanup_errors))
