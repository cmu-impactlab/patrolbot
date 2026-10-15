"""Real Nav2 floor transactions with synthetic scans and loopback SBC only.

Synthetic poses/scans test protocol ordering; they are not physical alignment
measurements or commissioning values. Never run against a real robot graph.
"""
import json
import copy
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
sys.path.insert(0, '/validation/ros2_ws/src/patrolbot_bridge/test')
from test_fake_sbc_integration import FakeSbc, wait_until
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from action_msgs.msg import GoalStatusArray
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from patrolbot_interfaces.action import LocalizeOnMap
from patrolbot_bridge.bridge_node import PatrolBotBridge
from patrolbot_bridge.map_catalog import MapCatalog, ActiveMapStore
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy


sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'diagnostics/localization'))
from geometry import MapRaster


def synthetic_scene(entry):
    raster=MapRaster(entry['path'],entry['map_id'],entry['revision'])
    chosen=None
    margin=12
    for y in range(margin,raster.height-margin,15):
        for x in range(margin,raster.width-margin,15):
            if all(raster.grid_cell(cx,cy)=='free'
                   for cy in range(y-margin,y+margin)
                   for cx in range(x-margin,x+margin)):
                chosen=x,y;break
        if chosen:break
    assert chosen, 'no synthetic free-space test pose'
    x,y=chosen;resolution=raster.resolution;origin=raster.origin
    lx,ly=(x+.5)*resolution,(y+.5)*resolution
    wx=origin[0]+lx*math.cos(origin[2])-ly*math.sin(origin[2])
    wy=origin[1]+lx*math.sin(origin[2])+ly*math.cos(origin[2])
    ranges=[]
    for i in range(361):
        hit=raster.raycast((wx+.037,wy),math.pi/2-i*math.pi/360,15.)
        ranges.append(str(hit['distance']) if hit['classification']=='occupied' else 'nan')
    print(json.dumps({'event':'fixture_identity','map':raster.identity,'pose':[wx,wy,0.]}),flush=True)
    return wx,wy,','.join(ranges)


launch=bridge=fake=executor=None
log_path=Path('/tmp/floor-navigation.log')
try:
    rclpy.init()
    node=Node('isolated_floor_acceptance')
    fake=FakeSbc(); fake.motors_enabled=True; fake.estop_pressed=False
    bridge=PatrolBotBridge(parameter_overrides=[Parameter('server_ip',value='127.0.0.1'),
        Parameter('server_port',value=fake.port),Parameter('reconnect_delay',value=.1)])
    executor=MultiThreadedExecutor(num_threads=6);executor.add_node(node);executor.add_node(bridge)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    log=log_path.open('w')
    launch=subprocess.Popen(['ros2','launch','patrolbot_navigation','bringup.launch.py'],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    from rosidl_runtime_py.convert import message_to_ordereddict
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from nav2_msgs.msg import ParticleCloud
    from diagnostic_msgs.msg import DiagnosticArray
    from tf2_msgs.msg import TFMessage
    capture_path=Path(os.environ.get('LOCALIZATION_CAPTURE','/tmp/floor-capture.jsonl'))
    capture_file=capture_path.open('w')
    capture_lock=threading.Lock()
    def capture(topic,message):
        record={'event':'sensor','topic':topic,'received_monotonic_ns':time.monotonic_ns(),
                'message':message_to_ordereddict(message)}
        with capture_lock:
            capture_file.write(json.dumps(record,allow_nan=True)+'\n');capture_file.flush()
    captures=[]
    for topic,kind in [('/scan',LaserScan),('/scan_unfiltered_candidate',LaserScan),
                       ('/odom',Odometry),('/amcl_pose',PoseWithCovarianceStamped),
                       ('/initialpose',PoseWithCovarianceStamped),
                       ('/amcl/accepted_initial_pose',PoseWithCovarianceStamped),
                       ('/particle_cloud',ParticleCloud),('/diagnostics',DiagnosticArray),
                       ('/tf',TFMessage),('/tf_static',TFMessage)]:
        # Reliable subscribers cannot receive the bridge's best-effort scans.
        qos=QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic=='/tf_static' else qos_profile_sensor_data
        captures.append(node.create_subscription(kind,topic,lambda m,t=topic:capture(t,m),qos))
    seed_events=[]
    def observed_seed(message):
        seed_events.append((bridge._epoch.stamp_ns(message),
                            bridge._epoch.maps.active['map_id']))
    seed_sub=node.create_subscription(PoseWithCovarianceStamped,'/initialpose',observed_seed,10)
    client=ActionClient(node,LocalizeOnMap,'/patrolbot/localize_on_map')
    assert client.wait_for_server(timeout_sec=15)
    catalog=MapCatalog('/opt/patrolbot_ws/install/patrolbot_navigation/share/patrolbot_navigation/maps')
    def report():
        while not bridge._stop_event.wait(10):
            epoch=bridge._epoch
            print(json.dumps({'event':'diagnostic','stage':epoch.status_stage,
                'phase':epoch._phase,'map_step':epoch._map_step,
                'manager_ready':epoch._localization_active.service_is_ready(),
                'future_done':epoch._future.done() if epoch._future else None,
                'map_error':epoch._map_error}),flush=True)
    threading.Thread(target=report,daemon=True).start()
    assert wait_until(lambda: bridge._epoch.maps.active is not None and
        bridge._epoch._navigation_idle_at is not None and bridge._epoch._future is None,timeout=90), (
        bridge._epoch.status_stage,bridge._epoch._phase)
    print(json.dumps({'event':'startup','map':bridge._epoch.maps.active['map_id'],
                      'motion_allowed':bridge._epoch.commands_allowed}),flush=True)
    assert not bridge._epoch.commands_allowed, 'startup must require a new operator seed'
    expected = os.environ.get('EXPECTED_STARTUP_MAP', 'cmuq-floor2')
    assert bridge._epoch.maps.active['map_id'] == expected
    if '--startup-only' in sys.argv:
        print(json.dumps({'passed':['restart_map_readback','fresh_localization_required'], 'map':expected}),flush=True)
        sequence = ()
    elif any(flag in sys.argv for flag in ('--cancel', '--quality-reject', '--measured-reject')):
        sequence = ('cmuq-floor2',)
    else:
        sequence = ('cmuq-floor2','cmuq-floor1','cmuq-floor2','cmuq-floor1')
    if '--quality-reject' in sys.argv:
        # Inject bad final evidence only after actual AMCL seed acceptance and
        # Nav2 activation. This tests coordinator failure handling, not AMCL
        # accuracy. Network isolation and FakeSbc assertions above still apply.
        original_fresh = bridge._epoch._map_localization_fresh
        def force_final_refresh():
            if bridge._epoch._map_step == 'costmap-map':
                return False
            return original_fresh()
        bridge._epoch._map_localization_fresh = force_final_refresh
        original_finish = bridge._epoch._map_finish_refresh
        def reject_final_covariance():
            epoch = bridge._epoch
            pose = epoch._pending_pose
            if (pose is not None and epoch._pending_pose_received_at is not None and
                    epoch._pending_pose_received_at >= epoch._map_refresh_started and
                    epoch.stamp_ns(pose) > epoch._map_refresh_baseline_stamp):
                pose.pose.covariance[7] = .25718849416411727
            return original_finish()
        bridge._epoch._map_finish_refresh = reject_final_covariance
    if '--measured-reject' in sys.argv:
        # Fault at the coordinator evidence boundary after real seed/ACK/scan;
        # this tests admission, not AMCL's ability to select a wall hypothesis.
        original_fresh = bridge._epoch._map_localization_fresh
        injected = []
        motion_actions = []
        def reject_displaced_measured_estimate():
            epoch = bridge._epoch
            estimate = epoch._pending_pose
            if (not injected and epoch._map_step == 'localization' and estimate is not None
                    and epoch._accepted_seed_stamp == epoch.maps.seed_stamp
                    and epoch.stamp_ns(estimate) > epoch.maps.seed_stamp):
                estimate = copy.deepcopy(estimate)
                estimate.pose.pose.position.y += 1.04
                epoch._pending_pose = estimate
                injected.append(epoch.stamp_ns(estimate))
            return original_fresh()
        bridge._epoch._map_localization_fresh = reject_displaced_measured_estimate
        def observe_motion(name, message):
            motion_actions.extend(name for status in message.status_list if status.status in (1, 2, 3))
        motion_subs = [node.create_subscription(GoalStatusArray, '/' + name + '/_action/status',
            lambda message, name=name: observe_motion(name, message), 10)
            for name in ('follow_path', 'backup', 'spin', 'wait')]
    for map_id in sequence:
        entry=catalog.maps[map_id];x,y,scan=synthetic_scene(entry)
        fake.laser_field=scan+"|LASER_AGE_MS:0"
        assert wait_until(lambda: bridge._epoch._navigation_idle_at is not None and
            time.monotonic()-bridge._epoch._navigation_idle_at<.4,timeout=10)
        goal=LocalizeOnMap.Goal();goal.request_id=str(uuid4());goal.map_id=map_id
        goal.map_revision=entry['revision'];goal.operator_authorized=True
        capture_file.write(json.dumps({'event':'transaction','request_id':goal.request_id,
            'map_id':map_id,'revision':entry['revision'],
            'fixture':MapRaster(entry['path'],map_id,entry['revision']).identity,
            'recovery_epoch':str(getattr(bridge._epoch,'epoch','unavailable'))})+'\n')
        goal.pose.pose.position.x=x;goal.pose.pose.position.y=y;goal.pose.pose.orientation.w=1.
        for index in (0,7,35): goal.pose.covariance[index]=.01
        if '--measured-reject' in sys.argv:
            goal.measured_pose = True
            goal.pose.covariance[0] = goal.pose.covariance[7] = .0009
            goal.pose.covariance[35] = .00030461742
        seed_count=len(seed_events)
        seen=[]
        def feedback(message):
            stage=message.feedback.stage
            if not seen or seen[-1]!=stage:
                seen.append(stage);print(json.dumps({'map':map_id,'stage':stage}),flush=True)
        future=client.send_goal_async(goal,feedback_callback=feedback)
        assert wait_until(future.done,timeout=10)
        handle=future.result();assert handle.accepted, (bridge._epoch.status_stage,bridge._epoch._phase)
        result=handle.get_result_async()
        assert wait_until(lambda: bridge._epoch.maps.hold,timeout=5)
        before=len(fake.received)
        velocity=Twist();velocity.linear.x=.1
        # Exercise the existing shared callback, connected only to loopback fake.
        bridge.cmd_vel_callback(velocity)
        time.sleep(.1)
        assert not any(line.startswith('DRIVE:0.1:') for _, line in fake.received[before:]), fake.received[before:]
        if '--cancel' in sys.argv:
            canceled = handle.cancel_goal_async()
            assert wait_until(canceled.done, timeout=10)
            assert wait_until(result.done, timeout=10)
            assert not result.result().result.localized and not bridge._epoch.commands_allowed
            before = len(fake.received)
            bridge.cmd_vel_callback(velocity)
            time.sleep(.2)
            assert not any(line.startswith('DRIVE:0.1:') for _,line in fake.received[before:])
            print(json.dumps({'passed':['cancel_holds_motion','failed_velocity_blocked'],
                'last_verified_map':bridge._epoch.maps.active['map_id']}),flush=True)
            break
        assert wait_until(result.done,timeout=115), (bridge._epoch._map_step,bridge._epoch.maps.error)
        response=result.result().result
        capture_file.write(json.dumps({'event':'result','request_id':goal.request_id,
            'response':message_to_ordereddict(response),'seed_stamp':bridge._epoch.maps.seed_stamp,
            'accepted_seed_stamp':bridge._epoch._accepted_seed_stamp})+'\n');capture_file.flush()
        if '--measured-reject' in sys.argv:
            assert response.code == response.SEED_INNOVATION_REJECTED and not response.localized, response
            assert injected and bridge._epoch.maps.hold and not bridge._epoch.commands_allowed
            assert seed_events[seed_count:] == [(bridge._epoch.maps.seed_stamp, map_id)]
            state = node.create_client(GetState, '/bt_navigator/get_state')
            assert state.wait_for_service(timeout_sec=5)
            future = state.call_async(GetState.Request())
            assert wait_until(future.done, timeout=5)
            assert future.result().current_state.id == 2, 'rejected measured seed activated navigation'
            destination = PoseStamped(); destination.header.frame_id = 'map'
            destination.pose.position.x = x + .2; destination.pose.position.y = y
            destination.pose.orientation.w = 1.
            for action, name in ((NavigateToPose, 'navigate_to_pose'),
                                 (NavigateThroughPoses, 'navigate_through_poses')):
                navigator = ActionClient(node, action, name)
                assert navigator.wait_for_server(timeout_sec=5)
                request = action.Goal()
                if name == 'navigate_to_pose': request.pose = destination
                elif hasattr(request.poses, 'goals'): request.poses.goals = [destination]
                else: request.poses = [destination]
                future = navigator.send_goal_async(request)
                assert wait_until(future.done, timeout=5)
                assert not future.result().accepted, name + ' accepted rejected localization'
                navigator.destroy()
            before = len(fake.received)
            bridge.cmd_vel_callback(velocity)
            time.sleep(.2)
            assert not any(line.startswith('DRIVE:0.1:') for _, line in fake.received[before:])
            assert not motion_actions, motion_actions
            print(json.dumps({'passed': ['measured_seed_rejected', 'single_seed',
                'both_navigators_refused', 'no_follow_or_motion_recovery', 'failed_velocity_blocked'],
                'detail': response.message, 'physical_acceptance': False}), flush=True)
            break
        if '--quality-reject' in sys.argv:
            assert response.code == response.FAILED and not response.localized
            assert response.message.startswith('LOCALIZATION_QUALITY_REJECTED:')
            assert response.map_activated and response.active_map_id == map_id
            assert not bridge._epoch.commands_allowed
            assert seed_events[seed_count:] == [(bridge._epoch.maps.seed_stamp, map_id)]
            before = len(fake.received)
            bridge.cmd_vel_callback(velocity)
            time.sleep(.2)
            assert not any(line.startswith('DRIVE:0.1:') for _, line in fake.received[before:])
            print(json.dumps({'passed': ['explicit_quality_failure', 'single_seed',
                'failed_velocity_blocked'], 'detail': response.message}), flush=True)
            break
        assert response.code==0 and response.localized, (response,bridge._epoch._map_step)
        assert response.active_map_id==map_id and bridge._epoch.commands_allowed
        assert wait_until(lambda: len(seed_events)>seed_count,timeout=3)
        assert seed_events[seed_count:]==[(bridge._epoch.maps.seed_stamp,map_id)]
        assert bridge._epoch._accepted_seed_stamp==bridge._epoch.maps.seed_stamp
        assert ActiveMapStore(os.environ.get('PATROLBOT_STATE_DIR','/var/lib/patrolbot'),catalog).load()['map_id']==map_id
        print(json.dumps({'event':'verified','map':map_id,'stages':seen}),flush=True)
    if not any(flag in sys.argv for flag in ('--startup-only', '--cancel', '--quality-reject', '--measured-reject')):
        print(json.dumps({'passed':['floor2-floor1-floor2','real_amcl_consumption','single_seed',
        'real_costmap_refresh','enabled_motor_hold','durable_selection']}),flush=True)
finally:
    if executor: executor.shutdown(timeout_sec=5)
    if bridge: bridge.destroy_node()
    if fake: fake.close()
    if rclpy.ok():rclpy.try_shutdown()
    if launch and launch.poll() is None:
        os.kill(launch.pid,signal.SIGINT)
        try:launch.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid,signal.SIGKILL);launch.wait()
    if log_path.exists():print(log_path.read_text(),flush=True)
