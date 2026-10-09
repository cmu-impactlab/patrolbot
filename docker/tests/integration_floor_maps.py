"""Real Nav2 floor transactions with synthetic scans and loopback SBC only.

Synthetic poses/scans test protocol ordering; they are not physical alignment
measurements or commissioning values. Never run against a real robot graph.
"""
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
sys.path.insert(0, '/validation/ros2_ws/src/patrolbot_bridge/test')
from test_fake_sbc_integration import FakeSbc, wait_until
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from patrolbot_interfaces.action import LocalizeOnMap
from patrolbot_bridge.bridge_node import PatrolBotBridge
from patrolbot_bridge.map_catalog import MapCatalog, ActiveMapStore
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


def synthetic_scene(entry):
    # Find a white free-space square, and ray-cast the actual canonical raster.
    import numpy as np
    yaml = Path(entry['path'])
    image = next(line.split(':', 1)[1].strip() for line in yaml.read_text().splitlines()
                 if line.startswith('image:'))
    data = (yaml.parent / image).read_bytes()
    pixels = np.frombuffer(data[-entry['width']*entry['height']:], dtype=np.uint8).reshape(entry['height'],entry['width'])[::-1]
    free = pixels > 250
    chosen = None
    margin = 12
    for y in range(margin,entry['height']-margin,15):
        for x in range(margin,entry['width']-margin,15):
            if free[y-margin:y+margin,x-margin:x+margin].all():
                chosen = x,y; break
        if chosen: break
    assert chosen, 'no synthetic free-space test pose'
    x,y = chosen; resolution=entry['resolution']
    ranges=[]
    for i in range(361):
        # Match the existing launch laser roll and x offset; test data only.
        angle=math.pi/2-i*math.pi/360
        distance=resolution
        while distance < 15.:
            cx=int(x+.5+.037/resolution+math.cos(angle)*distance/resolution)
            cy=int(y+.5+math.sin(angle)*distance/resolution)
            if not (0<=cx<entry['width'] and 0<=cy<entry['height']) or not free[cy,cx]: break
            distance+=resolution/2
        ranges.append(str(distance))
    return (entry['origin'][0]+(x+.5)*resolution,
            entry['origin'][1]+(y+.5)*resolution, ','.join(ranges))


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
    elif '--cancel' in sys.argv:
        sequence = ('cmuq-floor2',)
    else:
        sequence = ('cmuq-floor2','cmuq-floor1','cmuq-floor2','cmuq-floor1')
    for map_id in sequence:
        entry=catalog.maps[map_id];x,y,scan=synthetic_scene(entry)
        fake.laser_field=scan+"|LASER_AGE_MS:0"
        assert wait_until(lambda: bridge._epoch._navigation_idle_at is not None and
            time.monotonic()-bridge._epoch._navigation_idle_at<.4,timeout=10)
        goal=LocalizeOnMap.Goal();goal.request_id=str(uuid4());goal.map_id=map_id
        goal.map_revision=entry['revision'];goal.operator_authorized=True
        goal.pose.pose.position.x=x;goal.pose.pose.position.y=y;goal.pose.pose.orientation.w=1.
        for index in (0,7,35): goal.pose.covariance[index]=.01
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
        assert response.code==0 and response.localized, (response,bridge._epoch._map_step)
        assert response.active_map_id==map_id and bridge._epoch.commands_allowed
        assert wait_until(lambda: len(seed_events)>seed_count,timeout=3)
        assert seed_events[seed_count:]==[(bridge._epoch.maps.seed_stamp,map_id)]
        assert bridge._epoch._accepted_seed_stamp==bridge._epoch.maps.seed_stamp
        assert ActiveMapStore(os.environ.get('PATROLBOT_STATE_DIR','/var/lib/patrolbot'),catalog).load()['map_id']==map_id
        print(json.dumps({'event':'verified','map':map_id,'stages':seen}),flush=True)
    if '--startup-only' not in sys.argv and '--cancel' not in sys.argv:
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
