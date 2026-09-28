"""Bounded wall-clock scan outage soak of the real composed stack, isolated only."""
import json
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from collections import deque

assert os.environ.get('PATROLBOT_ISOLATED_TEST') == '1'
assert {p.name for p in Path('/sys/class/net').iterdir()} == {'lo'}, 'requires network-none isolation'
assert not any(Path('/dev').glob('ttyUSB*')) and not any(Path('/dev').glob('ttyACM*')), 'robot serial devices forbidden'
sys.path.insert(0, '/validation/ros2_ws/src/patrolbot_bridge/test')
from test_fake_sbc_integration import FakeSbc, wait_until
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.srv import ManageLifecycleNodes
from patrolbot_bridge.bridge_node import PatrolBotBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Empty
from tf2_ros import StaticTransformBroadcaster

log_path = Path(os.environ.get('PATROLBOT_SOAK_LOG', '/tmp/scan-outage.log'))
launch = None
fake = None
bridge = None
executor = None
forced_shutdown = False
try:
    log = log_path.open('w')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'patrolbot_navigation', 'bringup.launch.py'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    fake = FakeSbc()
    with fake._lock:
        # FakeSbc defaults to an asserted E-stop for command refusal tests.
        # Only this isolated simulator clears it; no hardware command is sent.
        fake.estop_pressed = False
        fake.received = deque(maxlen=100)
    scan = json.loads(Path('/validation/evidence/20260918-scan.json').read_text())
    fake.laser_field = ','.join('nan' if r is None else str(r) for r in scan['ranges'])
    rclpy.init()
    node = Node('isolated_epoch_navigation_acceptance')
    # Create persistent shutdown clients during setup, like the production
    # recovery coordinator. Late-created clients intermittently report ready
    # while their first request never reaches even an empty manager.
    shutdown_clients = {
        manager: node.create_client(
            ManageLifecycleNodes, f'/lifecycle_manager_{manager}/manage_nodes')
        for manager in ('navigation', 'localization')
    }
    bridge = PatrolBotBridge(parameter_overrides=[
        Parameter('server_ip', value='127.0.0.1'),
        Parameter('server_port', value=fake.port),
        Parameter('reconnect_delay', value=.1),
    ])
    broadcaster = StaticTransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.frame_id = 'base_link'
    tf.child_frame_id = 'laser_frame'
    tf.header.stamp = node.get_clock().now().to_msg()
    # Existing deployed laser extrinsics, also used by the recorded-map replay.
    tf.transform.translation.x = .037
    tf.transform.translation.z = .2
    tf.transform.rotation.x = 1.0
    broadcaster.sendTransform(tf)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(bridge)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    refresh = node.create_client(Empty, '/request_nomotion_update')
    events = [json.loads(line) for line in Path(
        '/validation/evidence/20260918-initialpose.jsonl').read_text().splitlines()]
    recorded = next(e['message'] for e in events if e['event'] == '/initialpose')

    def seed():
        assert wait_until(lambda: bridge._epoch.streams_allowed, timeout=90), (
            bridge._epoch.recovery.stage, bridge._epoch._phase, bridge._epoch._failed)
        assert wait_until(lambda: bridge._epoch.seed_tf_ready, timeout=10)
        assert wait_until(lambda: publisher.get_subscription_count() >= 2, timeout=10)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = node.get_clock().now().to_msg()
        p = recorded['pose']['pose']
        msg.pose.pose.position.x = p['position']['x']
        msg.pose.pose.position.y = p['position']['y']
        msg.pose.pose.orientation.z = p['orientation']['z']
        msg.pose.pose.orientation.w = p['orientation']['w']
        msg.pose.covariance = recorded['pose']['covariance']
        publisher.publish(msg)
        # One bounded isolated refresh ensures a scan stamped after the seed;
        # no standing refresh loop and no call to live ROS.
        time.sleep(.7)
        if refresh.service_is_ready():
            refresh.call_async(Empty.Request())
        ready = wait_until(lambda: bridge._epoch.commands_allowed, timeout=65)
        if not ready:
            pending = bridge._epoch._pending_pose
            print(json.dumps({'event': 'recovery_failure',
                'stage': bridge._epoch.recovery.stage, 'phase': bridge._epoch._phase,
                'failed': bridge._epoch._failed, 'seed_ns': bridge._epoch.recovery._seed_ns,
                'ack_ns': bridge._epoch._accepted_seed_stamp,
                'pending_pose_ns': bridge._epoch.stamp_ns(pending) if pending else None,
                'ack_publishers': node.count_publishers('/amcl/accepted_initial_pose'),
                'odom_pose_received': bridge._pose_received}), flush=True)
        assert ready, 'real navigation epoch recovery did not complete'

    seed()
    print(json.dumps({'event': 'initial_recovery_ready'}), flush=True)
    # Wall time is deliberate: accelerated clock tests cannot measure a soak.
    duration = float(os.environ.get('PATROLBOT_OUTAGE_SECONDS', '60'))
    assert 60 <= duration <= 28800, 'outage duration outside bounded test range'
    hold = {'last': 0.0, 'count': 0, 'nonzero': False}
    scans = {'last': 0.0}
    def on_hold(msg):
        hold['last'] = time.monotonic()
        hold['count'] += 1
        hold['nonzero'] |= msg.linear.x != 0.0 or msg.angular.z != 0.0
    hold_sub = node.create_subscription(Twist, '/input/safety_controller', on_hold, 10)
    scan_sub = node.create_subscription(LaserScan, '/scan', lambda msg: scans.update(last=time.monotonic()), qos_profile_sensor_data)
    time.sleep(2)
    assert time.monotonic() - scans['last'] < .5, 'no fresh scan before outage'
    assert wait_until(lambda: time.monotonic() - hold['last'] > 1, timeout=10), 'watchdog not released before outage'
    with fake._lock:
        saved_scan = fake.laser_field
        fake.laser_field = ''
    assert wait_until(lambda: time.monotonic() - scans['last'] > 1.0, timeout=5)
    assert wait_until(lambda: time.monotonic() - hold['last'] < .5, timeout=5)
    offset = log_path.stat().st_size
    start = time.monotonic()
    next_sample = start
    print(json.dumps({'event': 'outage_started', 'wall_seconds': duration}), flush=True)
    while time.monotonic() - start < duration:
        now = time.monotonic()
        assert launch.poll() is None, 'navigation launch died during outage'
        assert now - hold['last'] < 2, 'watchdog stopped publishing outage hold'
        assert not hold['nonzero'], 'watchdog published nonzero output'
        assert now - scans['last'] > .5, 'scan unexpectedly resumed during outage'
        assert log_path.stat().st_size < 64 * 1024 * 1024, 'test log exceeded 64 MiB safety cap'
        if now >= next_sample:
            with log_path.open() as f:
                f.seek(offset)
                recent = f.read()
            assert 'is current again' not in recent, 'costmap recovered without scan data'
            sample = {'event': 'outage_sample', 'elapsed_s': now-start,
                      'log_bytes': log_path.stat().st_size, 'hold_messages': hold['count']}
            for name in ('memory.current', 'memory.peak', 'cpu.stat'):
                path = Path('/sys/fs/cgroup') / name
                sample[name] = path.read_text().strip() if path.exists() else None
            print(json.dumps(sample), flush=True)
            next_sample = now + 60
        time.sleep(.1)
    with log_path.open() as f:
        f.seek(offset)
        outage_log = f.read()
    assert 'is current again' not in outage_log
    warnings = outage_log.count('observation buffer has not been updated')
    assert warnings > 0, 'no observation-buffer outage evidence'
    # Two configured scan observation buffers, each first warning plus 30 s repeats.
    assert warnings <= 2 * (math.ceil(duration / 30) + 2), warnings
    with fake._lock: fake.laser_field = saved_scan
    assert wait_until(lambda: time.monotonic() - scans['last'] < .5, timeout=10)
    assert wait_until(lambda: time.monotonic() - hold['last'] > 1, timeout=10), 'watchdog did not release'
    assert wait_until(lambda: 'is current again' in log_path.read_bytes()[offset:].decode(errors='replace'), timeout=10), 'missing costmap recovery event'
    print(json.dumps({'event':'outage_acceptance', 'wall_seconds':time.monotonic()-start,
                      'warning_count':warnings,'hold_messages':hold['count'],
                      'recovery_without_restart':True,'nonzero_hold':hold['nonzero']}),flush=True)
    # Shut down lifecycle-managed plugins while TF, DDS and their executors
    # still exist; then stop launch and the fake hardware process below.
    for manager in ('navigation', 'localization'):
        shutdown = shutdown_clients[manager]
        assert shutdown.wait_for_service(timeout_sec=5)
        request = ManageLifecycleNodes.Request()
        request.command = request.SHUTDOWN
        future = shutdown.call_async(request)
        assert wait_until(future.done, timeout=20), manager + ' shutdown timed out'
        assert future.result().success, manager + ' shutdown failed'
    print(json.dumps({'event': 'managed_shutdown_complete'}), flush=True)

finally:
    if executor is not None:
        executor.shutdown(timeout_sec=5)
    if bridge is not None:
        bridge.destroy_node()
    if fake is not None:
        fake.close()
    if rclpy.ok():
        rclpy.try_shutdown()
    if launch is not None and launch.poll() is None:
        # Launch owns its children and forwards the interrupt. Signaling the
        # whole group as well delivers duplicate shutdown signals to nodes.
        os.kill(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=12)
        except subprocess.TimeoutExpired:
            forced_shutdown = True
            os.killpg(launch.pid, signal.SIGKILL)
            launch.wait()
    if log_path.exists():
        logs = log_path.read_text()
        relevant = [line for line in logs.splitlines()
                    if '[amcl]' in line or 'ERROR' in line or 'SAFETY HOLD' in line or 'releasing priority' in line]
        print('\n'.join(relevant[:40]))
        print(f'... {max(0, len(relevant) - 80)} diagnostic lines omitted ...')
        print('\n'.join(relevant[-40:]))
        print(logs[-5000:])
        print('\n'.join(line for line in logs.splitlines() if 'SAFETY HOLD' in line or 'releasing priority' in line))
        assert 'SIGSEGV' not in logs and 'Segmentation Fault' not in logs, (
            'Nav2 composition crashed during isolated test teardown')

    assert not forced_shutdown, 'isolated Nav2 launch required forced termination'
    assert not re.search(r'process has died .*exit code (?!0[,\]])-?\d+', logs), (
        'a launched child exited unsuccessfully during teardown')
    assert launch is not None and launch.returncode == 0, (
        'ROS launch did not exit cleanly', None if launch is None else launch.returncode)
