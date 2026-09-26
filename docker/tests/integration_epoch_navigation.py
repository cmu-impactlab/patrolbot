"""Real Nav2 lifecycle/seed recovery with loopback hardware in isolation only."""
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
from uuid import uuid4

assert os.environ.get('PATROLBOT_ISOLATED_TEST') == '1'
sys.path.insert(0, '/validation/ros2_ws/src/patrolbot_bridge/test')
from test_fake_sbc_integration import FakeSbc, wait_until
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav2_msgs.action import Wait
from nav2_msgs.srv import ManageLifecycleNodes
from patrolbot_bridge.bridge_node import PatrolBotBridge
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Empty
from tf2_ros import StaticTransformBroadcaster

log_path = Path('/tmp/epoch-navigation.log')
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
    scan = json.loads(Path('/validation/evidence/20260918-scan.json').read_text())
    fake.laser_field = ','.join('nan' if r is None else str(r) for r in scan['ranges'])
    rclpy.init()
    node = Node('isolated_epoch_navigation_acceptance')
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
        assert wait_until(lambda: bridge._epoch.first_tf_ns is not None, timeout=10)
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
    client = ActionClient(node, Wait, '/wait')
    assert client.wait_for_server(timeout_sec=15)
    goal = Wait.Goal()
    goal.time.sec = 120
    future = client.send_goal_async(goal)
    assert wait_until(future.done, timeout=10)
    handle = future.result()
    assert handle.accepted
    result = handle.get_result_async()
    assert not result.done()
    old = bridge._epoch.guard.epoch
    with fake._lock:
        fake.odom_epoch = str(uuid4())
    fake.drop_connection()
    assert wait_until(lambda: bridge._epoch.guard.epoch != old, timeout=10)
    assert not bridge._epoch.commands_allowed
    seed()
    assert wait_until(result.done, timeout=10), 'old Wait goal survived Nav2 pause/resume'
    assert result.result().status in (5, 6), result.result().status
    print(json.dumps({'passed': ['real_amcl_seed_ack', 'origin_restart_hold',
                                 'real_navigation_pause_resume', 'old_goal_terminal'],
                      'old_wait_status': result.result().status}), flush=True)
    # Shut down lifecycle-managed plugins while TF, DDS and their executors
    # still exist; then stop launch and the fake hardware process below.
    for manager in ('navigation', 'localization'):
        shutdown = node.create_client(
            ManageLifecycleNodes, f'/lifecycle_manager_{manager}/manage_nodes')
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
                    if '[amcl]' in line or 'ERROR' in line]
        print('\n'.join(relevant[:40]))
        print(f'... {max(0, len(relevant) - 80)} diagnostic lines omitted ...')
        print('\n'.join(relevant[-40:]))
        print(logs[-5000:])
        assert 'SIGSEGV' not in logs and 'Segmentation Fault' not in logs, (
            'Nav2 composition crashed during isolated test teardown')

    assert not forced_shutdown, 'isolated Nav2 launch required forced termination'
    assert not re.search(r'process has died .*exit code (?!0[,\]])-?\d+', logs), (
        'a launched child exited unsuccessfully during teardown')
    assert launch is not None and launch.returncode == 0, (
        'ROS launch did not exit cleanly', None if launch is None else launch.returncode)
