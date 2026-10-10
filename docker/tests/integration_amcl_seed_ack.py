"""Run only in a network-isolated ROS container; never against live ROS."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.srv import ChangeState
from std_srvs.srv import Trigger
from rclpy.duration import Duration
from tf2_ros import TransformBroadcaster

assert os.environ.get('PATROLBOT_ISOLATED_TEST') == '1'
process = None
log_path = Path('/tmp/amcl-tf-retry.log')
try:
    log = log_path.open('w')
    process = subprocess.Popen([
        'ros2', 'run', 'nav2_amcl', 'amcl', '--ros-args',
        '--params-file', '/opt/patrolbot_ws/src/patrolbot_navigation/config/nav2_params.yaml',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('isolated_initial_pose_timing_test')
    publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    accepted = []
    ack_sub = node.create_subscription(
        PoseWithCovarianceStamped, '/amcl/accepted_initial_pose',
        lambda msg: accepted.append(msg.header.stamp), 10)
    broadcaster = TransformBroadcaster(node)
    readiness = node.create_client(Trigger, '/amcl/initial_pose_tf_ready')
    lifecycle = node.create_client(ChangeState, '/amcl/change_state')
    assert lifecycle.wait_for_service(timeout_sec=15)

    def spin(seconds, tf=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if tf:
                msg = TransformStamped()
                msg.header.frame_id = 'odom'
                msg.child_frame_id = 'base_link'
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.transform.rotation.w = 1.0
                broadcaster.sendTransform(msg)
            rclpy.spin_once(node, timeout_sec=.02)

    def transition(number):
        request = ChangeState.Request()
        request.transition.id = number
        future = lifecycle.call_async(request)
        end = time.monotonic() + 10
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        assert future.done() and future.result().success

    def seed(offset=0.0):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = (node.get_clock().now() + Duration(seconds=offset)).to_msg()
        msg.pose.pose.position.x = 1.0
        msg.pose.pose.position.y = 2.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = msg.pose.covariance[7] = .25
        msg.pose.covariance[35] = .0685
        publisher.publish(msg)
        return msg.header.stamp

    def count(text):
        return log_path.read_text().count(text)

    def ready():
        assert readiness.wait_for_service(timeout_sec=5)
        future = readiness.call_async(Trigger.Request())
        deadline = time.monotonic() + 3
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        assert future.done(), 'readiness service did not respond'
        return future.result().success

    transition(1)
    assert not ready(), 'inactive AMCL reported ready'
    transition(3)
    assert not ready(), 'AMCL ready without TF history'
    deadline = time.monotonic() + 5.0
    while publisher.get_subscription_count() != 1 and time.monotonic() < deadline:
        spin(.02, tf=False)
    assert publisher.get_subscription_count() == 1
    # A publisher existing is not proof that AMCL has received odometry TF.
    # Reject a seed with no TF history; later TF must not silently replay it.
    seed()
    spin(.15, tf=False)
    # Even TF arriving inside the retry window cannot reconstruct older history.
    spin(1.25)
    assert count('Rejecting initial pose') == 1, 'missing TF history was ignored'
    assert count('Setting pose') == 0 and not accepted
    spin(.5)
    assert count('Setting pose') == 0 and not accepted, 'replayed an expired seed'
    assert ready(), 'AMCL did not report received TF'
    # No transform covers this fresh seed until TF resumes 150 ms later.
    first_stamp = seed()
    spin(.15, tf=False)
    assert count('Setting pose') == 0, 'seed accepted without timestamped TF'
    assert not accepted, 'acknowledged before TF acceptance'
    spin(.4)
    assert count('Setting pose') == 1, 'pending seed did not recover with TF'
    assert accepted == [first_stamp]
    # Future source timestamp never becomes available within the 1s bound.
    seed(5.0)
    spin(1.4)
    assert count('Rejecting initial pose') == 2, 'retry deadline was extended'
    assert count('Setting pose') == 1, 'timeout used an identity fallback'
    assert accepted == [first_stamp], 'timed-out seed was acknowledged'
    # Superseding a future seed must create one fresh bounded request.
    seed(5.0)
    spin(.1)
    second_stamp = seed()
    spin(.4)
    assert count('Setting pose') == 2
    assert accepted == [first_stamp, second_stamp]
    spin(1.1)
    assert count('Rejecting initial pose') == 2, 'superseded request survived'
    # Pending callbacks must not apply after lifecycle deactivation.
    seed(5.0)
    spin(.1)
    transition(4)
    assert not ready(), 'deactivated AMCL reported ready'
    spin(1.2)
    assert count('Setting pose') == 2
    assert accepted == [first_stamp, second_stamp]
    assert count('Rejecting initial pose') == 2
    transition(3)
    spin(.3)
    assert ready()
    spin(1.2, tf=False)
    assert not ready(), 'stale TF reported ready'
    spin(.3)
    assert ready(), 'fresh TF did not restore readiness'
    future_tf = TransformStamped()
    future_tf.header.frame_id = 'odom'
    future_tf.child_frame_id = 'base_link'
    future_tf.header.stamp = (node.get_clock().now() + Duration(seconds=5)).to_msg()
    future_tf.transform.rotation.w = 1.0
    broadcaster.sendTransform(future_tf)
    spin(.1, tf=False)
    assert not ready(), 'future TF reported ready'
    transition(4)
    transition(2)
    transition(1)
    transition(3)
    assert not ready(), 'cleanup preserved previous TF readiness'
    transition(4)
    transition(2)
    print(json.dumps({'passed': ['readiness_missing_tf', 'readiness_inactive', 'readiness_fresh_tf', 'readiness_stale_tf', 'readiness_future_tf', 'readiness_lifecycle_reset', 'cold_start_missing_tf_rejected', 'expired_seed_not_replayed', 'fresh_seed_after_tf', 'delayed_tf', 'bounded_timeout', 'supersession', 'deactivation_cleanup', 'accepted_seed_correlation'],
                      'settings': count('Setting pose'), 'timeouts': count('Rejecting initial pose')}))
finally:
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
    if log_path.exists():
        print(log_path.read_text())
    if rclpy.ok():
        rclpy.shutdown()
