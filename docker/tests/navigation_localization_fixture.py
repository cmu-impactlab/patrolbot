"""Synthetic valid navigation context for network-isolated native BT tests."""
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from patrolbot_interfaces.msg import BaseState
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


def localization_fixture(node):
    state = dict(pose_age=0., update_pose=True, invalid_pose=False, moving=False,
                 generation=1, odom_enabled=True, accepted=True, bad_quaternion=False, bad_covariance=False,
                 pose_stamp_ns=None, pose_published=False, invalid_odom=False, bad_odom_frame=False, ack_lag=0., pose_stamps=[])
    barrier = node.get_clock().now().nanoseconds - 60_000_000_000
    base_pub = node.create_publisher(BaseState, '/patrolbot/base_state', 10)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/amcl_pose',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    def publish():
        now = node.get_clock().now()
        base = BaseState(); base.header.stamp = now.to_msg()
        base.link_connected = base.hardware_state_valid = True
        base.session_generation = state['generation']; base.odom_epoch = 'fixture'
        base.odom_epoch_generation = 1; base.odom_epoch_valid = True
        base.map_id = 'fixture'; base.map_revision = 'v1'
        base.map_activated_stamp_ns = base.localization_seed_stamp_ns = barrier
        base.lidar_state = 'READY'; base.dock_state_valid = True
        base.dock_state = 'CLEAR_CONFIRMED'
        if state['update_pose'] or state['pose_stamp_ns'] is None:
            state['pose_stamp_ns'] = now.nanoseconds - int(state['pose_age'] * 1e9)
            state['pose_stamps'] = (state['pose_stamps'] + [state['pose_stamp_ns']])[-64:]
        acknowledged = [s for s in state['pose_stamps'] if s <= now.nanoseconds-int(state['ack_lag']*1e9)]
        base.localization_pose_stamp_ns = acknowledged[-1] if acknowledged and state['accepted'] else 0
        base_pub.publish(base)
        pose = PoseWithCovarianceStamped(); pose.header.frame_id = 'map'
        stamp = state['pose_stamp_ns']
        pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        pose.pose.pose.orientation.w = 1.
        for i in (0, 7, 35): pose.pose.covariance[i] = .01
        if state['invalid_pose']: pose.pose.covariance[0] = float('nan')
        if state['bad_quaternion']: pose.pose.pose.orientation.w = .999
        if state['bad_covariance']: pose.pose.covariance[1] = pose.pose.covariance[6] = .02
        if state['update_pose'] or not state['pose_published']:
            pose_pub.publish(pose)
            state['pose_published'] = True
        if state['odom_enabled']:
            odom = Odometry(); odom.header.stamp = now.to_msg()
            odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_link'
            odom.pose.pose.orientation.w = 1.
            if state['invalid_odom']: odom.pose.pose.position.x = float('nan')
            if state['bad_odom_frame']: odom.header.frame_id = 'map'
            odom.twist.twist.linear.x = .02 if state['moving'] else 0.
            odom_pub.publish(odom)
    timer = node.create_timer(.05, publish)
    return state, timer, (base_pub, pose_pub, odom_pub)
