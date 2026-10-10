"""Subscriber-only reset interlocks. No ROS command/service calls are made."""
import json
import math
import sys
import time


def evaluate(base, odom, readiness, now, *, post_reset=False):
    def fresh(samples, limit):
        return bool(samples) and 0 <= now - samples[-1]['received'] < limit
    base_ok = len(base) >= 3 and fresh(base, .5) and all(
        m['link'] is True and m['hardware'] is True and m['epoch_valid'] is True and
        math.isfinite(m['age']) and 0 <= m['age'] < .5 and
        m['dock'] == 'DOCKED_CONFIRMED' and m['motors'] is False and m['undock'] is False and
        (m['recovery'] is True and m['seed'] == 0 if post_reset else True)
        for m in base)
    epochs = {m['epoch'] for m in base}
    base_ok = base_ok and len(epochs) == 1 and bool(next(iter(epochs), ''))
    odom_ok = len(odom) >= 3 and fresh(odom, .5) and all(
        all(math.isfinite(v) for v in m['values']) and
        math.isfinite(m['age']) and 0 <= m['age'] < .5 and
        abs(m['linear']) < .001 and abs(m['angular']) < .001 for m in odom)
    nav_ok = len(readiness) >= 2 and fresh(readiness, 1.0) and all(
        m['idle'] is True and m['motors'] is False and m['dock'] == 'DOCKED_CONFIRMED' and
        math.isfinite(m['base_age']) and 0 <= m['base_age'] < .5 for m in readiness)
    return {'safe': bool(base_ok and odom_ok and nav_ok), 'base_samples': len(base),
            'odom_samples': len(odom), 'readiness_samples': len(readiness),
            'base_safe': bool(base_ok), 'odom_safe': bool(odom_ok),
            'navigation_idle': bool(nav_ok), 'odom_epoch': base[-1]['epoch'] if base else ''}


def main():
    if sys.argv[1:] not in ([], ['--post-reset']):
        raise SystemExit(2)
    import rclpy
    from nav_msgs.msg import Odometry
    from patrolbot_interfaces.msg import BaseState, DockReadiness
    rclpy.init()
    node = rclpy.create_node('software_reset_readonly_preflight')
    base, odom, readiness = [], [], []
    def on_base(m):
        base.append(dict(received=time.monotonic(), link=m.link_connected,
            hardware=m.hardware_state_valid, epoch_valid=m.odom_epoch_valid,
            age=m.telemetry_age, dock=m.dock_state, motors=m.motors_enabled,
            undock=m.undock_active, recovery=m.localization_recovery_required,
            seed=m.localization_seed_stamp_ns, epoch=m.odom_epoch))
    def on_odom(m):
        p,q,t=m.pose.pose.position,m.pose.pose.orientation,m.twist.twist
        age=(node.get_clock().now().nanoseconds-m.header.stamp.sec*10**9-m.header.stamp.nanosec)/1e9
        odom.append(dict(received=time.monotonic(),age=age,linear=t.linear.x,angular=t.angular.z,
            values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w,t.linear.x,t.angular.z,*m.pose.covariance]))
    def on_readiness(m):
        readiness.append(dict(received=time.monotonic(), idle=m.navigation_idle,
            motors=m.motors_enabled,dock=m.dock_state,base_age=m.base_state_age))
    node.create_subscription(BaseState,'/patrolbot/base_state',on_base,10)
    node.create_subscription(Odometry,'/odom',on_odom,10)
    node.create_subscription(DockReadiness,'/patrolbot/dock_readiness',on_readiness,10)
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    result=evaluate(base,odom,readiness,time.monotonic(),post_reset=bool(sys.argv[1:]))
    result['captured_unix']=time.time()
    print(json.dumps(result),flush=True)
    node.destroy_node();rclpy.try_shutdown()
    raise SystemExit(0 if result['safe'] else 1)


if __name__=='__main__':main()
