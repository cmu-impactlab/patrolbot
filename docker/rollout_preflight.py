"""Read-only hardware capture; run inside the existing Pi ROS container."""
import json
import math
import time


# Match the existing half-second hardware/odometry freshness requirement.
MAX_BASE_RECEIPT_AGE_S = 0.5
MAX_ODOM_RECEIPT_AGE_S = 0.5


def evaluate(states, odometry, now):
    """Evaluate captured data and ensure each stream is still arriving at end."""
    base_safe = len(states) >= 3 and all(
        m['link_connected'] and m['hardware_state_valid'] and m['odom_epoch_valid'] and
        math.isfinite(m['telemetry_age']) and 0 <= m['telemetry_age'] < 0.5 and
        not m['motors_enabled'] and not m['undock_active'] and
        m['dock_state'] == 'DOCKED_CONFIRMED' for m in states)
    odom_safe = len(odometry) >= 3 and all(
        m['finite'] and math.isfinite(m['age']) and 0 <= m['age'] < 0.5 and
        abs(m['linear']) < 0.001 and abs(m['angular']) < 0.001
        for m in odometry)

    base_receipt_age = now - states[-1]['received'] if states else math.inf
    odom_receipt_age = now - odometry[-1]['received'] if odometry else math.inf
    base_fresh = 0 <= base_receipt_age < MAX_BASE_RECEIPT_AGE_S
    odom_fresh = 0 <= odom_receipt_age < MAX_ODOM_RECEIPT_AGE_S
    return {
        'base_samples': len(states),
        'odom_samples': len(odometry),
        'base_safe': bool(base_safe and base_fresh),
        'odom_safe': bool(odom_safe and odom_fresh),
        'base_receipt_age_s': base_receipt_age if math.isfinite(base_receipt_age) else None,
        'odom_receipt_age_s': odom_receipt_age if math.isfinite(odom_receipt_age) else None,
        'safe': bool(base_safe and base_fresh and odom_safe and odom_fresh),
    }


def main():
    import rclpy
    from rclpy.node import Node
    from patrolbot_interfaces.msg import BaseState
    from nav_msgs.msg import Odometry

    rclpy.init()
    node = Node('candidate_rollout_readonly_preflight')
    states, odometry = [], []

    def on_base(message):
        states.append({
            'received': time.monotonic(),
            'link_connected': message.link_connected,
            'hardware_state_valid': message.hardware_state_valid,
            'odom_epoch_valid': message.odom_epoch_valid,
            'telemetry_age': message.telemetry_age,
            'motors_enabled': message.motors_enabled,
            'undock_active': message.undock_active,
            'dock_state': message.dock_state,
        })

    def on_odom(message):
        pose, twist = message.pose.pose, message.twist.twist
        values = (pose.position.x, pose.position.y, pose.orientation.x,
                  pose.orientation.y, pose.orientation.z, pose.orientation.w,
                  twist.linear.x, twist.angular.z)
        age = (node.get_clock().now().nanoseconds -
               (message.header.stamp.sec * 10**9 + message.header.stamp.nanosec)) / 1e9
        odometry.append({'received': time.monotonic(),
                         'finite': all(math.isfinite(v) for v in values),
                         'age': age, 'linear': twist.linear.x,
                         'angular': twist.angular.z})

    node.create_subscription(BaseState, '/patrolbot/base_state', on_base, 10)
    node.create_subscription(Odometry, '/odom', on_odom, 10)
    end = time.monotonic() + 5
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    captured_at = time.monotonic()
    result = evaluate(states, odometry, captured_at)
    result['captured_unix'] = time.time()
    print(json.dumps(result), flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    raise SystemExit(0 if result['safe'] else 1)


if __name__ == '__main__':
    main()
