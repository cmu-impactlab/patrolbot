"""Read-only hardware capture; run inside the existing Pi ROS container."""
import json, math, time
import rclpy
from rclpy.node import Node
from patrolbot_interfaces.msg import BaseState
from nav_msgs.msg import Odometry

rclpy.init()
node = Node('candidate_rollout_readonly_preflight')
states, odometry = [], []

def on_odom(message):
    pose, twist = message.pose.pose, message.twist.twist
    values = (pose.position.x, pose.position.y, pose.orientation.x,
              pose.orientation.y, pose.orientation.z, pose.orientation.w,
              twist.linear.x, twist.angular.z)
    age = (node.get_clock().now().nanoseconds -
           (message.header.stamp.sec * 10**9 + message.header.stamp.nanosec)) / 1e9
    odometry.append({'finite': all(math.isfinite(v) for v in values),
                     'age': age, 'linear': twist.linear.x, 'angular': twist.angular.z})

node.create_subscription(BaseState, '/patrolbot/base_state', states.append, 10)
node.create_subscription(Odometry, '/odom', on_odom, 10)
end = time.monotonic() + 5
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
base_ok = len(states) >= 3 and all(
    m.link_connected and m.hardware_state_valid and m.odom_epoch_valid and
    math.isfinite(m.telemetry_age) and 0 <= m.telemetry_age < 0.5 and
    not m.motors_enabled and not m.undock_active and
    m.dock_state == 'DOCKED_CONFIRMED' for m in states)
odom_ok = len(odometry) >= 3 and all(
    m['finite'] and math.isfinite(m['age']) and 0 <= m['age'] < 0.5 and
    abs(m['linear']) < 0.001 and abs(m['angular']) < 0.001 for m in odometry)
result = {'base_samples': len(states), 'odom_samples': len(odometry),
          'base_safe': bool(base_ok), 'odom_safe': bool(odom_ok),
          'safe': bool(base_ok and odom_ok), 'captured_unix': time.time()}
print(json.dumps(result), flush=True)
node.destroy_node()
rclpy.try_shutdown()
raise SystemExit(0 if result['safe'] else 1)
