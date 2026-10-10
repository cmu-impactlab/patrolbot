"""Subscriber/fake-input test; requires isolated container network and DDS."""
import json
import os
import signal
import subprocess
import sys
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, BatteryState
from patrolbot_interfaces.msg import BaseState
from rclpy.qos import qos_profile_sensor_data
assert os.environ.get('PATROLBOT_ISOLATED_TEST') == '1'
rclpy.init()
node = rclpy.create_node('isolated_watchdog_outage_test')
watchdog_script = os.environ.get('PATROLBOT_WATCHDOG_SCRIPT')
command = ([sys.executable, watchdog_script] if watchdog_script else
           ['ros2', 'run', 'patrolbot_navigation', 'patrolbot_safety_watchdog.py'])
process = subprocess.Popen(command, start_new_session=True)
seen = []
sub = node.create_subscription(Twist, '/input/safety_controller', lambda msg: seen.append((time.monotonic(), msg)), 10)
pubs = {t: node.create_publisher(typ, t, qos_profile_sensor_data if t == '/scan' else 10)
        for t, typ in [('/scan', LaserScan), ('/odom', Odometry), ('/battery', BatteryState), ('/patrolbot/base_state', BaseState)]}
scan = LaserScan(); scan.header.frame_id = 'laser_frame'; scan.ranges = [1.0, 1.0, 1.0]
odom = Odometry(); odom.pose.pose.orientation.w = 1.0
battery = BatteryState(); battery.voltage = 24.0
base = BaseState(); base.hardware_state_valid = True; base.bumpers_valid = True; base.link_connected = True
base.odom_epoch_valid = True
base.localization_recovery_required = False
messages = {'/scan': scan, '/odom': odom, '/battery': battery, '/patrolbot/base_state': base}
def spin(seconds, scan_enabled=True):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for topic, msg in messages.items():
            if topic == '/scan' and not scan_enabled:
                continue
            msg.header.stamp = node.get_clock().now().to_msg()
            pubs[topic].publish(msg)
        rclpy.spin_once(node, timeout_sec=.025)
try:
    spin(2.0)
    seen.clear()
    spin(.5)
    assert not seen, 'watchdog failed to leave startup hold with fresh inputs'
    base.localization_recovery_required = True
    spin(.4)
    assert len(seen) >= 2, 'epoch recovery did not assert safety hold'
    assert all(m.linear.x == 0 and m.angular.z == 0 for _, m in seen)
    base.localization_recovery_required = False
    spin(.3)
    seen.clear()
    spin(.3)
    assert not seen, 'epoch recovery completion retained hold'
    spin(1.2, False)
    assert len(seen) >= 3, 'stale scan did not assert safety hold'
    assert all(m.linear.x == 0 and m.angular.z == 0 for _, m in seen)
    hold_messages = len(seen)
    spin(.5)
    seen.clear()
    spin(.5)
    assert not seen, 'scan recovery required a restart or retained hold'
    print(json.dumps({'passed': ['fresh_release', 'epoch_recovery_hold', 'stale_scan_zero_hold', 'recovery_without_restart'], 'hold_messages': hold_messages}))
finally:
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL); process.wait()
    node.destroy_node(); rclpy.shutdown()
    assert process.returncode == 0, 'watchdog shutdown failed'
