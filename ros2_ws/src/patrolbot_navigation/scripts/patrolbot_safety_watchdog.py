#!/usr/bin/env python3
"""
PatrolBot Safety Watchdog — sensor-loss safe-hold.

Monitors the freshness of the nav-critical sensor streams (/scan and /odom).
If either goes stale (sensor died, bridge died, SBC link dropped, or the SBC
started emitting an EMPTY laser because the SICK is disconnected), the robot
must NOT keep driving on stale data. This node holds it by publishing a zero
Twist to twist_mux's highest-priority input (`input/safety_controller`,
priority 10), which overrides navigation (5) and teleop (8).

When every monitored stream is fresh again, the node goes SILENT; twist_mux
times the safety input out (~1 s) and navigation resumes on its own — no
restart, no operator action.

Fail-safe by construction: before any data has ever arrived the streams count
as stale, so the default state is "hold". The robot can only move once it has
proven its sensors are live.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


class SafetyWatchdog(Node):
    def __init__(self):
        super().__init__('patrolbot_safety_watchdog')

        self.declare_parameter('scan_timeout', 0.5)   # s without /scan -> stale
        self.declare_parameter('odom_timeout', 0.5)   # s without /odom -> stale
        self.declare_parameter('rate', 10.0)          # Hz check/hold-publish
        g = lambda k: self.get_parameter(k).value
        self.scan_timeout = g('scan_timeout')
        self.odom_timeout = g('odom_timeout')
        dt = 1.0 / g('rate')

        # Use monotonic-ish ROS clock seconds for age math.
        self._last_scan = None
        self._last_odom = None
        self._holding = False

        # /scan is BEST_EFFORT (sensor QoS); /odom is RELIABLE (default).
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.pub = self.create_publisher(Twist, '/input/safety_controller', 10)
        self.create_timer(dt, self._tick)

        self.get_logger().info(
            f'Safety watchdog up. Holds robot if /scan >{self.scan_timeout}s or '
            f'/odom >{self.odom_timeout}s stale (via input/safety_controller).')

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _scan_cb(self, _):
        self._last_scan = self._now()

    def _odom_cb(self, _):
        self._last_odom = self._now()

    def _stale(self):
        now = self._now()
        reasons = []
        if self._last_scan is None or (now - self._last_scan) > self.scan_timeout:
            reasons.append('scan')
        if self._last_odom is None or (now - self._last_odom) > self.odom_timeout:
            reasons.append('odom')
        return reasons

    def _tick(self):
        reasons = self._stale()
        if reasons:
            # Hold: publish zero at the highest mux priority.
            self.pub.publish(Twist())
            if not self._holding:
                self._holding = True
                self.get_logger().warn(
                    f'SENSOR LOSS ({", ".join(reasons)}) — holding robot (safety stop).')
        elif self._holding:
            # Recovered: go silent so the safety input times out and nav resumes.
            self._holding = False
            self.get_logger().info('Sensors restored — releasing hold, nav resumes.')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
