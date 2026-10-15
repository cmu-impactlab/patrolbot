#!/usr/bin/env python3
"""
PatrolBot Safety Watchdog — system safety safe-hold.

Monitors nav sensor freshness, battery voltage/freshness, typed base hardware
state, E-stop, FAULTFLAGS, and (once physically commissioned) direction-aware
bumper state. Any invalid safety input publishes zero Twist to twist_mux's
highest-priority input (`input/safety_controller`, priority 10), overriding
navigation and teleop.

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
from nav_msgs.msg import Odometry
from patrolbot_interfaces.msg import BaseState
from rclpy.qos import qos_profile_sensor_data
from safety_policy import evaluate_safety, odometry_values_finite
from sensor_msgs.msg import BatteryState, LaserScan


def undock_turn_owned(message):
    goal = getattr(message, 'undock_turn_goal_id', '')
    return bool(getattr(message, 'undock_turn_active', False)
                and len(goal) == 32 and all(c in '0123456789abcdef' for c in goal)
                and message.link_connected and message.hardware_state_valid
                and getattr(message, 'odom_epoch_valid', False)
                and message.motors_enabled and message.dock_state_valid
                and message.dock_state == 'CLEAR_CONFIRMED'
                and not message.undock_active)


class SafetyWatchdog(Node):
    def __init__(self):
        super().__init__('patrolbot_safety_watchdog')

        self.declare_parameter('scan_timeout', 0.5)   # s without /scan -> stale
        self.declare_parameter('odom_timeout', 0.5)   # s without /odom -> stale
        self.declare_parameter('battery_timeout', 1.0)
        self.declare_parameter('base_state_timeout', 0.75)
        self.declare_parameter('battery_stop_voltage', 23.0)
        self.declare_parameter('stop_on_faults', True)
        self.declare_parameter('stop_on_estop', True)
        # Must remain false until all 12 bumper segments and masks are physically
        # mapped. When enabled, invalid mapping itself is a fail-closed hold.
        self.declare_parameter('bumper_enforcement_enabled', False)
        self.declare_parameter('rate', 10.0)          # Hz check/hold-publish
        g = lambda k: self.get_parameter(k).value
        self.scan_timeout = g('scan_timeout')
        self.odom_timeout = g('odom_timeout')
        self.battery_timeout = g('battery_timeout')
        self.base_state_timeout = g('base_state_timeout')
        self.battery_stop_voltage = g('battery_stop_voltage')
        self.stop_on_faults = g('stop_on_faults')
        self.stop_on_estop = g('stop_on_estop')
        self.bumpers_enabled = g('bumper_enforcement_enabled')
        dt = 1.0 / g('rate')

        # Use monotonic-ish ROS clock seconds for age math.
        self._last_scan = None
        self._last_odom = None
        self._invalid_odom_count = 0
        self._last_battery = None
        self._battery_voltage = float('nan')
        self._last_base_state = None
        self._hardware_state_valid = False
        self._fault_flags = 0
        self._estop_pressed = False
        self._bumpers_valid = False
        self._front_bumper_pressed = False
        self._rear_bumper_pressed = False
        self._desired_linear = 0.0
        self._desired_angular = 0.0
        self._holding = False
        self._holding_reasons = ()

        # /scan is BEST_EFFORT (sensor QoS); /odom is RELIABLE (default).
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(
            BatteryState, '/battery', self._battery_cb, 10)
        self.create_subscription(
            BaseState, '/patrolbot/base_state', self._base_state_cb, 10)
        # Observe the selected command before final smoothing. While a bumper
        # hold is active, zero output from this watchdog is ignored so the last
        # hazardous direction stays latched until the bumper releases.
        self.create_subscription(
            Twist, '/cmd_vel_out', self._desired_cmd_cb, 10)
        self.pub = self.create_publisher(Twist, '/input/safety_controller', 10)
        self.create_timer(dt, self._tick)

        self.get_logger().info(
            'Safety watchdog up: scan/odom, battery, typed hardware state, '
            'E-stop, and FAULTFLAGS are enforced via input/safety_controller; '
            f'bumper enforcement={self.bumpers_enabled}.')

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _scan_cb(self, _):
        self._last_scan = self._now()

    def _odom_cb(self, message):
        orientation = message.pose.pose.orientation
        if not odometry_values_finite(
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                (orientation.x, orientation.y, orientation.z, orientation.w),
                message.twist.twist.linear.x,
                message.twist.twist.angular.z):
            self._last_odom = None
            self._invalid_odom_count += 1
            self.get_logger().error(
                f'Rejected non-finite /odom; count={self._invalid_odom_count}.',
                throttle_duration_sec=5.0)
            return
        self._last_odom = self._now()

    def _battery_cb(self, message):
        self._last_battery = self._now()
        self._battery_voltage = float(message.voltage)

    def _base_state_cb(self, message):
        self._last_base_state = self._now()
        # 20261014: localization recovery no longer holds manual control.
        # A deadman-held human drives with eyes open; autonomy stays blocked
        # at the bridge (commands_allowed) and dashboard gate until re-seed.
        self._hardware_state_valid = bool(
            message.hardware_state_valid
            and getattr(message, "odom_epoch_valid", False))
        self._fault_flags = int(message.fault_flags)
        self._estop_pressed = bool(message.estop_pressed)
        self._bumpers_valid = bool(message.bumpers_valid)
        self._front_bumper_pressed = bool(message.front_bumper_pressed)
        self._rear_bumper_pressed = bool(message.rear_bumper_pressed)

    def _desired_cmd_cb(self, message):
        if self._holding:
            return
        self._desired_linear = float(message.linear.x)
        self._desired_angular = float(message.angular.z)

    def _reasons(self):
        now = self._now()
        return evaluate_safety(
            now=now,
            scan_at=self._last_scan,
            odom_at=self._last_odom,
            battery_at=self._last_battery,
            battery_voltage=self._battery_voltage,
            base_state_at=self._last_base_state,
            hardware_state_valid=self._hardware_state_valid,
            fault_flags=self._fault_flags,
            estop_pressed=self._estop_pressed,
            bumpers_enabled=self.bumpers_enabled,
            bumpers_valid=self._bumpers_valid,
            front_bumper_pressed=self._front_bumper_pressed,
            rear_bumper_pressed=self._rear_bumper_pressed,
            desired_linear=self._desired_linear,
            desired_angular=self._desired_angular,
            scan_timeout=self.scan_timeout,
            odom_timeout=self.odom_timeout,
            battery_timeout=self.battery_timeout,
            base_state_timeout=self.base_state_timeout,
            battery_stop_voltage=self.battery_stop_voltage,
            stop_on_faults=self.stop_on_faults,
            stop_on_estop=self.stop_on_estop)

    def _tick(self):
        reasons = self._reasons()
        if reasons:
            # Hold: publish zero at the highest mux priority.
            self.pub.publish(Twist())
            reason_tuple = tuple(reasons)
            if not self._holding or reason_tuple != self._holding_reasons:
                self._holding = True
                self._holding_reasons = reason_tuple
                self.get_logger().warn(
                    'SAFETY HOLD (' + ', '.join(reasons) +
                    ') — commanding priority-10 zero velocity.')
        elif self._holding:
            # Recovered: go silent so the safety input times out and nav resumes.
            self._holding = False
            self._holding_reasons = ()
            self._desired_linear = 0.0
            self._desired_angular = 0.0
            self.get_logger().info(
                'All enforced safety inputs valid — releasing priority-10 hold.')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
