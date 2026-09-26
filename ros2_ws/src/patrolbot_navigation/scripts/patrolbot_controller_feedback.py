#!/usr/bin/env python3
"""Finite F710 rumble alerts for active PatrolBot drive blockers.

This node has no motion publisher, action client, or service client. Its sole
output is ``/joy/set_feedback`` for the existing ``joy_node``.
"""

import signal

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from patrolbot_interfaces.msg import BaseState
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import BatteryState, Joy, JoyFeedback, LaserScan

from controller_feedback_policy import (
    PUBLISH_ZERO,
    PulseScheduler,
    feedback_blockers,
    joystick_drive_intent,
)


def _descriptor(description, minimum, maximum):
    return ParameterDescriptor(
        description=description,
        floating_point_range=[FloatingPointRange(
            from_value=float(minimum),
            to_value=float(maximum),
            step=0.0,
        )],
    )


class ControllerFeedback(Node):
    def __init__(self):
        super().__init__('patrolbot_controller_feedback')

        self.intensity = self._bounded(
            'intensity', 1.0, 0.0, 1.0, 'F710 rumble intensity')
        self.event_pulse = self._bounded(
            'event_pulse', 0.6, 0.05, 5.0,
            'Seconds for the first blocker pulse')
        self.retry_pulse = self._bounded(
            'retry_pulse', 0.35, 0.05, 5.0,
            'Seconds for a repeated blocked-drive pulse')
        self.retry_interval = self._bounded(
            'retry_interval', 1.5, 0.1, 30.0,
            'Minimum seconds between feedback pulses during a drive attempt')
        self.startup_grace = self._bounded(
            'startup_grace', 2.0, 0.0, 30.0,
            'Seconds to suppress expected startup telemetry blockers')
        self.joystick_timeout = self._bounded(
            'joystick_timeout', 0.4, 0.05, 5.0,
            'Maximum age of joystick input used for a drive attempt')
        self.deadzone = self._bounded(
            'deadzone', 0.12, 0.0, 0.95,
            'Stick deadzone used to recognize nonzero drive intent')
        if self.retry_interval < max(self.event_pulse, self.retry_pulse):
            raise ValueError(
                'retry_interval must be at least both pulse lengths')

        self._last_scan = None
        self._last_odom = None
        self._last_battery = None
        self._battery_voltage = float('nan')
        self._last_base_state = None
        self._hardware_state_valid = False
        self._fault_flags = 0
        self._estop_pressed = False
        self._bumpers_valid = False
        self._front_bumper_pressed = False
        self._rear_bumper_pressed = False
        self._motors_enabled = False
        self._dock_state_valid = False
        self._dock_state = 'UNKNOWN'
        self._desired_linear = 0.0
        self._desired_angular = 0.0
        self._last_joy = None
        self._joy_axes = []
        self._joy_buttons = []
        self._last_reasons = ()
        self._zero_sent_for_shutdown = False

        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(
            BatteryState, '/battery', self._battery_cb, 10)
        self.create_subscription(
            BaseState, '/patrolbot/base_state', self._base_state_cb, 10)
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(
            Twist, '/cmd_vel_out', self._desired_cmd_cb, 10)

        # joy_node accepts exactly one TYPE_RUMBLE, id=0 feedback stream. SDL's
        # LEFTRIGHT implementation applies that intensity to both the F710's
        # large and small motor magnitudes.
        self.feedback_pub = self.create_publisher(
            JoyFeedback, '/joy/set_feedback', 10)

        started_at = self._now()
        self._scheduler = PulseScheduler(
            started_at=started_at,
            event_pulse=self.event_pulse,
            retry_pulse=self.retry_pulse,
            retry_interval=self.retry_interval,
            startup_grace=self.startup_grace,
        )
        self.create_timer(0.02, self._tick)
        self.get_logger().info(
            'Controller feedback ready: alert-only finite rumble; no motion '
            'or motor-enable interface exists in this node.')

    def _bounded(self, name, default, minimum, maximum, description):
        return float(self.declare_parameter(
            name,
            default,
            _descriptor(description, minimum, maximum),
        ).value)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _scan_cb(self, _message):
        self._last_scan = self._now()

    def _odom_cb(self, _message):
        self._last_odom = self._now()

    def _battery_cb(self, message):
        self._last_battery = self._now()
        self._battery_voltage = float(message.voltage)

    def _base_state_cb(self, message):
        self._last_base_state = self._now()
        self._hardware_state_valid = bool(message.hardware_state_valid)
        self._fault_flags = int(message.fault_flags)
        self._estop_pressed = bool(message.estop_pressed)
        self._bumpers_valid = bool(message.bumpers_valid)
        self._front_bumper_pressed = bool(message.front_bumper_pressed)
        self._rear_bumper_pressed = bool(message.rear_bumper_pressed)
        self._motors_enabled = bool(message.motors_enabled)
        self._dock_state_valid = bool(message.dock_state_valid)
        self._dock_state = str(message.dock_state)

    def _joy_cb(self, message):
        self._last_joy = self._now()
        self._joy_axes = list(message.axes)
        self._joy_buttons = list(message.buttons)

    def _desired_cmd_cb(self, message):
        # Match the watchdog's direction latch: once a physical bumper blocks
        # a direction, its priority-10 zero must not erase the direction that
        # caused the block. Releasing the bumper permits the next command.
        previous_rotation = abs(self._desired_angular) > 0.01
        front_blocked = self._front_bumper_pressed and (
            self._desired_linear > 0.005 or previous_rotation)
        rear_blocked = self._rear_bumper_pressed and (
            self._desired_linear < -0.005 or previous_rotation)
        if self._bumpers_valid and (front_blocked or rear_blocked):
            return
        self._desired_linear = float(message.linear.x)
        self._desired_angular = float(message.angular.z)

    def _reasons(self, now, drive_attempt, joy_linear, joy_angular):
        desired_linear = self._desired_linear
        desired_angular = self._desired_angular
        if drive_attempt:
            desired_linear = joy_linear
            desired_angular = joy_angular
        return feedback_blockers(
            now=now,
            scan_at=self._last_scan,
            odom_at=self._last_odom,
            battery_at=self._last_battery,
            battery_voltage=self._battery_voltage,
            base_state_at=self._last_base_state,
            hardware_state_valid=self._hardware_state_valid,
            fault_flags=self._fault_flags,
            estop_pressed=self._estop_pressed,
            bumpers_valid=self._bumpers_valid,
            front_bumper_pressed=self._front_bumper_pressed,
            rear_bumper_pressed=self._rear_bumper_pressed,
            desired_linear=desired_linear,
            desired_angular=desired_angular,
            motors_enabled=self._motors_enabled,
            dock_state_valid=self._dock_state_valid,
            dock_state=self._dock_state,
            drive_attempt=drive_attempt,
        )

    def _publish_feedback(self, intensity):
        message = JoyFeedback()
        message.type = JoyFeedback.TYPE_RUMBLE
        message.id = 0
        message.intensity = float(max(0.0, min(1.0, intensity)))
        self.feedback_pub.publish(message)

    def _tick(self):
        now = self._now()
        drive_attempt, joy_linear, joy_angular = joystick_drive_intent(
            now=now,
            joy_at=self._last_joy,
            axes=self._joy_axes,
            buttons=self._joy_buttons,
            timeout=self.joystick_timeout,
            deadzone=self.deadzone,
        )
        reasons = tuple(self._reasons(
            now, drive_attempt, joy_linear, joy_angular))
        if reasons != self._last_reasons:
            if reasons:
                self.get_logger().warn(
                    'Controller feedback blocker: ' + ', '.join(reasons))
            elif self._last_reasons:
                self.get_logger().info(
                    'Controller feedback blockers cleared; sending zero.')
            self._last_reasons = reasons

        for action, _duration in self._scheduler.update(
                now=now, reasons=reasons, drive_attempt=drive_attempt):
            if action == PUBLISH_ZERO:
                self._publish_feedback(0.0)
            else:
                self._publish_feedback(self.intensity)

    def stop_feedback(self):
        if not self._zero_sent_for_shutdown:
            self._publish_feedback(0.0)
            self._zero_sent_for_shutdown = True


def main(args=None):
    # Keep the ROS context valid until the explicit zero has been published.
    # rclpy's default SIGINT/SIGTERM handlers shut the context down before a
    # finally block can use the publisher.
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = ControllerFeedback()
    shutdown_requested = False

    def request_shutdown(_signum, _frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    try:
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_feedback()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
