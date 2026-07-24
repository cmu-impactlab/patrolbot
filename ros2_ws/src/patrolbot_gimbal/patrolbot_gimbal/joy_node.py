"""Joystick to gimbal command.

Subscribes to /joy and publishes gimbal/input/joy. It reads the same physical
controller as the drive teleop but claims only inputs that teleop leaves free,
and it never publishes anything the velocity chain consumes.

All the arithmetic lives in gimbal_policy, which is tested without ROS.
"""

import rclpy
from patrolbot_interfaces.msg import GimbalCommand
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

from . import gimbal_policy as policy


class GimbalJoy(Node):

    def __init__(self):
        super().__init__('gimbal_joy')

        self.declare_parameter('max_pan_deg_s', 45.0)
        self.declare_parameter('max_tilt_deg_s', 45.0)
        self.declare_parameter('max_roll_deg_s', 30.0)
        self.declare_parameter('invert_pan', False)
        self.declare_parameter('invert_tilt', False)
        self.declare_parameter('invert_roll', False)
        self.declare_parameter('deadzone', 0.12)
        self.declare_parameter('rate_hz', 30.0)

        g = lambda k: self.get_parameter(k).value        # noqa: E731

        self.cfg = policy.GimbalConfig(
            max_pan_deg_s=g('max_pan_deg_s'),
            max_tilt_deg_s=g('max_tilt_deg_s'),
            max_roll_deg_s=g('max_roll_deg_s'),
            invert_pan=g('invert_pan'),
            invert_tilt=g('invert_tilt'),
            invert_roll=g('invert_roll'),
            deadzone=g('deadzone'),
        )

        self._axes = []
        self._buttons = []
        self._prev_buttons = []
        self._last_joy = None
        self._was_valid = False

        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.pub = self.create_publisher(GimbalCommand, 'gimbal/input/joy', 10)
        self._home_client = self.create_client(Trigger, 'gimbal/home')

        self.create_timer(1.0 / float(g('rate_hz')), self._tick)

        self.get_logger().info(
            'gimbal joy ready: hold LB and use the right stick to look, '
            'triggers to roll, Back to recenter. RB still drives the robot.')

    def _on_joy(self, msg: Joy):
        self._axes = list(msg.axes)
        self._buttons = list(msg.buttons)
        self._last_joy = self.get_clock().now().nanoseconds * 1e-9

        if policy.rising_edge(
                policy.button_pressed(self._buttons, policy.BTN_HOME),
                policy.button_pressed(self._prev_buttons, policy.BTN_HOME)):
            self._request_home()
        self._prev_buttons = list(self._buttons)

    def _request_home(self):
        if self._home_client.service_is_ready():
            self._home_client.call_async(Trigger.Request())
        else:
            self.get_logger().warning('gimbal/home unavailable; ignoring')

    def _joy_age(self):
        if self._last_joy is None:
            # Never heard from the controller: treat as maximally stale so the
            # deadman cannot be considered held.
            return policy.JOY_TIMEOUT_S + 1.0
        return (self.get_clock().now().nanoseconds * 1e-9) - self._last_joy

    def _tick(self):
        rates = policy.joy_to_command(
            self._axes, self._buttons, self.cfg, self._joy_age())

        # Publish continuously while active, and keep publishing an explicit
        # invalid command after release. Going silent would be indistinguishable
        # from this node dying, and the mux must be able to tell those apart.
        if not rates.valid and not self._was_valid:
            self._publish(rates)
            return

        self._publish(rates)
        self._was_valid = rates.valid

    def _publish(self, rates):
        msg = GimbalCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = (GimbalCommand.MODE_RATE if rates.valid
                    else GimbalCommand.MODE_HOLD)
        msg.pan = float(rates.pan)
        msg.tilt = float(rates.tilt)
        msg.roll = float(rates.roll)
        msg.valid = bool(rates.valid)
        msg.source = 'joy'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
