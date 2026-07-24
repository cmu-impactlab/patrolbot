"""Gimbal command arbiter.

Subscribes to gimbal/input/joy and gimbal/input/web, publishes gimbal/cmd.
Precedence lives in MuxCore, which is tested without ROS.

Separate from twist_mux by design: this decides where the camera looks and
shares nothing with the commissioned velocity arbiter.
"""

import rclpy
from patrolbot_interfaces.msg import GimbalCommand
from rclpy.node import Node

from .mux_core import SOURCE_JOY, SOURCE_WEB, MuxCore


class GimbalMux(Node):

    def __init__(self):
        super().__init__('gimbal_mux')

        self.declare_parameter('input_timeout', 0.5)
        self.declare_parameter('rate_hz', 30.0)

        self.core = MuxCore(
            clock=lambda: self.get_clock().now().nanoseconds * 1e-9,
            input_timeout_s=float(self.get_parameter('input_timeout').value),
        )

        self.create_subscription(
            GimbalCommand, 'gimbal/input/joy',
            lambda m: self._on_input(SOURCE_JOY, m), 10)
        self.create_subscription(
            GimbalCommand, 'gimbal/input/web',
            lambda m: self._on_input(SOURCE_WEB, m), 10)

        self.pub = self.create_publisher(GimbalCommand, 'gimbal/cmd', 10)
        self.create_timer(
            1.0 / float(self.get_parameter('rate_hz').value), self._tick)

        self._last_active = None
        self.get_logger().info(
            'gimbal mux ready: joystick outranks dashboard')

    def _on_input(self, source, msg: GimbalCommand):
        self.core.submit(source, {
            'mode': int(msg.mode),
            'pan': float(msg.pan),
            'tilt': float(msg.tilt),
            'roll': float(msg.roll),
            'valid': bool(msg.valid),
        })

    def _tick(self):
        chosen = self.core.select()
        if chosen is None:
            return          # nothing has ever spoken; stay quiet

        active = self.core.active_source()
        if active != self._last_active:
            self.get_logger().info(f'gimbal control: {active or "idle"}')
            self._last_active = active

        command = chosen.command
        msg = GimbalCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = int(command.get('mode', GimbalCommand.MODE_HOLD))
        if not command.get('valid', False):
            msg.mode = GimbalCommand.MODE_HOLD
        msg.pan = float(command.get('pan', 0.0))
        msg.tilt = float(command.get('tilt', 0.0))
        msg.roll = float(command.get('roll', 0.0))
        msg.valid = bool(command.get('valid', False))
        msg.source = chosen.source
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalMux()
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
