"""Gimbal driver node: the only thing in PatrolBot that opens the gimbal port.

Deliberately thin. Every decision lives in DriverCore and every byte in
sbgc_protocol, both of which are tested without ROS. What remains here is
plumbing: parameters, timers, topics, and reconnection.

This node publishes nothing that can reach the robot velocity chain.
"""

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from patrolbot_interfaces.msg import GimbalCommand, GimbalState
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from . import gimbal_policy as policy
from . import sbgc_protocol as protocol
from .driver_core import DriverCore
from .sbgc_transport import SbgcTransport


class GimbalDriver(Node):

    def __init__(self):
        super().__init__('gimbal_driver')

        self.declare_parameter('port', '/dev/patrolbot_gimbal')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('state_rate_hz', 10.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('reconnect_delay', 2.0)

        self.declare_parameter('max_pan_deg_s', 45.0)
        self.declare_parameter('max_tilt_deg_s', 45.0)
        self.declare_parameter('max_roll_deg_s', 30.0)
        self.declare_parameter('pan_min_deg', -170.0)
        self.declare_parameter('pan_max_deg', 170.0)
        self.declare_parameter('tilt_min_deg', -90.0)
        self.declare_parameter('tilt_max_deg', 40.0)
        self.declare_parameter('roll_min_deg', -45.0)
        self.declare_parameter('roll_max_deg', 45.0)
        self.declare_parameter('invert_pan', False)
        self.declare_parameter('invert_tilt', False)
        self.declare_parameter('invert_roll', False)

        g = lambda k: self.get_parameter(k).value        # noqa: E731

        cfg = policy.GimbalConfig(
            max_pan_deg_s=g('max_pan_deg_s'),
            max_tilt_deg_s=g('max_tilt_deg_s'),
            max_roll_deg_s=g('max_roll_deg_s'),
            invert_pan=g('invert_pan'),
            invert_tilt=g('invert_tilt'),
            invert_roll=g('invert_roll'),
            pan_min_deg=g('pan_min_deg'), pan_max_deg=g('pan_max_deg'),
            tilt_min_deg=g('tilt_min_deg'), tilt_max_deg=g('tilt_max_deg'),
            roll_min_deg=g('roll_min_deg'), roll_max_deg=g('roll_max_deg'),
        )

        self._port_name = g('port')
        self._baud = int(g('baud'))
        self._reconnect_delay = float(g('reconnect_delay'))
        self._next_reconnect = 0.0

        self.core = DriverCore(
            cfg=cfg,
            clock=lambda: self.get_clock().now().nanoseconds * 1e-9,
            command_timeout_s=float(g('command_timeout')),
        )
        self.link = SbgcTransport()

        # Subscribes to the arbitrated output of the gimbal mux only.
        self.create_subscription(
            GimbalCommand, 'gimbal/cmd', self._on_command, 10)
        self.state_pub = self.create_publisher(GimbalState, 'gimbal/state', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.create_service(SetBool, 'gimbal/set_motors', self._srv_set_motors)
        self.create_service(Trigger, 'gimbal/home', self._srv_home)
        self.create_service(Trigger, 'gimbal/level', self._srv_level)

        self.create_timer(1.0 / float(g('rate_hz')), self._control_tick)
        self.create_timer(1.0 / float(g('state_rate_hz')), self._state_tick)

        self._try_connect()
        self.get_logger().info(
            f'gimbal driver ready on {self._port_name} @ {self._baud}; '
            'holds until a valid command arrives')

    # ------------------------------------------------------------ link --

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _try_connect(self):
        try:
            self.link.open(self._port_name, self._baud)
            self.core.set_connected(True)
            self.get_logger().info(f'gimbal connected on {self._port_name}')
        except IOError as exc:
            self.core.set_connected(False, str(exc))
            self._next_reconnect = self._now() + self._reconnect_delay

    # -------------------------------------------------------- callbacks --

    def _on_command(self, msg: GimbalCommand):
        if not msg.valid or msg.mode == GimbalCommand.MODE_HOLD:
            self.core.submit_invalid()
        elif msg.mode == GimbalCommand.MODE_RATE:
            self.core.submit_rate(msg.pan, msg.tilt, msg.roll)
        elif msg.mode == GimbalCommand.MODE_ANGLE:
            self.core.submit_angle(msg.pan, msg.tilt, msg.roll)
        else:
            # An unrecognised mode is not a reason to keep moving.
            self.core.submit_invalid()

    def _control_tick(self):
        if not self.link.connected:
            if self._now() >= self._next_reconnect:
                self._try_connect()
            return

        payload = self.core.tick()
        try:
            self.link.send(protocol.CMD_CONTROL, payload)
            self.link.send(protocol.CMD_GET_ANGLES)
        except IOError as exc:
            self.core.set_connected(False, str(exc))
            self.get_logger().warning(f'gimbal link lost: {exc}')
            self._next_reconnect = self._now() + self._reconnect_delay
            return

        for command_id, body in self.link.poll(timeout_s=0.0):
            self.core.on_frame(command_id, body)

        if not self.link.connected:
            self.core.set_connected(False, self.link.last_error)
            self._next_reconnect = self._now() + self._reconnect_delay

    # ----------------------------------------------------------- publish --

    def _state_tick(self):
        snap = self.core.state()
        snap.connected = self.link.connected

        msg = GimbalState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gimbal_link'
        msg.connected = snap.connected
        msg.motors_on = snap.motors_on
        msg.imu_pan, msg.imu_tilt, msg.imu_roll = (
            snap.imu_pan, snap.imu_tilt, snap.imu_roll)
        msg.target_pan, msg.target_tilt, msg.target_roll = (
            snap.target_pan, snap.target_tilt, snap.target_roll)
        msg.pan_min, msg.pan_max = snap.pan_min, snap.pan_max
        msg.tilt_min, msg.tilt_max = snap.tilt_min, snap.tilt_max
        msg.roll_min, msg.roll_max = snap.roll_min, snap.roll_max
        msg.command_timeout = snap.command_timeout
        msg.fault = snap.fault
        self.state_pub.publish(msg)

        self._publish_diagnostics(snap)

    def _publish_diagnostics(self, snap):
        status = DiagnosticStatus()
        status.name = 'patrolbot/gimbal'
        status.hardware_id = 'simplebgc32'

        if not snap.connected:
            status.level = DiagnosticStatus.ERROR
            status.message = snap.fault or 'gimbal not connected'
        elif snap.fault:
            status.level = DiagnosticStatus.WARN
            status.message = snap.fault
        elif snap.command_timeout:
            status.level = DiagnosticStatus.OK
            status.message = 'holding (no active command)'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'active'

        status.values = [
            KeyValue(key='connected', value=str(snap.connected)),
            KeyValue(key='motors_on', value=str(snap.motors_on)),
            KeyValue(key='command_timeout', value=str(snap.command_timeout)),
            KeyValue(key='imu_pan', value=f'{snap.imu_pan:.2f}'),
            KeyValue(key='imu_tilt', value=f'{snap.imu_tilt:.2f}'),
            KeyValue(key='imu_roll', value=f'{snap.imu_roll:.2f}'),
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diag_pub.publish(array)

    # ---------------------------------------------------------- services --

    def _send_or_fail(self, command_id, payload=b''):
        try:
            self.link.send(command_id, payload)
            return True, ''
        except IOError as exc:
            return False, str(exc)

    def _srv_set_motors(self, request, response):
        command = (protocol.CMD_MOTORS_ON if request.data
                   else protocol.CMD_MOTORS_OFF)
        ok, err = self._send_or_fail(command)
        if ok:
            self.core.set_motors(request.data)
        response.success = ok
        response.message = 'ok' if ok else err
        return response

    def _srv_home(self, request, response):
        ok, err = self._send_or_fail(
            protocol.CMD_CONTROL, protocol.build_home_payload())
        if ok:
            self.core.set_position(0.0, 0.0, 0.0)
        response.success = ok
        response.message = 'ok' if ok else err
        return response

    def _srv_level(self, request, response):
        ok, err = self._send_or_fail(
            protocol.CMD_CONTROL, protocol.build_level_payload())
        if ok:
            self.core.set_position(0.0, 0.0, 0.0)
        response.success = ok
        response.message = 'ok' if ok else err
        return response

    def shutdown(self):
        """Stop the gimbal before going away."""
        try:
            if self.link.connected:
                self.link.send(protocol.CMD_CONTROL, self.core.hold_payload())
        except IOError:
            pass
        self.link.close()


def main(args=None):
    rclpy.init(args=args)
    node = GimbalDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
