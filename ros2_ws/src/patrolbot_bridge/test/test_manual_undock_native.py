"""Actual DDS + loopback SBC transport, explicitly with no AMCL/Nav2 nodes.

The SBC here is scripted: this verifies transport/ownership, not physical
release/turn accuracy or a native ARIA server build. Run only in a network-none
container with PATROLBOT_RUN_MANUAL_UNDOCK_NATIVE=1.
"""

import os
import threading
import time

from patrolbot_bridge.bridge_node import PatrolBotBridge
from patrolbot_bridge.epoch_recovery import EpochRecovery
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy

from test_fake_sbc_integration import FakeSbc, wait_until

pytestmark = pytest.mark.skipif(
    os.environ.get('PATROLBOT_RUN_MANUAL_UNDOCK_NATIVE') != '1',
    reason='isolated native manual-undock test requires explicit opt-in')


class ManualSbc(FakeSbc):
    def _send(self, connection, payload):
        if payload.startswith(b'HELLO:'):
            payload = payload.replace(b'FEATURES=UNDOCK,', b'FEATURES=UNDOCK,MANUAL_UNDOCK_V1,')
        super()._send(connection, payload)

    def _reply(self, connection, line):
        if line.startswith('MANUAL_UNDOCK:'):
            # Existing scripted fake stays stationary; the C++ fake-base tests
            # separately exercise actual reverse+turn controller calculations.
            line = line.replace('MANUAL_UNDOCK:', 'UNDOCK:', 1)
        super()._reply(connection, line)


def test_start_dispatches_once_without_nav2_or_localization_reset_ack(tmp_path, monkeypatch):
    monkeypatch.setenv('PATROLBOT_STATE_DIR', str(tmp_path))
    fake = ManualSbc()
    fake.estop_pressed = False
    rclpy.init()
    bridge = None
    joy_node = Node('joy_node')
    pub = joy_node.create_publisher(Joy, '/joy', 10)
    control = {'start': False}

    def send():
        msg = Joy()
        msg.header.stamp = joy_node.get_clock().now().to_msg()
        msg.axes = [0., 0., 1., 0., 0., 1., 0., 0.]
        msg.buttons = [0] * 11
        msg.buttons[7] = int(control['start'])
        pub.publish(msg)
    joy_node.create_timer(.05, send)
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        bridge = PatrolBotBridge(parameter_overrides=[
            Parameter('server_ip', value='127.0.0.1'),
            Parameter('server_port', value=fake.port),
            Parameter('receive_timeout', value=.5),
            Parameter('connect_timeout', value=.2),
            Parameter('reconnect_delay', value=.05),
        ])
        executor.add_node(bridge)
        executor.add_node(joy_node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        assert wait_until(lambda: bridge._manual_undock.sensors_ready(), timeout=5.)
        assert bridge._epoch.recovery.stage == EpochRecovery.WAIT_RESET
        assert not bridge._pose_received  # Navigation publication stays gated.
        assert wait_until(lambda: bridge._manual_undock.gesture.armed, timeout=5.)
        control['start'] = True
        assert wait_until(lambda: any(
            line.startswith('MANUAL_UNDOCK:') for _, line in fake.received), timeout=4.)
        assert wait_until(lambda: not bridge._manual_undock.busy, timeout=5.)
        time.sleep(.3)
        assert sum(line.startswith('MANUAL_UNDOCK:') for _, line in fake.received) == 1
        assert not any(line.startswith('DRIVE:') and line != 'DRIVE:0.0:0.0'
                       for _, line in fake.received)
        assert bridge._epoch.recovery.stage == EpochRecovery.WAIT_RESET
        assert not bridge._epoch.commands_allowed
        assert not bridge._epoch.undock_hold
    finally:
        control['start'] = False
        executor.shutdown(timeout_sec=4.)
        if bridge is not None:
            bridge.destroy_node()
        joy_node.destroy_node()
        rclpy.try_shutdown()
        fake.close()
