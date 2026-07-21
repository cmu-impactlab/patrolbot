"""Isolated loopback integration tests for the real Pi bridge node."""

import os
import socket
import threading
import time
from uuid import uuid4

from geometry_msgs.msg import Twist
from patrolbot_bridge.bridge_node import PatrolBotBridge
from patrolbot_interfaces.srv import ChargeRelease
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


pytestmark = pytest.mark.skipif(
    os.environ.get('PATROLBOT_RUN_INTEGRATION') != '1',
    reason='set PATROLBOT_RUN_INTEGRATION=1 in an isolated ROS domain')


class FakeSbc:
    """Small protocol peer which never opens the real hardware port."""

    def __init__(self):
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(('127.0.0.1', 0))
        self.listener.listen(4)
        self.listener.settimeout(0.1)
        self.port = self.listener.getsockname()[1]
        self.running = True
        self.charge_state = 2
        self.motors_enabled = False
        self.estop_pressed = True
        self.connections = 0
        self.received = []
        self.malformed_ids = set()
        self._connection = None
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while self.running:
            try:
                connection, _address = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            connection.settimeout(0.02)
            with self._lock:
                self.connections += 1
                connection_number = self.connections
                self._connection = connection
            buffer = ''
            next_telemetry = 0.0
            while self.running:
                now = time.monotonic()
                if now >= next_telemetry:
                    with self._lock:
                        charge_state = self.charge_state
                        motors_enabled = self.motors_enabled
                        estop_pressed = self.estop_pressed
                    packet = (
                        'ODOM:0,0,0,0,0|LASER:1.0,1.0,1.0\n'
                        f'AUX:SONAR=|BATT=24.0,80,{charge_state},25.0|'
                        f'FLAGS=0,0,0,{int(motors_enabled)},'
                        f'{int(estop_pressed)}\n')
                    try:
                        connection.sendall(packet.encode('ascii'))
                    except OSError:
                        break
                    next_telemetry = now + 0.05
                try:
                    data = connection.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not data:
                    break
                buffer += data.decode('ascii')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    with self._lock:
                        self.received.append((connection_number, line))
                    self._reply(connection, line)
            with self._lock:
                if self._connection is connection:
                    self._connection = None
            try:
                connection.close()
            except OSError:
                pass

    def _reply(self, connection, line):
        fields = line.split(':', 1)
        if len(fields) != 2 or fields[0] != 'CHARGE_RELEASE':
            return
        request_id = fields[1]
        if request_id in self.malformed_ids:
            connection.sendall(b'RESULT:malformed\n')
            return
        with self._lock:
            self.charge_state = 0
            self.motors_enabled = False
        response = (
            f'ACK:CHARGE_RELEASE:{request_id}:ACCEPTED\n'
            f'RESULT:CHARGE_RELEASE:{request_id}:OK:0:released:0:0:1\n')
        connection.sendall(response.encode('ascii'))

    def drop_connection(self):
        with self._lock:
            connection = self._connection
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                connection.close()
            except OSError:
                pass

    def close(self):
        self.running = False
        self.drop_connection()
        try:
            self.listener.close()
        except OSError:
            pass
        self._thread.join(timeout=2.0)


def wait_until(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def harness():
    fake = FakeSbc()
    rclpy.init()
    bridge = PatrolBotBridge(parameter_overrides=[
        Parameter('server_ip', value='127.0.0.1'),
        Parameter('server_port', value=fake.port),
        Parameter('connect_timeout', value=0.2),
        Parameter('receive_timeout', value=0.3),
        Parameter('reconnect_delay', value=0.05),
        Parameter('control_timeout', value=0.4),
    ])
    client_node = Node('fake_sbc_integration_client')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    assert wait_until(lambda: bridge._link_connected)
    assert wait_until(lambda: bridge._pose_received)
    assert wait_until(lambda: bridge._charge_state_at > 0 and bridge._flags_at > 0)
    yield fake, bridge, client_node, executor
    executor.shutdown(timeout_sec=2.0)
    bridge.destroy_node()
    client_node.destroy_node()
    spin_thread.join(timeout=2.0)
    rclpy.shutdown()
    fake.close()


def call_release(client_node, request_id):
    client = client_node.create_client(
        ChargeRelease, '/patrolbot/charge_release')
    assert client.wait_for_service(timeout_sec=1.0)
    request = ChargeRelease.Request()
    request.request_id = request_id
    future = client.call_async(request)
    assert wait_until(future.done, timeout=2.0)
    return future.result()


def test_release_round_trip_and_duplicate_never_emit_velocity(harness):
    fake, bridge, client_node, _executor = harness
    with fake._lock:
        # Reproduce manual dock charging observed on the real PatrolBot:
        # positive charge state with motors still reported enabled.
        fake.motors_enabled = True
        fake.estop_pressed = False
    assert wait_until(lambda: bridge._motors_enabled)
    request_id = str(uuid4())
    first = call_release(client_node, request_id)
    second = call_release(client_node, request_id)
    assert first.success and first.charge_state == 0
    assert second.success and second.charge_state == 0
    assert all(not line.startswith('DRIVE:') for _session, line in fake.received)
    releases = [line for _session, line in fake.received
                if line == f'CHARGE_RELEASE:{request_id}']
    assert len(releases) == 2


def test_malformed_terminal_reply_times_out_fail_closed(harness):
    fake, _bridge, client_node, _executor = harness
    request_id = str(uuid4())
    fake.malformed_ids.add(request_id)
    response = call_release(client_node, request_id)
    assert not response.success
    assert response.code == ChargeRelease.Response.TIMEOUT
    assert all(not line.startswith('DRIVE:') for _session, line in fake.received)


def test_velocity_is_not_replayed_after_hardware_reconnect(harness):
    fake, bridge, client_node, _executor = harness
    publisher = client_node.create_publisher(Twist, '/cmd_vel', 10)
    command = Twist()
    command.linear.x = 0.04
    publisher.publish(command)
    assert wait_until(
        lambda: any(line.startswith('DRIVE:') for _session, line in fake.received))
    first_generation = bridge._session_generation
    fake.drop_connection()
    assert wait_until(lambda: bridge._session_generation > first_generation)
    second_connection = fake.connections
    time.sleep(0.25)
    assert not any(
        session == second_connection and line.startswith('DRIVE:')
        for session, line in fake.received)
