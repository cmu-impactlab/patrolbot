"""Isolated loopback integration tests for the real Pi bridge node."""

import os
import socket
import threading
import time
from uuid import uuid4

from geometry_msgs.msg import Twist
from patrolbot_bridge.bridge_node import PatrolBotBridge
from patrolbot_interfaces.action import HardwareUndock
from patrolbot_interfaces.srv import ChargeRelease, MotorEnable
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


pytestmark = pytest.mark.skipif(
    os.environ.get('PATROLBOT_RUN_INTEGRATION') != '1',
    reason='set PATROLBOT_RUN_INTEGRATION=1 in an isolated ROS domain')


class FakeSbc:
    """Small protocol peer which never opens the real hardware port."""

    def __init__(self, advertise_undock=True):
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
        self.advertise_undock = advertise_undock
        # Stops the telemetry stream while leaving the socket open: the SBC
        # gone quiet, rather than the connection dropping.
        self.telemetry_paused = False
        # The undock scripts now run off the receive loop, so two threads write
        # to the same socket. sendall is not atomic, and an interleaved write
        # would split a protocol line down the middle.
        self._send_lock = threading.Lock()
        self._scripts = []
        # Dock observer state, mirrored into the AUX DOCK= section.
        self.dock_state = 'DOCKED_CONFIRMED'
        self.dock_phase = 0
        self.dock_phase_name = 'IDLE'
        self.undock_active = False
        self.profile_commissioned = True
        self.redock_inhibited = False
        # Scripted undock behaviour.
        self.undock_events = 3
        self.undock_should_succeed = True
        self.undock_stall = False
        self._active_undock = None
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
            try:
                features = 'UNDOCK' if self.advertise_undock else ''
                self._send(
                    connection,
                    f'HELLO:PROTOCOL=2:FEATURES={features}\n'.encode('ascii'))
            except OSError:
                pass
            buffer = ''
            next_telemetry = 0.0
            while self.running:
                now = time.monotonic()
                if now >= next_telemetry and not self.telemetry_paused:
                    try:
                        self._send(connection, self._telemetry())
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

    def _telemetry(self):
        with self._lock:
            charge_state = self.charge_state
            motors_enabled = self.motors_enabled
            estop_pressed = self.estop_pressed
            dock = (
                f'{self.dock_state},{self.dock_phase},{self.dock_phase_name},'
                f'{int(self.undock_active)},0,99.0,1,'
                f'{int(self.redock_inhibited)},0,'
                f'{int(self.profile_commissioned)}')
        return (
            'ODOM:0,0,0,0,0|LASER:1.0,1.0,1.0\n'
            f'AUX:SONAR=|BATT=24.0,80,{charge_state},25.0|'
            f'FLAGS=0,0,0,{int(motors_enabled)},{int(estop_pressed)}|'
            f'DOCK={dock}\n').encode('ascii')

    def _reply(self, connection, line):
        fields = line.split(':')
        if not fields:
            return
        verb = fields[0]
        if verb == 'CHARGE_RELEASE' and len(fields) == 2:
            self._reply_charge_release(connection, fields[1])
        elif verb == 'MOTOR_ENABLE' and len(fields) == 2:
            with self._lock:
                self.motors_enabled = True
            self._send(connection, (
                f'ACK:MOTOR_ENABLE:{fields[1]}:ACCEPTED\n'
                f'RESULT:MOTOR_ENABLE:{fields[1]}:OK:0:motors_enabled:0:1:0\n'
            ).encode('ascii'))
        elif verb == 'UNDOCK' and len(fields) == 6:
            # Scripted off the receive loop. Both undock scripts sleep between
            # frames, and running them inline froze this loop -- and so the
            # telemetry stream -- for the whole operation, which meant the
            # DOCK section carrying undock_active=1 was never sent while the
            # undock was active. The real SBC keeps reporting throughout.
            self._spawn(self._start_undock, connection, fields[1])
        elif verb == 'UNDOCK_CANCEL' and len(fields) == 2:
            self._spawn(self._cancel_undock, connection, fields[1])

    def _spawn(self, target, *args):
        thread = threading.Thread(target=target, args=args, daemon=True)
        with self._lock:
            self._scripts.append(thread)
        thread.start()

    def _send(self, connection, payload):
        """Serialise writes so two threads cannot split a protocol line."""
        with self._send_lock:
            connection.sendall(payload)

    def _reply_charge_release(self, connection, request_id):
        if request_id in self.malformed_ids:
            self._send(connection, b'RESULT:malformed\n')
            return
        with self._lock:
            self.charge_state = 0
            self.motors_enabled = False
        self._send(connection, (
            f'ACK:CHARGE_RELEASE:{request_id}:ACCEPTED\n'
            f'RESULT:CHARGE_RELEASE:{request_id}:OK:0:released:0:0:1\n'
        ).encode('ascii'))

    def _start_undock(self, connection, request_id):
        # Runs on its own thread now, so it can outlive the socket when a test
        # drops the connection or tears the harness down mid-undock. A closed
        # peer is a normal end to the script, not an error.
        try:
            self._send(
                connection,
                f'ACK:UNDOCK:{request_id}:ACCEPTED\n'.encode('ascii'))
            with self._lock:
                self._active_undock = request_id
                self.undock_active = True
                self.dock_state = 'DEPARTING'
            for index in range(self.undock_events):
                distance = 0.1 * (index + 1)
                self._send(connection, (
                    f'UNDOCK_EVENT:{request_id}:4:BACKING_OUT:{distance:.4f}:'
                    f'0.0000:0.0000:3:1:99.0000:0:DEPARTING\n'
                ).encode('ascii'))
                time.sleep(0.05)
        except OSError:
            return
        if self.undock_stall:
            # Never emits a terminal frame: exercises the bridge backstop.
            return
        self._finish_undock(connection, request_id, self.undock_should_succeed,
                            0 if self.undock_should_succeed else 11,
                            'undocked' if self.undock_should_succeed
                            else 'rear_path_blocked')

    def _cancel_undock(self, connection, target_id):
        self._send(
            connection,
            f'ACK:UNDOCK_CANCEL:{target_id}:ACCEPTED\n'.encode('ascii'))
        with self._lock:
            active = self._active_undock
        if active != target_id:
            return
        # Terminal only after the stop, exactly as the real SBC behaves.
        time.sleep(0.05)
        self._finish_undock(connection, target_id, False, 13, 'canceled')

    def _finish_undock(self, connection, request_id, success, code, message):
        with self._lock:
            self._active_undock = None
            self.undock_active = False
            self.dock_state = (
                'CLEAR_CONFIRMED' if success else 'FAULT_STOPPED')
            self.motors_enabled = bool(success)
            self.redock_inhibited = bool(success)
            phase = 6 if success else 7
            phase_name = 'HANDOFF_READY' if success else 'STOPPING'
        try:
            self._send(connection, (
                f'UNDOCK_RESULT:{request_id}:{"OK" if success else "ERROR"}:'
                f'{code}:{message}:{phase}:{phase_name}:0.5300:0.0010:0.0020:'
                f'3:{int(success)}:0:99.0000:1:'
                f'{"CLEAR_CONFIRMED" if success else "FAULT_STOPPED"}:'
                f'{int(success)}\n').encode('ascii'))
        except OSError:
            pass

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
        with self._lock:
            scripts = list(self._scripts)
        for script in scripts:
            script.join(timeout=2.0)


def wait_until(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def build_harness(fake):
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
    return bridge, client_node, executor, spin_thread


def teardown_harness(bridge, client_node, executor, spin_thread, fake):
    executor.shutdown(timeout_sec=2.0)
    bridge.destroy_node()
    client_node.destroy_node()
    spin_thread.join(timeout=2.0)
    rclpy.shutdown()
    fake.close()


@pytest.fixture
def harness():
    fake = FakeSbc()
    bridge, client_node, executor, spin_thread = build_harness(fake)
    yield fake, bridge, client_node, executor
    teardown_harness(bridge, client_node, executor, spin_thread, fake)


@pytest.fixture
def harness_without_undock():
    fake = FakeSbc(advertise_undock=False)
    bridge, client_node, executor, spin_thread = build_harness(fake)
    yield fake, bridge, client_node, executor
    teardown_harness(bridge, client_node, executor, spin_thread, fake)


def call_release(client_node, request_id):
    client = client_node.create_client(
        ChargeRelease, '/patrolbot/charge_release')
    assert client.wait_for_service(timeout_sec=1.0)
    request = ChargeRelease.Request()
    request.request_id = request_id
    future = client.call_async(request)
    assert wait_until(future.done, timeout=2.0)
    return future.result()


def send_undock(client_node, clearance=0.53, speed=0.10, duration=30.0,
                feedback=None, timeout=10.0):
    client = ActionClient(client_node, HardwareUndock, '/patrolbot/hardware_undock')
    assert client.wait_for_server(timeout_sec=2.0)
    goal = HardwareUndock.Goal()
    goal.clearance_distance = clearance
    goal.reverse_speed = speed
    goal.maximum_duration = duration
    goal.validation_mode = False
    send_future = client.send_goal_async(goal, feedback_callback=feedback)
    assert wait_until(send_future.done, timeout=3.0)
    handle = send_future.result()
    return client, handle


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


def test_motor_enable_is_refused_while_the_dock_state_says_docked(harness):
    fake, bridge, client_node, _executor = harness
    with fake._lock:
        fake.estop_pressed = False
        fake.dock_state = 'DOCKED_CONFIRMED'
    assert wait_until(lambda: bridge._dock_state == 'DOCKED_CONFIRMED')
    client = client_node.create_client(MotorEnable, '/patrolbot/motor_enable')
    assert client.wait_for_service(timeout_sec=1.0)
    request = MotorEnable.Request()
    request.request_id = str(uuid4())
    future = client.call_async(request)
    assert wait_until(future.done, timeout=2.0)
    response = future.result()
    assert not response.success
    assert response.code == MotorEnable.Response.PRECONDITION_FAILED
    # Refused before it ever reached the wire.
    assert not any(
        line.startswith('MOTOR_ENABLE:') for _session, line in fake.received)


def test_hardware_undock_reports_success_with_motors_left_enabled(harness):
    fake, _bridge, client_node, _executor = harness
    seen = []
    _client, handle = send_undock(
        client_node, feedback=lambda msg: seen.append(msg.feedback))
    assert handle.accepted
    result_future = handle.get_result_async()
    assert wait_until(result_future.done, timeout=10.0)
    result = result_future.result().result
    assert result.success
    assert result.code == HardwareUndock.Result.OK
    assert result.distance_traveled == pytest.approx(0.53)
    assert result.motors_enabled
    assert result.physically_clear
    assert result.final_dock_state == 'CLEAR_CONFIRMED'
    assert seen and seen[-1].phase_name == 'BACKING_OUT'
    assert all(not line.startswith('DRIVE:') for _session, line in fake.received)


def test_hardware_undock_suppresses_cmd_vel_while_active(harness):
    fake, bridge, client_node, _executor = harness
    with fake._lock:
        fake.undock_events = 8
    publisher = client_node.create_publisher(Twist, '/cmd_vel', 10)
    _client, handle = send_undock(client_node)
    assert handle.accepted
    assert wait_until(lambda: bridge._undock_active, timeout=3.0)
    command = Twist()
    command.linear.x = -0.05
    for _ in range(5):
        publisher.publish(command)
        time.sleep(0.02)
    assert all(not line.startswith('DRIVE:') for _session, line in fake.received)
    result_future = handle.get_result_async()
    assert wait_until(result_future.done, timeout=10.0)


def test_hardware_undock_failure_reports_disabled_motors(harness):
    fake, _bridge, client_node, _executor = harness
    with fake._lock:
        fake.undock_should_succeed = False
    _client, handle = send_undock(client_node)
    result_future = handle.get_result_async()
    assert wait_until(result_future.done, timeout=10.0)
    result = result_future.result().result
    assert not result.success
    assert result.code == HardwareUndock.Result.SENSOR_BLOCKED
    assert not result.motors_enabled
    assert not result.physically_clear
    assert result.final_dock_state == 'FAULT_STOPPED'


def test_hardware_undock_cancel_is_forwarded_and_terminal(harness):
    fake, _bridge, client_node, _executor = harness
    with fake._lock:
        fake.undock_events = 20
    _client, handle = send_undock(client_node)
    assert handle.accepted
    time.sleep(0.2)
    handle.cancel_goal_async()
    result_future = handle.get_result_async()
    assert wait_until(result_future.done, timeout=10.0)
    result = result_future.result().result
    assert not result.success
    assert result.code == HardwareUndock.Result.CANCELED
    assert any(
        line.startswith('UNDOCK_CANCEL:') for _session, line in fake.received)


def test_hardware_undock_aborts_when_the_session_disappears(harness):
    fake, bridge, client_node, _executor = harness
    with fake._lock:
        fake.undock_events = 40
    _client, handle = send_undock(client_node)
    assert handle.accepted
    assert wait_until(lambda: bridge._undock_request_id is not None, timeout=3.0)
    fake.drop_connection()
    result_future = handle.get_result_async()
    assert wait_until(result_future.done, timeout=10.0)
    result = result_future.result().result
    assert not result.success
    assert result.code == HardwareUndock.Result.SESSION_CHANGED


def test_hardware_undock_is_refused_by_a_server_without_the_feature(
        harness_without_undock):
    fake, _bridge, client_node, _executor = harness_without_undock
    client = ActionClient(
        client_node, HardwareUndock, '/patrolbot/hardware_undock')
    assert client.wait_for_server(timeout_sec=2.0)
    goal = HardwareUndock.Goal()
    goal.clearance_distance = 0.53
    goal.reverse_speed = 0.10
    goal.maximum_duration = 30.0
    send_future = client.send_goal_async(goal)
    assert wait_until(send_future.done, timeout=3.0)
    # Fail closed at admission rather than sending a command that will never be
    # answered and then waiting out the whole timeout.
    assert not send_future.result().accepted
    assert not any(
        line.startswith('UNDOCK:') for _session, line in fake.received)


def test_odom_tf_stops_rather_than_going_stale_when_the_link_dies(harness):
    """A dead SBC link must remove odom->base_link, not re-stamp the last pose.

    The TF timer runs at 50 Hz off its own clock and re-stamps whatever pose it
    last saw, so an SBC outage produced a transform that was always perfectly
    fresh by its header and increasingly wrong in fact. Every consumer that
    checks TF age -- the costmaps, the collision monitor, AMCL -- was told the
    robot's position was current. Dropping the transform makes them report a
    lookup failure instead, which is the failure they know how to handle.
    """
    fake, bridge, client_node, _executor = harness
    from tf2_msgs.msg import TFMessage

    seen = []
    client_node.create_subscription(
        TFMessage, '/tf',
        lambda msg: seen.extend(
            t for t in msg.transforms if t.child_frame_id == 'base_link'),
        10)
    assert wait_until(lambda: len(seen) > 0), 'no odom->base_link to begin with'

    # The SBC goes quiet with the socket still open. The last pose is now
    # frozen, but the TF timer keeps ticking off its own clock.
    fake.telemetry_paused = True
    timeout = bridge.get_parameter('odom_tf_timeout').value
    time.sleep(timeout + 0.3)
    settled = len(seen)
    time.sleep(0.5)   # 25 TF ticks at 50 Hz
    assert len(seen) == settled, 'kept broadcasting a stale pose'

    # It must resume when telemetry does, rather than latching off for good.
    fake.telemetry_paused = False
    assert wait_until(lambda: len(seen) > settled), 'never resumed'
