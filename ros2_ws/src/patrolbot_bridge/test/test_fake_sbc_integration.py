"""Isolated loopback integration tests for the real Pi bridge node."""

import math
import os
import socket
import threading
import time
from uuid import uuid4

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from patrolbot_bridge.bridge_node import PatrolBotBridge
from patrolbot_interfaces.action import HardwareUndock
from patrolbot_interfaces.srv import ChargeRelease, MotorEnable, SetLidarPower
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

pytestmark = pytest.mark.skipif(
    os.environ.get('PATROLBOT_RUN_INTEGRATION') != '1',
    reason='set PATROLBOT_RUN_INTEGRATION=1 in an isolated ROS domain',
)


class FakeSbc:
    """Small protocol peer which never opens the real hardware port."""

    def __init__(self, advertise_undock=True):
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(('127.0.0.1', 0))
        self.listener.listen(4)
        self.listener.settimeout(0.1)
        self.port = self.listener.getsockname()[1]
        self.odom_epoch = str(uuid4())
        self.running = True
        self.charge_state = 2
        self.motors_enabled = False
        self.estop_pressed = True
        self.connections = 0
        self.received = []
        self.malformed_ids = set()
        self.advertise_undock = advertise_undock
        self.lidar_state = 'READY'
        self.lidar_supported = True
        self.lidar_scan_age_ms = 0
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
        # Reported ARIA pose. Settable so a test can drive the fake robot and
        # watch what that does to the covariance on the wire.
        self.odom_pose = (0.0, 0.0, 0.0)
        # Body of the LASER: section, verbatim. Settable so a test can put a
        # field on the wire that does not parse as a number and watch what the
        # bridge does with the beams either side of it.
        self.laser_field = '1.0,1.0,1.0'
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
                features = 'UNDOCK,LIDAR_POWER' if self.advertise_undock else 'LIDAR_POWER'
                self._send(
                    connection,
                    f'HELLO:PROTOCOL=2:FEATURES={features}:ODOM_EPOCH={self.odom_epoch}\n'.encode(
                        'ascii'
                    ),
                )
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
                f'{int(self.profile_commissioned)}'
            )
            odom_x, odom_y, odom_th = self.odom_pose
            laser_field = self.laser_field if self.lidar_state == 'READY' else ''
        return (
            f'ODOM:{odom_x},{odom_y},{odom_th},0,0|LASER:{laser_field}'
            f'|ODOM_EPOCH:{self.odom_epoch}\n'
            f'AUX:SONAR=|BATT=24.0,80,{charge_state},25.0|'
            f'FLAGS=0,0,0,{int(motors_enabled)},{int(estop_pressed)}|'
            f'DOCK={dock}|LIDAR={self.lidar_state},{int(self.lidar_supported)},'
            f'{self.lidar_scan_age_ms}\n'
        ).encode('ascii')

    def _reply(self, connection, line):
        fields = line.split(':')
        if not fields:
            return
        verb = fields[0]
        if verb in ('LIDAR_ON', 'LIDAR_OFF') and len(fields) == 2:
            with self._lock:
                self.lidar_state = 'STARTING' if verb == 'LIDAR_ON' else 'OFF'
                self.lidar_scan_age_ms = -1
            self._send(
                connection,
                (
                    f'ACK:{verb}:{fields[1]}:ACCEPTED\n'
                    f'RESULT:{verb}:{fields[1]}:OK:0:commanded:2:0:1\n'
                ).encode('ascii'),
            )
            if verb == 'LIDAR_ON':
                self._spawn(self._fresh_lidar)
        elif verb == 'CHARGE_RELEASE' and len(fields) == 2:
            self._reply_charge_release(connection, fields[1])
        elif verb == 'MOTOR_ENABLE' and len(fields) == 2:
            with self._lock:
                self.motors_enabled = True
            self._send(
                connection,
                (
                    f'ACK:MOTOR_ENABLE:{fields[1]}:ACCEPTED\n'
                    f'RESULT:MOTOR_ENABLE:{fields[1]}:OK:0:motors_enabled:0:1:0\n'
                ).encode('ascii'),
            )
        elif verb == 'UNDOCK' and len(fields) == 6:
            # Scripted off the receive loop. Both undock scripts sleep between
            # frames, and running them inline froze this loop -- and so the
            # telemetry stream -- for the whole operation, which meant the
            # DOCK section carrying undock_active=1 was never sent while the
            # undock was active. The real SBC keeps reporting throughout.
            self._spawn(self._start_undock, connection, fields[1])
        elif verb == 'UNDOCK_CANCEL' and len(fields) == 2:
            self._spawn(self._cancel_undock, connection, fields[1])

    def _fresh_lidar(self):
        time.sleep(0.15)
        with self._lock:
            self.lidar_state = 'READY'
            self.lidar_scan_age_ms = 0

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
        self._send(
            connection,
            (
                f'ACK:CHARGE_RELEASE:{request_id}:ACCEPTED\n'
                f'RESULT:CHARGE_RELEASE:{request_id}:OK:0:released:0:0:1\n'
            ).encode('ascii'),
        )

    def _start_undock(self, connection, request_id):
        # Runs on its own thread now, so it can outlive the socket when a test
        # drops the connection or tears the harness down mid-undock. A closed
        # peer is a normal end to the script, not an error.
        try:
            self._send(connection, f'ACK:UNDOCK:{request_id}:ACCEPTED\n'.encode('ascii'))
            with self._lock:
                self._active_undock = request_id
                self.undock_active = True
                self.dock_state = 'DEPARTING'
            for index in range(self.undock_events):
                distance = 0.1 * (index + 1)
                # Checked and sent under the same lock. _finish_undock clears
                # the active request while holding it and only then writes the
                # terminal frame, so a cancel can neither slip a terminal frame
                # between this check and this send, nor lose the race and let
                # an event trail the terminal. The terminal is always last.
                with self._lock:
                    if self._active_undock != request_id:
                        return
                    self._send(
                        connection,
                        (
                            f'UNDOCK_EVENT:{request_id}:4:BACKING_OUT:'
                            f'{distance:.4f}:'
                            f'0.0000:0.0000:3:1:99.0000:0:DEPARTING\n'
                        ).encode('ascii'),
                    )
                time.sleep(0.05)
        except OSError:
            return
        if self.undock_stall:
            # Never emits a terminal frame: exercises the bridge backstop.
            return
        self._finish_undock(
            connection,
            request_id,
            self.undock_should_succeed,
            0 if self.undock_should_succeed else 11,
            'undocked' if self.undock_should_succeed else 'rear_path_blocked',
        )

    def _cancel_undock(self, connection, target_id):
        self._send(connection, f'ACK:UNDOCK_CANCEL:{target_id}:ACCEPTED\n'.encode('ascii'))
        with self._lock:
            active = self._active_undock
        if active != target_id:
            return
            # Terminal only after the stop, exactly as the real SBC behaves.
        time.sleep(0.05)
        self._finish_undock(connection, target_id, False, 13, 'canceled')

    def _finish_undock(self, connection, request_id, success, code, message):
        with self._lock:
            if self._active_undock != request_id:
                # Exactly one terminal frame per request, as the real SBC
                # emits. The event script and a cancel run on separate threads
                # and would otherwise both report an outcome for the same id --
                # and disagree about it.
                return
            self._active_undock = None
            self.undock_active = False
            self.dock_state = 'CLEAR_CONFIRMED' if success else 'FAULT_STOPPED'
            self.motors_enabled = bool(success)
            self.redock_inhibited = bool(success)
            phase = 6 if success else 7
            phase_name = 'HANDOFF_READY' if success else 'STOPPING'
        try:
            self._send(
                connection,
                (
                    f"UNDOCK_RESULT:{request_id}:{'OK' if success else 'ERROR'}:"
                    f'{code}:{message}:{phase}:{phase_name}:0.5300:0.0010:0.0020:'
                    f'3:{int(success)}:0:99.0000:1:'
                    f"{'CLEAR_CONFIRMED' if success else 'FAULT_STOPPED'}:"
                    f'{int(success)}\n'
                ).encode('ascii'),
            )
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


class FakeAmcl(Node):
    """Lifecycle peer for bridge integration; not physical convergence proof."""

    def __init__(self):
        super().__init__('amcl')
        self.declare_parameter('set_initial_pose', True)
        self.declare_parameter('always_reset_initial_pose', False)
        self.state = 3
        self.transitions = []
        self.navigation_transitions = []
        self.create_service(
            ManageLifecycleNodes,
            '/lifecycle_manager_navigation/manage_nodes',
            self.manage_navigation,
        )
        self.create_service(
            Trigger, '/lifecycle_manager_localization/is_active', self.localization_active
        )
        self.create_service(GetState, '/amcl/get_state', self.get_state)
        self.create_service(Trigger, '/amcl/initial_pose_tf_ready', self.tf_ready)
        self.create_service(ChangeState, '/amcl/change_state', self.change_state)
        self.create_service(GetMap, '/map_server/map', self.map_readback)
        self.create_service(Trigger, '/amcl/map_consumed', self.map_consumed)
        self.create_service(Trigger, '/bt_navigator/navigation_idle', self.localization_active)
        self.ack = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl/accepted_initial_pose', 10
        )
        self.pose = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.seed, 10)

    def map_readback(self, _request, response):
        response.map.header.frame_id = 'map'
        response.map.info.width = 3192
        response.map.info.height = 2205
        response.map.info.resolution = 0.075
        response.map.info.origin.position.x = -1.0
        response.map.info.origin.position.y = -1.0
        response.map.info.origin.orientation.w = 1.0
        response.map.info.map_load_time.sec = 1
        return response

    def map_consumed(self, _request, response):
        response.success = self.state == 3
        response.message = '1:0:3192:2205'
        return response

    def tf_ready(self, _request, response):
        response.success = self.state == 3
        return response

    def localization_active(self, _request, response):
        response.success = True
        return response

    def manage_navigation(self, request, response):
        self.navigation_transitions.append(request.command)
        response.success = True
        return response

    def get_state(self, _request, response):
        response.current_state.id = self.state
        return response

    def change_state(self, request, response):
        self.transitions.append(request.transition.id)
        self.state = {4: 2, 2: 1, 1: 2, 3: 3}[request.transition.id]
        response.success = True
        return response

    def seed(self, message):
        if self.state != 3:
            return
        self.ack.publish(message)
        message.header.stamp = self.get_clock().now().to_msg()
        self.pose.publish(message)


def seed_harness(bridge, client_node):
    assert wait_until(lambda: bridge._epoch.seed_tf_ready)
    publisher = client_node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    assert wait_until(lambda: publisher.get_subscription_count() >= 2)
    message = PoseWithCovarianceStamped()
    message.header.frame_id = 'map'
    message.header.stamp = client_node.get_clock().now().to_msg()
    message.pose.pose.orientation.w = 1.0
    for index in (0, 7, 35):
        message.pose.covariance[index] = 0.1
    publisher.publish(message)
    assert wait_until(lambda: bridge._epoch.commands_allowed)
    client_node.destroy_publisher(publisher)


def build_harness(fake):
    rclpy.init()
    amcl = FakeAmcl()
    bridge = PatrolBotBridge(
        parameter_overrides=[
            Parameter('server_ip', value='127.0.0.1'),
            Parameter('server_port', value=fake.port),
            Parameter('connect_timeout', value=0.2),
            Parameter('receive_timeout', value=0.3),
            Parameter('reconnect_delay', value=0.05),
            Parameter('control_timeout', value=0.4),
        ]
    )
    client_node = Node('fake_sbc_integration_client')
    client_node.fake_amcl = amcl
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(amcl)
    executor.add_node(bridge)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        assert wait_until(lambda: bridge._link_connected)
        assert wait_until(lambda: bridge._pose_received)
        assert wait_until(lambda: bridge._charge_state_at > 0 and bridge._flags_at > 0)
        seed_harness(bridge, client_node)
    except BaseException:
        # A failed seed must not leak nodes/threads or poison every subsequent
        # test with an already-initialized ROS context.
        teardown_harness(bridge, client_node, executor, spin_thread, fake)
        raise
    return bridge, client_node, executor, spin_thread


def teardown_harness(bridge, client_node, executor, spin_thread, fake):
    assert executor.shutdown(timeout_sec=3.0), 'executor failed to drain callbacks'
    spin_thread.join(timeout=2.0)
    assert not spin_thread.is_alive(), 'executor spin thread survived shutdown'
    for node in (bridge, client_node.fake_amcl, client_node):
        executor.remove_node(node)
        node.destroy_node()
    rclpy.try_shutdown()
    fake.close()


@pytest.fixture
def harness(tmp_path, monkeypatch):
    monkeypatch.setenv('PATROLBOT_STATE_DIR', str(tmp_path))
    fake = FakeSbc()
    bridge, client_node, executor, spin_thread = build_harness(fake)
    yield fake, bridge, client_node, executor
    teardown_harness(bridge, client_node, executor, spin_thread, fake)


@pytest.fixture
def harness_without_undock(tmp_path, monkeypatch):
    monkeypatch.setenv('PATROLBOT_STATE_DIR', str(tmp_path))
    fake = FakeSbc(advertise_undock=False)
    bridge, client_node, executor, spin_thread = build_harness(fake)
    yield fake, bridge, client_node, executor
    teardown_harness(bridge, client_node, executor, spin_thread, fake)


def call_release(client_node, request_id):
    client = client_node.create_client(ChargeRelease, '/patrolbot/charge_release')
    assert client.wait_for_service(timeout_sec=1.0)
    request = ChargeRelease.Request()
    request.request_id = request_id
    future = client.call_async(request)
    assert wait_until(future.done, timeout=2.0)
    return future.result()


def send_undock(
    client_node, clearance=0.53, speed=0.10, duration=30.0, feedback=None, timeout=10.0
):
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
    releases = [line for _session, line in fake.received if line == f'CHARGE_RELEASE:{request_id}']
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
    assert wait_until(lambda: any(line.startswith('DRIVE:') for _session, line in fake.received))
    first_generation = bridge._session_generation
    fake.drop_connection()
    assert wait_until(lambda: bridge._session_generation > first_generation)
    second_connection = fake.connections
    time.sleep(0.25)
    assert not any(
        session == second_connection and line.startswith('DRIVE:')
        for session, line in fake.received
    )


def test_motor_enable_is_independent_of_dock_state(harness):
    fake, bridge, client_node, _executor = harness
    with fake._lock:
        fake.estop_pressed = False
        fake.dock_state = 'DOCKED_CONFIRMED'
    assert wait_until(
        lambda: bridge._dock_state == 'DOCKED_CONFIRMED' and not bridge._estop_pressed
    )
    client = client_node.create_client(MotorEnable, '/patrolbot/motor_enable')
    assert client.wait_for_service(timeout_sec=1.0)
    request = MotorEnable.Request()
    request.request_id = str(uuid4())
    future = client.call_async(request)
    assert wait_until(future.done, timeout=2.0)
    response = future.result()
    assert response.success and response.motors_enabled
    assert any(line.startswith('MOTOR_ENABLE:') for _session, line in fake.received)


def test_hardware_undock_reports_success_with_motors_left_enabled(harness):
    fake, _bridge, client_node, _executor = harness
    seen = []
    _client, handle = send_undock(client_node, feedback=lambda msg: seen.append(msg.feedback))
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
    assert any(line.startswith('UNDOCK_CANCEL:') for _session, line in fake.received)


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


def test_hardware_undock_is_refused_by_a_server_without_the_feature(harness_without_undock):
    fake, _bridge, client_node, _executor = harness_without_undock
    client = ActionClient(client_node, HardwareUndock, '/patrolbot/hardware_undock')
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
    assert not any(line.startswith('UNDOCK:') for _session, line in fake.received)


def test_odom_tf_stops_rather_than_going_stale_when_the_link_dies(harness):
    """
    A dead SBC link must remove odom->base_link, not re-stamp the last pose.

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
        TFMessage,
        '/tf',
        lambda msg: seen.extend(t for t in msg.transforms if t.child_frame_id == 'base_link'),
        10,
    )
    assert wait_until(lambda: len(seen) > 0), 'no odom->base_link to begin with'

    # The SBC goes quiet with the socket still open. The last pose is now
    # frozen, but the TF timer keeps ticking off its own clock.
    fake.telemetry_paused = True
    timeout = bridge.get_parameter('odom_tf_timeout').value
    time.sleep(timeout + 0.3)
    settled = len(seen)
    time.sleep(0.5)  # 25 TF ticks at 50 Hz
    assert len(seen) == settled, 'kept broadcasting a stale pose'

    # It must resume when telemetry does, rather than latching off for good.
    fake.telemetry_paused = False
    assert wait_until(lambda: len(seen) > settled), 'never resumed'


def test_the_transform_leads_the_scan_out_of_a_telemetry_gap(harness):
    """
    After a gap the first scan must not outrun its transform.

    /odom and /scan are published from the telemetry worker the moment a frame
    arrives, while TF came only from a 50 Hz timer on a different thread. With
    the transform withdrawn during the gap, the first scan after a reconnect
    could reach the costmap message filter up to a full timer period before any
    transform covered it -- the precise ordering this node publishes TF at
    50 Hz to avoid.
    """
    fake, bridge, client_node, _executor = harness
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    events = []
    client_node.create_subscription(
        TFMessage,
        '/tf',
        lambda msg: events.extend('tf' for t in msg.transforms if t.child_frame_id == 'base_link'),
        50,
    )
    # /scan is published with sensor QoS; a default RELIABLE subscription is
    # silently incompatible and receives nothing.
    from rclpy.qos import qos_profile_sensor_data

    client_node.create_subscription(
        LaserScan, '/scan', lambda _msg: events.append('scan'), qos_profile_sensor_data
    )
    assert wait_until(lambda: 'tf' in events and 'scan' in events)

    fake.telemetry_paused = True
    time.sleep(bridge.get_parameter('odom_tf_timeout').value + 0.3)
    events.clear()
    time.sleep(0.2)
    assert events == [], 'still publishing during the gap'

    fake.telemetry_paused = False
    assert wait_until(lambda: 'scan' in events), 'no scan after the gap'
    assert events[0] == 'tf', f'first frame out of the gap was {events[0]!r}, not the transform'


def test_odom_publishes_a_usable_covariance_not_a_block_of_zeros(harness):
    """
    /odom carried 36 zeros in both blocks, which is worse than no answer.

    Zero covariance is not "unknown": it asserts the wheel odometry is exact,
    and the matrix is singular, so an estimator that weights the measurement
    by inverting it cannot use the message at all. Everything downstream of
    this bridge that wants to reason about odometry quality -- an EKF, a
    localization health check -- needs real numbers on the wire.
    """
    fake, bridge, client_node, _executor = harness
    from nav_msgs.msg import Odometry
    from patrolbot_bridge.odom_covariance import UNOBSERVED_VARIANCE

    seen = []
    client_node.create_subscription(Odometry, '/odom', lambda msg: seen.append(msg), 10)
    assert wait_until(lambda: len(seen) > 0), 'no /odom at all'

    pose = list(seen[-1].pose.covariance)
    twist = list(seen[-1].twist.covariance)
    assert len(pose) == 36 and len(twist) == 36

    # x, y and yaw are measured, so they carry a real, positive variance.
    for axis in (0, 7, 35):
        assert pose[axis] > 0.0, f'pose diagonal {axis} still zero'
        assert twist[axis] > 0.0, f'twist diagonal {axis} still zero'

        # z, roll and pitch are not measured by a planar base, and must say so.
    for axis in (14, 21, 28):
        assert pose[axis] == UNOBSERVED_VARIANCE

        # Nothing here estimates a cross-correlation, so none is claimed.
    diagonal = {0, 7, 14, 21, 28, 35}
    assert all(pose[i] == 0.0 for i in range(36) if i not in diagonal)


def test_odom_pose_covariance_grows_as_the_robot_drives(harness):
    """
    Dead reckoning drifts, and the message has to admit it over time.

    A constant covariance would claim the pose is as trustworthy at the far
    end of a patrol as it was at the dock -- the one thing wheel odometry is
    guaranteed not to be.
    """
    fake, bridge, client_node, _executor = harness
    from nav_msgs.msg import Odometry

    seen = []
    client_node.create_subscription(Odometry, '/odom', lambda msg: seen.append(msg), 50)
    assert wait_until(lambda: len(seen) > 0)
    at_rest = seen[-1].pose.covariance[0]

    # Drive the fake robot forward in steps small enough to be real motion
    # rather than a rejected discontinuity.
    for step in range(1, 21):
        fake.odom_pose = (0.005 * step, 0.0, 0.0)
        time.sleep(0.05)

    assert wait_until(
        lambda: seen[-1].pose.covariance[0] > at_rest
    ), 'covariance never grew despite 0.1 m of driving'


def test_link_reconnect_preserves_odometry_covariance(harness):
    """Losing the Pi socket leaves ARIA's origin and accrued drift intact."""
    fake, bridge, client_node, _executor = harness
    from nav_msgs.msg import Odometry

    seen = []
    client_node.create_subscription(Odometry, '/odom', lambda msg: seen.append(msg), 50)
    assert wait_until(lambda: len(seen) > 0)
    at_rest = seen[-1].pose.covariance[0]
    for step in range(1, 21):
        fake.odom_pose = (0.005 * step, 0.0, 0.0)
        time.sleep(0.05)
    assert wait_until(lambda: abs(seen[-1].pose.pose.position.x - 0.1) < 1e-6)
    driven = list(seen[-1].pose.covariance)
    assert driven[0] > at_rest

    for _ in range(2):
        generation = bridge._session_generation
        fake.drop_connection()
        assert wait_until(
            lambda: bridge._session_generation > generation, timeout=6.0
        ), 'never reconnected'
        # Require a newly accepted frame, not a queued pre-disconnect message.
        before = bridge._pose_at
        assert wait_until(lambda: bridge._pose_at > before)
        with bridge._pose_lock:
            assert bridge._odom_covariance.pose_covariance() == driven

    fake.odom_pose = (0.11, 0.0, 0.0)
    assert wait_until(
        lambda: seen[-1].pose.covariance[0] > driven[0]
    ), 'valid motion after reconnect did not accumulate covariance'

    # ----------------------------------------------------------------------------
    # Scan geometry. The SBC sends ranges only -- ARIA's per-reading angles never
    # reach the wire -- so a beam's bearing is purely its index in the list, and
    # `_laser_scan_message` reconstructs the sweep as angle_min = -pi/2 with
    # angle_increment = pi/(N-1). N is therefore not a count, it is the angular
    # calibration: add or drop one element and every beam after it is reported at
    # the wrong bearing, in a scan that still looks entirely plausible in RViz.
    # ----------------------------------------------------------------------------


def _scan_from(seen, field_count, second_range):
    """
    Wait for a scan that could only have come from the field just installed.

    A scan built from the previous LASER: body can still be in flight, so
    waiting for "any scan" samples the old one and fails intermittently. The
    identifying mark is the second range, which differs from the 1.0 the
    default field produces. Deliberately accepts a short scan too: a buggy
    bridge drops the bad element and publishes N-1 beams, and this has to fail
    on the assertions below rather than hang here.
    """

    def matches(msg):
        if len(msg.ranges) not in (field_count - 1, field_count):
            return False
        return len(msg.ranges) > 1 and msg.ranges[1] == pytest.approx(second_range)

    assert wait_until(
        lambda: any(matches(m) for m in seen)
    ), 'no scan was produced from the installed LASER: field'
    return next(m for m in seen if matches(m))


def _subscribe_to_scan(client_node):
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    seen = []
    client_node.create_subscription(
        LaserScan, '/scan', lambda msg: seen.append(msg), qos_profile_sensor_data
    )
    return seen


def test_an_unparsable_range_does_not_rotate_the_rest_of_the_scan(harness):
    """
    A field that will not parse must occupy its slot, not vanish.

    Silently skipping it shortens N, which widens angle_increment across the
    whole sweep and slides every later beam off its true bearing. From AMCL's
    side that is indistinguishable from a miscalibrated laser -- and it is
    exactly the failure that gets misdiagnosed as bad odometry.
    """
    fake, _bridge, client_node, _executor = harness
    seen = _subscribe_to_scan(client_node)
    assert wait_until(lambda: len(seen) > 0), 'no scan on a clean stream'

    # The corrupt field is deliberately neither first nor last: an off-by-one
    # at either end would still pass a bare length check.
    fake.laser_field = '1.0,nonsense,3.0,4.0'
    seen.clear()
    scan = _scan_from(seen, field_count=4, second_range=float('inf'))

    assert len(scan.ranges) == 4, (
        f'published {len(scan .ranges)} beams for 4 fields -- the unparsable '
        'one was dropped rather than marked invalid, so every beam after it '
        'is now reported at the wrong bearing'
    )
    assert scan.angle_increment == pytest.approx(math.pi / 3.0)
    assert math.isinf(scan.ranges[1])
    assert scan.ranges[2] == pytest.approx(
        3.0
    ), 'the beam after the bad field shifted into the wrong slot'
    assert scan.ranges[3] == pytest.approx(4.0)


def test_a_blank_field_inside_the_sweep_also_keeps_its_slot(harness):
    """
    An empty field mid-sweep is the same defect as an unparsable one.

    Both require a corrupt frame, and both must cost one invalid beam rather
    than re-aiming the remainder of the scan.
    """
    fake, _bridge, client_node, _executor = harness
    seen = _subscribe_to_scan(client_node)
    assert wait_until(lambda: len(seen) > 0), 'no scan on a clean stream'

    fake.laser_field = '1.0,,3.0,4.0'
    seen.clear()
    scan = _scan_from(seen, field_count=4, second_range=float('inf'))

    assert len(scan.ranges) == 4, (
        f'published {len(scan .ranges)} beams for 4 fields -- the blank one '
        'was dropped, re-aiming every beam after it'
    )
    assert scan.angle_increment == pytest.approx(math.pi / 3.0)
    assert math.isinf(scan.ranges[1])
    assert scan.ranges[2] == pytest.approx(3.0)
    assert scan.ranges[3] == pytest.approx(4.0)


def test_the_servers_trailing_comma_does_not_add_a_phantom_beam(harness):
    """
    The real SBC terminates every LASER: body with a comma. Pin that.

    `patrolbot_server.cpp` writes `range,` per reading, so the production wire
    format always ends in a separator and splitting it yields a final empty
    element. That one must stay ignored. This is the counter-case to the two
    tests above and the reason they cannot be satisfied by blindly treating
    every empty field as a beam: doing so would append a phantom +inf return
    to every single scan the robot ever publishes and mis-aim the whole sweep
    by one increment. Nothing covered this before -- the fake omitted the
    trailing comma the real server always sends.
    """
    fake, _bridge, client_node, _executor = harness
    seen = _subscribe_to_scan(client_node)
    assert wait_until(lambda: len(seen) > 0), 'no scan on a clean stream'

    fake.laser_field = '1.0,2.0,3.0,'
    seen.clear()
    scan = _scan_from(seen, field_count=4, second_range=2.0)

    assert len(scan.ranges) == 3, (
        f'published {len(scan .ranges)} beams for 3 readings plus the '
        "server's trailing comma -- the terminator was counted as a beam"
    )
    assert scan.angle_increment == pytest.approx(math.pi / 2.0)
    assert scan.ranges == pytest.approx([1.0, 2.0, 3.0])


def test_a_one_beam_frame_is_dropped_without_disturbing_the_stream(harness):
    """
    A frame too short to describe the sweep must be skipped deliberately.

    `angle_increment` is `pi/(N-1)`, so a single-beam frame divides by zero.
    That used to raise inside `_parse_telemetry` and be swallowed whole by the
    broad `except Exception` -- the right outcome reached the wrong way: the
    scan was correctly dropped, but nothing recorded that it happened, so on
    the live robot a corrupt frame was indistinguishable from a quiet laser.
    The guard is now explicit and warns, throttled, on exactly one beam; zero
    beams stays silent because that is the ordinary laser-down path.

    What must hold either way: no scan is ever published with fewer than two
    beams (there is no honest geometry for one), `/odom` keeps flowing because
    it is published before the scan block, and the stream recovers on the next
    good frame rather than wedging.
    """
    from nav_msgs.msg import Odometry

    fake, _bridge, client_node, _executor = harness
    seen = _subscribe_to_scan(client_node)
    odom_seen = []
    client_node.create_subscription(Odometry, '/odom', lambda msg: odom_seen.append(msg), 50)
    assert wait_until(lambda: len(seen) > 0), 'no scan on a clean stream'

    # One reading plus the server's terminator: N == 1 after the terminator is
    # discarded, which is exactly the degenerate case.
    fake.laser_field = '1.0,'
    seen.clear()
    odom_seen.clear()

    assert wait_until(lambda: len(odom_seen) > 2), (
        'odometry stopped while the laser field was degenerate -- a bad scan '
        'must not take the pose stream down with it'
    )
    assert all(len(msg.ranges) >= 2 for msg in seen), (
        'published a scan with fewer than two beams, which has no valid ' 'angle_increment'
    )

    # The next good frame must come straight back.
    fake.laser_field = '1.0,2.0,3.0,'
    scan = _scan_from(seen, field_count=4, second_range=2.0)
    assert len(scan.ranges) == 3, 'the stream did not recover'
    assert scan.angle_increment == pytest.approx(math.pi / 2.0)


@pytest.mark.parametrize(
    'odom,reason',
    [
        ('nan,0,0,0,0', 'non-finite'),
        ('0,0,0,inf,0', 'non-finite'),
        ('0,0,0,0.31,0', 'linear-speed-limit'),
        ('0,0,0,0,1.21', 'angular-speed-limit'),
        ('100,0,0,0,0', 'position-discontinuity'),
        ('0,0,3.0,0,0', 'heading-discontinuity'),
    ],
)
def test_invalid_odometry_cannot_refresh_pose_and_valid_stream_recovers(harness, odom, reason):
    fake, bridge, _client_node, _executor = harness
    assert wait_until(lambda: bridge._pose_at > 0)
    fake.telemetry_paused = True
    time.sleep(0.2)
    before = bridge._pose_at
    bridge._parse_telemetry('ODOM:' + odom + '|LASER:1,1,1|ODOM_EPOCH:' + fake.odom_epoch)
    assert bridge._odom_rejections.get(reason, 0) >= 1
    assert bridge._pose_at == before
    fake.telemetry_paused = False
    assert wait_until(lambda: bridge._pose_at > before)


@pytest.mark.parametrize(
    'metadata',
    [
        'LASER_AGE_MS:-1',
        'LASER_AGE_MS:501',
        'LASER_AGE_MS:nan',
        'LASER_AGE_MS:1|LASER_AGE_MS:2',
    ],
)
def test_invalid_scan_age_drops_only_scan_and_recovers(harness, metadata):
    fake, bridge, _client, _executor = harness
    assert wait_until(lambda: bridge._pose_at > 0)
    fake.telemetry_paused = True
    time.sleep(0.2)
    published = []
    original = bridge._publish_with_timing
    bridge._publish_with_timing = lambda pub, msg, topic: published.append((topic, msg))
    try:
        bridge._parse_telemetry(
            'ODOM:0,0,0,0,0|LASER:1,2,3,|' + metadata + '|ODOM_EPOCH:' + fake.odom_epoch
        )
        assert [topic for topic, _ in published] == ['/odom']
        published.clear()
        bridge._parse_telemetry(
            'ODOM:0,0,0,0,0|LASER:1,2,3,|LASER_AGE_MS:120' + '|ODOM_EPOCH:' + fake.odom_epoch
        )
        assert [topic for topic, _ in published] == ['/odom', '/scan']
        odom, scan = [msg for _, msg in published]

        def ns(msg):
            return msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec

        assert 120000000 <= ns(odom) - ns(scan) < 140000000
        assert list(scan.ranges) == [1, 2, 3]
    finally:
        bridge._publish_with_timing = original
        fake.telemetry_paused = False


def test_scan_age_includes_local_queue_wait(harness):
    from unittest.mock import patch

    fake, bridge, _client, _executor = harness
    assert wait_until(lambda: bridge._pose_at > 0)
    fake.telemetry_paused = True
    time.sleep(0.2)
    published = []
    original = bridge._publish_with_timing
    bridge._publish_with_timing = lambda pub, msg, topic: published.append((topic, msg))
    try:
        line = 'ODOM:0,0,0,0,0|LASER:1,2,3,|LASER_AGE_MS:120' + '|ODOM_EPOCH:' + fake.odom_epoch
        clock = bridge.get_clock()
        receipt_clock = clock.now()
        received_at = time.monotonic() - 0.1
        earliest_queue_age = time.monotonic() - received_at
        with patch.object(clock, 'now', return_value=receipt_clock):
            bridge._parse_telemetry(line, received_at=received_at)
        latest_queue_age = time.monotonic() - received_at
        odom, scan = [msg for _, msg in published]

        def ns(msg):
            return msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            # Queue delay ages both stamps; acquisition age separates them.

        assert ns(odom) - ns(scan) == 120000000
        odom_age = receipt_clock.nanoseconds - ns(odom)
        assert int(earliest_queue_age * 1e9) <= odom_age <= int(latest_queue_age * 1e9)
        assert receipt_clock.nanoseconds - ns(scan) == odom_age + 120000000
        # The next delayed frame must still have a newer network receipt.
        time.sleep(0.31)
        published.clear()
        bridge._parse_telemetry(line, received_at=time.monotonic() - 0.4)
        assert [topic for topic, _ in published] == ['/odom']
    finally:
        bridge._publish_with_timing = original
        fake.telemetry_paused = False


def test_explicit_origin_change_holds_commands_until_new_seed(harness):
    from unittest.mock import patch

    fake, bridge, client, _executor = harness
    old_session = bridge._session_generation
    old_epoch = fake.odom_epoch
    with bridge._epoch.lock:
        bridge._odom_covariance.update(0.0, 0.0, 0.0)
        bridge._odom_covariance.update(0.1, 0.0, 0.0)
        before = bridge._odom_covariance.pose_covariance()[0]
    with patch.object(
        bridge._odom_covariance, 'reset', wraps=bridge._odom_covariance.reset
    ) as reset:
        with fake._lock:
            fake.odom_epoch = str(uuid4())
        fake.drop_connection()
        assert wait_until(lambda: bridge._epoch.guard.epoch == fake.odom_epoch)
        assert wait_until(lambda: bridge._epoch.streams_allowed)
        assert not bridge._epoch.commands_allowed
        assert bridge._odom_covariance.pose_covariance()[0] < before
        assert reset.call_count == 1
        fake.telemetry_paused = True
        time.sleep(0.1)
        previous = bridge._pose_at
        bridge._parse_telemetry(
            'ODOM:0,0,0,0,0|LASER:1,1|ODOM_EPOCH:' + old_epoch, session=old_session
        )
        assert bridge._pose_at == previous
        command = Twist()
        command.linear.x = 0.05
        with fake._lock:
            command_count = len(fake.received)
        bridge.cmd_vel_callback(command)
        time.sleep(0.05)
        with fake._lock:
            assert not any(
                line.startswith('DRIVE:0.05:') for _, line in fake.received[command_count:]
            )
        fake.telemetry_paused = False
        seed_harness(bridge, client)
        assert reset.call_count == 1
        assert client.fake_amcl.navigation_transitions[-2:] == [1, 2]


def test_close_dock_returns_survive_without_changing_bearings(harness):
    """Keep measured dock returns and reject invalid slots without re-aiming."""
    fake, _bridge, client_node, _executor = harness
    seen = _subscribe_to_scan(client_node)
    assert wait_until(lambda: len(seen) > 0)
    fake.laser_field = '0.099,0.124,0.15,0.249,0.25,0.1,0,nan,-1,inf,'
    seen.clear()
    scan = _scan_from(seen, field_count=10, second_range=0.124)
    assert len(scan.ranges) == 10
    assert scan.range_min == pytest.approx(0.10)
    assert scan.angle_increment == pytest.approx(math.pi / 9)
    assert list(scan.ranges[1:6]) == pytest.approx([0.124, 0.15, 0.249, 0.25, 0.1])
    assert all(math.isinf(scan.ranges[i]) for i in [0, 6, 7, 8, 9])


def test_lidar_power_roundtrip_keeps_odometry_and_requires_fresh_pose(harness):
    fake, bridge, client_node, _ = harness
    client = client_node.create_client(SetLidarPower, '/patrolbot/set_lidar_power')
    assert client.wait_for_service(timeout_sec=2.0)
    seen = []
    subscription = client_node.create_subscription(Odometry, '/odom', seen.append, 10)
    assert wait_until(lambda: bridge._epoch._navigation_idle_at is not None)
    off = client.call_async(SetLidarPower.Request(request_id=str(uuid4()), enabled=False))
    assert wait_until(off.done)
    assert off.result().success and off.result().state == 'OFF'
    assert wait_until(lambda: bridge._lidar.snapshot(time.monotonic())[0] == 'OFF')
    before = len(seen)
    assert wait_until(lambda: len(seen) > before + 3)
    assert not bridge._epoch.commands_allowed
    old_seed = bridge._epoch.recovery._seed_ns
    initial = PoseWithCovarianceStamped()
    initial.header.frame_id = 'map'
    initial.header.stamp = client_node.get_clock().now().to_msg()
    initial.pose.pose.orientation.w = 1.0
    for index in (0, 7, 35):
        initial.pose.covariance[index] = 0.1
    bridge._epoch.seed(initial)
    assert bridge._epoch.recovery._seed_ns == old_seed
    request_count = len(fake.received)
    velocity = Twist()
    velocity.linear.x = 0.1
    bridge.cmd_vel_callback(velocity)
    assert not any(line.startswith('DRIVE:0.1:') for _, line in fake.received[request_count:])
    on = client.call_async(SetLidarPower.Request(request_id=str(uuid4()), enabled=True))
    assert wait_until(on.done)
    assert on.result().success and on.result().state == 'STARTING'
    assert wait_until(bridge._lidar_ready)
    assert not bridge._epoch.commands_allowed
    seed_harness(bridge, client_node)
    assert bridge._epoch.commands_allowed
    client_node.destroy_subscription(subscription)


def test_ready_scans_without_power_commissioning_do_not_advertise_control(harness):
    fake, bridge, client_node, _ = harness
    fake.lidar_supported = False
    assert wait_until(lambda: not bridge._lidar.snapshot(time.monotonic())[1])
    assert bridge._lidar_ready()
    client = client_node.create_client(SetLidarPower, '/patrolbot/set_lidar_power')
    assert client.wait_for_service(timeout_sec=2.0)
    before = len(fake.received)
    future = client.call_async(SetLidarPower.Request(request_id=str(uuid4()), enabled=False))
    assert wait_until(future.done)
    assert not future.result().success
    assert not any(line.startswith('LIDAR_') for _, line in fake.received[before:])
