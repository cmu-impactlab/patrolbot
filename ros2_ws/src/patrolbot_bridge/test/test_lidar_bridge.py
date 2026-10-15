"""Exercise the bridge service and socket gates with real generated ROS types."""
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import Twist
from patrolbot_interfaces.srv import SetLidarPower, MotorEnable
from patrolbot_bridge.bridge_node import PatrolBotBridge
from patrolbot_bridge.lidar_state import LidarState
from patrolbot_bridge.undock_protocol import parse_hello

ID = '123e4567-e89b-12d3-a456-426614174000'


@pytest.fixture
def bridge():
    b = PatrolBotBridge.__new__(PatrolBotBridge)
    b._state_lock = threading.Lock(); b._pose_lock = threading.Lock()
    b._control_lock = threading.Lock(); b.sock_lock = threading.Lock()
    b._hardware_operation_lock = threading.Lock()
    b._undock_goal_lock = threading.Lock(); b._undock_goal_reserved = False
    b._pending_controls = {}; b._lidar = LidarState()
    b._lidar_pending = False; b._lidar_hold = False; b._lidar_cutoff = 0.
    b._session_generation = 1; b._link_connected = True
    b._charge_state = 0; b._motors_enabled = False; b._estop_pressed = False
    b._fault_flags = 0; b._undock_active = False
    b._dock_state = 'CLEAR_CONFIRMED'
    now = time.monotonic()
    b._charge_state_at = b._flags_at = b._dock_state_at = b._pose_at = now
    b._vx = b._vth = 0.; b.CONTROL_TIMEOUT = .001
    b._lidar.update('READY,1,0', now)
    b._server_hello = parse_hello('HELLO:PROTOCOL=2:FEATURES=LIDAR_POWER')
    b._epoch = NS(lock=threading.RLock(), commands_allowed=True,
        maps=NS(hold=False), _map_goal=None, _future=None, _map_future=None,
        _navigation_status=False, _navigation_idle_at=now,
        require_lidar_seed=Mock())
    b.get_logger = Mock(return_value=Mock())
    b.sock = Mock()
    b.sent = []
    def send(payload):
        b.sent.append(payload)
        if payload.startswith((b'LIDAR_', b'MOTOR_ENABLE:')):
            if payload.startswith(b'LIDAR_'):
                assert b._lidar_hold and b._lidar_pending
            verb, request_id = payload.decode().strip().split(':')
            b._handle_control_reply(f'ACK:{verb}:{request_id}:ACCEPTED')
            motors = int(verb == 'MOTOR_ENABLE')
            b._handle_control_reply(f'RESULT:{verb}:{request_id}:OK:0:commanded:0:{motors}:0')
    b.sock.sendall.side_effect = send
    return b


def power(b, enabled):
    request = SetLidarPower.Request(request_id=ID, enabled=enabled)
    return b._set_lidar_power_callback(request, SetLidarPower.Response())


def test_off_establishes_hold_before_send_and_requires_new_seed(bridge):
    response = power(bridge, False)
    assert response.success and response.state == 'OFF'
    assert bridge.sent[0] == b'DRIVE:0.0:0.0\n'
    assert bridge.sent[1] == f'LIDAR_OFF:{ID}\n'.encode()
    bridge._epoch.require_lidar_seed.assert_called_once()
    assert not bridge._lidar_ready()


@pytest.mark.parametrize('conflict', ['moving', 'nan', 'navigation', 'idle-stale',
    'map', 'undock-reserved', 'uncommissioned', 'stale', 'missing-capability', 'starting'])
def test_power_conflicts_send_nothing(bridge, conflict):
    if conflict == 'moving': bridge._vx = .1
    if conflict == 'nan': bridge._vth = float('nan')
    if conflict == 'navigation': bridge._epoch._navigation_status = True
    if conflict == 'idle-stale': bridge._epoch._navigation_idle_at = 0.
    if conflict == 'map': bridge._epoch.maps.hold = True
    if conflict == 'undock-reserved': bridge._undock_goal_reserved = True
    if conflict == 'uncommissioned': bridge._lidar.update('READY,0,0', time.monotonic())
    if conflict == 'stale': bridge._lidar.reset()
    if conflict == 'starting': bridge._lidar.update('STARTING,1,-1', time.monotonic())
    if conflict == 'missing-capability': bridge._server_hello = None
    assert not power(bridge, False).success
    assert bridge.sent == []


def test_on_is_possible_without_localization(bridge):
    bridge._epoch.commands_allowed = False
    bridge._lidar.update('OFF,1,-1', time.monotonic())
    response = power(bridge, True)
    assert response.success and response.state == 'STARTING'


def test_timeout_and_late_result_cannot_release_motion(bridge):
    bridge.sock.sendall.side_effect = lambda p: bridge.sent.append(p)
    response = power(bridge, False)
    assert response.code == 5 and bridge._lidar_hold
    bridge._handle_control_reply(f'RESULT:LIDAR_OFF:{ID}:OK:0:late:0:0:0')
    bridge._lidar.update('READY,1,0', time.monotonic())
    bridge._epoch.commands_allowed = True
    twist = Twist(); twist.linear.x = .1
    bridge.cmd_vel_callback(twist)
    assert len(bridge.sent) == 2 and not bridge._lidar_ready()


def test_one_hardware_operation_and_pending_barrier(bridge):
    bridge._hardware_operation_lock.acquire()
    assert not power(bridge, False).accepted
    bridge._hardware_operation_lock.release()
    bridge._lidar_pending = True
    assert not bridge._lidar_ready()


@pytest.mark.parametrize('state', ['OFF,1,-1', 'STARTING,1,-1', 'UNKNOWN,0,-1'])
def test_motor_power_is_available_but_nonzero_drive_requires_lidar(bridge, state):
    bridge._lidar.update(state, time.monotonic())
    response = bridge._motor_enable_callback(MotorEnable.Request(request_id=ID), MotorEnable.Response())
    assert response.success and response.motors_enabled
    assert bridge.sent == [f'MOTOR_ENABLE:{ID}\n'.encode()]
    bridge.sent.clear()
    twist = Twist(); twist.linear.x = .1
    bridge.cmd_vel_callback(twist)
    assert not bridge.sent
    bridge.cmd_vel_callback(Twist())
    assert bridge.sent == [b'DRIVE:0.0:0.0\n']


def test_queued_ready_from_before_transition_is_ignored(bridge):
    received = time.monotonic()
    power(bridge, False)
    bridge._record_lidar_section('READY,1,0', received)
    assert not bridge._lidar_ready()
