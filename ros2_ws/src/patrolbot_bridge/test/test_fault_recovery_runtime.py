"""Run the real zero-motion recovery methods with a recording SBC transport."""

import ast
import math
from pathlib import Path
import threading
import time
from types import MethodType, SimpleNamespace as NS
from unittest.mock import Mock

from patrolbot_bridge import control_protocol as protocol
import pytest


REQUEST = '11111111-2222-3333-4444-555555555555'


@pytest.fixture
def node():
    path = Path(__file__).parents[1] / 'patrolbot_bridge' / 'bridge_node.py'
    tree = ast.parse(path.read_text())
    klass = next(n for n in tree.body if isinstance(n, ast.ClassDef) and
                 n.name == 'PatrolBotBridge')
    names = {'_control_service_guarded', '_dock_reset_callback', '_handle_undock_result'}
    methods = [n for n in klass.body if isinstance(n, ast.FunctionDef) and n.name in names]
    pending = next(n for n in tree.body if isinstance(n, ast.ClassDef) and
                   n.name == 'PendingControl')
    namespace = dict(vars(protocol), time=time, threading=threading, math=math)
    namespace['parse_undock_result'] = lambda result: result
    exec(compile(ast.Module(body=[pending] + methods, type_ignores=[]), str(path), 'exec'),
         namespace)
    now = time.monotonic()
    e = NS(lock=threading.RLock(), maps=None, commands_allowed=False, undock_hold=False,
           tuning_hold=False, _failed=False, _navigation_status={}, _future=None,
           _map_goal=None, _map_future=None)
    n = NS(_epoch=e, _state_lock=threading.Lock(), _pose_lock=threading.Lock(),
           _control_lock=threading.Lock(), sock_lock=threading.Lock(), _session_generation=1,
           _link_connected=True, _charge_state_at=now, _flags_at=now, _charge_state=0,
           _motors_enabled=True, _estop_pressed=False, _fault_flags=0,
           _dock_state='FAULT_STOPPED', _undock_active=False, _vx=0., _vth=0., _pose_at=now,
           _undock_lock=threading.Lock(), _undock_request_id=REQUEST,
           _undock_event=threading.Event(),
           _lidar_ready=lambda: True, _pending_controls={}, _manual_active_at=0.,
           _web_teleop_active_at=0., _undock_goal_reserved=False, CONTROL_TIMEOUT=.01,
           _manual_undock=NS(busy=False, unknown=False, web_session_active=False),
           _undock_turn=NS(busy=False, actions={}, revoke=Mock()), get_logger=Mock(),
           get_clock=lambda: NS(now=lambda: NS(nanoseconds=123456789)))
    writes = []

    def send(payload):
        writes.append(payload)
        p = n._pending_controls[(protocol.COMMAND_DOCK_RESET, REQUEST)]
        p.reply = NS(success=True, code=0, message='dock_state_reset', accepted=True)
        p.event.set()

    def fill(response, reply, code, message, started, **kwargs):
        response.success = bool(reply and reply.success)
        response.code, response.message = code, message
        return response

    n.sock = NS(sendall=send)
    n._fill_control_response = fill
    n._discard_pending_control = lambda key, _: n._pending_controls.pop(key, None)
    for name in names:
        setattr(n, name, MethodType(namespace[name], n))
    n._control_service = n._control_service_guarded
    n.writes = writes
    return n


def test_recovery_without_amcl_sends_only_dock_reset_and_never_enables_motors(node):
    node._operation_fault = ('turn_path_blocked', REQUEST)
    result = node._dock_reset_callback(NS(request_id=REQUEST), NS())
    assert result.success
    assert node.writes == [f'DOCK_RESET:{REQUEST}\n'.encode()]
    assert node._operation_fault == ('', '')
    assert not node._epoch.commands_allowed  # Never restore navigation/old seed.
    assert node._manual_undock.input_after_ns == 123456789


def test_only_a_correlated_failed_result_updates_the_persistent_reason(node):
    node._handle_undock_result(NS(request_id='unrelated', success=False, message='wrong'))
    assert not hasattr(node, '_operation_fault')
    assert not node._undock_event.is_set()
    node._handle_undock_result(NS(request_id=REQUEST, success=False, message='turn_path_blocked'))
    assert node._operation_fault == ('turn_path_blocked', REQUEST)
    assert node._undock_event.is_set()
    assert node.writes == []


@pytest.mark.parametrize('block', [
    'estop', 'fault', 'moving', 'stale', 'unknown', 'active', 'navigation',
    'web-session', 'web-input', 'controller-input', 'tuning', 'mutation', 'lidar',
    'unknown-turn'])
def test_recovery_refusal_preserves_reason_and_sends_nothing(node, block):
    node._operation_fault = ('turn_path_blocked', REQUEST)
    if block == 'estop':
        node._estop_pressed = True
    if block == 'fault':
        node._fault_flags = 256
    if block == 'moving':
        node._vx = .1
    if block == 'stale':
        node._pose_at -= 1.
    if block == 'unknown':
        node._manual_undock.unknown = True
    if block == 'unknown-turn':
        node._undock_turn.unknown = True
    if block == 'active':
        node._undock_active = True
    if block == 'navigation':
        node._epoch._navigation_status = {'old': True}
    if block == 'web-session':
        node._manual_undock.web_session_active = True
    if block == 'web-input':
        node._web_teleop_active_at = time.monotonic()
    if block == 'controller-input':
        node._manual_active_at = time.monotonic()
    if block == 'tuning':
        node._epoch.tuning_hold = True
    if block == 'mutation':
        node._epoch._failed = True
    if block == 'lidar':
        node._lidar_ready = lambda: False
    result = node._dock_reset_callback(NS(request_id=REQUEST), NS())
    assert not result.success
    assert node.writes == []
    assert node._operation_fault == ('turn_path_blocked', REQUEST)
