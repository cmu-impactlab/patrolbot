"""Exercise the production undock admission callback without ROS or hardware."""

import ast
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest


@pytest.fixture
def admission():
    source = Path(__file__).parents[1] / 'patrolbot_bridge' / 'bridge_node.py'
    tree = ast.parse(source.read_text())
    bridge = next(node for node in tree.body
                  if isinstance(node, ast.ClassDef) and node.name == 'PatrolBotBridge')
    method = next(node for node in bridge.body
                  if isinstance(node, ast.FunctionDef)
                  and node.name == '_undock_goal_callback')
    namespace = {'math': math, 'GoalResponse': NS(ACCEPT='accept', REJECT='reject')}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'),
         namespace)
    return namespace['_undock_goal_callback']


@pytest.fixture
def bridge():
    return NS(
        _epoch=NS(lock=threading.RLock(), commands_allowed=True),
        _state_lock=threading.Lock(), _undock_goal_lock=threading.Lock(),
        _link_connected=True, _server_hello=NS(supports_undock=lambda: True),
        _lidar_ready=Mock(return_value=True), _undock_goal_reserved=False,
        _undock_turn=NS(reserve_reverse=Mock(return_value=True)),
        get_logger=Mock(return_value=Mock()), sock=Mock())


def _request():
    return NS(clearance_distance=0.53, reverse_speed=0.1,
              maximum_duration=30.0, validation_mode=False)


@pytest.mark.parametrize('condition', ['epoch_down', 'lidar_down', 'both_down'])
def test_epoch_and_lidar_state_do_not_gate_undock_admission(
        admission, bridge, condition):
    """Authorized 2026-10-16: admission lives at the dock manager (E-stop
    plus charging) and the SBC. Epoch recovery, TF/nav readiness and scan
    freshness must not refuse an undock goal here; post-reverse motion
    stays gated elsewhere."""
    if condition in ('epoch_down', 'both_down'):
        bridge._epoch.commands_allowed = False
    if condition in ('lidar_down', 'both_down'):
        bridge._lidar_ready.return_value = False
    request = _request()
    assert admission(bridge, request) == 'accept'
    assert bridge._undock_goal_reserved
    bridge.get_logger().error.assert_not_called()
    bridge.sock.sendall.assert_not_called()


def test_turn_reserve_refusal_rejects_silently_without_reserving(
        admission, bridge):
    bridge._undock_turn.reserve_reverse.return_value = False
    assert admission(bridge, _request()) == 'reject'
    bridge.get_logger().error.assert_not_called()
    assert not bridge._undock_goal_reserved
    bridge.sock.sendall.assert_not_called()


@pytest.mark.parametrize('setup,expected', [
    ('bad_params', 'must be finite and positive'),
    ('no_link', 'no SBC link'),
    ('no_features', 'FEATURES=UNDOCK'),
    ('reserved', 'another goal is active'),
])
def test_transport_and_identity_gates_still_refuse(
        admission, bridge, setup, expected):
    if setup == 'bad_params':
        request = NS(clearance_distance=float('nan'), reverse_speed=0.1,
                     maximum_duration=30.0, validation_mode=False)
    else:
        request = _request()
    reserved_before = bridge._undock_goal_reserved
    if setup == 'no_link':
        bridge._link_connected = False
    elif setup == 'no_features':
        bridge._server_hello = NS(supports_undock=lambda: False)
    elif setup == 'reserved':
        bridge._undock_goal_reserved = True
        reserved_before = True
    assert admission(bridge, request) == 'reject'
    bridge.get_logger().error.assert_called_once()
    assert expected in bridge.get_logger().error.call_args.args[0]
    assert bridge._undock_goal_reserved == reserved_before
    bridge.sock.sendall.assert_not_called()


def test_healthy_admission_still_reserves_without_sending(admission, bridge):
    assert admission(bridge, _request()) == 'accept'
    assert bridge._undock_goal_reserved
    bridge.get_logger().error.assert_not_called()
    bridge.sock.sendall.assert_not_called()
