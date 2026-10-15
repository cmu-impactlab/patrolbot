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
        get_logger=Mock(return_value=Mock()), sock=Mock())


@pytest.mark.parametrize('gate,expected', [
    ('epoch', 'Hardware undock rejected: epoch commands_allowed=False.'),
    ('lidar', 'Hardware undock rejected: lidar_not_ready.'),
])
def test_rejection_names_gate_without_reserving_or_sending(admission, bridge, gate, expected):
    if gate == 'epoch':
        bridge._epoch.commands_allowed = False
    else:
        bridge._lidar_ready.return_value = False
    request = NS(clearance_distance=0.53, reverse_speed=0.1, maximum_duration=30.0)
    assert admission(bridge, request) == 'reject'
    bridge.get_logger().error.assert_called_once_with(expected)
    assert not bridge._undock_goal_reserved
    bridge.sock.sendall.assert_not_called()


def test_healthy_admission_still_reserves_without_sending(admission, bridge):
    request = NS(clearance_distance=0.53, reverse_speed=0.1, maximum_duration=30.0)
    assert admission(bridge, request) == 'accept'
    assert bridge._undock_goal_reserved
    bridge.get_logger().error.assert_not_called()
    bridge.sock.sendall.assert_not_called()
