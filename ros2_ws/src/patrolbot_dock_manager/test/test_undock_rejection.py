"""Production rejection/result mapping contracts, without ROS or hardware."""

import ast
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest


@pytest.fixture
def manager_class():
    package = Path(__file__).parents[1]
    source = package / 'patrolbot_dock_manager' / 'dock_manager.py'
    tree = ast.parse(source.read_text())
    manager = next(node for node in tree.body
                   if isinstance(node, ast.ClassDef) and node.name == 'DockManager')
    manager.bases = []  # Omit ROS Node construction, not the production methods.
    methods = {'_run_hardware_undock', '_undock_result_code', '_execute_callback'}
    manager.body = [node for node in manager.body
                    if isinstance(node, ast.FunctionDef) and node.name in methods]

    def action(name):
        text = (package.parent / 'patrolbot_interfaces' / 'action' / f'{name}.action').read_text()
        codes = {}
        for line in text.splitlines():
            if line.startswith('uint16 ') and '=' in line:
                key, value = line.split()[1].split('=')
                codes[key] = int(value)
        return NS(Result=NS(**codes), Goal=NS)

    namespace = {'Undock': action('Undock'), 'HardwareUndock': action('HardwareUndock'),
                 'time': time, 'uuid4': lambda: 'offline-test'}
    exec(compile(ast.Module(body=[manager], type_ignores=[]), str(source), 'exec'),
         namespace)
    return namespace['DockManager']


@pytest.fixture
def manager(manager_class):
    manager = manager_class()
    manager._hardware_undock_client = Mock()
    manager._wait_future = Mock(return_value=True)
    manager.get_parameter = Mock(return_value=NS(value=1.0))
    manager._hardware_feedback_callback = Mock()
    return manager


@pytest.mark.parametrize('acceptance,expected_code', [(NS(accepted=False), 1), (None, 8)])
def test_only_explicit_refusal_maps_to_rejected(manager, acceptance, expected_code):
    client = manager._hardware_undock_client
    client.send_goal_async.return_value.result.return_value = acceptance
    goal = NS(is_cancel_requested=False, request=NS(validation_mode=False))
    ok, reason, result = manager._run_hardware_undock(goal, 'offline-test')
    assert not ok and result is None
    assert manager._undock_result_code(goal, result, reason) == expected_code
    if acceptance is None:
        assert reason == 'hardware undock goal acceptance unknown'
    else:
        assert reason == 'hardware undock goal rejected'


@pytest.mark.parametrize('reason', [
    '', 'hardware undock action unavailable', 'hardware undock goal acceptance timeout',
    'hardware undock goal acceptance unknown', 'hardware undock produced no result',
    'hardware undock returned no result',
])
def test_unknown_outcomes_keep_motion_failed(manager, reason):
    assert manager._undock_result_code(NS(is_cancel_requested=False), None, reason) == 8


def test_cancellation_keeps_precedence_over_rejection(manager):
    assert manager._undock_result_code(
        NS(is_cancel_requested=True), None, 'hardware undock goal rejected') == 9


def test_terminal_motion_failure_is_not_reclassified(manager):
    # HardwareUndock.MOTION_FAILED=10 maps to Undock.MOTION_FAILED=8.
    assert manager._undock_result_code(
        NS(is_cancel_requested=False), NS(code=10), 'hardware undock goal rejected') == 8


def test_execute_relays_rejection_code_and_preserves_manual_recovery(manager):
    goal = NS(is_cancel_requested=False, request=NS(validation_mode=False))
    manager._hardware_undock_client.send_goal_async.return_value.result.return_value = NS(
        accepted=False)
    manager._transition = Mock()
    manager._snapshot = Mock(return_value=NS(
        charge_state=2, session_generation=7, command_path_valid=True))
    manager._wait_for_precheck = Mock(return_value=(True, ''))
    manager._wait_for_localization = Mock(return_value=True)
    manager._refresh_command_path = Mock()
    manager._abort = Mock(return_value='aborted')
    manager._goal_lock = threading.Lock()
    manager._lock = threading.Lock()
    manager._state_name = 'HARDWARE_UNDOCK'
    assert manager._execute_callback(goal) == 'aborted'
    manager._abort.assert_called_once_with(
        goal, 1, 'hardware undock goal rejected', 2, manual_recovery=True, distance=0.0)
    assert manager._transition.call_args.args[0] == 'HARDWARE_UNDOCK'
