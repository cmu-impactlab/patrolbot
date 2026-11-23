"""Exercise production terminal result handling without ROS or hardware."""

import ast
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest


@pytest.fixture
def finish():
    source = Path(__file__).parents[1] / 'patrolbot_bridge/bridge_node.py'
    bridge = next(n for n in ast.parse(source.read_text()).body
                  if isinstance(n, ast.ClassDef) and n.name == 'PatrolBotBridge')
    method = next(n for n in bridge.body
                  if isinstance(n, ast.FunctionDef) and n.name == '_undock_finish')
    namespace = dict(HardwareUndock=NS(Result=NS), CANCELED=13,
                     time=NS(monotonic=lambda: 102.0))
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    return namespace['_undock_finish']


@pytest.mark.parametrize('cancel_requested', [False, True])
@pytest.mark.parametrize('code', range(15))
def test_terminal_transition_preserves_sbc_result(finish, cancel_requested, code):
    bridge = NS(get_logger=Mock())
    goal = Mock(is_cancel_requested=cancel_requested)

    def canceled():
        assert goal.is_cancel_requested, 'EXECUTING -> CANCELED is illegal'
    goal.canceled.side_effect = canceled
    reply = NS(success=code == 0, code=code, message='SBC terminal evidence',
               distance_traveled=0.012, lateral_error=0.003, heading_error=-0.02,
               release_attempts=2, charge_state=2, motors_enabled=False,
               dock_state='FAULT_STOPPED', physically_clear=False)
    result = finish(bridge, goal, reply, 100.0, 7, '')
    assert result.success == reply.success and result.code == code
    assert result.message == reply.message
    for field in ('distance_traveled', 'lateral_error', 'heading_error',
                  'release_attempts', 'motors_enabled', 'physically_clear'):
        assert getattr(result, field) == getattr(reply, field)
    assert result.final_charge_state == 2
    assert result.final_dock_state == 'FAULT_STOPPED'
    assert result.elapsed_seconds == 2.0 and result.session_generation == 7
    expected = 'succeed' if code == 0 else (
        'canceled' if code == 13 and cancel_requested else 'abort')
    for transition in ('succeed', 'canceled', 'abort'):
        assert getattr(goal, transition).call_count == int(transition == expected)


@pytest.mark.parametrize('message', ['', 'SBC stopped after cancellation'])
def test_local_revocation_reason_survives_canceled_reply(finish, message):
    reason = 'hardware telemetry expired'
    bridge = NS(get_logger=Mock())
    goal = Mock(is_cancel_requested=False)
    reply = NS(success=False, code=13, message=message, distance_traveled=0.0,
               lateral_error=0.0, heading_error=0.0, release_attempts=1,
               charge_state=2, motors_enabled=False, dock_state='FAULT_STOPPED',
               physically_clear=False)
    result = finish(bridge, goal, reply, 100.0, 7, reason)
    assert result.code == 13 and not result.success
    assert reason in result.message and result.message.startswith(message)
    assert reply.message == message  # Keep the received terminal frame intact.
    goal.abort.assert_called_once()
    goal.canceled.assert_not_called()
    assert reason in bridge.get_logger().info.call_args.args[0]
