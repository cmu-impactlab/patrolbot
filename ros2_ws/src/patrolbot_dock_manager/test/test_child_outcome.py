"""A child action's silence cannot finish the public undock action."""
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from patrolbot_dock_manager.dock_manager import DockManager
from patrolbot_interfaces.action import HardwareUndock, Undock, UndockTurn
from unique_identifier_msgs.msg import UUID
import pytest
from rclpy.executors import ExternalShutdownException


@pytest.fixture(autouse=True)
def running_ros():
    with patch('patrolbot_dock_manager.dock_manager.rclpy.ok', return_value=True):
        yield


class Future:
    def __init__(self, value=None, ready=False):
        self.value, self.ready = value, ready
        self.add_done_callback = Mock()

    def done(self):
        return self.ready

    def result(self):
        return self.value


def manager():
    node = DockManager.__new__(DockManager)
    node._hardware_goal_id = UUID(uuid=list(range(16)))
    node._hardware_distance = node._hardware_release_attempts = 0
    node.get_parameter = lambda _: NS(value=45.)
    node.get_logger = Mock()
    node._publish_feedback = Mock()
    node._wait_for_odom_zero = Mock(return_value=True)
    return node


@pytest.mark.parametrize('child', ['hardware', 'turn'])
def test_stop_before_late_acceptance_waits_for_that_child_terminal(child):
    node = manager()
    terminal = Future()
    handle = NS(accepted=True, goal_id=node._hardware_goal_id,
                get_result_async=lambda: terminal, cancel_goal_async=Mock())
    acceptance = Future()
    client = NS(wait_for_server=lambda **_: True,
                send_goal_async=Mock(return_value=acceptance))
    node._hardware_undock_client = node._turn_client = client
    goal = NS(is_cancel_requested=False, request=Undock.Goal())
    goal.request.reverse_distance, goal.request.reverse_speed = .53, .1
    clock = NS(now=0.)
    terminal_at = 6.
    result = (HardwareUndock.Result() if child == 'hardware'
              else UndockTurn.Result())
    result.success, result.code, result.message = False, result.CANCELED, 'stopped'

    def sleep(delay):
        clock.now += delay
        if clock.now >= 5.1:
            goal.is_cancel_requested = True
        if clock.now >= 5.4:
            acceptance.value, acceptance.ready = handle, True
        if clock.now >= terminal_at:
            terminal.value, terminal.ready = NS(status=5, result=result), True

    with patch('patrolbot_dock_manager.dock_manager.time.monotonic',
               side_effect=lambda: clock.now), patch(
                   'patrolbot_dock_manager.dock_manager.time.sleep', side_effect=sleep):
        if child == 'hardware':
            outcome = node._run_hardware_undock(goal, 'test')
        else:
            outcome = node._run_spin(goal, 7, 3.141592653589793, 'test')

    assert not outcome[0]
    assert clock.now >= terminal_at, 'public operation returned before child terminal'
    handle.cancel_goal_async.assert_called_once()
    client.send_goal_async.assert_called_once()
    assert any(call.kwargs.get('state') in ('UNKNOWN', 'STOPPING')
               for call in node._publish_feedback.call_args_list)


@pytest.mark.parametrize('child', ['hardware', 'turn'])
def test_expired_result_wait_holds_until_actual_child_terminal(child):
    node = manager()
    terminal = Future()
    handle = NS(accepted=True, goal_id=node._hardware_goal_id,
                get_result_async=lambda: terminal, cancel_goal_async=Mock())
    client = NS(wait_for_server=lambda **_: True,
                send_goal_async=Mock(return_value=Future(handle, ready=True)))
    node._hardware_undock_client = node._turn_client = client
    goal = NS(is_cancel_requested=False, request=Undock.Goal())
    goal.request.reverse_distance, goal.request.reverse_speed = .53, .1
    clock = NS(now=0.)
    terminal_at = 77. if child == 'hardware' else 47.
    result = (HardwareUndock.Result() if child == 'hardware'
              else UndockTurn.Result())
    result.success, result.code, result.message = False, result.CANCELED, 'stopped'

    def sleep(delay):
        clock.now += delay
        if clock.now >= terminal_at:
            terminal.value, terminal.ready = NS(status=5, result=result), True

    with patch('patrolbot_dock_manager.dock_manager.time.monotonic',
               side_effect=lambda: clock.now), patch(
                   'patrolbot_dock_manager.dock_manager.time.sleep', side_effect=sleep):
        if child == 'hardware':
            outcome = node._run_hardware_undock(goal, 'test')
        else:
            outcome = node._run_spin(goal, 7, 3.141592653589793, 'test')

    assert not outcome[0]
    assert clock.now >= terminal_at, 'timeout is not an actual child terminal'
    handle.cancel_goal_async.assert_called_once()
    assert any(call.kwargs.get('state') == 'STOPPING'
               for call in node._publish_feedback.call_args_list)


@pytest.mark.parametrize('outcome', ['exception', 'none', 'nonterminal', 'missing-result'])
def test_unknown_child_outcome_keeps_public_owner_and_reports_uncertainty(outcome):
    node = manager()
    future = Future(NS(status=0), ready=True)
    if outcome == 'exception':
        future.result = Mock(side_effect=RuntimeError('lost RPC'))
    elif outcome == 'none':
        future.value = None
    elif outcome == 'missing-result':
        future.value = NS(status=4, result=None)
    child = NS(cancel_goal_async=Mock())
    goal = NS(is_cancel_requested=False)
    class ObserveHold(Exception):
        pass
    with patch('patrolbot_dock_manager.dock_manager.time.monotonic', return_value=1.), \
            patch('patrolbot_dock_manager.dock_manager.time.sleep', side_effect=ObserveHold):
        with pytest.raises(ObserveHold):
            node._wait_child_future(future, goal, 2., 'test', child=child)
    node._publish_feedback.assert_called_once_with(goal, 2., 'test', state='UNKNOWN')
    child.cancel_goal_async.assert_not_called()


def test_explicit_child_decline_needs_no_wait_or_cancellation():
    node = manager()
    client = NS(wait_for_server=lambda **_: True, send_goal_async=Mock(
        return_value=Future(NS(accepted=False), ready=True)))
    node._turn_client = client
    goal = NS(is_cancel_requested=False)
    with patch('patrolbot_dock_manager.dock_manager.time.sleep') as sleep:
        result = node._run_spin(goal, 7, 3.141592653589793, 'test')
    assert result == (False, 'undock turn rejected')
    sleep.assert_not_called()


def test_aborted_hardware_action_cannot_advance_using_an_embedded_success():
    node = manager()
    result = HardwareUndock.Result()
    result.success, result.physically_clear = True, True
    terminal = Future(NS(status=6, result=result), ready=True)
    handle = NS(accepted=True, goal_id=node._hardware_goal_id,
                get_result_async=lambda: terminal, cancel_goal_async=Mock())
    node._hardware_undock_client = NS(wait_for_server=lambda **_: True,
        send_goal_async=Mock(return_value=Future(handle, ready=True)))
    goal = NS(is_cancel_requested=False, request=Undock.Goal())
    goal.request.reverse_distance, goal.request.reverse_speed = .53, .1
    outcome = node._run_hardware_undock(goal, 'test')
    assert not outcome[0]
    assert outcome[1] == 'hardware undock action did not succeed'


def test_unknown_wait_drains_on_ros_shutdown_without_faking_a_child_terminal():
    node = manager()
    with patch('patrolbot_dock_manager.dock_manager.rclpy.ok', return_value=False):
        with pytest.raises(ExternalShutdownException):
            node._wait_child_future(None, NS(is_cancel_requested=True), 1., 'test')
    node._publish_feedback.assert_not_called()


def test_feedback_failure_cannot_release_an_unknown_child_owner():
    node = manager()
    node._publish_feedback.side_effect = RuntimeError('feedback output failed')
    class ObserveHold(Exception):
        pass
    with patch('patrolbot_dock_manager.dock_manager.time.sleep', side_effect=ObserveHold):
        with pytest.raises(ObserveHold):
            node._wait_child_future(None, NS(is_cancel_requested=False),
                                    0., 'test', child=NS(cancel_goal_async=Mock()))
    node.get_logger.return_value.error.assert_called_once()


def test_feedback_shutdown_exception_still_drains_the_unknown_wait():
    node = manager()
    node._publish_feedback.side_effect = ExternalShutdownException()
    with pytest.raises(ExternalShutdownException):
        node._wait_child_future(None, NS(is_cancel_requested=False), 0., 'test')
    node.get_logger.return_value.error.assert_not_called()
