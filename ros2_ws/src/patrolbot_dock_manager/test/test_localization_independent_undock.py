"""Execute production orchestration with timed child futures, without ROS/DDS."""
import ast
from concurrent.futures import Future
from dataclasses import replace
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from patrolbot_dock_manager.contracts import (
    SafetySnapshot, automatic_docking_allowed, precheck_failures, resume_failures,
    transition_allowed,
)


class Clock:
    def __init__(self):
        self.now = 100.0
        self.jobs = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        for at, callback in list(self.jobs):
            if self.now >= at:
                self.jobs.remove((at, callback))
                callback()

    def later(self, seconds, value):
        future = Future()
        self.jobs.append((self.now + seconds, lambda: future.set_result(value)))
        return future


def wire(name):
    source = Path(__file__).parents[2] / 'patrolbot_interfaces/action' / (name + '.action')
    codes = {}
    for line in source.read_text().splitlines():
        if line.startswith('uint16 ') and '=' in line:
            key, value = line.split()[1].split('=')
            codes[key] = int(value)
    return NS(Result=type(name + 'Result', (NS,), codes), Goal=NS)


@pytest.fixture
def rig():
    source = Path(__file__).parents[1] / 'patrolbot_dock_manager/dock_manager.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'DockManager')
    cls.bases = []
    methods = {
        '_execute_callback', '_transition', '_wait_for_precheck', '_wait_for_handoff',
        '_run_hardware_undock', '_run_spin', '_wait_child_future', '_wait_future',
        '_wait_for_odom_zero', '_undock_result_code', '_result', '_abort', '_canceled',
        '_wait_for_localization', '_localization_update_attempt_limit', '_readiness_timer',
    }
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in methods]
    clock = Clock()
    namespace = dict(time=clock, math=math, rclpy=NS(ok=lambda: True),
                     uuid4=lambda: 'offline',
                     precheck_failures=precheck_failures, resume_failures=resume_failures,
                     automatic_docking_allowed=automatic_docking_allowed,
                     transition_allowed=transition_allowed,
                     GoalStatus=NS(STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6),
                     ExternalShutdownException=RuntimeError,
                     DockReadiness=lambda: NS(header=NS()),
                     **{name: wire(name) for name in ('Undock', 'HardwareUndock', 'UndockTurn')})
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), namespace)
    manager = namespace['DockManager']()
    state = SafetySnapshot(
        link_connected=True, hardware_state_valid=True, charge_state_valid=True,
        charge_state=2, session_generation=7, estop_pressed=False, bumpers_valid=True,
        sonar_fresh=True, rear_clear=True, odom_fresh=True, stationary=True,
        command_path_valid=True, navigation_idle=True, dock_state_valid=True,
        dock_state='DOCKED_CONFIRMED', undock_profile_commissioned=True,
    )
    rig = NS(manager=manager, clock=clock, state=state, turn_calls=[], hardware_calls=[],
             cancel_phase=None, hardware_success=True, zero=True)
    manager._snapshot = lambda **kw: rig.state
    params = dict(precheck_timeout=1.5, handoff_timeout=0.5, undock_turn_yaw=math.pi,
                  undock_clearance_distance=0.53, undock_reverse_speed=0.10,
                  undock_max_duration=45.0, localization_update_max_attempts=5)
    manager.get_parameter = lambda key: NS(value=params[key])
    manager.get_logger = Mock()
    manager._publish_feedback = Mock()
    manager._refresh_command_path = Mock()
    manager._hardware_feedback_callback = Mock()
    manager._request_localization_update = Mock(return_value=True)
    manager._initial_pose_pub = Mock()
    manager._goal_lock = threading.Lock()
    manager._lock = threading.Lock()
    manager._goal_reserved = True
    manager._state_name = 'IDLE'
    manager._localization = None
    manager._localization_at = 99.9
    manager._path_reason = 'invalid test path'
    goal = Mock(is_cancel_requested=False, request=NS(validation_mode=False))
    rig.goal = goal

    def child(kind):
        handle = Mock(accepted=True, goal_id=NS(uuid=list(range(16))))

        def send(request, **kwargs):
            getattr(rig, kind + '_calls').append(request)
            if rig.cancel_phase == kind:
                clock.jobs.append((clock.now + 0.10,
                                   lambda: setattr(goal, 'is_cancel_requested', True)))
            result = NS(success=rig.hardware_success if kind == 'hardware' else True,
                        code=0, message='hardware fault', distance_traveled=0.53,
                        release_attempts=1, physically_clear=True,
                        final_dock_state='CLEAR_CONFIRMED', motors_enabled=True)

            def complete():
                if kind == 'hardware':
                    rig.state = replace(rig.state, charge_state=0, motors_enabled=True,
                                        dock_state='CLEAR_CONFIRMED', physically_clear=True)
                elif not rig.zero:
                    rig.state = replace(rig.state, stationary=False)
                terminal.set_result(NS(status=5 if goal.is_cancel_requested else 4, result=result))
            terminal = Future()
            clock.jobs.append((clock.now + 0.25, complete))
            handle.get_result_async.return_value = terminal
            return clock.later(0.05, handle)
        client = Mock()
        client.send_goal_async.side_effect = send
        return client, handle
    manager._hardware_undock_client, rig.hardware_handle = child('hardware')
    manager._turn_client, rig.turn_handle = child('turn')
    return rig


@pytest.mark.parametrize('pose', ['missing', 'stale', 'invalid', 'fresh'])
def test_standard_reverse_turn_and_zero_need_no_amcl(rig, pose):
    rig.state = replace(rig.state, localization_fresh=pose in ('invalid', 'fresh'),
                        localization_valid=pose in ('stale', 'fresh'))
    result = rig.manager._execute_callback(rig.goal)
    assert result.success, result.message
    assert result.distance_traveled == 0.53
    assert len(rig.hardware_calls) == len(rig.turn_calls) == 1
    assert rig.hardware_calls[0].clearance_distance == 0.53
    assert rig.turn_calls[0].target_yaw == math.pi
    assert rig.clock.now >= 101.75  # precheck, both children and stable zero
    assert rig.manager._state_name == 'IDLE'
    assert not rig.manager._goal_reserved
    rig.manager._request_localization_update.assert_not_called()
    rig.manager._initial_pose_pub.publish.assert_not_called()
    rig.goal.succeed.assert_called_once()


def test_delayed_hardware_handoff_does_not_request_amcl(rig):
    rig.state = replace(rig.state, dock_state='DEPARTING', motors_enabled=True)
    rig.clock.jobs.append((100.15, lambda: setattr(
        rig, 'state', replace(rig.state, dock_state='CLEAR_CONFIRMED'))))
    assert rig.manager._wait_for_handoff(rig.goal, 7, 0.5, 'offline') == (True, '')
    rig.manager._request_localization_update.assert_not_called()


@pytest.mark.parametrize('localized', [False, True])
@pytest.mark.parametrize('bad', [
    dict(hardware_state_valid=False), dict(fault_flags=1), dict(estop_pressed=True),
    dict(charge_state=0), dict(dock_state='UNKNOWN'),
    dict(undock_profile_commissioned=False), dict(navigation_idle=False),
    dict(command_path_valid=False),
])
def test_localization_never_changes_precheck_safety(rig, localized, bad):
    rig.state = replace(rig.state, localization_valid=localized,
                        localization_fresh=localized, **bad)
    result = rig.manager._execute_callback(rig.goal)
    assert not result.success
    assert not rig.hardware_calls and not rig.turn_calls


@pytest.mark.parametrize('bad', [dict(odom_fresh=False), dict(stationary=False),
                                 dict(session_generation=8), dict(fault_flags=1),
                                 dict(navigation_idle=False)])
def test_handoff_hardware_failure_still_blocks_turn(rig, bad):
    rig.state = replace(rig.state, dock_state='CLEAR_CONFIRMED', motors_enabled=True, **bad)
    ok, reason = rig.manager._wait_for_handoff(rig.goal, 7, 0.5, 'offline')
    assert not ok and reason
    rig.manager._request_localization_update.assert_not_called()


@pytest.mark.parametrize('phase', ['hardware', 'turn'])
def test_cancel_reaches_current_child_without_success_or_next_motion(rig, phase):
    rig.cancel_phase = phase
    result = rig.manager._execute_callback(rig.goal)
    assert not result.success and result.code == 9
    getattr(rig, phase + '_handle').cancel_goal_async.assert_called_once()
    assert len(rig.turn_calls) == (phase == 'turn')
    rig.goal.succeed.assert_not_called()


def test_zero_confirmation_remains_required(rig):
    rig.zero = False
    result = rig.manager._execute_callback(rig.goal)
    assert not result.success
    assert 'zero velocity' in result.message


def test_readiness_uses_hardware_undock_snapshot_instead_of_navigation(rig):
    manager = rig.manager
    snapshot = manager._snapshot
    manager._snapshot = Mock(side_effect=snapshot)
    manager._base_state_at = manager._sonar_at = manager._odom_at = rig.clock.now
    manager._redock_inhibit_remaining = 0.0
    manager._readiness_pub = Mock()
    manager.get_clock = Mock()
    manager._readiness_timer()
    manager._snapshot.assert_called_once_with(for_undock=True)
    message = manager._readiness_pub.publish.call_args.args[0]
    assert message.ready_for_undock
    assert not message.localization_valid
