"""Offline safety boundaries for the real Nav2 closed-loop runner."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[2]


def test_both_trees_bound_recovery_and_recheck_goal_updates():
    for path in (ROOT / 'ros2_ws/src/patrolbot_navigation/config/behavior_trees').glob('*.xml'):
        tree = ET.parse(path)
        recovery = tree.find('.//RecoveryNode[@name="NavigateRecovery"]')
        assert int(recovery.attrib['number_of_retries']) == 9
        fallback = recovery.find('./Sequence/ReactiveFallback')
        assert fallback[0].tag == 'GoalUpdated'
        actions = fallback.find('./RoundRobin')
        assert [node.tag for node in actions] == ['Sequence', 'BackUp', 'Spin', 'Wait']
        assert float(actions.find('./BackUp').attrib['backup_dist']) == .10
        assert float(actions.find('./BackUp').attrib['backup_speed']) == .05


def load_contract():
    spec = importlib.util.spec_from_file_location('navigation_capture', ROOT / 'docker/tests/navigation_capture.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_success_requires_action_and_external_plant_arrival():
    check = load_contract().validate_outcome
    capture = {'scenario': 'reach', 'status': 4, 'distance_m': .5,
               'position_error_m': .05, 'yaw_error_rad': .02, 'drive_nonzero': True}
    check(capture)
    for field, value in [('status', 6), ('distance_m', 0.), ('position_error_m', .4), ('drive_nonzero', False)]:
        with pytest.raises(AssertionError):
            check(dict(capture, **{field: value}))


@pytest.mark.parametrize('scenario,status', [('cancel', 5), ('progress-failure', 6)])
def test_terminal_failure_or_cancel_requires_a_stopped_plant(scenario, status):
    check = load_contract().validate_outcome
    capture = {'scenario': scenario, 'status': status, 'stopped': True,
               'terminal_commands': [(1., 'DRIVE:0:0')]}
    check(capture)
    with pytest.raises(AssertionError):
        check(dict(capture, stopped=False))


@pytest.mark.parametrize('scenario', ['stale-scan', 'lidar-off', 'origin-change'])
def test_sensor_or_origin_hold_requires_no_forwarded_motion(scenario):
    check = load_contract().validate_outcome
    capture = {'scenario': scenario, 'stopped': True, 'blocked_nonzero': True}
    check(capture)
    for key in ('stopped', 'blocked_nonzero'):
        with pytest.raises(AssertionError):
            check(dict(capture, **{key: False}))


def test_partial_startup_cleanup_preserves_original_error_and_emits_evidence(capsys):
    import ast
    import json
    from types import SimpleNamespace

    source = ast.parse((ROOT / 'docker/tests/integration_navigation_closed_loop.py').read_text())
    final = next(node.finalbody for node in source.body if isinstance(node, ast.Try))
    def unavailable():
        raise AttributeError('partially constructed epoch')
    def denied(_):
        raise PermissionError('read-only evidence directory')
    capture = {'passed': False, 'failure': 'original startup error'}
    namespace = dict(sys=SimpleNamespace(exc_info=lambda: (ValueError, None, None)),
        capture=capture, bridge=SimpleNamespace(destroy_node=lambda: None), readiness=unavailable, fake=None,
        node=None, executor=None, launch=None, base_launch=None, log=None,
        rclpy=SimpleNamespace(ok=lambda: False), json=json,
        args=SimpleNamespace(output=SimpleNamespace(write_text=denied)))
    exec(compile(ast.Module(body=final, type_ignores=[]), '<runner teardown>', 'exec'), namespace)
    assert capture['failure'] == 'original startup error'
    assert 'partially constructed epoch' in capture['cleanup_errors'][0]
    assert 'original startup error' in capsys.readouterr().out


def test_runner_launches_production_arbiter_without_relaying_velocity():
    source = (ROOT / 'docker/tests/integration_navigation_closed_loop.py').read_text()
    assert "['ros2', 'launch', 'patrolbot-launch', 'bringup.xml']" in source
    assert 'create_publisher(Twist' not in source


def test_frozen_plant_does_not_mask_continuing_nonzero_drive():
    check = load_contract().validate_outcome
    for commands in ([], [(1., 'DRIVE:0.1:0')],
                     [(1., 'DRIVE:0:0'), (2., 'DRIVE:0:0.2')]):
        with pytest.raises(AssertionError, match='DRIVE'):
            check({'scenario': 'progress-failure', 'status': 6, 'stopped': True,
                   'terminal_commands': commands})


def test_terminal_output_allows_bounded_smoother_deceleration():
    check = load_contract().validate_outcome
    check({'scenario':'cancel','status':5,'stopped':True,
        'terminal_commands':[(1,'DRIVE:.15:0'),(1,'DRIVE:.08:0'),
                             (1,'DRIVE:0:0'),(1,'DRIVE:0:0')]})


def test_stall_log_requires_ordered_failure_clear_and_retry():
    parse = load_contract().stall_recovery_evidence
    lines = [
        '[controller_server]: Failed to make progress',
        '[local_costmap.local_costmap]: Received request to clear entirely the local_costmap',
        '[controller_server]: Received a goal, begin computing control effort.']
    assert parse('\n'.join(lines)) == lines
    for bad in (lines[:1], lines[1:], list(reversed(lines)),
                [lines[0], lines[2], lines[1]]):
        assert not parse('\n'.join(bad))


def test_bounded_stall_cancel_requires_real_recovery_and_unchanged_plant():
    check = load_contract().validate_outcome
    capture = dict(scenario='bounded-stall-cancel', status=5, stopped=True,
        terminal_commands=[(1., 'DRIVE:0:0')], stall_recovery_log=[
            '[controller_server]: Failed to make progress',
            '[local_costmap.local_costmap]: Received request to clear entirely the local_costmap',
            '[controller_server]: Received a goal, begin computing control effort.'],
        frozen_start_pose=[1., 2., 0.], world_pose=[1., 2., 0.],
        no_success_before_cancel=True, case_elapsed_s=90.)
    check(capture)
    for key,value in [('status',4), ('stall_recovery_log',[]),
                      ('world_pose',[1.01,2.,0.]), ('no_success_before_cancel',False),
                      ('case_elapsed_s',121.), ('terminal_commands',[(1.,'DRIVE:.1:0')])]:
        with pytest.raises(AssertionError):
            check(dict(capture, **{key:value}))
