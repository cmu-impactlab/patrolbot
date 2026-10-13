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


def test_actual_recovery_requires_both_cycles_motion_and_safe_terminal_state():
    check = load_contract().validate_outcome
    capture = dict(scenario='blocked-goal', status=6, case_elapsed_s=90., stopped=True,
        terminal_latched_drive='DRIVE:0:0', terminal_commands=[],
        terminal_observation_s=.6, terminal_feedback_fresh=True, drive_nonzero=True,
        actual_recovery_stages=['backup', 'spin', 'wait'] * 2,
        outer_recovery_stages=['clear', 'backup', 'spin', 'wait'] * 2 + ['clear'],
        planner_failure_observed=True, full_outer_clears=3, navigate_error_code=204,
        actual_backup_motion=True, actual_spin_motion=True)
    check(capture)
    for key, value in [('status', 4), ('actual_recovery_stages', ['backup']),
        ('planner_failure_observed', False), ('full_outer_clears', 1),
        ('drive_nonzero', False), ('actual_backup_motion', False),
        ('actual_spin_motion', False), ('navigate_error_code', 0),
        ('case_elapsed_s', 121.), ('stopped', False),
        ('terminal_latched_drive', 'DRIVE:.1:0'), ('terminal_latched_drive', None),
        ('terminal_commands', [(1., 'DRIVE:0:.2')]), ('terminal_observation_s', .2),
        ('terminal_feedback_fresh', False), ('outer_recovery_stages', ['clear', 'backup'])]:
        with pytest.raises(AssertionError):
            check(dict(capture, **{key: value}))


def test_outer_recovery_pairs_both_costmaps_and_excludes_contextual_global_clears():
    parse = load_contract().outer_recovery_stages
    local = '[local_costmap.local_costmap]: Received request to clear entirely the local_costmap'
    global_ = '[global_costmap.global_costmap]: Received request to clear entirely the global_costmap'
    backup = '[behavior_server]: Running backup'
    spin = '[behavior_server]: Running spin'
    wait = '[behavior_server]: Running wait'
    assert parse('\n'.join([local, global_, global_, backup, spin, wait,
        local, global_, backup, spin, wait, local, global_, global_])) == [
            'clear', 'backup', 'spin', 'wait'] * 2 + ['clear']
    assert parse('\n'.join([global_, backup])) == ['backup']


def test_recovery_evidence_waits_for_log_producer_shutdown(tmp_path):
    import ast
    import json
    import time
    from types import SimpleNamespace

    source = ast.parse((ROOT / 'docker/tests/integration_navigation_closed_loop.py').read_text())
    final = next(node.finalbody for node in source.body if isinstance(node, ast.Try))
    local = '[local_costmap.local_costmap]: Received request to clear entirely the local_costmap'
    global_ = '[global_costmap.global_costmap]: Received request to clear entirely the global_costmap'
    cycle = [local, global_] + ['[behavior_server]: Running ' + name for name in ('backup', 'spin', 'wait')]
    log_path = tmp_path / 'raw.log'
    log_path.write_text('\n'.join(cycle * 2))
    def flush_last_clear():
        with log_path.open('a') as stream:
            stream.write('\n' + local + '\n' + global_)
    capture = {'scenario': 'blocked-goal'}
    def validate(value):
        assert value['outer_recovery_stages'] == ['clear', 'backup', 'spin', 'wait'] * 2 + ['clear']
    namespace = dict(sys=SimpleNamespace(exc_info=lambda: (None, None, None)),
        capture=capture, bridge=None, fake=None, node=None, executor=None,
        launch=None, base_launch=None, log=SimpleNamespace(close=flush_last_clear),
        log_path=log_path, navigation_log_offset=0,
        rclpy=SimpleNamespace(ok=lambda: False), json=json, time=time,
        deadline=time.monotonic()+120, outer_recovery_stages=load_contract().outer_recovery_stages,
        validate_outcome=validate, args=SimpleNamespace(output=tmp_path / 'capture.json'))
    exec(compile(ast.Module(body=final, type_ignores=[]), '<runner teardown>', 'exec'), namespace)
    assert capture.get('passed') is True
