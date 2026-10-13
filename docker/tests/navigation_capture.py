"""Assertions shared by isolated Nav2 integration and offline regression tests."""


def validate_outcome(capture):
    scenario = capture['scenario']
    if scenario in ('cancel', 'progress-failure', 'bounded-stall-cancel'):
        assert terminal_output_stopped(capture['terminal_commands']), 'terminal navigation retained nonzero DRIVE output'
    if scenario == 'reach':
        assert capture['status'] == 4, 'NavigateToPose did not succeed'
        assert capture['drive_nonzero'], 'Nav2 never commanded motion'
        assert capture['distance_m'] > .25, 'feedback plant did not traverse a path'
        assert capture['position_error_m'] <= .25, 'plant missed commanded map position'
        assert capture['yaw_error_rad'] <= .50, 'plant missed commanded heading'
    elif scenario == 'cancel':
        assert capture['status'] == 5, 'canceled goal did not terminate canceled'
        assert capture['stopped'], 'plant kept moving after cancellation'
    elif scenario == 'bounded-stall-cancel':
        assert capture['status'] in (5, 6), 'stalled navigation succeeded or did not terminate'
        assert capture['no_success_before_cancel'], 'stalled navigation succeeded before cancellation'
        assert stall_recovery_evidence('\n'.join(capture['stall_recovery_log'])), 'no actual progress failure and contextual retry'
        assert capture['world_pose'] == capture['frozen_start_pose'], 'frozen plant moved'
        assert capture['stopped'], 'stalled plant did not stop'
        assert capture['case_elapsed_s'] <= 120., 'stall case exceeded original budget'
    elif scenario == 'progress-failure':
        assert capture['status'] == 6, 'frozen plant did not abort after bounded recovery'
        assert capture['stopped'], 'terminal navigation failure left motion commanded'
    elif scenario == 'blocked-goal':
        assert capture['status'] == 6, 'outside-map goal did not abort'
        assert capture['case_elapsed_s'] <= 120., 'original case budget exceeded'
        assert capture['planner_failure_observed'], 'actual planner failure not observed'
        assert capture['actual_recovery_stages'] == ['backup', 'spin', 'wait'] * 2, 'full behavior sequence missing'
        assert capture['outer_recovery_stages'] == ['clear', 'backup', 'spin', 'wait'] * 2 + ['clear'], 'full outer sequence missing'
        assert capture['full_outer_clears'] == 3, 'full outer clearing sequence missing'
        assert capture['navigate_error_code'] != 0, 'terminal planner error code missing'
        assert capture['actual_backup_motion'] and capture['actual_spin_motion'], 'recovery feedback motion missing'
        assert capture['drive_nonzero'] and capture['stopped'], 'plant motion or terminal stop missing'
        assert capture['terminal_observation_s'] >= .6, 'terminal observation incomplete'
        assert capture['terminal_feedback_fresh'], 'no stopped feedback after terminal status'
        baseline = capture['terminal_latched_drive']
        assert isinstance(baseline, str) and terminal_output_stopped([(0, baseline)]), 'terminal DRIVE state was nonzero or absent'
        assert terminal_output_stopped([(0, baseline)] + capture['terminal_commands']), 'terminal DRIVE became nonzero'
    elif scenario in ('stale-scan', 'lidar-off', 'origin-change', 'false-amcl-jump'):
        assert capture['stopped'], 'stale scans did not stop the feedback plant'
        assert capture['blocked_nonzero'], 'stale scan hold forwarded nonzero DRIVE'
    else:
        raise AssertionError('unknown navigation scenario')


def terminal_output_stopped(commands):
    drives = [tuple(map(float, line.split(':')[1:])) for _, line in commands
              if line.startswith('DRIVE:')]
    if not drives or any(len(values) != 2 for values in drives):
        return False
    stopped = [all(abs(value) < .001 for value in values) for values in drives]
    # Normal cancellation ramps through the existing smoother; zero must arrive
    # inside the runner's bounded observation and remain zero afterward.
    return any(stopped) and all(stopped[stopped.index(True):])


def stall_recovery_evidence(log):
    markers = (
        '[controller_server]: Failed to make progress',
        '[local_costmap.local_costmap]: Received request to clear entirely the local_costmap',
        '[controller_server]: Received a goal, begin computing control effort.')
    evidence = []
    for line in log.splitlines():
        if markers[len(evidence)] in line:
            evidence.append(line)
            if len(evidence) == len(markers):
                return evidence
    return []


def outer_recovery_stages(log):
    markers = [('local', '[local_costmap.local_costmap]: Received request to clear entirely the local_costmap'),
               ('global', '[global_costmap.global_costmap]: Received request to clear entirely the global_costmap')]
    markers += [(name, '[behavior_server]: Running ' + name) for name in ('backup', 'spin', 'wait')]
    events = [name for line in log.splitlines() for name, marker in markers if marker in line]
    return ['clear' if name == 'global' else name for i, name in enumerate(events)
            if name in ('backup', 'spin', 'wait') or (name == 'global' and i and events[i-1] == 'local')]
