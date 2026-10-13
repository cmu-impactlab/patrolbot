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
