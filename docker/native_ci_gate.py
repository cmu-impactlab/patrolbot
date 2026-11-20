"""Bounded native pytest runner and evidence plugin; no ROS mocks or retries."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


EXCLUSIONS = ('test/test_copyright.py', 'test/test_flake8.py', 'test/test_pep257.py')
SENTINELS = {
    'bridge': (
        'test/test_fake_sbc_integration.py::test_release_round_trip_and_duplicate_never_emit_velocity',
        'test/test_manual_undock_native.py::test_start_dispatches_once_without_nav2_or_localization_reset_ack',
    ),
    'dock_manager': (
        'test/test_manager_integration.py::test_operator_dock_pose_initialization_and_hardware_undock',
    ),
    'navigation': (
        'test/test_collision_monitor_integration.py::test_front_zone_blocks_forward_but_not_reverse',
        'test/test_collision_monitor_integration.py::test_rear_zone_blocks_reverse_but_not_forward',
        'test/test_collision_monitor_integration.py::test_stale_rear_source_stops_reverse',
    ),
}


# These hooks run only in the child pytest process, including on collection failure.
def event(kind, **fields):
    with open(os.environ['PATROLBOT_NATIVE_EVENTS'], 'a') as stream:
        stream.write(json.dumps({'event': kind, **fields}) + '\n')


def pytest_collection_finish(session):
    event('collection', nodeids=[item.nodeid for item in session.items])


def pytest_deselected(items):
    event('deselected', nodeids=[item.nodeid for item in items])


def pytest_collectreport(report):
    if report.failed or report.skipped:
        event('collection_problem', nodeid=report.nodeid, outcome=report.outcome,
              reason=str(report.longrepr))


def pytest_runtest_logreport(report):
    event('report', nodeid=report.nodeid, when=report.when,
          outcome=report.outcome, reason=str(report.longrepr) if report.longrepr else '')


def pytest_warning_recorded(warning_message, when, nodeid, location):
    event('warning', category=warning_message.category.__name__,
          reason=str(warning_message.message), when=when, nodeid=nodeid)


def pytest_sessionfinish(session, exitstatus):
    event('finish', exitstatus=int(exitstatus))


def check_evidence(directory, sentinels):
    """Require collection, every test phase, clean shutdown, and parseable JUnit."""
    issues = []
    try:
        events = [json.loads(line) for line in (directory / 'events.jsonl').read_text().splitlines()]
    except (OSError, ValueError) as error:
        return [f'missing or invalid pytest evidence: {error}']
    collections = [item for item in events if item['event'] == 'collection']
    nodeids = collections[-1]['nodeids'] if collections else []
    if not nodeids:
        issues.append('empty or missing collection')
    finishes = [item for item in events if item['event'] == 'finish']
    if not finishes or finishes[-1]['exitstatus'] != 0:
        issues.append('missing or unsuccessful pytest session finish')
    reports = {}
    for item in events:
        if item['event'] == 'report':
            reports.setdefault(item['nodeid'], {})[item['when']] = item['outcome']
            if item['outcome'] != 'passed':
                issues.append(f"{item['nodeid']} {item['when']}: {item['outcome']}: {item['reason']}")
        elif item['event'] == 'collection_problem':
            issues.append(f"collection {item['outcome']}: {item['nodeid']}: {item['reason']}")
        elif item['event'] == 'deselected' and item['nodeids']:
            issues.append(f"unexpected deselected tests: {item['nodeids']}")
        elif item['event'] == 'warning' and (
                item['category'] in (
                    'PytestUnhandledThreadExceptionWarning', 'PytestUnraisableExceptionWarning')
                or (item['category'] == 'RuntimeWarning' and 'was never awaited' in item['reason'])):
            issues.append(f"{item['category']}: {item['reason']}")
    for nodeid in nodeids:
        if reports.get(nodeid) != dict.fromkeys(('setup', 'call', 'teardown'), 'passed'):
            issues.append(f'incomplete or unsuccessful setup/call/teardown: {nodeid}')
    for nodeid in sentinels:
        if nodeid not in nodeids:
            issues.append(f'missing required integration sentinel: {nodeid}')
    try:
        junit = ET.parse(directory / 'junit.xml')
        if len(junit.findall('.//testcase')) != len(nodeids) or not nodeids:
            issues.append('JUnit testcase coverage differs from collection')
    except (OSError, ET.ParseError) as error:
        issues.append(f'missing or invalid JUnit: {error}')
    return issues


def retire_process_group(process):
    """Signal only this suite's group and bound both retirement waits."""
    started = time.monotonic()
    signals = []
    for sig, grace in ((signal.SIGTERM, 5), (signal.SIGKILL, 1)):
        process.poll()  # Reap pytest before probing for surviving children.
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return {'signals_sent': signals, 'forced_cleanup': 'SIGKILL' in signals,
                    'group_retired': True, 'elapsed_seconds': time.monotonic() - started}
        signals.append(sig.name)
        deadline = time.monotonic() + grace
        while True:
            process.poll()
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return {'signals_sent': signals, 'forced_cleanup': 'SIGKILL' in signals,
                        'group_retired': True, 'elapsed_seconds': time.monotonic() - started}
            if time.monotonic() >= deadline:
                break
            time.sleep(0.05)
    return {'signals_sent': signals, 'forced_cleanup': 'SIGKILL' in signals,
            'group_retired': False, 'elapsed_seconds': time.monotonic() - started}


def run_suite(name, package, evidence, deadline, paths=()):
    directory = evidence / name
    directory.mkdir(parents=True, exist_ok=False)
    (directory / 'ros').mkdir()
    command = [sys.executable, '-B', '-m', 'pytest', '-q', '-ra', '--strict-config',
               '-p', 'no:cacheprovider', '-p', 'native_ci_gate',
               '-W', 'error::pytest.PytestUnhandledThreadExceptionWarning',
               '-W', 'error::pytest.PytestUnraisableExceptionWarning',
               '-W', 'error:coroutine .* was never awaited:RuntimeWarning',
               f'--junitxml={directory / "junit.xml"}',
               *(f'--ignore={path}' for path in EXCLUSIONS), *(paths or ('test',))]
    env = {**os.environ, 'PYTHONPATH': os.pathsep.join(
        filter(None, (str(Path(__file__).resolve().parent), os.environ.get('PYTHONPATH')))),
        'PATROLBOT_NATIVE_EVENTS': str(directory / 'events.jsonl'),
        'ROS_LOG_DIR': str(directory / 'ros'), 'PYTHONDONTWRITEBYTECODE': '1'}
    for key in ('PYTEST_ADDOPTS', 'PYTEST_PLUGINS', 'PYTEST_DISABLE_PLUGIN_AUTOLOAD'):
        env.pop(key, None)
    started = time.monotonic()
    timed_out = False
    leaked_children = False
    with (directory / 'pytest.log').open('w') as output:
        process = subprocess.Popen(command, cwd=package, env=env, stdout=output,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        try:
            process.wait(timeout=deadline)
        except subprocess.TimeoutExpired:
            timed_out = True
        finally:
            # A completed pytest must not leave native children for the next suite.
            if not timed_out:
                try:
                    os.killpg(process.pid, 0)
                    leaked_children = True
                except ProcessLookupError:
                    pass
            cleanup = retire_process_group(process)
    issues = check_evidence(directory, SENTINELS.get(name, ()))
    if leaked_children:
        issues.append('pytest left a live process group after shutdown')
    if timed_out:
        issues.append(f'suite deadline exceeded: {deadline}s')
    if cleanup['forced_cleanup']:
        issues.append('suite process group required SIGKILL')
    if not cleanup['group_retired']:
        issues.append('suite process group did not retire within cleanup deadline')
    if process.returncode:
        issues.append(f'pytest process exit: {process.returncode}')
    summary = {'suite': name, 'command': command, 'cwd': str(package),
               'expected_exclusions': list(EXCLUSIONS), 'allowed_skips': [],
               'deadline_seconds': deadline, 'elapsed_seconds': time.monotonic() - started,
               'returncode': process.returncode, 'timed_out': timed_out,
               'leaked_children': leaked_children,
               'process_group_cleanup': cleanup, 'issues': issues}
    (directory / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(f'=== {name} ===', flush=True)
    with (directory / 'pytest.log').open() as output:
        for line in output:
            print(line, end='', flush=True)
    print(json.dumps(summary), flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    paths = sys.argv[1:]
    workspace = Path(os.environ.get('PATROLBOT_TEST_WORKSPACE', '/opt/patrolbot')).resolve()
    evidence = Path(os.environ.get('PATROLBOT_TEST_EVIDENCE', '/tmp/native-test-evidence')).resolve()
    evidence.mkdir(parents=True, exist_ok=True)
    deadline = float(os.environ.get('PATROLBOT_TEST_TIMEOUT', '300'))
    if not 0 < deadline <= 600:
        parser.error('PATROLBOT_TEST_TIMEOUT must be in (0, 600] seconds')
    suites = [('custom', workspace)] if paths else [
        (name, workspace / 'ros2_ws/src' / f'patrolbot_{name}') for name in SENTINELS]
    results = []
    for name, package in suites:
        try:
            results.append(run_suite(name, package, evidence, deadline, paths))
        except Exception as error:
            # A runner/evidence failure must not hide subsequent package results.
            results.append({'suite': name, 'issues': [f'runner failure: {error}']})
    (evidence / 'summary.json').write_text(json.dumps(results, indent=2) + '\n')
    return int(any(result['issues'] for result in results))


if __name__ == '__main__':
    sys.exit(main())
