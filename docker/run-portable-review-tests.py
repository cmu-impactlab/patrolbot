#!/usr/bin/env python3
"""Run offline Python and ARIA-free C++ coverage, never native integration.

Separate pytest processes isolate test-local import stubs and duplicate helper
module names. Native/lint exclusions are explicit rather than relying on an
import failure or an environment-dependent skip. Each C++ binary runs even if
another fails, so an undock failure cannot hide the remaining hardware checks.
"""

import argparse
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
EXCLUDED = {
    # Native ROS types (even when individual tests do not create DDS nodes).
    'test_bridge_queue.py',
    'test_epoch_runtime.py',
    'test_epoch_tuning_hold.py',
    'test_lidar_bridge.py',
    'test_map_runtime.py',
    'test_motor_power.py',
    # Opt-in native ROS / live-stack / Docker suites: never portable defaults.
    'test_deployed_odom_malformed.py',
    'test_fake_sbc_integration.py',
    'test_manager_integration.py',
    'test_collision_monitor_integration.py',
    'test_manual_undock_native.py',
    # Require the ament lint environment, not the offline dependency set.
    'test_copyright.py',
    'test_flake8.py',
    'test_pep257.py',
}
GROUPS = (
    'contracts',
    'ros2_ws/src/patrolbot_bridge/test',
    'ros2_ws/src/patrolbot_navigation/test',
    'ros2_ws/src/patrolbot_dock_manager/test',
    'docker/tests',
    'docker/system-reset',
    'diagnostics/localization',
    'diagnostics/navigation',
    'docker',
)
HARDWARE_TESTS = (
    'tests/protocol_test',
    'tests/undock_test',
    'tests/scan_timing_test',
    'tests/lidar_power_test',
    'tests/encoder_capture_test',
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list', action='store_true', help='list selected files without running')
    args = parser.parse_args()
    env = os.environ.copy()
    # Own source tree comes first, including when the interpreter has editable
    # installs pointing at a different checkout.
    sources = [ROOT / 'ros2_ws/src' / name for name in (
        'patrolbot_bridge', 'patrolbot_navigation', 'patrolbot_dock_manager')]
    env['PYTHONPATH'] = os.pathsep.join(map(str, sources))
    # Defense in depth: an inherited opt-in must never turn this into a live run.
    env['PATROLBOT_RUN_INTEGRATION'] = '0'
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    # Do not inherit additional pytest paths/options from a native test shell.
    env.pop('PYTEST_ADDOPTS', None)
    failures = []
    for group in GROUPS:
        files = [str(path.relative_to(ROOT)) for path in sorted((ROOT / group).glob('test*.py'))
                 if path.name not in EXCLUDED]
        if not files:
            raise RuntimeError('empty portable test group: ' + group)
        print('\nOffline group: ' + group, flush=True)
        if args.list:
            print('\n'.join(files))
            continue
        result = subprocess.run(
            [sys.executable, '-B', '-m', 'pytest', '-q', '-p', 'no:cacheprovider', *files],
            cwd=ROOT, env=env, check=False)
        if result.returncode:
            failures.append(group)
    # This artifact check is a script with main(), not a pytest test function.
    artifact = 'docker/nav2-patches/test_patch_artifact.py'
    if args.list:
        print(artifact)
    elif subprocess.run([sys.executable, '-B', artifact], cwd=ROOT, env=env, check=False).returncode:
        failures.append(artifact)
    hardware = ROOT / 'hardware/patrolbot_hw_server'
    build = ['make', '-C', str(hardware), '-B', *HARDWARE_TESTS]
    print('\nOffline hardware build: ' + ' '.join(build), flush=True)
    if args.list:
        print('\n'.join('hardware/patrolbot_hw_server/' + path for path in HARDWARE_TESTS))
    elif subprocess.run(build, cwd=ROOT, env=env, check=False).returncode:
        failures.append('hardware build')
    else:
        for binary in HARDWARE_TESTS:
            print('\nOffline hardware test: ' + binary, flush=True)
            if subprocess.run([str(hardware / binary)], cwd=hardware, env=env, check=False).returncode:
                failures.append(binary)
    if failures:
        print('Failed groups: ' + ', '.join(failures), file=sys.stderr)
    return bool(failures)


if __name__ == '__main__':
    sys.exit(main())
