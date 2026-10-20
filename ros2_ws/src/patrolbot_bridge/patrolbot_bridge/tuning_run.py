"""Launch a fixed consumer role while retaining the shared initialization guard."""

import argparse
import os
from pathlib import Path

from .tuning_initialize import RuntimeGuard
from .tuning_startup import resolve, state_expected


def prepare(role):
    if role not in ('bringup', 'navigation'):
        raise ValueError('unknown fixed robot role')
    default_state = Path(os.environ.get('PATROLBOT_STATE_DIR', '/var/lib/patrolbot')) / 'config'
    state_dir = os.environ.get('PATROLBOT_TUNING_DIR', str(default_state))
    guard = None
    try:
        guard = RuntimeGuard(state_dir, read_only=True)
        os.set_inheritable(guard.fd, True)
    except FileNotFoundError:
        if (os.environ.get('PATROLBOT_TUNING_ENABLED', '0') == '1'
                or state_expected(state_dir)):
            raise
    if role == 'bringup':
        from ament_index_python.packages import get_package_share_directory
        default = (Path(get_package_share_directory('patrolbot-launch'))
                   / 'param/defaults/smoother.yaml')
        os.environ['PATROLBOT_SMOOTHER_PARAMS_FILE'] = resolve(default, role)
        command = ['ros2', 'launch', 'patrolbot-launch', 'bringup.xml']
    else:
        command = ['ros2', 'launch', 'patrolbot_navigation', 'bringup.launch.py']
    return guard, command


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', choices=['bringup', 'navigation'], required=True)
    args = parser.parse_args(argv)
    guard, command = prepare(args.role)
    try:
        os.execvpe(command[0], command, os.environ)
    finally:
        if guard is not None:
            guard.close()
