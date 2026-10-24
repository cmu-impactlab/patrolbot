"""Launch a fixed consumer role while retaining the shared initialization guard."""

import argparse
import os
from pathlib import Path

from .tuning_initialize import RuntimeGuard
from .tuning_startup import resolve, state_expected


def prepare(role):
    if role not in ('bringup', 'navigation'):
        raise ValueError('unknown fixed robot role')
    # Never inherit a candidate path across an ordinary Docker/process restart.
    os.environ.pop('PATROLBOT_NAVIGATION_PARAMS_FILE', None)
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
        from .tuning_nav2 import BOOT_SOCKET, request
        # An absent helper means ordinary ACTIVE startup. An installed but
        # unavailable/malformed helper cannot silently substitute another boot.
        if Path(BOOT_SOCKET).exists():
            from ament_index_python.packages import get_package_share_directory
            from .tuning_startup import navigation_boot
            default = Path(get_package_share_directory('patrolbot_navigation')) / 'config/nav2_params.yaml'
            registry = Path(get_package_share_directory('patrolbot_bridge')) / 'contracts/robot-tuning.registry.json'
            reply = request(BOOT_SOCKET, {'op': 'boot'})
            path = navigation_boot(default, state_dir=state_dir, registry_file=registry, boot_reply=reply)
            os.environ['PATROLBOT_NAVIGATION_PARAMS_FILE'] = path
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
