"""
Resolve only a validated, explicitly committed ACTIVE startup configuration.

Legacy startup is unchanged until tuning is deliberately enabled. This resolver
never initializes a store, promotes DESIRED state, or contacts any ROS node.
"""

import argparse
import os
from pathlib import Path
import tempfile

from .tuning_json import loads
from .tuning_store import _read_bytes, RevisionStore, StoreUnavailable
from .tuning_yaml import strict_yaml_dump, strict_yaml_load


def merge(base, overlay):
    result = dict(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge(result[key], value)
        else:
            result[key] = value
    return result


def state_expected(state_dir):
    """Previously initialized or interrupted state never permits a defaults fallback."""
    state = Path(state_dir)
    marker = state.parent / 'tuning-initialized.json'
    if marker.exists() or marker.is_symlink() or state.is_symlink():
        return True
    try:
        return any(path.name != 'lock' for path in state.iterdir())
    except FileNotFoundError:
        return False
    except OSError:
        return True


def resolve(default_file, scope, *, enabled=None, state_dir=None, registry_file=None):
    if scope not in ('bringup', 'navigation'):
        raise ValueError('unknown tuning scope')
    enabled = (os.environ.get('PATROLBOT_TUNING_ENABLED', '0') == '1'
               if enabled is None else enabled)
    state_dir = state_dir or os.environ.get(
        'PATROLBOT_TUNING_DIR',
        str(Path(os.environ.get('PATROLBOT_STATE_DIR', '/var/lib/patrolbot')) / 'config'))
    if not enabled and not state_expected(state_dir):
        return str(default_file)
    if registry_file is None:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('patrolbot_bridge'))
        registry_file = share / 'contracts/robot-tuning.registry.json'
    registry = loads(_read_bytes(Path(registry_file), 512 * 1024).decode('utf-8'), 512 * 1024)
    store = RevisionStore(state_dir, registry)
    pointer = store.read_startup()
    active = store.snapshot_state()['active_revision']
    if active != pointer['revision']:
        raise StoreUnavailable(
            'startup_revision_mismatch', 'refusing a partially promoted artifact')
    defaults = strict_yaml_load(_read_bytes(Path(default_file), 1024 * 1024).decode('utf-8'))
    overlay = strict_yaml_load(store.load_revision(active)['generated'][scope + '.yaml'])
    document = merge(defaults, overlay)
    fd, path = tempfile.mkstemp(prefix='patrolbot-active-' + scope + '-', suffix='.yaml')
    with os.fdopen(fd, 'w') as stream:
        stream.write(strict_yaml_dump(document))
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scope', required=True, choices=['bringup', 'navigation'])
    args = parser.parse_args()
    from ament_index_python.packages import get_package_share_directory
    package, relative = (('patrolbot-launch', 'param/defaults/smoother.yaml')
                         if args.scope == 'bringup'
                         else ('patrolbot_navigation', 'config/nav2_params.yaml'))
    print(resolve(Path(get_package_share_directory(package)) / relative, args.scope))
