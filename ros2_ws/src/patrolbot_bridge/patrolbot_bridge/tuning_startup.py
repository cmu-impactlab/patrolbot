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


def resolve_candidate(default_file, scope, *, store, authorization, public_key,
                      launch_receipt, now, host_boot_id):
    """Explicit host-authorized candidate API, NEVER called by ordinary resolve.

    Returns (temporary YAML path, exact boot binding). The caller must establish
    retained command maintenance before launching consumers, authenticate their
    new identities and verify the complete revision before promotion/release.
    Shipped tuning_run intentionally does not use this until those hooks exist.
    """
    from .tuning_candidate_boot import authorize
    binding = authorize(store, scope=scope, authorization=authorization,
                        public_key=public_key, launch_receipt=launch_receipt,
                        now=now, host_boot_id=host_boot_id)
    defaults = strict_yaml_load(_read_bytes(Path(default_file), 1024 * 1024).decode('utf-8'))
    document = merge(defaults, strict_yaml_load(binding['artifact']))
    fd, path = tempfile.mkstemp(prefix='patrolbot-candidate-' + scope + '-', suffix='.yaml')
    with os.fdopen(fd, 'w') as stream:
        stream.write(strict_yaml_dump(document))
    return path, {key: value for key, value in binding.items() if key != 'artifact'}


def navigation_boot(default_file, *, state_dir, registry_file, boot_reply):
    """Validate the fixed helper's one-use boot reply; never select DESIRED.

    Only tuning_run calls this after receiving data from its private boot socket.
    The old three-container signed-candidate API above is not used by this path.
    """
    if not isinstance(boot_reply, dict) or boot_reply.get('ok') is not True:
        raise StoreUnavailable('navigation_boot_unavailable')
    if boot_reply.get('state') == 'active' and set(boot_reply) == {'ok', 'state'}:
        return resolve(default_file, 'navigation', state_dir=state_dir, registry_file=registry_file)
    from .tuning_nav2 import candidate_for_job
    from .tuning_store import _canonical_uuid
    keys = {'ok', 'state', 'job_id', 'digest', 'plan_hash', 'revision', 'artifact', 'boot_id'}
    if (set(boot_reply) != keys or boot_reply['state'] != 'candidate'
            or not _canonical_uuid(boot_reply['boot_id'])):
        raise StoreUnavailable('invalid_navigation_boot_binding')
    registry = loads(_read_bytes(Path(registry_file), 512 * 1024).decode(), 512 * 1024)
    store = RevisionStore(state_dir, registry)
    candidate = candidate_for_job(store, {key: boot_reply[key] for key in ('job_id', 'digest', 'plan_hash')})
    if any(boot_reply[key] != value for key, value in candidate.items()):
        raise StoreUnavailable('navigation_boot_candidate_mismatch')
    defaults = strict_yaml_load(_read_bytes(Path(default_file), 1024 * 1024).decode())
    document = merge(defaults, strict_yaml_load(candidate['artifact']))
    fd, path = tempfile.mkstemp(prefix='patrolbot-nav2-boot-', suffix='.yaml')
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
