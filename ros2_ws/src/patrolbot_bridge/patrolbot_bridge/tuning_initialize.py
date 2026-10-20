"""
Explicit, local, reviewed clean-install initialization; never a ROS command.

The core process holds a shared runtime guard. This tool needs its exclusive
guard, so initialization cannot race a live core. A durable marker outside the
config directory prevents deletion/interruption from looking like a clean install.
"""

import argparse
import fcntl
import json
import os
from pathlib import Path
import stat
import time

from . import tuning_spec
from .tuning_json import loads
from .tuning_store import _atomic_write, RevisionStore, StoreUnavailable


def marker_path(state_dir):
    return Path(state_dir).parent / 'tuning-initialized.json'


class RuntimeGuard:
    """Serialize local initialization against the existing core process."""

    def __init__(self, state_dir, *, exclusive=False, read_only=False):
        path = Path(state_dir).parent / 'tuning-runtime.lock'
        if not path.parent.is_dir() or path.parent.is_symlink():
            raise StoreUnavailable('durable_directory_unavailable')
        flags = os.O_RDONLY if read_only else os.O_RDWR | os.O_CREAT
        self.fd = os.open(path, flags | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0), 0o600)
        try:
            if not stat.S_ISREG(os.fstat(self.fd).st_mode):
                raise StoreUnavailable('invalid_runtime_guard')
            mode = fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH
            fcntl.flock(self.fd, mode | fcntl.LOCK_NB)
        except BaseException:
            os.close(self.fd)
            self.fd = None
            raise

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


def reviewed_candidate(registry):
    """Compute a reviewable candidate from the current YAML, never library defaults."""
    index = tuning_spec.index_registry(registry)
    values = tuning_spec.yaml_starting_values(index)
    artifacts = tuning_spec.generated_artifacts(values, index)
    return {
        'registry_digest': tuning_spec.registry_digest(registry),
        'defaults_digest': tuning_spec.defaults_digest(),
        'generated_digest': tuning_spec.artifacts_digest(artifacts),
    }


def initialize_clean(state_dir, registry, *, expected, reason, confirm_clean_install=False,
                     _effective_uid=None):
    """
    Initialize a clean install only after an explicit local administrator review.

    This does not activate runtime nodes. Existing/missing prior state is never
    repaired or reset. The UID injection is for unit tests, not a wire input.
    """
    uid = os.geteuid() if _effective_uid is None else _effective_uid
    if type(uid) is not int or uid != 0:
        raise StoreUnavailable('local_administrator_required')
    if confirm_clean_install is not True:
        raise StoreUnavailable('explicit_clean_install_confirmation_required')
    if not isinstance(reason, str) or not reason.strip() or len(reason) > 512:
        raise StoreUnavailable('explicit_initialization_reason_required')
    candidate = reviewed_candidate(registry)
    if not isinstance(expected, dict) or expected != candidate:
        raise StoreUnavailable('reviewed_candidate_changed')
    store = RevisionStore(state_dir, registry)
    with RuntimeGuard(state_dir, exclusive=True):
        if marker_path(state_dir).exists() or marker_path(state_dir).is_symlink():
            raise StoreUnavailable('previous_initialization_requires_inspection')
        if store.state()['status'] != 'uninitialized':
            raise StoreUnavailable('already_initialized')
        # Revalidate immediately before the irreversible marker/write sequence.
        if reviewed_candidate(registry) != candidate:
            raise StoreUnavailable('reviewed_candidate_changed')
        values = tuning_spec.yaml_starting_values(store.index)
        artifacts = tuning_spec.generated_artifacts(values, store.index)
        if tuning_spec.artifacts_digest(artifacts) != candidate['generated_digest']:
            raise StoreUnavailable('reviewed_candidate_changed')
        _atomic_write(marker_path(state_dir), json.dumps({
            'schema_version': 1, 'candidate': candidate, 'started_at': time.time(),
        }, sort_keys=True).encode())
        result = store._initialize(
            actor={'id': 0, 'username': 'local-root', 'role': 'administrator'},
            reason=reason, provenance={'initialization': 'explicit_local_yaml_review',
                                       'reviewed_candidate': candidate},
            initial_values=values, expected_artifacts=artifacts,
            expected_defaults_digest=candidate['defaults_digest'])
        actual = store.load_revision(result['desired_revision'])['meta']['generated_digest']
        if actual != candidate['generated_digest']:
            raise StoreUnavailable('initialization_outcome_unknown')
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--registry', required=True)
    parser.add_argument('--state-dir', required=True)
    parser.add_argument('--initialize', action='store_true')
    parser.add_argument('--confirm-clean-install', action='store_true')
    parser.add_argument('--registry-digest')
    parser.add_argument('--defaults-digest')
    parser.add_argument('--generated-digest')
    parser.add_argument('--reason', default='')
    args = parser.parse_args(argv)
    try:
        path = Path(args.registry)
        if path.is_symlink() or not path.is_file() or path.stat().st_size > 512 * 1024:
            raise StoreUnavailable('invalid_registry_file')
        registry = loads(path.read_text(), 512 * 1024)
        if not args.initialize:
            print(json.dumps({'candidate': reviewed_candidate(registry),
                              'write_performed': False}))
            return 0
        result = initialize_clean(
            args.state_dir, registry, reason=args.reason,
            confirm_clean_install=args.confirm_clean_install,
            expected={'registry_digest': args.registry_digest,
                      'defaults_digest': args.defaults_digest,
                      'generated_digest': args.generated_digest})
        print(json.dumps({'outcome': 'initialized', 'revision': result['desired_revision'],
                          'runtime_activated': False}))
        return 0
    except (ValueError, OSError) as exc:
        print(json.dumps({'outcome': 'refused', 'detail': str(exc), 'runtime_activated': False}))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
