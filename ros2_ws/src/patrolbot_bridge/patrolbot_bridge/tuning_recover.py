"""Explicit offline last-good rollback of an interrupted configuration job."""

import argparse
import json
import os
from pathlib import Path

from .tuning_initialize import RuntimeGuard
from .tuning_json import loads
from .tuning_store import RevisionStore, SCOPES, StoreUnavailable


def recover(state_dir, registry, *, job_id, revision, digest, reason,
            confirm_stopped=False, _effective_uid=None):
    """
    Restore exact last-good startup files while every guarded consumer is stopped.

    No running service is restarted or rearmed. Repeated ambiguous mutations
    are not retried: this is a separate, explicit local administrator recovery.
    """
    uid = os.geteuid() if _effective_uid is None else _effective_uid
    if type(uid) is not int or uid != 0 or confirm_stopped is not True:
        raise StoreUnavailable('local_administrator_and_stopped_confirmation_required')
    if not isinstance(reason, str) or not reason.strip() or len(reason) > 512:
        raise StoreUnavailable('explicit_recovery_reason_required')
    store = RevisionStore(state_dir, registry)
    actor = {'id': 0, 'username': 'local-root', 'role': 'administrator'}
    with RuntimeGuard(state_dir, exclusive=True), store.writer():
        manifest = store.require_ready()
        if manifest.get('latest_job') != job_id:
            raise StoreUnavailable('recovery_job_changed')
        job = store.get_job(job_id)
        admission = store.navigation_admission()
        unsettled_admission = bool(admission and admission['pending'])
        if (job is None or (job.get('state') not in ('unknown', 'pending', 'running')
                            and not unsettled_admission)):
            raise StoreUnavailable('no_unresolved_recovery_job')
        if type(revision) is not int or revision != manifest.get('last_known_good'):
            raise StoreUnavailable('recovery_requires_exact_last_good_revision')
        state = store.load_revision(revision)
        if digest != state['meta']['generated_digest']:
            raise StoreUnavailable('recovery_digest_changed')
        store.promote(revision=revision, scopes=SCOPES, actor=actor,
                      detail='explicit offline recovery: ' + reason)
        # A partially committed/unknown candidate is not a newly proven good
        # restore target merely because its revision appeared in the manifest.
        store._write_manifest(dict(store.require_ready(), last_known_good=revision))
        job = dict(job, state='failed', detail='Explicit offline rollback to last-good startup.',
                   recovery={'revision': revision, 'digest': digest, 'reason': reason,
                             'runtime_verified': False, 'fresh_operator_seed_required': True})
        store.record_job(job)
        if unsettled_admission:
            # Every guarded consumer is stopped; its in-process navigator gate
            # is gone. Normal startup still requires fresh operator localization.
            store.record_navigation_admission(dict(admission, pending=False))
        store._audit({'event': 'offline_recovery', 'actor': actor, 'job_id': job_id,
                      'revision': revision, 'digest': digest, 'reason': reason,
                      'at': store.clock(), 'runtime_verified': False})
        return job


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('state-dir', 'registry', 'job-id', 'digest', 'reason'):
        parser.add_argument('--' + name, required=True)
    parser.add_argument('--revision', required=True, type=int)
    parser.add_argument('--confirm-all-consumers-stopped', action='store_true')
    args = parser.parse_args(argv)
    try:
        path = Path(args.registry)
        if not path.is_file() or path.is_symlink() or path.stat().st_size > 512 * 1024:
            raise StoreUnavailable('invalid_registry_file')
        result = recover(args.state_dir, loads(path.read_text(), 512 * 1024),
                         job_id=args.job_id, revision=args.revision, digest=args.digest,
                         reason=args.reason,
                         confirm_stopped=args.confirm_all_consumers_stopped)
        print(json.dumps(result, sort_keys=True))
        return 0
    except (ValueError, OSError) as exc:
        print(json.dumps({'outcome': 'refused', 'detail': str(exc)}))
        return 2
