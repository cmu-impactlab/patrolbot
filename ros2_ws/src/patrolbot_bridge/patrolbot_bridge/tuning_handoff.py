"""Durable, non-authorizing Docker handoff. No Docker, keys, ROS or promotion.

A handoff is staging, NOT permission. Its unsettled job remains the durable
motion hold. Host authorization and replacement-runtime verification are distinct
steps; neither a staged artifact nor a successful Docker exit commits ACTIVE.
"""

import hashlib
import json
import re

from .tuning_store import (
    SCOPES, StoreUnavailable, _atomic_write, _canonical_uuid, _fsync_dir, _read_json,
)


SERVICES = ('bringup', 'bridge', 'navigation')
UNSETTLED = ('pending', 'running', 'unknown')


def canonical_bytes(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def digest(value):
    return 'sha256:' + hashlib.sha256(canonical_bytes(value)).hexdigest()


def is_digest(value):
    return isinstance(value, str) and re.fullmatch(r'sha256:[0-9a-f]{64}', value) is not None


def _path(store, job_id):
    if not _canonical_uuid(job_id):
        raise StoreUnavailable('invalid_handoff_job')
    path = store.base / 'handoffs' / (job_id + '.json')
    store._reject_links(path)
    return path


def _binding(store, job_id):
    manifest = store.require_ready()
    job = store.get_job(job_id)
    if (manifest['latest_job'] != job_id or not isinstance(job, dict)
            or job.get('state') not in UNSETTLED):
        raise StoreUnavailable('handoff_requires_exact_unsettled_job')
    plan = job.get('execution_plan')
    if (not isinstance(plan, dict) or plan.get('execution_kind') != 'robot_docker'
            or plan.get('executable') is not True or plan.get('reasons') != []
            or plan.get('services') != list(SERVICES)
            or plan.get('seed_required') is not True
            or plan.get('verification_scopes') != list(SCOPES)
            or plan.get('plan_hash') != digest({k: v for k, v in plan.items()
                                               if k != 'plan_hash'})
            or job.get('plan_hash') != plan['plan_hash']
            or type(job.get('revision')) is not int
            or job['revision'] != plan.get('revision')
            or job.get('digest') != plan.get('digest')):
        raise StoreUnavailable('handoff_requires_reviewed_fixed_docker_plan')
    candidate = store.load_revision(job['revision'])
    pointer = store.read_startup()
    if (set(manifest['active_revisions'].values()) != {pointer['revision']}
            or plan.get('base_revision') != pointer['revision']
            or plan.get('base_digest') != pointer['generated_digest']
            or plan.get('desired_revision') != manifest['desired_revision']
            or candidate['meta']['generated_digest'] != plan['digest']):
        raise StoreUnavailable('handoff_revision_changed')
    return manifest, job, plan, candidate, pointer


def stage(store, *, job_id):
    """Called only after the coordinator durably owns acknowledged command holds.

    This records an immutable pointer to an already immutable revision. It never
    modifies startup files, ACTIVE, DESIRED, last-good, or navigation admission.
    The caller must separately collect the real hold acknowledgments; this module
    cannot infer them from a Boolean, a file, or a zero velocity message.
    """
    with store.writer():
        manifest, job, plan, candidate, pointer = _binding(store, job_id)
        if job.get('state') != 'running' or job.get('mutation_started') is not False:
            raise StoreUnavailable('handoff_requires_pre_mutation_running_job')
        path = _path(store, job_id)
        if path.exists():
            # A crash/repeated staging request is not new execution permission.
            return read(store, job_id=job_id)
        record = {
            'version': 1, 'job_id': job_id, 'revision': job['revision'],
            'digest': job['digest'], 'plan_hash': plan['plan_hash'],
            'store_generation': manifest['generation'],
            'base_revision': pointer['revision'], 'base_digest': pointer['generated_digest'],
            'generated': candidate['meta']['generated'], 'services': list(SERVICES),
            'hold_required': True, 'fresh_operator_seed_required': True,
        }
        record['handoff_digest'] = digest(record)
        path.parent.mkdir(mode=0o700, exist_ok=True)
        _fsync_dir(store.base)
        _atomic_write(path, canonical_bytes(record))
        # A crash between these writes is a closed gate: read() requires both.
        store.record_job(dict(job, docker_handoff={
            'handoff_digest': record['handoff_digest'], 'hold_required': True,
            'state': 'staged',
        }))
        store._audit({'event': 'docker_handoff_staged', 'job_id': job_id,
                      'handoff_digest': record['handoff_digest'], 'at': store.clock()})
        return record


def read(store, *, job_id):
    """Revalidate history, job ownership and full artifact binding on every read."""
    manifest, job, plan, candidate, pointer = _binding(store, job_id)
    record = _read_json(_path(store, job_id))
    expected = {
        'version': 1, 'job_id': job_id, 'revision': job['revision'],
        'digest': job['digest'], 'plan_hash': plan['plan_hash'],
        'store_generation': manifest['generation'],
        'base_revision': pointer['revision'], 'base_digest': pointer['generated_digest'],
        'generated': candidate['meta']['generated'], 'services': list(SERVICES),
        'hold_required': True, 'fresh_operator_seed_required': True,
    }
    expected['handoff_digest'] = digest(expected)
    if record != expected or job.get('docker_handoff', {}).get('hold_required') is not True:
        raise StoreUnavailable('corrupt_docker_handoff')
    if job['docker_handoff'].get('handoff_digest') != record['handoff_digest']:
        raise StoreUnavailable('docker_handoff_job_binding_changed')
    if job['docker_handoff'].get('state') not in ('staged', 'dispatching', 'unknown', 'verifying'):
        raise StoreUnavailable('docker_handoff_not_pending')
    return record


def mark_unknown(store, *, job_id, detail):
    """Ambiguity persists the hold; it is never retry or promotion permission."""
    with store.writer():
        read(store, job_id=job_id)
        job = store.get_job(job_id)
        store.record_job(dict(job, state='unknown', mutation_started=True,
                              detail=str(detail)[:512], docker_handoff=dict(
                                  job['docker_handoff'], state='unknown')))
        store._audit({'event': 'docker_handoff_unknown', 'job_id': job_id,
                      'at': store.clock()})
