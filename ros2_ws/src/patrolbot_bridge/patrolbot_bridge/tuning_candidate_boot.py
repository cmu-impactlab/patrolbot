"""Explicit candidate boot API, deliberately separate from ordinary ACTIVE startup.

No environment-variable auto-selection, DESIRED lookup, signature bypass or
fallback-to-defaults. The host must supply a new signed boot capability and a
host-owned launch receipt. Missing native maintenance/policy integration remains
a blocker to using this API in the shipped launcher.
"""

from . import tuning_handoff as handoff
from .tuning_host_auth import bind_handoff, verify
from .tuning_store import SCOPES, StoreUnavailable


def authorize(store, *, scope, authorization, public_key, launch_receipt,
              now, host_boot_id):
    """Return exact immutable candidate contents, with the durable hold retained.

    launch_receipt must come from the separately protected host control directory,
    never from robot-writable state. Caller must keep the candidate maintenance
    hold and authenticate a new runtime identity before any verification/commit.
    This function itself cannot attest ROS command ownership or consumed caches.
    """
    if scope not in SCOPES:
        raise StoreUnavailable('invalid_candidate_scope')
    claims = verify(authorization, public_key, kind='candidate_boot', now=now,
                    host_boot_id=host_boot_id)
    with store.reader():
        staged = handoff.read(store, job_id=claims['job_id'])
        bind_handoff(claims, staged)
        expected_receipt = {
            'version': 1, 'state': 'dispatching',
            'boot_id': claims['boot_id'], 'job_id': claims['job_id'],
            'authorization_id': claims['execution_authorization_id'],
            'boot_authorization_id': claims['authorization_id'],
            'handoff_digest': claims['handoff_digest'],
            'config_digest': claims['config_digest'], 'volume_id': claims['volume_id'],
            'host_boot_id': claims['host_boot_id'],
        }
        if launch_receipt != expected_receipt:
            raise StoreUnavailable('candidate_host_launch_receipt_mismatch')
        job = store.get_job(claims['job_id'])
        owner = job.get('docker_handoff', {})
        if (job.get('mutation_started') is not True or owner.get('state') != 'dispatching'
                or owner.get('boot_id') != claims['boot_id']
                or owner.get('authorization_id') != claims['execution_authorization_id']):
            raise StoreUnavailable('candidate_boot_not_dispatched')
        candidate = store.load_revision(claims['revision'])
        return {'job_id': claims['job_id'], 'revision': claims['revision'],
                'digest': claims['digest'], 'plan_hash': claims['plan_hash'],
                'boot_id': claims['boot_id'], 'hold_required': True,
                'fresh_operator_seed_required': True,
                'artifact': candidate['generated'][scope + '.yaml']}
