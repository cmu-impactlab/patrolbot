"""Offline recovery never races consumers or resets desired tuning history."""

import json
from pathlib import Path

from patrolbot_bridge.tuning_initialize import RuntimeGuard
from patrolbot_bridge.tuning_recover import recover
from patrolbot_bridge.tuning_store import RevisionStore, StoreUnavailable
import pytest


REGISTRY = json.loads((Path(__file__).resolve().parents[4]
                       / 'contracts/robot-tuning.registry.json').read_text())
ADMIN = {'id': 1, 'username': 'fixture', 'role': 'administrator'}
JOB = '22222222-2222-4222-8222-222222222222'


def setup(tmp_path):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize(actor=ADMIN, reason='unit fixture')
    store.save(request_id='11111111-1111-4111-8111-111111111111', base_revision=1,
               changes=[{'id': 'amcl.alpha1', 'value': .1}], actor=ADMIN)
    store.record_job({'job_id': JOB, 'state': 'unknown', 'revision': 2})
    store._write_manifest(dict(store.require_ready(), latest_job=JOB))
    return store, store.load_revision(1)['meta']['generated_digest']


def run(store, digest, **kwargs):
    return recover(store.base, REGISTRY, job_id=JOB, revision=1, digest=digest,
                   reason='explicit unit recovery', confirm_stopped=True, _effective_uid=0,
                   **kwargs)


def test_offline_last_good_restore_preserves_saved_pending_desired_state(tmp_path):
    store, digest = setup(tmp_path)
    job = run(store, digest)
    assert job['state'] == 'failed'
    assert job['recovery']['runtime_verified'] is False
    assert store.read_startup()['revision'] == 1
    assert store.require_ready()['desired_revision'] == 2
    assert store.require_ready()['last_known_good'] == 1
    assert store.desired_overrides()['amcl.alpha1'] == .1
    assert 'amcl.alpha1' in store.require_ready()['pending_keys']
    with pytest.raises(StoreUnavailable, match='no_unresolved'):
        run(store, digest)


def test_running_consumer_excludes_offline_recovery(tmp_path):
    store, digest = setup(tmp_path)
    before = store.manifest_path.read_bytes()
    with RuntimeGuard(store.base):
        with pytest.raises(BlockingIOError):
            run(store, digest)
    assert store.manifest_path.read_bytes() == before
    assert store.get_job(JOB)['state'] == 'unknown'


def test_wrong_digest_or_non_admin_never_promotes(tmp_path):
    store, digest = setup(tmp_path)
    before = store.manifest_path.read_bytes()
    with pytest.raises(StoreUnavailable, match='digest'):
        run(store, 'sha256:' + '0' * 64)
    with pytest.raises(StoreUnavailable, match='administrator'):
        recover(store.base, REGISTRY, job_id=JOB, revision=1, digest=digest,
                reason='fixture', confirm_stopped=True, _effective_uid=1000)
    assert store.manifest_path.read_bytes() == before


def test_offline_recovery_can_resolve_completed_reload_with_pending_navigation_admission(tmp_path):
    store, digest = setup(tmp_path)
    store.record_job({'job_id': JOB, 'state': 'succeeded', 'revision': 2})
    state = store.load_revision(2)
    store.record_navigation_admission({
        'pending': True, 'revision': 2, 'digest': state['meta']['generated_digest'],
        'map_identity': '1:2:100:200',
    })
    run(store, digest)
    assert store.navigation_admission()['pending'] is False
    assert store.read_startup()['revision'] == 1
