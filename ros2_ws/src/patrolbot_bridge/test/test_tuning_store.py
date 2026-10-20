"""Immutable revision store: atomicity, CAS, dedup, fail-closed. No ROS."""

import json
from pathlib import Path

from patrolbot_bridge import tuning_store as store_mod
from patrolbot_bridge.tuning_store import RevisionStore, StoreUnavailable
import pytest

REPO = Path(__file__).resolve().parents[4]
REGISTRY = json.loads((REPO / 'contracts' / 'robot-tuning.registry.json').read_text())

ADMIN = {'id': 1, 'username': 'admin', 'role': 'administrator'}
UUID = '11111111-1111-4111-8111-111111111111'


def make(tmp_path):
    store = RevisionStore(tmp_path / 'config', REGISTRY, clock=lambda: 1000.0)
    return store


def initialized(tmp_path):
    store = make(tmp_path)
    store.initialize(actor=ADMIN, reason='test')
    return store


def test_clean_install_is_uninitialized_and_mutation_fails_closed(tmp_path):
    store = make(tmp_path)
    assert store.state()['status'] == 'uninitialized'
    with pytest.raises(StoreUnavailable):
        store.require_ready()
    with pytest.raises(StoreUnavailable):
        store.save(request_id=UUID, base_revision=0,
                   changes=[{'id': 'amcl.alpha1', 'value': 0.06}], actor=ADMIN)


def test_initialize_builds_revision_one_and_active_startup(tmp_path):
    store = initialized(tmp_path)
    state = store.snapshot_state()
    assert state['desired_revision'] == 1
    assert state['active_revision'] == 1
    assert state['pending_keys'] == []
    pointer = store.read_startup()
    assert pointer['revision'] == 1
    assert set(pointer['checksums']) == {'bringup.yaml', 'navigation.yaml'}
    assert store.initialized_path.exists()


def test_initialize_refuses_to_reset_existing_state(tmp_path):
    store = initialized(tmp_path)
    with pytest.raises(StoreUnavailable, match='already_initialized'):
        store.initialize(actor=ADMIN, reason='again')


def test_missing_sentinel_next_to_state_fails_closed(tmp_path):
    store = initialized(tmp_path)
    store.initialized_path.unlink()
    with pytest.raises(StoreUnavailable, match='corrupt_missing_sentinel'):
        store.state()


def test_corrupt_manifest_is_not_a_faster_reset(tmp_path):
    store = initialized(tmp_path)
    store.manifest_path.write_text('{not json')
    with pytest.raises(StoreUnavailable):
        store.state()
    with pytest.raises(StoreUnavailable):
        store.initialize(actor=ADMIN, reason='reset attempt')


def test_corrupt_sentinel_fails_closed(tmp_path):
    store = initialized(tmp_path)
    store.initialized_path.write_text(json.dumps({'schema_version': 99}))
    with pytest.raises(StoreUnavailable, match='corrupt_sentinel'):
        store.state()


def test_save_creates_new_revision_and_pending_survives_restart(tmp_path):
    store = initialized(tmp_path)
    result = store.save(request_id=UUID, base_revision=1,
                        changes=[{'id': 'amcl.alpha1', 'value': 0.06}], actor=ADMIN)
    assert result['outcome'] == 'ok'
    assert result['revision'] == 2
    assert store.snapshot_state()['pending_keys'] == ['amcl.alpha1']
    # A "process restart" is a fresh store object over the same directory.
    restarted = RevisionStore(tmp_path / 'config', REGISTRY)
    state = restarted.snapshot_state()
    assert state['desired_revision'] == 2
    assert state['active_revision'] == 1
    assert state['pending_keys'] == ['amcl.alpha1']
    # Boot reads ACTIVE, so restart still yields the old active revision.
    assert restarted.read_startup()['revision'] == 1


def test_compare_and_swap_conflict_between_two_admins(tmp_path):
    store = initialized(tmp_path)
    first = store.save(request_id=UUID, base_revision=1,
                       changes=[{'id': 'amcl.alpha1', 'value': 0.06}], actor=ADMIN)
    assert first['outcome'] == 'ok'
    second = store.save(request_id='22222222-2222-4222-8222-222222222222',
                        base_revision=1,
                        changes=[{'id': 'amcl.alpha1', 'value': 0.07}], actor=ADMIN)
    assert second['outcome'] == 'conflict'
    # The losing write left no revision behind.
    assert store.snapshot_state()['desired_revision'] == 2


def test_duplicate_request_is_idempotent(tmp_path):
    store = initialized(tmp_path)
    store.save(request_id=UUID, base_revision=1,
               changes=[{'id': 'amcl.alpha1', 'value': 0.06}], actor=ADMIN)
    repeat = store.save(request_id=UUID, base_revision=1,
                        changes=[{'id': 'amcl.alpha1', 'value': 0.06}], actor=ADMIN)
    assert repeat['outcome'] == 'ok'
    assert repeat.get('deduplicated') is True
    assert store.snapshot_state()['desired_revision'] == 2


def test_interrupted_request_reports_unknown_never_retries(tmp_path):
    store = initialized(tmp_path)
    store.requests_dir.mkdir(exist_ok=True)
    store.record_request(UUID, {
        'request_id': UUID, 'operation': 'save', 'state': 'in_progress', 'started_at': 1.0})
    result = store.save(request_id=UUID, base_revision=1,
                        changes=[{'id': 'amcl.alpha1', 'value': 0.06}], actor=ADMIN)
    assert result['outcome'] == 'unknown'
    assert store.snapshot_state()['desired_revision'] == 1


def test_refused_patch_writes_no_revision(tmp_path):
    store = initialized(tmp_path)
    result = store.save(request_id=UUID, base_revision=1,
                        changes=[{'id': 'max_linear_m_s', 'value': 0.31}], actor=ADMIN)
    assert result['outcome'] == 'refused'
    assert 'out_of_bounds' in result['detail']
    assert store.snapshot_state()['desired_revision'] == 1


def test_atomic_write_failure_leaves_manifest_intact(tmp_path, monkeypatch):
    store = initialized(tmp_path)
    original_replace = store_mod.os.replace

    def boom(src, dst):
        if str(dst).endswith('manifest.json'):
            raise OSError('disk full')
        return original_replace(src, dst)

    monkeypatch.setattr(store_mod.os, 'replace', boom)
    with pytest.raises(OSError):
        store.save(request_id=UUID, base_revision=1,
                   changes=[{'id': 'amcl.alpha1', 'value': 0.06}], actor=ADMIN)
    monkeypatch.setattr(store_mod.os, 'replace', original_replace)
    # Manifest never advanced; no stray temp files remain.
    assert store.snapshot_state()['desired_revision'] == 1
    leftovers = [p.name for p in (tmp_path / 'config').glob('.*.tmp')]
    assert leftovers == []


def test_revision_checksum_tamper_is_detected(tmp_path):
    store = initialized(tmp_path)
    overrides = store.revision_dir(1) / 'overrides.yaml'
    overrides.write_text('amcl.alpha1: 0.99\n')
    with pytest.raises(StoreUnavailable, match='checksum_mismatch'):
        store.load_revision(1)


def test_promote_commits_active_startup_and_clears_pending(tmp_path):
    store = initialized(tmp_path)
    store.save(request_id=UUID, base_revision=1,
               changes=[{'id': 'amcl.alpha1', 'value': 0.06}], actor=ADMIN)
    manifest = store.promote(revision=2, scopes=store_mod.SCOPES, actor=ADMIN)
    assert manifest['active_revisions'] == {'bringup': 2, 'navigation': 2}
    assert manifest['last_known_good'] == 1
    assert manifest['pending_keys'] == []
    assert store.read_startup()['revision'] == 2
    # A process restart now boots the promoted revision.
    assert RevisionStore(tmp_path / 'config', REGISTRY).read_startup()['revision'] == 2


def test_audit_log_is_durable_and_secret_free(tmp_path):
    store = initialized(tmp_path)
    store.save(request_id=UUID, base_revision=1,
               changes=[{'id': 'amcl.alpha1', 'value': 0.06}], actor=ADMIN,
               reason='operator request')
    lines = store.audit_path.read_text().splitlines()
    assert lines
    records = [json.loads(line) for line in lines]
    assert any(r['event'] == 'save' and r['actor']['username'] == 'admin' for r in records)
    joined = store.audit_path.read_text()
    assert 'password' not in joined and 'token' not in joined


def test_symlinked_base_is_refused(tmp_path):
    real = tmp_path / 'real'
    real.mkdir()
    link = tmp_path / 'config'
    link.symlink_to(real)
    with pytest.raises(StoreUnavailable, match='symlinked_state_dir'):
        RevisionStore(link, REGISTRY)


def test_request_and_job_ids_must_be_canonical_uuids(tmp_path):
    store = initialized(tmp_path)
    with pytest.raises(store_mod.StoreError):
        store.get_request('../../etc/passwd')
    with pytest.raises(store_mod.StoreError):
        store.get_job('not-a-uuid')
