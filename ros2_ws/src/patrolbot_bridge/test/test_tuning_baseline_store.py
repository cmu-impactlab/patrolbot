"""Unit tests for importing an explicit baseline without restoring faster defaults."""

from concurrent.futures import ThreadPoolExecutor
import copy
import json
from pathlib import Path
import threading

from patrolbot_bridge import tuning_spec
from patrolbot_bridge.tuning_startup import resolve
from patrolbot_bridge.tuning_store import RevisionStore, StoreError, StoreUnavailable
from patrolbot_bridge.tuning_yaml import strict_yaml_load
import pytest


ROOT = Path(__file__).resolve().parents[4]
REGISTRY_PATH = ROOT / 'contracts/robot-tuning.registry.json'
REGISTRY = json.loads(REGISTRY_PATH.read_text())
ACTOR = {'id': 1, 'username': 'fixture-admin', 'role': 'administrator'}
CAPTURE = '11111111-1111-4111-8111-111111111111'
SESSION = '22222222-2222-4222-8222-222222222222'


def _bundle():
    index = tuning_spec.index_registry(REGISTRY)
    values = {pid: entry['default'] for pid, entry in index.items()
              if entry.get('accepts_edits') is True
              and entry.get('protected') is not True
              and entry.get('not_applicable') is not True}
    values.update({'max_linear_m_s': .15, 'max_angular_rad_s': .35,
                   'nav2.controller.desired_linear_vel': .1})
    values.update(tuning_spec.speed_producer_targets(values, index))
    return {
        'contract': 'patrolbot-tuning-baseline', 'version': 1,
        'capture_id': CAPTURE, 'source_session_id': SESSION,
        'registry_digest': tuning_spec.registry_digest(REGISTRY),
        'defaults_digest': tuning_spec.defaults_digest(),
        'observed_values': values,
        'observed_artifacts': tuning_spec.generated_artifacts(values, index),
    }


def _tree(path):
    return {str(p.relative_to(path)): p.read_bytes()
            for p in path.rglob('*') if p.is_file()}


def test_baseline_initialization_preserves_lower_caps_and_cruise(tmp_path):
    bundle = _bundle()
    original = copy.deepcopy(bundle)
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    manifest = store.initialize_from_baseline(bundle, actor=ACTOR, reason='unit fixture')
    assert manifest['desired_revision'] == 1
    assert set(manifest['active_revisions'].values()) == {1}
    assert manifest['provenance']['baseline']['capture_id'] == CAPTURE
    assert manifest['provenance']['baseline']['source_session_id'] == SESSION
    revision = store.load_revision(1)
    assert revision['overrides'] == original['observed_values']
    assert revision['generated'] == original['observed_artifacts']
    final = strict_yaml_load(revision['generated']['bringup.yaml'])
    assert final['teleop_velocity_smoother']['ros__parameters']['max_velocity'] == [.15, 0, .35]
    nav = strict_yaml_load(revision['generated']['navigation.yaml'])
    assert nav['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'] == .1
    assert bundle == original


def test_reopen_and_pending_save_boot_only_the_imported_active_baseline(tmp_path):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize_from_baseline(_bundle(), actor=ACTOR, reason='unit fixture')
    reopened = RevisionStore(store.base, REGISTRY)
    assert reopened.read_startup()['revision'] == 1
    saved = reopened.save(
        request_id='33333333-3333-4333-8333-333333333333', base_revision=1,
        changes=[{'id': 'max_linear_m_s', 'value': .05}], actor=ACTOR)
    assert saved['outcome'] == 'ok'
    params = resolve(ROOT / 'ros2_ws/src/patrolbot-launch/param/defaults/smoother.yaml',
                     'bringup', enabled=True, state_dir=store.base, registry_file=REGISTRY_PATH)
    startup = strict_yaml_load(Path(params).read_text())
    assert startup['teleop_velocity_smoother']['ros__parameters']['max_velocity'][0] == .15
    assert reopened.read_startup()['revision'] == 1
    assert reopened.snapshot_state()['desired_revision'] == 2


@pytest.mark.parametrize('role', ['operator', 'observer', 'administrator '])
def test_baseline_initialization_requires_a_trusted_administrator_actor(tmp_path, role):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    with pytest.raises(StoreError, match='administrator'):
        store.initialize_from_baseline(_bundle(), actor=dict(ACTOR, role=role), reason='fixture')
    assert not store.base.exists()


def test_nonrepresentable_lower_producer_is_refused_before_state_creation(tmp_path):
    bundle = _bundle()
    nav = strict_yaml_load(bundle['observed_artifacts']['navigation.yaml'])
    nav['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'] = .04
    bundle['observed_artifacts']['navigation.yaml'] = tuning_spec.strict_yaml_dump(nav)
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    with pytest.raises(StoreError):
        store.initialize_from_baseline(bundle, actor=ACTOR, reason='unit fixture')
    assert not store.base.exists()


def test_logical_producer_claim_cannot_hide_a_lower_observed_artifact(tmp_path):
    bundle = _bundle()
    # min(cap, target) would generate the same .15 artifact from .26. Import
    # must not retain .26 as an observed source target and revive it later.
    bundle['observed_values']['nav2.velocity_smoother.max_velocity_x'] = .26
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    with pytest.raises(StoreError):
        store.initialize_from_baseline(bundle, actor=ACTOR, reason='unit fixture')
    assert not store.base.exists()


def test_reinitialization_never_overwrites_baseline_or_history(tmp_path):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize_from_baseline(_bundle(), actor=ACTOR, reason='unit fixture')
    before = _tree(store.base)
    with pytest.raises(StoreUnavailable, match='already_initialized'):
        store.initialize_from_baseline(_bundle(), actor=ACTOR, reason='do not reset')
    assert _tree(store.base) == before


def test_packaged_source_drift_before_commit_never_creates_active_pointer(tmp_path, monkeypatch):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    initialize = store._initialize

    def drift_before_lock(**kwargs):
        monkeypatch.setattr(tuning_spec, 'defaults_digest', lambda: 'sha256:' + '0' * 64)
        return initialize(**kwargs)

    monkeypatch.setattr(store, '_initialize', drift_before_lock)
    with pytest.raises(StoreUnavailable, match='baseline_changed'):
        store.initialize_from_baseline(_bundle(), actor=ACTOR, reason='unit fixture')
    assert not store.startup_path.exists()
    assert not store.manifest_path.exists()


@pytest.mark.parametrize('actor', [None, {}, dict(ACTOR, id=True), dict(ACTOR, username='')])
def test_invalid_actor_never_creates_tuning_state(tmp_path, actor):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    with pytest.raises(StoreError, match='administrator'):
        store.initialize_from_baseline(_bundle(), actor=actor, reason='unit fixture')
    assert not store.base.exists()


def test_crash_before_manifest_never_resets_existing_baseline(tmp_path, monkeypatch):
    store = RevisionStore(tmp_path / 'config', REGISTRY)

    def full_disk(*args):
        raise OSError('disk full')

    monkeypatch.setattr(store, '_write_manifest', full_disk)
    with pytest.raises(OSError, match='disk full'):
        store.initialize_from_baseline(_bundle(), actor=ACTOR, reason='unit fixture')
    reopened = RevisionStore(store.base, REGISTRY)
    before = _tree(store.base)
    with pytest.raises(StoreUnavailable, match='corrupt_missing_manifest'):
        reopened.require_ready()
    with pytest.raises(StoreUnavailable):
        reopened.initialize_from_baseline(_bundle(), actor=ACTOR, reason='must not repair')
    assert _tree(store.base) == before
    assert not store.startup_path.exists()


def test_two_initializers_have_one_durable_winner(tmp_path):
    barrier = threading.Barrier(2)
    bundle = _bundle()

    def initialize():
        store = RevisionStore(tmp_path / 'config', REGISTRY)
        barrier.wait(timeout=5)
        try:
            store.initialize_from_baseline(bundle, actor=ACTOR, reason='unit fixture')
            return 'initialized'
        except StoreUnavailable as exc:
            return exc.code

    with ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(lambda _: initialize(), range(2)))
    assert sorted(results) == ['already_initialized', 'initialized']
    reopened = RevisionStore(tmp_path / 'config', REGISTRY)
    assert reopened.read_startup()['revision'] == 1
    assert len(list(reopened.revisions_dir.iterdir())) == 1
