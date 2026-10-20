"""Regression probes for persistence, active/desired separation and fail-closed policy."""
import json
import os
from pathlib import Path

from patrolbot_bridge.tuning_policy import PolicyCache
from patrolbot_bridge.tuning_startup import resolve
from patrolbot_bridge.tuning_store import RevisionStore, StoreError, StoreUnavailable
from patrolbot_bridge.tuning_yaml import strict_yaml_load
import pytest

ROOT = Path(__file__).resolve().parents[4]
REGISTRY_PATH = ROOT / 'contracts/robot-tuning.registry.json'
REGISTRY = json.loads(REGISTRY_PATH.read_text())
ACTOR = {'id': 1, 'username': 'fixture', 'role': 'administrator'}
REQUEST = '11111111-1111-4111-8111-111111111111'


def store(tmp_path):
    result = RevisionStore(tmp_path / 'config', REGISTRY)
    result.initialize(actor=ACTOR, reason='offline fixture only')
    return result


def test_startup_uses_committed_active_not_desired(tmp_path):
    state = store(tmp_path)
    result = state.save(request_id=REQUEST, base_revision=1,
                        changes=[{'id': 'max_linear_m_s', 'value': .1}], actor=ACTOR)
    assert result['outcome'] == 'ok'
    default = ROOT / 'ros2_ws/src/patrolbot_navigation/config/nav2_params.yaml'
    path = resolve(default, 'navigation', enabled=True, state_dir=state.base,
                   registry_file=REGISTRY_PATH)
    document = strict_yaml_load(Path(path).read_text())
    follow_path = document['controller_server']['ros__parameters']['FollowPath']
    assert follow_path['desired_linear_vel'] == .22
    assert state.read_startup()['revision'] == 1
    assert 'max_linear_m_s' in state.snapshot_state()['pending_keys']
    assert resolve(default, 'navigation', enabled=False) == str(default)


def test_initialized_active_settings_survive_missing_enable_flag(tmp_path):
    state = store(tmp_path)
    default = ROOT / 'ros2_ws/src/patrolbot-launch/param/defaults/smoother.yaml'
    result = resolve(default, 'bringup', enabled=False, state_dir=state.base,
                     registry_file=REGISTRY_PATH)
    assert result != str(default)
    assert strict_yaml_load(Path(result).read_text()) == strict_yaml_load(
        state.load_revision(1)['generated']['bringup.yaml'])


def test_missing_previously_initialized_state_never_loads_faster_defaults(tmp_path):
    (tmp_path / 'tuning-initialized.json').write_text('{}')
    default = ROOT / 'ros2_ws/src/patrolbot-launch/param/defaults/smoother.yaml'
    with pytest.raises(StoreUnavailable):
        resolve(default, 'bringup', enabled=False, state_dir=tmp_path / 'config',
                registry_file=REGISTRY_PATH)


def test_changed_artifact_compiler_requires_explicit_compatibility_not_reinterpretation(
        tmp_path, monkeypatch):
    state = store(tmp_path)
    from patrolbot_bridge import tuning_spec
    monkeypatch.setattr(tuning_spec, 'compiler_digest', lambda: 'sha256:' + '0' * 64)
    with pytest.raises(StoreUnavailable, match='incompatible_compiler'):
        state.load_revision(1)


def test_nonregular_state_files_refuse_without_waiting_for_a_writer(tmp_path):
    state = store(tmp_path)
    state.initialized_path.unlink()
    os.mkfifo(state.initialized_path)
    with pytest.raises(StoreUnavailable, match='non_regular_state'):
        state.state()


def test_required_missing_policy_never_falls_back_to_higher_legacy_cap():
    state = {'valid': True}

    def probe():
        if not state['valid']:
            raise ValueError('corrupt')
        return 'generation', {'max_linear_m_s': .1, 'max_angular_rad_s': .2}
    cache = PolicyCache(probe, required=True)
    cache.refresh()
    assert cache.clamp(.3, .7)[:2] == (.1, .2)
    state['valid'] = False
    cache.refresh()
    assert cache.clamp(.3, .7)[:2] == (0.0, 0.0)


def test_partial_scope_promotion_refused_before_startup_write(tmp_path):
    state = store(tmp_path)
    original = state.startup_path.read_bytes()
    with pytest.raises(StoreError):
        state.promote(revision=1, scopes=['bringup'], actor=ACTOR)
    assert state.startup_path.read_bytes() == original


def test_idempotency_key_cannot_acknowledge_a_different_patch(tmp_path):
    state = store(tmp_path)
    first = state.save(request_id=REQUEST, base_revision=1,
                       changes=[{'id': 'amcl.alpha1', 'value': .06}], actor=ACTOR)
    assert first['outcome'] == 'ok'
    second = state.save(request_id=REQUEST, base_revision=1,
                        changes=[{'id': 'amcl.alpha1', 'value': .08}], actor=ACTOR)
    assert second['outcome'] == 'refused'
    assert state.desired_overrides()['amcl.alpha1'] == .06


def test_startup_checksum_filenames_are_allowlisted(tmp_path):
    state = store(tmp_path)
    pointer = json.loads(state.startup_path.read_text())
    pointer['checksums']['../../escape.yaml'] = 'sha256:' + '0'*64
    state.startup_path.write_text(json.dumps(pointer))
    with pytest.raises(StoreUnavailable):
        state.read_startup()


def test_symlinked_revision_directory_fails_closed(tmp_path):
    state = store(tmp_path)
    (state.revisions_dir / '2').symlink_to(state.revision_dir(1), target_is_directory=True)
    with pytest.raises(StoreUnavailable):
        state.load_revision(2)


def test_orphan_job_is_not_a_clean_install(tmp_path):
    base = tmp_path / 'config'
    (base / 'jobs').mkdir(parents=True)
    with pytest.raises(StoreUnavailable):
        RevisionStore(base, REGISTRY).initialize(actor=ACTOR, reason='must refuse')


def test_probability_dependency_both_directions(tmp_path):
    state = store(tmp_path)
    result = state.save(request_id=REQUEST, base_revision=1,
                        changes=[{'id': 'amcl.z_rand', 'value': .1}], actor=ACTOR)
    assert result['outcome'] == 'refused'
