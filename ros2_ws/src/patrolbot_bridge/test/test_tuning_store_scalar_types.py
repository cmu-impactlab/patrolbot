"""New JSON numbers compile as ROS doubles; immutable old types fail closed."""

import json
from pathlib import Path
import uuid

import pytest

from patrolbot_bridge import tuning_spec
from patrolbot_bridge.tuning_startup import resolve
from patrolbot_bridge.tuning_store import RevisionStore, StoreUnavailable
from patrolbot_bridge.tuning_yaml import strict_yaml_load


ROOT = Path(__file__).resolve().parents[4]
REGISTRY_PATH = ROOT / 'contracts/robot-tuning.registry.json'
REGISTRY = json.loads(REGISTRY_PATH.read_text())
ADMIN = {'id': 1, 'username': 'scalar-fixture', 'role': 'administrator'}


def setup(tmp_path):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize(actor=ADMIN, reason='isolated scalar fixture')
    return store


def save(store, changes, *, request_id=None):
    return store.save(request_id=request_id or str(uuid.uuid4()), base_revision=1,
                      changes=changes, actor=ADMIN)


@pytest.mark.parametrize('value', [0, 1, 0.0, 1.0])
def test_json_integral_number_becomes_double_only_in_new_revision(tmp_path, value):
    store = setup(tmp_path)
    before = {str(p): p.read_bytes() for p in store.revision_dir(1).rglob('*') if p.is_file()}
    old_startup = store.startup_path.read_bytes()
    compiler = tuning_spec.compiler_digest()
    request_id = str(uuid.uuid4())
    changes = [{'id': 'amcl.alpha1', 'value': value},
               {'id': 'amcl.max_beams', 'value': 61}]
    original = [dict(change) for change in changes]
    result = save(store, changes, request_id=request_id)
    assert result['outcome'] == 'ok'
    candidate = store.load_revision(2)
    amcl = strict_yaml_load(candidate['generated']['navigation.yaml'])['amcl']['ros__parameters']
    assert type(amcl['alpha1']) is float
    assert amcl['alpha1'] == float(value)
    assert type(candidate['overrides']['amcl.alpha1']) is float
    assert type(amcl['max_beams']) is int
    assert type(candidate['overrides']['amcl.max_beams']) is int
    assert changes == original  # caller's canonical wire intent was not mutated
    assert save(store, changes, request_id=request_id)['deduplicated'] is True
    assert before == {str(p): p.read_bytes() for p in store.revision_dir(1).rglob('*') if p.is_file()}
    assert store.startup_path.read_bytes() == old_startup
    assert store.require_ready()['active_revisions'] == {'bringup': 1, 'navigation': 1}
    assert tuning_spec.compiler_digest() == compiler


def test_dedup_fingerprint_remains_original_wire_intent_before_normalization(tmp_path):
    store = setup(tmp_path)
    request_id = str(uuid.uuid4())
    assert save(store, [{'id': 'amcl.alpha1', 'value': 0}], request_id=request_id)['outcome'] == 'ok'
    changed_wire = save(store, [{'id': 'amcl.alpha1', 'value': 0.0}], request_id=request_id)
    assert changed_wire['outcome'] == 'refused'
    assert store.require_ready()['desired_revision'] == 2


@pytest.mark.parametrize('value', [True, False, '0', None])
def test_normalization_never_rescues_invalid_numeric_input(tmp_path, value):
    store = setup(tmp_path)
    assert save(store, [{'id': 'amcl.alpha1', 'value': value}])['outcome'] == 'refused'
    assert store.require_ready()['desired_revision'] == 1


def test_historical_checksum_valid_integer_ros_artifact_is_not_rewritten_or_loaded(tmp_path):
    store = setup(tmp_path)
    # Reproduce a legacy compiler artifact using the deliberately unchanged
    # compiler API, bypassing the NEW-save normalization for this fixture only.
    values = store.load_revision(1)['overrides']
    values['amcl.alpha1'] = 0
    store._write_revision(2, values, parent=1, actor=ADMIN,
                          reason='offline historical type fixture', provenance={})
    path = store.revision_dir(2)
    original = {str(p): p.read_bytes() for p in path.rglob('*') if p.is_file()}
    with pytest.raises(StoreUnavailable, match='incompatible_artifact_type'):
        store.load_revision(2)
    startup_before = store.startup_path.read_bytes()
    with pytest.raises(StoreUnavailable, match='incompatible_artifact_type'):
        store.promote(revision=2, scopes=['bringup', 'navigation'], actor=ADMIN)
    assert store.startup_path.read_bytes() == startup_before
    store._write_manifest(dict(store.require_ready(), active_revisions={
        'bringup': 2, 'navigation': 2}, desired_revision=2))
    default = ROOT / 'ros2_ws/src/patrolbot_navigation/config/nav2_params.yaml'
    with pytest.raises(StoreUnavailable):
        resolve(default, 'navigation', enabled=True, state_dir=store.base,
                registry_file=REGISTRY_PATH)
    assert original == {str(p): p.read_bytes() for p in path.rglob('*') if p.is_file()}
    assert store.read_startup()['revision'] == 1
