"""Starting defaults read from the packaged consumer YAML, not the registry.

These tests prove the reviewed registry catalog is not the starting source: the
packaged consumer YAML is, and only for logical ids the registry marks full,
editable, non-protected and applicable. A synthetic store and a stub
coordinator keep the suite offline; nothing here touches ROS, the robot, the
network or any deployed state.
"""

import copy
import json
from pathlib import Path

from patrolbot_bridge import config_bootstrap as bootstrap
from patrolbot_bridge import tuning_spec as spec
from patrolbot_bridge.tuning_service import _catalog_entry, TuningService
from patrolbot_bridge.tuning_store import RevisionStore, StoreUnavailable
import pytest

REPO = Path(__file__).resolve().parents[4]
REGISTRY_PATH = REPO / 'contracts' / 'robot-tuning.registry.json'
REGISTRY = json.loads(REGISTRY_PATH.read_text())
INDEX = spec.index_registry(REGISTRY)
ACTOR = {'id': 1, 'username': 'fixture', 'role': 'administrator'}
SESSION = '123e4567-e89b-12d3-a456-426614174000'

SMOOTHER_FILE = spec.CONSUMER_SCOPES['bringup']['source_file']
NAV_FILE = spec.CONSUMER_SCOPES['navigation']['source_file']


def _editable_ids(index):
    return {pid for pid, entry in index.items()
            if entry.get('accepts_edits') is True
            and entry.get('protected') is not True
            and entry.get('not_applicable') is not True}


def _copied_documents():
    """Real packaged documents, deep-copied so a test can mutate them safely."""
    return {scope: copy.deepcopy(spec.packaged_defaults(scope))
            for scope in spec.CONSUMER_SCOPES}


def _patch_packaged_yaml(monkeypatch, documents):
    monkeypatch.setattr(
        spec, 'packaged_defaults', lambda scope: copy.deepcopy(documents[scope]))


def _synthetic_entry(**source):
    entry = {
        'id': 'synthetic.value',
        'type': 'number',
        'accepts_edits': True,
        'protected': False,
        'not_applicable': False,
        'bounds': {'lower': 0.0, 'upper': 1.0,
                   'lower_inclusive': True, 'upper_inclusive': True},
        'source': source,
    }
    return entry


class _StubCoordinator:
    """Offline stand-in; activation is never supported in these tests."""

    activation_supported = False

    def safe_apply_state(self):
        return {'allowed': False, 'reason': 'not initialized'}

    def status(self, job_id=None):
        return None


# ----------------------------------------------------------------------
# The helper: scope mapping, dotted/literal keys, indexes and strictness
# ----------------------------------------------------------------------
def test_starting_values_come_from_the_packaged_yaml():
    values = spec.yaml_starting_values(INDEX)
    assert set(values) == _editable_ids(INDEX)
    assert values['amcl.alpha1'] == 0.05                          # literal key
    assert values['nav2.controller.desired_linear_vel'] == 0.22   # dotted key
    assert values['max_linear_m_s'] == 0.30                       # indexed slot 0
    assert values['max_angular_rad_s'] == 0.70                    # indexed slot 2
    assert values['amcl.min_particles'] == 1000                   # strict integer


def test_documents_can_be_injected_and_are_not_mutated():
    documents = _copied_documents()
    before = copy.deepcopy(documents)
    values = spec.yaml_starting_values(INDEX, documents=documents)
    assert values['amcl.alpha1'] == 0.05
    assert documents == before


def test_dotted_node_and_dotted_key_are_followed():
    documents = {'navigation': {
        'outer': {'inner': {'ros__parameters': {
            'FollowPath': {'desired_linear_vel': 0.11}}}}}}
    index = {'synthetic.value': _synthetic_entry(
        file=NAV_FILE, node='outer.inner', key='FollowPath.desired_linear_vel',
        index=None)}
    values = spec.yaml_starting_values(index, documents=documents)
    assert values['synthetic.value'] == 0.11


def test_starting_values_writes_nothing(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('starting values must not write')

    for name in ('write_text', 'write_bytes', 'mkdir', 'touch', 'unlink'):
        monkeypatch.setattr(Path, name, forbidden, raising=False)
    values = spec.yaml_starting_values(INDEX)
    assert values['amcl.alpha1'] == 0.05


# ----------------------------------------------------------------------
# The YAML is the source of truth; the registry default is not a fallback
# ----------------------------------------------------------------------
def test_packaged_yaml_wins_over_a_stale_registry_default(monkeypatch):
    documents = _copied_documents()
    documents['navigation']['amcl']['ros__parameters']['alpha1'] = 0.06
    assert INDEX['amcl.alpha1']['default'] == 0.05
    _patch_packaged_yaml(monkeypatch, documents)
    values = spec.yaml_starting_values(INDEX)
    assert values['amcl.alpha1'] == 0.06
    assert INDEX['amcl.alpha1']['default'] == 0.05  # registry never mutated


def test_missing_yaml_key_raises_instead_of_falling_back_to_the_registry():
    documents = _copied_documents()
    del documents['navigation']['amcl']['ros__parameters']['alpha1']
    with pytest.raises(spec.SpecError) as info:
        spec.yaml_starting_values(INDEX, documents=documents)
    assert 'amcl.alpha1' in str(info.value)


def test_missing_node_raises():
    documents = _copied_documents()
    del documents['navigation']['amcl']
    index = {'synthetic.value': _synthetic_entry(
        file=NAV_FILE, node='amcl', key='alpha1', index=None)}
    with pytest.raises(spec.SpecError):
        spec.yaml_starting_values(index, documents=documents)


def test_missing_document_for_scope_raises():
    index = {'synthetic.value': _synthetic_entry(
        file=NAV_FILE, node='amcl', key='alpha1', index=None)}
    with pytest.raises(spec.SpecError):
        spec.yaml_starting_values(
            index, documents={'bringup': spec.packaged_defaults('bringup')})


def test_out_of_bounds_yaml_value_raises():
    documents = _copied_documents()
    documents['navigation']['amcl']['ros__parameters']['alpha1'] = 5.0
    with pytest.raises(spec.SpecError) as info:
        spec.yaml_starting_values(INDEX, documents=documents)
    assert 'out_of_bounds' in str(info.value)


@pytest.mark.parametrize('pid, value, reason', [
    ('amcl.alpha1', True, 'bool_as_number'),
    ('amcl.min_particles', 1000.5, 'bad_type'),
    ('amcl.alpha1', float('nan'), 'bad_type'),
    ('amcl.alpha1', float('inf'), 'bad_type'),
])
def test_non_strict_yaml_values_are_refused(pid, value, reason):
    documents = _copied_documents()
    source = INDEX[pid]['source']
    params = documents['navigation'][source['node']]['ros__parameters']
    params[source['key']] = value
    with pytest.raises(spec.SpecError) as info:
        spec.yaml_starting_values(INDEX, documents=documents)
    assert reason in str(info.value)


@pytest.mark.parametrize('position', [-1, True, 1.5, '0'])
def test_source_index_must_be_a_non_negative_integer(position):
    index = {'synthetic.value': _synthetic_entry(
        file=SMOOTHER_FILE, node='teleop_velocity_smoother',
        key='max_velocity', index=position)}
    with pytest.raises(spec.SpecError) as info:
        spec.yaml_starting_values(index)
    assert 'non-negative integer' in str(info.value)


def test_source_index_out_of_range_raises():
    index = {'synthetic.value': _synthetic_entry(
        file=SMOOTHER_FILE, node='teleop_velocity_smoother',
        key='max_velocity', index=7)}
    with pytest.raises(spec.SpecError) as info:
        spec.yaml_starting_values(index)
    assert 'out of range' in str(info.value)


def test_unalowlisted_source_file_is_refused():
    index = {'synthetic.value': _synthetic_entry(
        file='etc/passwd', node='teleop_velocity_smoother',
        key='max_velocity', index=0)}
    with pytest.raises(spec.SpecError) as info:
        spec.yaml_starting_values(index)
    assert 'allowlisted' in str(info.value)


# ----------------------------------------------------------------------
# No bypass through arbitrary mapping keys or unknown/malformed sources
# ----------------------------------------------------------------------
def test_protected_and_not_applicable_sources_are_skipped_before_reading():
    # The reviewed registry has protected entries whose ``source.file`` is not
    # an allowlisted consumer scope (for example the mux arbitration file). The
    # skip must happen before any traversal, so a skipped entry can never be
    # turned into a read of an arbitrary file or key.
    index = {
        'locked.value': {
            'id': 'locked.value', 'type': 'number',
            'accepts_edits': False, 'protected': True, 'not_applicable': False,
            'source': {'file': 'etc/shadow', 'node': 'root', 'key': 'hash',
                       'index': None},
        },
        'inapplicable.value': {
            'id': 'inapplicable.value', 'type': 'number',
            'accepts_edits': True, 'protected': False, 'not_applicable': True,
            'source': {'file': 'etc/shadow', 'node': 'root', 'key': 'hash',
                       'index': None},
        },
    }
    assert spec.yaml_starting_values(index) == {}


def test_unknown_scope_keys_in_the_document_mapping_are_ignored():
    documents = _copied_documents()
    documents['../../etc/passwd'] = {'amcl': {'ros__parameters': {'alpha1': 9.9}}}
    documents['anything'] = {'amcl': {'ros__parameters': {'alpha1': 8.8}}}
    values = spec.yaml_starting_values(INDEX, documents=documents)
    assert values['amcl.alpha1'] == 0.05


def test_non_mapping_source_is_refused():
    index = {'synthetic.value': {
        'id': 'synthetic.value', 'type': 'number',
        'accepts_edits': True, 'protected': False, 'not_applicable': False,
        'source': 'etc/passwd',
    }}
    with pytest.raises(spec.SpecError) as info:
        spec.yaml_starting_values(index)
    assert 'source is not a mapping' in str(info.value)


def test_source_node_without_ros_parameters_is_refused():
    documents = {'navigation': {'amcl': {'alpha1': 0.05}}}
    index = {'synthetic.value': _synthetic_entry(
        file=NAV_FILE, node='amcl', key='alpha1', index=None)}
    with pytest.raises(spec.SpecError) as info:
        spec.yaml_starting_values(index, documents=documents)
    assert 'ros__parameters' in str(info.value)


def test_missing_source_key_raises_instead_of_reading_a_default():
    documents = {'navigation': {'amcl': {'ros__parameters': {}}}}
    index = {'synthetic.value': _synthetic_entry(
        file=NAV_FILE, node='amcl', key='alpha1', index=None)}
    with pytest.raises(spec.SpecError) as info:
        spec.yaml_starting_values(index, documents=documents)
    assert 'missing source key' in str(info.value)


def test_source_index_against_a_non_list_value_is_refused():
    documents = {'bringup': {'teleop_velocity_smoother': {
        'ros__parameters': {'max_velocity': 0.30}}}}
    index = {'synthetic.value': _synthetic_entry(
        file=SMOOTHER_FILE, node='teleop_velocity_smoother',
        key='max_velocity', index=0)}
    with pytest.raises(spec.SpecError) as info:
        spec.yaml_starting_values(index, documents=documents)
    assert 'out of range' in str(info.value)


# ----------------------------------------------------------------------
# Wiring: store initialization, bootstrap candidate, service snapshot
# ----------------------------------------------------------------------
def test_store_initialize_seeds_from_packaged_yaml(monkeypatch, tmp_path):
    documents = _copied_documents()
    documents['navigation']['amcl']['ros__parameters']['alpha1'] = 0.06
    _patch_packaged_yaml(monkeypatch, documents)
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize(actor=ACTOR, reason='yaml seed fixture')
    assert store.desired_overrides()['amcl.alpha1'] == 0.06
    assert INDEX['amcl.alpha1']['default'] == 0.05


def test_bootstrap_candidate_seeds_from_packaged_yaml(tmp_path):
    report = bootstrap.build_plan(REGISTRY_PATH, tmp_path / 'config')
    assert report['bootstrap_status'] == 'clean_uninitialized'
    assert report['candidate_defaults']['source'] == 'packaged_yaml_starting_values'
    assert report['candidate_defaults']['validation'] == 'ok'
    assert report['candidate_defaults']['reasons'] == []
    assert not (tmp_path / 'config').exists()


def test_bootstrap_candidate_uses_the_yaml_helper(monkeypatch, tmp_path):
    real = spec.yaml_starting_values

    def poisoned(index, *, documents=None):
        values = dict(real(index, documents=documents))
        values['max_linear_m_s'] = 0.9
        return values

    monkeypatch.setattr(bootstrap.tuning_spec, 'yaml_starting_values', poisoned)
    report = bootstrap.build_plan(REGISTRY_PATH, tmp_path / 'config')
    assert report['bootstrap_status'] == 'refused'
    assert report['candidate_defaults']['validation'] == 'rejected'
    assert any('max_linear_m_s' in reason and 'out_of_bounds' in reason
               for reason in report['candidate_defaults']['reasons'])


def test_uninitialized_snapshot_uses_yaml_starting_values(monkeypatch, tmp_path):
    documents = _copied_documents()
    documents['navigation']['amcl']['ros__parameters']['alpha1'] = 0.06
    _patch_packaged_yaml(monkeypatch, documents)
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    service = TuningService(store, _StubCoordinator(), session_id=SESSION)
    snapshot = service.runtime_snapshot()
    assert snapshot['desired']['amcl.alpha1'] == 0.06
    assert snapshot['desired']['max_linear_m_s'] == 0.30
    assert snapshot['active']['amcl.alpha1'] is None
    assert snapshot['active_known'] is False
    assert snapshot['desired_revision'] is None
    assert snapshot['committed_active_revision'] is None
    assert snapshot['committed_active'] is None
    assert snapshot['policy_required'] is False
    assert snapshot['advertised_capability'] is None
    assert snapshot['safe_apply']['allowed'] is False
    catalog = {entry['id']: entry for entry in snapshot['catalog']}
    assert catalog['amcl.alpha1']['default'] == 0.06
    assert catalog['safety_watchdog.scan_timeout']['default'] == 0.5
    assert INDEX['amcl.alpha1']['default'] == 0.05
    assert not store.base.exists()


def test_initialized_snapshot_keeps_saved_desired_and_catalog_default(tmp_path):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize(actor=ACTOR, reason='ready fixture')
    service = TuningService(store, _StubCoordinator(), session_id=SESSION)
    snapshot = service.runtime_snapshot()
    assert snapshot['desired_revision'] == 1
    assert snapshot['desired']['amcl.alpha1'] == INDEX['amcl.alpha1']['default']
    catalog = {entry['id']: entry for entry in snapshot['catalog']}
    assert catalog['amcl.alpha1']['default'] == INDEX['amcl.alpha1']['default']


def test_corrupt_saved_revision_never_falls_back_to_yaml_starting_values(tmp_path):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize(actor=ACTOR, reason='ready fixture')
    service = TuningService(store, _StubCoordinator(), session_id=SESSION)
    (store.revision_dir(1) / 'overrides.yaml').write_text('amcl.alpha1: 0.99\n')
    with pytest.raises(StoreUnavailable):
        service.runtime_snapshot()


def test_catalog_projection_does_not_mutate_the_registry():
    entry = INDEX['amcl.alpha1']
    before = copy.deepcopy(entry)
    projected = _catalog_entry(entry, 0.06)
    assert projected['default'] == 0.06
    assert entry == before
    assert _catalog_entry(entry)['default'] == 0.05
