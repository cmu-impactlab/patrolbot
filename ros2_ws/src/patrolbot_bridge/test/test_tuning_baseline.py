"""Offline baseline-bundle validation and exact representability. No ROS."""

import copy
import json
from pathlib import Path

from patrolbot_bridge import tuning_spec as spec
from patrolbot_bridge.tuning_baseline import (
    ARTIFACT_SCOPES,
    BaselineError,
    BUNDLE_FIELDS,
    CONTRACT,
    editable_ids,
    EVIDENCE_KIND,
    prepare_baseline,
    VERSION,
)
import pytest

REPO = Path(__file__).resolve().parents[4]
REGISTRY = json.loads((REPO / 'contracts' / 'robot-tuning.registry.json').read_text())
INDEX = spec.index_registry(REGISTRY)
CAPTURE = 'abcdef01-2345-4678-89ab-cdef01234567'
SESSION = '22222222-2222-4222-8222-222222222222'
SOURCE = Path(__file__).resolve().parents[1] / 'patrolbot_bridge' / 'tuning_baseline.py'


def lower_values():
    """Every editable value, with both ceilings lowered below the defaults."""
    values = spec.reviewed_defaults(INDEX)
    values = {pid: values[pid] for pid in editable_ids(INDEX)}
    values['max_linear_m_s'] = 0.15
    values['max_angular_rad_s'] = 0.35
    return values


def build_bundle(**changes):
    values = lower_values()
    bundle = {
        'contract': CONTRACT,
        'version': VERSION,
        'capture_id': CAPTURE,
        'source_session_id': SESSION,
        'registry_digest': spec.registry_digest(REGISTRY),
        'defaults_digest': spec.defaults_digest(),
        'observed_values': values,
        'observed_artifacts': spec.generated_artifacts(values, INDEX),
    }
    bundle.update(changes)
    return bundle


def test_valid_lower_baseline_returns_deterministic_candidate():
    bundle = build_bundle()
    result = prepare_baseline(bundle, REGISTRY)
    assert set(result) == {'overrides', 'artifacts', 'provenance', 'digest'}
    assert result['overrides'] == bundle['observed_values']
    assert result['overrides']['max_linear_m_s'] == 0.15
    assert result['artifacts'] == spec.generated_artifacts(result['overrides'], INDEX)
    assert set(result['artifacts']) == set(ARTIFACT_SCOPES)
    assert result['provenance'] == {
        'evidence_kind': EVIDENCE_KIND,
        'capture_id': CAPTURE,
        'source_session_id': SESSION,
        'registry_digest': spec.registry_digest(REGISTRY),
        'defaults_digest': spec.defaults_digest(),
        'baseline_digest': result['digest'],
        'runtime_verified': False,
        'authorized': False,
    }
    assert result['digest'].startswith('sha256:')
    assert prepare_baseline(copy.deepcopy(bundle), REGISTRY) == result


def test_a_lower_source_specific_producer_is_preserved_not_raised():
    values = lower_values()
    values['nav2.controller.desired_linear_vel'] = 0.12
    bundle = build_bundle(observed_values=values,
                          observed_artifacts=spec.generated_artifacts(values, INDEX))
    result = prepare_baseline(bundle, REGISTRY)
    assert result['overrides']['nav2.controller.desired_linear_vel'] == 0.12
    assert 'desired_linear_vel: 0.12' in result['artifacts']['navigation.yaml']


def test_json_text_and_mapping_inputs_are_equivalent():
    bundle = build_bundle()
    assert prepare_baseline(json.dumps(bundle), REGISTRY) == prepare_baseline(bundle, REGISTRY)


def test_prepare_never_mutates_its_inputs():
    bundle = build_bundle()
    registry = copy.deepcopy(REGISTRY)
    before_bundle = copy.deepcopy(bundle)
    before_registry = copy.deepcopy(registry)
    result = prepare_baseline(bundle, registry)
    assert bundle == before_bundle
    assert registry == before_registry
    result['overrides']['max_linear_m_s'] = 9.9
    result['artifacts']['bringup.yaml'] = 'changed'
    assert bundle['observed_values']['max_linear_m_s'] == 0.15


def test_digest_binds_every_field():
    first = prepare_baseline(build_bundle(), REGISTRY)['digest']
    second = prepare_baseline(build_bundle(capture_id=SESSION), REGISTRY)['digest']
    third = prepare_baseline(build_bundle(source_session_id=CAPTURE), REGISTRY)['digest']
    assert len({first, second, third}) == 3


def test_editable_ids_match_the_store_initialize_filter():
    assert set(editable_ids(INDEX)) == {pid for pid in INDEX
                                        if INDEX[pid].get('accepts_edits') is True
                                        and INDEX[pid].get('protected') is not True
                                        and INDEX[pid].get('not_applicable') is not True}
    assert 'amcl.alpha5' not in editable_ids(INDEX)
    assert 'safety_watchdog.scan_timeout' not in editable_ids(INDEX)


def test_missing_logical_value_is_refused_not_defaulted():
    bundle = build_bundle()
    del bundle['observed_values']['amcl.alpha1']
    with pytest.raises(BaselineError) as info:
        prepare_baseline(bundle, REGISTRY)
    assert info.value.code == 'observed_scope_mismatch'


def test_unknown_logical_id_is_refused():
    bundle = build_bundle()
    bundle['observed_values']['does.not.exist'] = 0.0
    with pytest.raises(BaselineError) as info:
        prepare_baseline(bundle, REGISTRY)
    assert info.value.code == 'observed_scope_mismatch'


def test_missing_artifact_scope_is_refused():
    bundle = build_bundle()
    del bundle['observed_artifacts']['bringup.yaml']
    with pytest.raises(BaselineError) as info:
        prepare_baseline(bundle, REGISTRY)
    assert info.value.code == 'observed_scope_mismatch'


def test_unknown_artifact_scope_is_refused():
    bundle = build_bundle()
    bundle['observed_artifacts']['extra.yaml'] = 'a: 1\n'
    with pytest.raises(BaselineError) as info:
        prepare_baseline(bundle, REGISTRY)
    assert info.value.code == 'observed_scope_mismatch'


def test_unknown_bundle_field_is_refused():
    with pytest.raises(BaselineError) as info:
        prepare_baseline(build_bundle(extra='x'), REGISTRY)
    assert info.value.code == 'invalid_bundle'
    assert 'observed_artifacts' in BUNDLE_FIELDS


@pytest.mark.parametrize('version', [True, False, '1', 1.0, 2, 0, None])
def test_version_must_be_the_strict_integer(version):
    with pytest.raises(BaselineError) as info:
        prepare_baseline(build_bundle(version=version), REGISTRY)
    assert info.value.code == 'unsupported_version'


def test_contract_must_match():
    with pytest.raises(BaselineError) as info:
        prepare_baseline(build_bundle(contract='other'), REGISTRY)
    assert info.value.code == 'unsupported_contract'


@pytest.mark.parametrize('field', ['registry_digest', 'defaults_digest'])
def test_stale_digest_is_refused(field):
    with pytest.raises(BaselineError) as info:
        prepare_baseline(build_bundle(**{field: 'sha256:' + '0' * 64}), REGISTRY)
    assert info.value.code == field + '_mismatch'


@pytest.mark.parametrize('field', ['capture_id', 'source_session_id'])
@pytest.mark.parametrize('value', ['not-a-uuid', CAPTURE.upper(), CAPTURE + 'x', 7])
def test_identifiers_must_be_canonical_uuids(field, value):
    with pytest.raises(BaselineError) as info:
        prepare_baseline(build_bundle(**{field: value}), REGISTRY)
    assert info.value.code == 'invalid_' + field


def test_non_mapping_bundle_is_refused():
    for bundle in ([], 3, '[]', '{}'):
        with pytest.raises(BaselineError) as info:
            prepare_baseline(bundle, REGISTRY)
        assert info.value.code == 'invalid_bundle'


def test_duplicate_json_key_is_refused():
    text = '{"contract":"patrolbot-tuning-baseline","contract":"x"}'
    with pytest.raises(BaselineError) as info:
        prepare_baseline(text, REGISTRY)
    assert info.value.code == 'invalid_bundle'


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf'), 2 ** 64])
def test_nonfinite_or_oversized_number_is_refused(bad):
    bundle = build_bundle()
    bundle['observed_values']['max_linear_m_s'] = bad
    with pytest.raises(BaselineError) as info:
        prepare_baseline(bundle, REGISTRY)
    assert info.value.code == 'invalid_bundle'


def test_deeply_nested_bundle_is_refused():
    nested = {}
    cursor = nested
    for _ in range(40):
        cursor['child'] = {}
        cursor = cursor['child']
    bundle = build_bundle()
    bundle['observed_values'] = nested
    with pytest.raises(BaselineError) as info:
        prepare_baseline(bundle, REGISTRY)
    assert info.value.code == 'invalid_bundle'


def test_boolean_is_not_accepted_as_a_number():
    bundle = build_bundle()
    bundle['observed_values']['max_linear_m_s'] = True
    with pytest.raises(BaselineError) as info:
        prepare_baseline(bundle, REGISTRY)
    assert info.value.code == 'invalid_observed_value'


def test_out_of_bounds_value_is_refused():
    bundle = build_bundle()
    bundle['observed_values']['max_linear_m_s'] = 0.9
    with pytest.raises(BaselineError) as info:
        prepare_baseline(bundle, REGISTRY)
    assert info.value.code == 'invalid_observed_value'


def test_incoherent_non_speed_dependency_is_refused():
    bundle = build_bundle()
    bundle['observed_values']['amcl.min_particles'] = 5000
    with pytest.raises(BaselineError) as info:
        prepare_baseline(bundle, REGISTRY)
    assert info.value.code == 'dependency_violation'


def test_speed_producer_above_its_cap_is_representable():
    bundle = build_bundle()
    assert bundle['observed_values']['nav2.velocity_smoother.max_velocity_theta'] == 1.0
    result = prepare_baseline(bundle, REGISTRY)
    assert 'max_velocity:\n    - 0.15\n    - 0.0\n    - 0.35' in (
        result['artifacts']['navigation.yaml'])


def test_protected_artifact_change_is_refused_not_merged():
    bundle = build_bundle()
    navigation = bundle['observed_artifacts']['navigation.yaml']
    assert 'scan_timeout: 0.5' in navigation
    mutated = navigation.replace('scan_timeout: 0.5', 'scan_timeout: 5.0', 1)
    assert mutated != navigation
    bundle['observed_artifacts']['navigation.yaml'] = mutated
    with pytest.raises(BaselineError) as info:
        prepare_baseline(bundle, REGISTRY)
    assert info.value.code == 'artifact_mismatch'


def test_lower_artifact_producer_is_refused_not_widened():
    bundle = build_bundle()
    navigation = bundle['observed_artifacts']['navigation.yaml']
    assert 'desired_linear_vel: 0.15' in navigation
    mutated = navigation.replace('desired_linear_vel: 0.15', 'desired_linear_vel: 0.12', 1)
    assert mutated != navigation
    bundle['observed_artifacts']['navigation.yaml'] = mutated
    with pytest.raises(BaselineError) as info:
        prepare_baseline(bundle, REGISTRY)
    assert info.value.code == 'artifact_mismatch'


def test_duplicate_yaml_key_in_artifact_is_refused():
    bundle = build_bundle()
    bundle['observed_artifacts']['bringup.yaml'] = 'a: 1\na: 2\n'
    with pytest.raises(BaselineError) as info:
        prepare_baseline(bundle, REGISTRY)
    assert info.value.code == 'invalid_observed_artifact'


def test_yaml_anchor_bomb_in_artifact_is_refused():
    bundle = build_bundle()
    bundle['observed_artifacts']['navigation.yaml'] = 'base: &b {a: 1}\nchild:\n  <<: *b\n'
    with pytest.raises(BaselineError) as info:
        prepare_baseline(bundle, REGISTRY)
    assert info.value.code == 'invalid_observed_artifact'


def test_non_text_artifact_is_refused():
    bundle = build_bundle()
    bundle['observed_artifacts']['bringup.yaml'] = {'not': 'text'}
    with pytest.raises(BaselineError) as info:
        prepare_baseline(bundle, REGISTRY)
    assert info.value.code == 'invalid_observed_artifact'


def test_prepare_writes_nothing(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('the baseline validator must not write')

    for name in ('write_text', 'write_bytes', 'mkdir', 'touch', 'unlink',
                 'rename', 'replace', 'chmod', 'symlink_to'):
        monkeypatch.setattr(Path, name, forbidden, raising=False)
    result = prepare_baseline(build_bundle(), REGISTRY)
    assert result['digest'].startswith('sha256:')


def test_module_never_imports_ros():
    assert 'rclpy' not in SOURCE.read_text()
