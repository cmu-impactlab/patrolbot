"""Source-backed dependency previews, never fake evidence of native execution."""

import copy
import json
from pathlib import Path

import pytest

from patrolbot_bridge.tuning_plan import build_plan, canonical_digest
from patrolbot_bridge.tuning_store import RevisionStore

ROOT = Path(__file__).resolve().parents[4]
REGISTRY = json.loads((ROOT / 'contracts/robot-tuning.registry.json').read_text())
ACTOR = {'id': 1, 'username': 'fixture-admin', 'role': 'administrator'}
SESSION = '11111111-1111-4111-8111-111111111111'
REQUEST = '22222222-2222-4222-8222-222222222222'


def setup(tmp_path, changes):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize(actor=ACTOR, reason='isolated fixture')
    result = store.save(request_id=REQUEST, base_revision=1, changes=changes, actor=ACTOR)
    assert result['outcome'] == 'ok'
    return store


def preview(store, **patch):
    arguments = dict(candidate=store.load_revision(2), active=store.load_revision(1),
                     index=store.index, registry_digest=store.registry_digest,
                     session_id=SESSION, generation=(1, 1), desired_revision=2)
    arguments.update(patch)
    return build_plan(**arguments)


@pytest.mark.parametrize('pid,value,nodes', [
    ('max_linear_m_s', .2, {'teleop_velocity_smoother', 'velocity_smoother',
                          'controller_server', 'patrolbot_joy_teleop'}),
    ('max_angular_rad_s', .5, {'teleop_velocity_smoother', 'velocity_smoother',
                             'controller_server', 'patrolbot_joy_teleop', 'behavior_server'}),
    ('nav2.controller.lookahead_dist', .5, {'controller_server'}),
    ('nav2.velocity_smoother.max_velocity_x', .25, {'velocity_smoother'}),
    ('nav2.behavior.max_rotational_vel', .5, {'behavior_server'}),
    ('amcl.alpha1', .06, {'amcl'}),
])
def test_exact_compiled_closure_not_registry_scope(tmp_path, pid, value, nodes):
    store = setup(tmp_path, [{'id': pid, 'value': value}])
    before = store.manifest_path.read_bytes()
    plan = preview(store)
    assert set(plan['nodes']) == nodes
    assert plan['seed_required'] is ('amcl' in nodes)
    assert plan['verification_scopes'] == ['bringup', 'navigation']
    assert plan['changes'] == [{'id': pid, 'before': store.load_revision(1)['overrides'][pid], 'after': value}]
    assert not plan['executable']
    assert store.manifest_path.read_bytes() == before
    assert store.read_startup()['revision'] == 1
    assert store.latest_job() is None


def test_complete_arrays_and_lower_producer_ceilings_are_explicit(tmp_path):
    store = setup(tmp_path, [{'id': 'max_linear_m_s', 'value': .2}])
    values = {step['node']: step['parameters'] for step in preview(store)['operations']}
    assert values['teleop_velocity_smoother'] == {
        'max_velocity': [.2, 0., .7], 'min_velocity': [-.2, 0., -.7]}
    assert values['controller_server']['FollowPath.desired_linear_vel'] == .2
    assert 'FollowPath.min_approach_linear_velocity' not in values['controller_server']
    assert 'amcl' not in values


@pytest.mark.parametrize('part', ['session_id', 'generation', 'implementation', 'operation', 'desired_revision'])
def test_plan_hash_binds_all_review_context_not_only_candidate_digest(tmp_path, part):
    store = setup(tmp_path, [{'id': 'amcl.alpha1', 'value': .06}])
    first = preview(store)
    patch = {'session_id': '33333333-3333-4333-8333-333333333333', 'generation': (2, 1),
             'implementation': 'different-pinned-image', 'operation': 'restore', 'desired_revision': 3}
    second = preview(store, **{part: patch[part]})
    assert first['digest'] == second['digest']
    assert first['plan_hash'] != second['plan_hash']


def test_plan_is_deterministic_and_hash_covers_exact_scope_and_seed_loss(tmp_path):
    store = setup(tmp_path, [{'id': 'amcl.alpha1', 'value': .06}])
    first = preview(store)
    assert preview(store) == first
    content = {key: value for key, value in first.items() if key != 'plan_hash'}
    assert canonical_digest(content) == first['plan_hash']
    modified = copy.deepcopy(content)
    modified['seed_required'] = False
    assert canonical_digest(modified) != first['plan_hash']


def test_constructor_only_parameter_cannot_claim_hot_update_support(tmp_path):
    store = setup(tmp_path, [{'id': 'nav2.controller.approach_velocity_scaling_dist', 'value': .7}])
    plan = preview(store, support={'controller_server': 'parameters'}, implementation='test-only-pinned-source')
    assert not plan['executable']
    assert plan['operations'][0]['kind'] == 'lifecycle'


def test_unproven_policies_cannot_be_covered_by_node_service_availability(tmp_path):
    store = setup(tmp_path, [{'id': 'max_linear_m_s', 'value': .2}])
    plan = preview(store, support={node: 'lifecycle' for node in (
        'teleop_velocity_smoother', 'velocity_smoother', 'controller_server', 'patrolbot_joy_teleop')},
        implementation='test-only-native-image')
    assert not plan['executable']
    assert any(reason['code'] == 'policy_handshake_unverified' for reason in plan['reasons'])


def test_invalid_runtime_domain_keeps_saved_targets_readable_but_unexecutable(tmp_path):
    store = setup(tmp_path, [{'id': 'max_linear_m_s', 'value': 0.}])
    plan = preview(store)
    assert not plan['executable']
    assert any(reason['code'] == 'runtime_domain_invalid' for reason in plan['reasons'])
    assert store.snapshot_state()['desired_revision'] == 2


def test_no_arbitrary_native_nodes_or_operation_names(tmp_path):
    store = setup(tmp_path, [{'id': 'amcl.alpha1', 'value': .06}])
    with pytest.raises(ValueError):
        preview(store, support={'../../bin/sh': 'lifecycle'})
    with pytest.raises(ValueError):
        preview(store, support={'amcl': 'shell'})
