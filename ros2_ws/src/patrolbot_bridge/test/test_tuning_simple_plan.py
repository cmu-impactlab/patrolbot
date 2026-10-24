"""Reduced two-mode policy; no per-node lifecycle orchestration or broad restart."""

import json
from pathlib import Path

import pytest

from patrolbot_bridge.tuning_mutability import NAVIGATION_NODES, classify
from patrolbot_bridge.tuning_backend import NODES
from patrolbot_bridge.tuning_simple_plan import build_plan
from patrolbot_bridge.tuning_store import RevisionStore

ROOT = Path(__file__).resolve().parents[4]
REGISTRY = json.loads((ROOT / 'contracts/robot-tuning.registry.json').read_text())
ACTOR = {'id': 1, 'username': 'isolated-plan-fixture', 'role': 'administrator'}
STATES = dict.fromkeys(NODES, 3)
STATES['patrolbot_joy_teleop'] = 'running'


def plan(tmp_path, changes, **patch):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize(actor=ACTOR, reason='isolated reduced-plan fixture')
    assert store.save(request_id='22222222-2222-4222-8222-222222222222', base_revision=1,
                      changes=changes, actor=ACTOR)['outcome'] == 'ok'
    arguments = dict(candidate=store.load_revision(2), active=store.load_revision(1),
        index=store.index, registry_digest=store.registry_digest,
        session_id='11111111-1111-4111-8111-111111111111', generation=(1, 1), desired_revision=2,
        states=STATES, hot_support=NODES, nav2_redeploy_supported=True, policy_support=True,
        implementation='OFFLINE-FIXTURE-NOT-NATIVE-ACCEPTANCE')
    arguments.update(patch)
    result = build_plan(**arguments)
    assert store.read_startup()['revision'] == 1
    assert store.latest_job() is None
    return result


@pytest.mark.parametrize('node,key,state', [
    *[('amcl', key, 3) for key in ('alpha1', 'alpha2', 'alpha3', 'alpha4', 'update_min_a', 'update_min_d',
                                  'max_beams', 'sigma_hit', 'z_hit', 'z_rand')],
    *[('controller_server', 'FollowPath.' + key, 2) for key in ('desired_linear_vel', 'lookahead_dist',
        'min_lookahead_dist', 'max_lookahead_dist', 'lookahead_time', 'rotate_to_heading_angular_vel',
        'min_approach_linear_velocity', 'regulated_linear_scaling_min_speed')],
    ('velocity_smoother', 'max_velocity', 3), ('velocity_smoother', 'min_velocity', 3),
    ('teleop_velocity_smoother', 'max_velocity', 3), ('patrolbot_joy_teleop', 'max_linear_cap', 'running'),
])
def test_source_backed_hot_callbacks_are_not_blanket_classified_restart(node, key, state):
    assert classify(node, [key], state)[0] == 'hot'


@pytest.mark.parametrize('node,key,state', [
    *[('amcl', key, 3) for key in ('min_particles', 'max_particles', 'pf_err', 'pf_z')],
    ('controller_server', 'FollowPath.approach_velocity_scaling_dist', 3),
    ('behavior_server', 'max_rotational_vel', 3), ('behavior_server', 'min_rotational_vel', 2),
    ('amcl', 'alpha1', 2), ('velocity_smoother', 'max_velocity', 2),
])
def test_constructor_reset_or_missing_inactive_callback_requires_whole_nav2(node, key, state):
    assert classify(node, [key], state)[0] == 'nav2_redeploy'


@pytest.mark.parametrize('pid,value', [('amcl.alpha1', .06), ('amcl.update_min_d', .2),
    ('amcl.max_beams', 64), ('amcl.sigma_hit', .25), ('nav2.controller.lookahead_dist', .5),
    ('nav2.velocity_smoother.max_velocity_x', .25), ('max_linear_m_s', .2)])
def test_hot_only_batch_restarts_nothing_and_preserves_localization(tmp_path, pid, value):
    result = plan(tmp_path, [{'id': pid, 'value': value}])
    assert result['executable'], result['reasons']
    assert result['execution_kind'] == 'scoped_parameters'
    assert result['services'] == [] and not result['seed_required']
    assert all(step['kind'] == 'parameters' for step in result['operations'])
    assert 'image_rebuild' in result and result['image_rebuild'] is False


@pytest.mark.parametrize('pid,value', [('amcl.min_particles', 1500), ('amcl.max_particles', 4500),
    ('amcl.pf_err', .06), ('amcl.pf_z', 1.2), ('nav2.controller.approach_velocity_scaling_dist', .7),
    ('nav2.behavior.max_rotational_vel', .5), ('max_angular_rad_s', .5)])
def test_any_restart_field_selects_one_navigation_container_not_single_nodes_or_core(tmp_path, pid, value):
    result = plan(tmp_path, [{'id': pid, 'value': value}])
    assert result['executable'], result['reasons']
    assert result['execution_kind'] == 'nav2_redeploy'
    assert result['services'] == ['navigation'] and result['seed_required'] is True
    assert set(NAVIGATION_NODES) <= set(result['nodes'])
    assert not set(result['services']) & {'bridge', 'bringup', 'web-bridge', 'gimbal', 'sbc', 'dashboard'}
    assert not any(step['kind'] == 'lifecycle' for step in result['operations'])


def test_mixed_batch_never_hot_writes_old_nav2_before_restarting_it(tmp_path):
    result = plan(tmp_path, [{'id': 'nav2.controller.lookahead_dist', 'value': .5},
                             {'id': 'nav2.behavior.max_rotational_vel', 'value': .5}])
    assert result['execution_kind'] == 'nav2_redeploy'
    assert result['services'] == ['navigation']
    assert {step['kind'] for step in result['operations']} == {'nav2_restart'}


def test_inactive_smoother_escalation_is_reviewed_before_any_mutation(tmp_path):
    states = dict(STATES, velocity_smoother=2)
    result = plan(tmp_path, [{'id': 'max_linear_m_s', 'value': .2}], states=states)
    assert result['execution_kind'] == 'nav2_redeploy' and result['seed_required']


def test_unavailable_nav2_helper_is_refusal_not_silent_lifecycle_fallback(tmp_path):
    result = plan(tmp_path, [{'id': 'nav2.behavior.max_rotational_vel', 'value': .5}],
                  nav2_redeploy_supported=False)
    assert not result['executable']
    assert result['execution_kind'] == 'nav2_redeploy'
    assert any(reason['code'] == 'nav2_redeploy_unavailable' for reason in result['reasons'])


def test_missing_state_never_mislabels_a_known_constructor_edit_as_hot(tmp_path):
    result = plan(tmp_path, [{'id': 'nav2.behavior.max_rotational_vel', 'value': .5}], states={})
    assert not result['executable']
    assert result['execution_kind'] == 'nav2_redeploy' and result['services'] == ['navigation']
    assert any(reason['code'] == 'consumer_state_unsupported' for reason in result['reasons'])


def test_unavailable_external_smoother_does_not_expand_nav2_to_whole_robot(tmp_path):
    result = plan(tmp_path, [{'id': 'max_linear_m_s', 'value': .2}],
                  states=dict(STATES, teleop_velocity_smoother=2))
    assert not result['executable']
    assert 'bringup' not in result['services'] and 'bridge' not in result['services']


def test_unknown_and_transitional_states_never_authorize_hot_or_automatic_repair():
    for state in (None, True, 10, 'active'):
        assert classify('controller_server', ['FollowPath.lookahead_dist'], state)[0] == 'unsupported'


def test_policy_unavailability_and_domain_error_stay_honest(tmp_path):
    result = plan(tmp_path, [{'id': 'max_linear_m_s', 'value': .2}], policy_support=False)
    assert not result['executable'] and any(reason['code'] == 'policy_handshake_unverified' for reason in result['reasons'])
