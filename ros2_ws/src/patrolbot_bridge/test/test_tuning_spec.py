"""Speed mapping and deterministic consumer generation. No ROS."""

import json
from pathlib import Path

from patrolbot_bridge import tuning_spec as spec
import pytest
import yaml

REPO = Path(__file__).resolve().parents[4]
REGISTRY = json.loads((REPO / 'contracts' / 'robot-tuning.registry.json').read_text())
INDEX = spec.index_registry(REGISTRY)


def test_reviewed_defaults_never_exceed_the_hard_envelope():
    limits = spec.effective_limits({}, INDEX)
    assert limits == {'max_linear_m_s': 0.30, 'max_angular_rad_s': 0.70}


def test_lowering_linear_cap_preserves_lower_source_targets():
    overrides = {
        'max_linear_m_s': 0.15,
        'nav2.velocity_smoother.max_velocity_x': 0.15,
        'nav2.controller.desired_linear_vel': 0.12,
    }
    targets = spec.speed_producer_targets(overrides, INDEX)
    assert targets['nav2.controller.desired_linear_vel'] == 0.12
    assert targets['nav2.velocity_smoother.max_velocity_x'] == 0.15
    assert targets['nav2.controller.min_approach_linear_velocity'] == 0.08


def test_producers_are_clamped_to_the_cap_not_raised():
    # A source default above the cap must come down; nothing above the cap.
    overrides = {'max_angular_rad_s': 0.5}
    targets = spec.speed_producer_targets(overrides, INDEX)
    assert targets['nav2.velocity_smoother.max_velocity_theta'] == 0.5
    assert targets['nav2.controller.rotate_to_heading_angular_vel'] == 0.5
    assert targets['nav2.behavior.max_rotational_vel'] == 0.5
    for pid, value in targets.items():
        kind = INDEX[pid]['value_kind']
        cap = 0.5 if kind == 'angular_speed' else 0.30
        assert value <= cap + 1e-9


def test_bringup_overlay_is_symmetric_with_zero_lateral():
    overlay = spec.bringup_overlay({'max_linear_m_s': 0.2, 'max_angular_rad_s': 0.4}, INDEX)
    params = overlay['teleop_velocity_smoother']['ros__parameters']
    assert params['max_velocity'] == [0.2, 0.0, 0.4]
    assert params['min_velocity'] == [-0.2, 0.0, -0.4]


def test_f710_caps_follow_the_ceiling_and_remove_the_bypass():
    policy = spec.f710_policy({'max_angular_rad_s': 0.4, 'max_linear_m_s': 0.2}, INDEX)
    assert policy['max_angular_cap'] == 0.4
    assert policy['max_linear_cap'] == 0.2
    assert policy['max_linear'] <= policy['max_linear_cap']
    assert policy['max_angular'] <= policy['max_angular_cap']
    # The reviewed source hard-coded 3.0; the effective policy is far below it.
    assert policy['max_angular_cap'] < 3.0
    assert policy['min_angular'] <= 0.4
    assert policy['min_linear'] <= 0.2


def test_f710_minima_clamp_when_cap_falls_below_default_minimum():
    policy = spec.f710_policy({'max_linear_m_s': 0.02, 'max_angular_rad_s': 0.05}, INDEX)
    assert policy['min_linear'] == 0.02
    assert policy['min_angular'] == 0.05
    assert policy['max_linear_cap'] == 0.02


def test_generated_artifacts_are_deterministic_and_parseable():
    overrides = {'max_linear_m_s': 0.18, 'max_angular_rad_s': 0.6}
    first = spec.generated_artifacts(overrides, INDEX)
    second = spec.generated_artifacts(overrides, INDEX)
    assert first == second
    assert set(first) == {'bringup.yaml', 'navigation.yaml'}
    for text in first.values():
        assert isinstance(yaml.safe_load(text), dict)
    assert spec.artifacts_digest(first) == spec.artifacts_digest(second)


def test_navigation_overlay_uses_nested_plugin_keys_and_f710_policy():
    overrides = {'max_linear_m_s': 0.2}
    overlay = spec.navigation_overlay(overrides, INDEX)
    assert overlay['controller_server']['ros__parameters']['FollowPath'][
        'desired_linear_vel'] <= 0.2
    assert overlay['velocity_smoother']['ros__parameters']['max_velocity'] == [0.2, 0.0, 0.7]
    assert overlay['patrolbot_joy_teleop']['ros__parameters']['max_linear_cap'] == 0.2


def test_protected_defaults_are_preserved_in_full_generated_configuration():
    from patrolbot_bridge.tuning_yaml import strict_yaml_load
    document = strict_yaml_load(spec.generated_artifacts({}, INDEX)['navigation.yaml'])
    defaults = spec.packaged_defaults('navigation')
    for node, key in (('patrolbot_safety_watchdog', 'scan_timeout'),
                      ('patrolbot_safety_watchdog', 'odom_timeout'),
                      ('amcl', 'laser_model_type'), ('collision_monitor', 'source_timeout')):
        assert document[node]['ros__parameters'][key] == defaults[node]['ros__parameters'][key]
    assert document['amcl']['ros__parameters']['set_initial_pose'] is False
    assert document['amcl']['ros__parameters']['always_reset_initial_pose'] is True


def test_validate_change_rejects_protected_unknown_and_out_of_bounds():
    assert spec.validate_change(INDEX, 'amcl.laser_model_type', 'x')
    assert spec.validate_change(INDEX, 'safety_watchdog.scan_timeout', 1.0)
    assert spec.validate_change(INDEX, 'does.not.exist', 1)
    assert spec.validate_change(INDEX, 'max_linear_m_s', 0.9)
    assert spec.validate_change(INDEX, 'max_linear_m_s', True)
    assert spec.validate_change(INDEX, 'max_linear_m_s', 0.25) == []


def test_validate_overrides_enforces_reverse_dependency_edges():
    # Lowering max_particles alone leaves min_particles > max_particles.
    problems = spec.validate_overrides(INDEX, spec.reviewed_defaults(INDEX),
                                       [{'id': 'amcl.max_particles', 'value': 500}])
    assert problems
    # A coherent pair passes.
    assert spec.validate_overrides(
        INDEX, spec.reviewed_defaults(INDEX),
        [{'id': 'amcl.min_particles', 'value': 100},
         {'id': 'amcl.max_particles', 'value': 500}]) == []


def test_validate_overrides_requires_producers_in_line_with_cap():
    problems = spec.validate_overrides(INDEX, spec.reviewed_defaults(INDEX),
                                       [{'id': 'max_linear_m_s', 'value': 0.1}])
    assert problems == []
    targets = spec.speed_producer_targets({'max_linear_m_s': 0.1}, INDEX)
    assert targets['nav2.controller.desired_linear_vel'] == 0.1
    assert spec.validate_overrides(
        INDEX, spec.reviewed_defaults(INDEX),
        [{'id': 'max_linear_m_s', 'value': 0.1},
         {'id': 'nav2.velocity_smoother.max_velocity_x', 'value': 0.1},
         {'id': 'nav2.controller.desired_linear_vel', 'value': 0.1}]) == []


def test_unknown_registry_id_never_validates():
    with pytest.raises(spec.SpecError):
        spec.index_registry({'contract': spec.CONTRACT, 'version': 1,
                             'parameters': [{'id': 'x'}]})
