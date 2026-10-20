"""Unit checks for documented Nav2 domains and the stricter robot edit envelope."""

import json
from pathlib import Path

from patrolbot_bridge import tuning_spec
import pytest


ROOT = Path(__file__).resolve().parents[4]
REGISTRY = json.loads((ROOT / 'contracts/robot-tuning.registry.json').read_text())
INDEX = tuning_spec.index_registry(REGISTRY)


@pytest.mark.parametrize('pid,bad,good', [
    ('amcl.sigma_hit', 0.0, .2),
    ('nav2.controller.min_approach_linear_velocity', .01, .08),
    ('nav2.controller.approach_velocity_scaling_dist', 1.5, .6),
    ('amcl.pf_z', 0.0, 2.33),
    ('amcl.min_particles', 0, 1000),
    ('amcl.max_beams', 1, 60),
    ('max_linear_m_s', .31, .30),
    ('max_angular_rad_s', .71, .70),
])
def test_invalid_domain_boundary_is_refused_but_yaml_start_is_accepted(pid, bad, good):
    assert tuning_spec.validate_change(INDEX, pid, bad)
    assert tuning_spec.validate_change(INDEX, pid, good) == []


def test_probability_weights_must_obey_documented_mixture_sum():
    changes = [{'id': 'amcl.z_hit', 'value': .8}, {'id': 'amcl.z_rand', 'value': .5}]
    assert tuning_spec.validate_overrides(INDEX, {}, changes)
    changes[1]['value'] = .2
    assert tuning_spec.validate_overrides(INDEX, {}, changes) == []
