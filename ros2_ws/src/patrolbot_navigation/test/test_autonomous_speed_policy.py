"""Portable tests for the offline autonomous speed profile policy."""

import sys
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from autonomous_speed_policy import (  # noqa: E402
    REALLY_FAST_TARGET,
    resolve_autonomous_speed,
)


def test_default_and_presets():
    assert resolve_autonomous_speed() == resolve_autonomous_speed('Fast') == (0.22, 0.70)
    assert resolve_autonomous_speed('Slow') == (0.12, 0.40)


def test_really_fast_target_is_not_an_available_profile():
    assert REALLY_FAST_TARGET == (2.00, 0.70)
    with pytest.raises(ValueError, match='uncommissioned and unavailable'):
        resolve_autonomous_speed('Really Fast')
    with pytest.raises(ValueError, match='uncommissioned and unavailable'):
        resolve_autonomous_speed('Really Fast', commissioned_linear_mps=2.0)
    with pytest.raises(TypeError):
        resolve_autonomous_speed('Really Fast', commissioned=True)


@pytest.mark.parametrize('profile', ['fast', 'Unknown', '', None, True, []])
def test_invalid_profile_never_falls_back(profile):
    with pytest.raises(ValueError, match='unknown'):
        resolve_autonomous_speed(profile)


@pytest.mark.parametrize(('profile', 'bounds', 'axis'), [
    ('Slow', {'commissioned_linear_mps': 0.119}, 'linear'),
    ('Slow', {'commissioned_angular_radps': 0.399}, 'angular'),
    ('Fast', {'commissioned_linear_mps': 0.219}, 'linear'),
    ('Fast', {'commissioned_angular_radps': 0.699}, 'angular'),
])
def test_presets_reject_tighter_bounds_instead_of_clamping(profile, bounds, axis):
    with pytest.raises(ValueError, match=f'{axis} cap'):
        resolve_autonomous_speed(profile, **bounds)


def test_preset_at_commissioned_bounds_is_accepted():
    assert resolve_autonomous_speed(
        'Slow', commissioned_linear_mps=0.12, commissioned_angular_radps=0.40,
    ) == (0.12, 0.40)


def test_presets_cannot_take_custom_caps_even_if_the_replacement_would_fit():
    with pytest.raises(ValueError, match='only be supplied for Custom'):
        resolve_autonomous_speed('Slow', custom_linear_mps=0.10, custom_angular_radps=0.30)


def test_custom_needs_both_caps():
    with pytest.raises(ValueError, match='linear cap'):
        resolve_autonomous_speed('Custom', custom_angular_radps=0.40)
    with pytest.raises(ValueError, match='angular cap'):
        resolve_autonomous_speed('Custom', custom_linear_mps=0.12)


def test_custom_accepts_absolute_limits_tighter_limits_and_zero_as_a_cap():
    assert resolve_autonomous_speed(
        'Custom', custom_linear_mps=0.26, custom_angular_radps=0.70,
    ) == (0.26, 0.70)
    assert resolve_autonomous_speed(
        'Custom', custom_linear_mps=0, custom_angular_radps=0.30,
        commissioned_linear_mps=0, commissioned_angular_radps=0.30,
    ) == (0.0, 0.30)


@pytest.mark.parametrize(('linear', 'angular', 'axis'), [
    (0.21, 0.50, 'linear'),
    (0.20, 0.51, 'angular'),
])
def test_custom_rejects_caps_above_tighter_commissioned_bounds(linear, angular, axis):
    with pytest.raises(ValueError, match=f'{axis} cap'):
        resolve_autonomous_speed(
            'Custom', custom_linear_mps=linear, custom_angular_radps=angular,
            commissioned_linear_mps=0.20, commissioned_angular_radps=0.50,
        )


@pytest.mark.parametrize('field', [
    'custom_linear_mps', 'custom_angular_radps',
    'commissioned_linear_mps', 'commissioned_angular_radps',
])
@pytest.mark.parametrize('bad', [
    True, False, float('nan'), float('inf'), -float('inf'), -0.01, '0.12', None, 2.0,
])
def test_resolver_rejects_invalid_caps_and_bounds(field, bad):
    args = dict(custom_linear_mps=0.12, custom_angular_radps=0.40)
    args[field] = bad
    with pytest.raises(ValueError, match='must be a finite number'):
        resolve_autonomous_speed('Custom', **args)


def test_commissioned_bounds_cannot_raise_either_absolute_limit():
    with pytest.raises(ValueError, match='commissioned_linear_mps'):
        resolve_autonomous_speed('Fast', commissioned_linear_mps=0.260001)
    with pytest.raises(ValueError, match='commissioned_angular_radps'):
        resolve_autonomous_speed('Fast', commissioned_angular_radps=0.700001)


def test_huge_integer_caps_are_rejected_without_conversion_overflow():
    with pytest.raises(ValueError, match='linear cap'):
        resolve_autonomous_speed('Custom', custom_linear_mps=10**400, custom_angular_radps=0.40)
