"""Offline guard tests for ordinary controller commands; no ROS transport."""

from dataclasses import replace
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from autonomous_speed_output import (  # noqa: E402
    ActiveOutputFence,
    CommandEpoch,
    ControllerSample,
    guard_autonomous_speed_output as guard,
)
from autonomous_speed_policy import resolve_autonomous_speed  # noqa: E402


STOP = (0.0, 0.0)


@pytest.fixture
def fence():
    return ActiveOutputFence(CommandEpoch(), resolve_autonomous_speed(
        'Custom', custom_linear_mps=0.20, custom_angular_radps=0.60))


def sample(fence, linear=0.24, angular=0.0, received=10.0):
    return ControllerSample(fence.epoch, linear, angular, received)


def test_speed_limit_message_cannot_raise_the_confirmed_linear_cap(fence):
    # Raw RPP had 0.20 storage readback but output 0.24 after an absolute
    # SpeedLimit. The offline guard must enforce the confirmed policy anyway.
    assert guard(sample(fence), fence, now_monotonic_s=10.1) == (0.20, 0.0)
    assert guard(sample(fence, 0.15), fence, now_monotonic_s=10.1) == (0.15, 0.0)


@pytest.mark.parametrize('angular', [0.35, -0.35, 0.70, -0.70])
def test_turning_limit_scales_a_single_ratio_without_widening(fence, angular):
    lower = replace(fence, caps=(0.20, 0.40))
    x, z = guard(sample(lower, 0.26, angular), lower, now_monotonic_s=10.1)
    assert 0.0 < x <= 0.20
    assert 0.0 < abs(z) <= 0.40
    assert z / x == pytest.approx(angular / 0.26)
    assert max(x / 0.20, abs(z) / 0.40) == pytest.approx(1.0)


def test_rotating_without_forward_is_supported_inside_the_angular_limit(fence):
    x, z = guard(sample(fence, 0.0, -0.70), fence, now_monotonic_s=10.0)
    assert x == 0.0 and z == pytest.approx(-0.60) and abs(z) <= 0.60


@pytest.mark.parametrize('time', [9.9, 10.25, 11.0, float('nan'), float('inf'), True, 10**400])
def test_stale_future_or_invalid_timing_stops(fence, time):
    assert guard(sample(fence), fence, now_monotonic_s=time) == STOP


def test_epoch_change_or_missing_current_authority_stops(fence):
    data = sample(fence)
    for current in (None, replace(fence, epoch=CommandEpoch()), replace(fence, epoch=None)):
        assert guard(data, current, now_monotonic_s=10.1) == STOP
    assert guard(None, fence, now_monotonic_s=10.1) == STOP


@pytest.mark.parametrize('linear,angular', [
    (-0.001, 0.0), (2.00, 0.0), (0.26001, 0.0), (0.1, -0.701),
    (0.1, 0.701), (True, 0.0), (0.1, True), (float('nan'), 0.1),
    (0.1, float('inf')), (10**400, 0.0),
])
def test_invalid_reverse_or_uncommissioned_raw_sample_stops(fence, linear, angular):
    assert guard(sample(fence, linear, angular), fence, now_monotonic_s=10.0) == STOP


@pytest.mark.parametrize('caps', [None, (2.0, 0.70), (0.26, True), (0.26,), [0.26, 0.70]])
def test_malformed_policy_does_not_revert_to_baseline(fence, caps):
    assert guard(sample(fence), replace(fence, caps=caps), now_monotonic_s=10.0) == STOP


def test_zero_axis_stops_rather_than_turning_a_curve_into_a_line(fence):
    turning = sample(fence, 0.10, 0.20)
    for caps in ((0.0, 0.40), (0.20, 0.0)):
        assert guard(turning, replace(fence, caps=caps), now_monotonic_s=10.0) == STOP
