"""Explicit release and buffer evidence never defaults missing data to safe idle."""

from patrolbot_bridge.tuning_runtime import TuningObservations
import pytest


def test_releases_are_fresh_observations_not_absence_of_manual_intent():
    clock = [10.0]
    obs = TuningObservations(lambda: clock[0])
    assert obs.evidence().physical_deadman_held is None
    obs.deadman('physical', True)
    assert obs.evidence().physical_deadman_held is True
    obs.deadman('physical', False)
    assert obs.evidence().physical_deadman_held is False
    clock[0] += .51
    assert obs.evidence().physical_deadman_held is None
    assert obs.evidence().physical_deadman_fresh is False


def test_zero_buffers_require_both_observations_and_publisher_liveness():
    clock = [10.0]
    obs = TuningObservations(lambda: clock[0])
    obs.healthy_publishers('publishers-1')
    obs.velocity('/cmd_vel', 0, 0)
    assert not obs.evidence().command_buffers_zero
    obs.velocity('/cmd_vel_out', 0, 0)
    assert obs.evidence().command_buffers_zero
    clock[0] += .51
    assert not obs.evidence().command_buffers_zero
    obs.healthy_publishers('publishers-1')
    assert obs.evidence().command_buffers_zero
    obs.healthy_publishers('publishers-2')
    assert not obs.evidence().command_buffers_zero


@pytest.mark.parametrize('value', [.1, float('nan'), float('inf'), True])
def test_nonzero_or_invalid_velocity_never_proves_stopped(value):
    obs = TuningObservations(lambda: 10.)
    obs.healthy_publishers('same')
    obs.velocity('/cmd_vel_out', 0, 0)
    obs.velocity('/cmd_vel', value, 0)
    assert not obs.evidence().command_buffers_zero


def test_session_change_discards_released_deadmen_and_prior_zeros():
    obs = TuningObservations(lambda: 10.)
    obs.deadman('physical', False)
    obs.deadman('web', False)
    obs.reset(2)
    assert obs.evidence().physical_deadman_held is None
    assert obs.evidence().web_deadman_held is None
    assert not obs.evidence().command_buffers_zero


def test_future_receipt_after_clock_regression_is_unknown():
    clock = [10.]
    obs = TuningObservations(lambda: clock[0])
    obs.deadman('web', False)
    clock[0] = 9.
    assert obs.evidence().web_deadman_held is None
