import math

from patrolbot_bridge.odom_validation import OdomSample, OdomValidator
import pytest


def validator():
    # Exact existing contracts: patrolbot_server classifyDrive() accepts
    # <=0.30 m/s and <=1.20 rad/s; telemetry is emitted every 50 ms.
    return OdomValidator(
        max_linear_speed=0.30,
        max_angular_speed=1.20,
        expected_period=0.05,
    )


def sample(x=0.0, y=0.0, yaw=0.0, linear=0.0, angular=0.0):
    return OdomSample(x, y, yaw, linear, angular)


@pytest.mark.parametrize('field', range(5))
@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_every_non_finite_wire_value_is_rejected(field, bad):
    values = list(sample().values())
    values[field] = bad
    result = validator().check_and_accept(OdomSample(*values), 1.0)
    assert not result.accepted
    assert result.reason == 'non-finite'


def test_reported_twist_must_respect_the_sbc_drive_contract():
    check = validator()
    assert not check.check_and_accept(sample(linear=0.300001), 1.0).accepted
    assert not check.check_and_accept(sample(angular=-1.200001), 1.0).accepted
    assert check.check_and_accept(sample(linear=0.30, angular=-1.20), 1.0).accepted


def test_continuity_uses_elapsed_time_speed_limits_and_one_sample_period():
    check = validator()
    assert check.check_and_accept(sample(), 10.0).accepted
    # 0.1 s elapsed + one 0.05 s sample period at 0.30 m/s = 0.045 m.
    assert check.check_and_accept(sample(x=0.045), 10.1).accepted
    result = check.check_and_accept(sample(x=0.091), 10.2)
    assert not result.accepted
    assert result.reason == 'position-discontinuity'


def test_heading_wrap_is_not_a_false_discontinuity():
    check = validator()
    assert check.check_and_accept(sample(yaw=math.pi - 0.01), 1.0).accepted
    assert check.check_and_accept(sample(yaw=-math.pi + 0.01), 1.05).accepted


def test_rejected_frame_never_becomes_the_next_reference():
    check = validator()
    assert check.check_and_accept(sample(), 1.0).accepted
    assert not check.check_and_accept(sample(x=50.0), 1.05).accepted
    assert check.check_and_accept(sample(x=0.02), 1.10).accepted


def test_link_delay_does_not_invent_a_fixed_distance_threshold():
    check = validator()
    assert check.check_and_accept(sample(), 1.0).accepted
    # Two seconds at the enforced 0.30 m/s bound plus one sample period.
    assert check.check_and_accept(sample(x=0.60), 3.0).accepted
    assert not check.check_and_accept(sample(x=0.70), 3.1).accepted


def test_epoch_reset_is_explicit_not_a_side_effect_of_rejection():
    check = validator()
    assert check.check_and_accept(sample(x=10.0), 1.0).accepted
    assert not check.check_and_accept(sample(x=0.0), 1.05).accepted
    check.reset_epoch()
    assert check.check_and_accept(sample(x=0.0), 1.10).accepted


@pytest.mark.parametrize(
    'kwargs',
    [
        {'max_linear_speed': 0.0, 'max_angular_speed': 1.2, 'expected_period': 0.05},
        {'max_linear_speed': 0.3, 'max_angular_speed': math.nan, 'expected_period': 0.05},
        {'max_linear_speed': 0.3, 'max_angular_speed': 1.2, 'expected_period': -0.05},
    ],
)
def test_invalid_validator_configuration_fails_closed(kwargs):
    with pytest.raises(ValueError):
        OdomValidator(**kwargs)


def test_receipt_time_regression_does_not_change_reference():
    check = validator()
    assert check.check_and_accept(sample(), 10.0).accepted
    rejected = check.check_and_accept(sample(x=0.01), 9.9)
    assert rejected.reason == 'receipt-time-regressed'
    assert check.check_and_accept(sample(x=0.02), 10.05).accepted


def test_queue_delay_must_not_be_used_as_motion_time():
    check = validator()
    assert check.check_and_accept(sample(), 10.0).accepted
    # Parsing this frame at 12.0 would falsely admit 0.6 m of motion.
    # Its socket receipt at 10.05 only admits 0.03 m.
    rejected = check.check_and_accept(sample(x=0.6), 10.05)
    assert rejected.reason == 'position-discontinuity'
