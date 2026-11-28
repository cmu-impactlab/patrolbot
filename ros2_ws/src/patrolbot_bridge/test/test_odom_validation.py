import math

from patrolbot_bridge.odom_validation import OdomSample, OdomSource, OdomValidator, parse_odom_source
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


def test_replay_shape_repeated_frames_do_not_shrink_the_pose_allowance():
    timed = validator()
    legacy = validator()
    # Reconstruct the 29.2 mm step / 44.1 ms receive interval from passive
    # replay. The capture did not have SIP time; these source times are a fixture.
    for received, stamp, age, x in [
        (10.0000, 1000, 5, 0.0),
        (10.0441, 1000, 49, 0.0),
        (10.0882, 1000, 93, 0.0),
        (10.1323, 1100, 38, 0.0292),
    ]:
        result = timed.check_and_accept(sample(x=x), received, source=OdomSource(stamp, age))
        assert result.accepted
        if stamp == 1000:
            assert result.fresh_at == pytest.approx(9.995)
            assert result.repeated == (received != 10.0)
        legacy_result = legacy.check_and_accept(sample(x=x), received)
    assert legacy_result.reason == 'position-discontinuity'
    assert not result.repeated


def test_new_stationary_sip_is_not_a_repeat_and_burst_cannot_mint_motion_time():
    check = validator()
    assert check.check_and_accept(sample(), 10.0, source=OdomSource(1000, 0)).accepted
    # Two queued frames can have the same receipt time; a truly new stationary
    # update remains valid but can't add any Pi elapsed time or freshness.
    result = check.check_and_accept(sample(), 10.0, source=OdomSource(1100, 0))
    assert result.accepted and not result.repeated and result.fresh_at == 10.0
    bad = check.check_and_accept(sample(x=0.22), 10.01, source=OdomSource(2000, 0))
    assert bad.reason == 'position-discontinuity'
    assert 'limit_m=0.018' in bad.detail
    assert 'source_elapsed_s=0.9' in bad.detail


def test_transport_delay_does_not_mint_motion_time_either():
    check = validator()
    assert check.check_and_accept(sample(), 10.0, source=OdomSource(1000, 0)).accepted
    bad = check.check_and_accept(sample(x=.20), 10.8, source=OdomSource(1020, 0))
    assert bad.reason == 'position-discontinuity'
    assert 'limit_m=0.021' in bad.detail


def test_wrapped_heading_is_valid_but_a_real_jump_keeps_bounded_evidence():
    check = validator()
    yaw = math.pi - .01
    assert check.check_and_accept(sample(yaw=yaw), 10, source=OdomSource(1000, 0)).accepted
    yaw = -math.pi + .01
    assert check.check_and_accept(sample(yaw=yaw), 10.1, source=OdomSource(1100, 0)).accepted
    bad = check.check_and_accept(sample(yaw=yaw + 1), 10.2, source=OdomSource(1200, 0))
    assert bad.reason == 'heading-discontinuity'
    assert 'step_rad=1' in bad.detail and 'limit_rad=0.18' in bad.detail
    good = check.check_and_accept(sample(yaw=yaw), 10.3, source=OdomSource(1300, 0))
    assert good.accepted


@pytest.mark.parametrize('s, reason', [
    (sample(linear=.33), 'linear-speed-limit'),
    (sample(angular=-1.21), 'angular-speed-limit'),
])
def test_source_timing_never_waives_twist_limits(s, reason):
    check = validator()
    assert check.check_and_accept(sample(), 10, source=OdomSource(1000, 0)).accepted
    assert check.check_and_accept(s, 10.1, source=OdomSource(1100, 0)).reason == reason
    # Repeating the invalid stamp with a different payload can't rehabilitate it.
    result = check.check_and_accept(sample(), 10.2, source=OdomSource(1100, 100))
    assert result.reason == 'conflicting-odometry-source'


@pytest.mark.parametrize('parts,reason', [
    ([], 'missing-odometry-source'),
    (['ODOM_SOURCE_MS:1,0', 'ODOM_SOURCE_MS:1,0'], 'malformed-odometry-source'),
    (['ODOM_SOURCE_MS=1,0'], 'malformed-odometry-source'),
    (['ODOM_SOURCE_MS:-1,-1'], 'unavailable-odometry-source'),
    (['ODOM_SOURCE_MS:1,-1'], 'malformed-odometry-source'),
    (['ODOM_SOURCE_MS:1,nan'], 'malformed-odometry-source'),
    (['ODOM_SOURCE_MS:1,0,0'], 'malformed-odometry-source'),
    (['ODOM_SOURCE_MS:' + '9' * 4096 + ',0'], 'malformed-odometry-source'),
])
def test_advertised_source_metadata_is_required_and_strict(parts, reason):
    with pytest.raises(ValueError, match=reason):
        parse_odom_source(parts, required=True)


def test_legacy_metadata_is_explicit_and_cannot_switch_clock_mid_epoch():
    assert parse_odom_source([], required=False) is None
    parts = ['ODOM_SOURCE_MS:12340,12']
    with pytest.raises(ValueError, match='unadvertised-odometry-source'):
        parse_odom_source(parts, required=False)
    source = parse_odom_source(parts, required=True)
    check = validator()
    assert check.check_and_accept(sample(), 10, source=source).accepted
    assert check.check_and_accept(sample(), 10.1).reason == 'odometry-source-mode-changed'
    assert not check.check_and_accept(sample(), 10.2, source=OdomSource(12, 0)).accepted
    check.reset_epoch()
    assert check.check_and_accept(sample(x=50), 10.2, source=OdomSource(12, 0)).accepted


def test_source_and_assembly_regression_and_stale_age_fail_closed():
    check = validator()
    assert check.check_and_accept(sample(), 10, source=OdomSource(1000, 80)).accepted
    for stamp, age in [(990, 150), (1000, 0), (1050, 0)]:
        result = check.check_and_accept(sample(), 10.01, source=OdomSource(stamp, age))
        assert result.reason == 'odometry-source-time-regressed'
    result = check.check_and_accept(sample(), 10.1, source=OdomSource(1100, 400), queue_age=.2)
    assert result.reason == 'stale-or-future-odometry-source'
    result = check.check_and_accept(sample(), 10.5, source=OdomSource(1100, 400))
    assert result.reason == 'unaccepted-odometry-source'
    # A distinct, locally fresh observation with forward assembly time recovers.
    assert check.check_and_accept(sample(), 10.6, source=OdomSource(1501, 0)).accepted


def test_repeated_pose_expires_even_if_repeated_reported_age_remains_young():
    check = validator()
    assert check.check_and_accept(sample(), 10, source=OdomSource(1000, 10)).accepted
    gentle = check.check_and_accept(sample(), 10.1, source=OdomSource(1000, 10))
    assert gentle.repeated and gentle.fresh_at == pytest.approx(9.99)
    result = check.check_and_accept(sample(), 10.49, source=OdomSource(1000, 10), queue_age=.001)
    assert result.reason == 'stale-repeated-odometry-source'


def test_locally_older_source_projection_cannot_refresh_pose():
    check = validator()
    assert check.check_and_accept(sample(), 10, source=OdomSource(1000, 0)).accepted
    result = check.check_and_accept(sample(), 10.05, source=OdomSource(1050, 60))
    assert result.reason == 'odometry-source-freshness-regressed'
