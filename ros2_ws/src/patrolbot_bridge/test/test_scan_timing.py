from patrolbot_bridge.scan_timing import (
    LaserAgeResult,
    compute_scan_stamp_ns,
    parse_laser_age,
)


def test_absent_age_is_explicit_legacy_result():
    result = parse_laser_age('ODOM:0,0,0,0,0|LASER:1,2,3')
    assert result == LaserAgeResult(True, True, None, 'legacy-no-age')


def test_valid_age_is_accepted_from_full_telemetry_sections():
    result = parse_laser_age(
        'ODOM:0,0,0,0,0|LASER:1,2,3|LASER_AGE_MS:120')
    assert result == LaserAgeResult(True, False, 120, 'age-valid')


def test_age_boundaries_are_valid():
    assert parse_laser_age('|LASER_AGE_MS:0').age_ms == 0
    assert parse_laser_age('|LASER_AGE_MS:500').age_ms == 500


def test_malformed_values_are_rejected_without_legacy_fallback():
    for value in ('', '-1', '+1', '1.0', ' 1', '1 ', 'nan', '501'):
        result = parse_laser_age('|LASER_AGE_MS:' + value)
        assert not result.accepted
        assert not result.legacy
        assert result.age_ms is None


def test_duplicate_age_fields_are_rejected():
    result = parse_laser_age('|LASER_AGE_MS:10|LASER_AGE_MS:20')
    assert not result.accepted
    assert result.reason == 'duplicate-age-field'


def test_wrong_age_separator_is_rejected():
    result = parse_laser_age('|LASER_AGE_MS=10')
    assert not result.accepted
    assert result.reason == 'malformed-age-field'


def test_corrected_stamp_subtracts_elapsed_age():
    assert compute_scan_stamp_ns(2_000_000_000, 120_000_000) == 1_880_000_000


def test_corrected_stamp_rejects_negative_age_and_result():
    import pytest

    with pytest.raises(ValueError):
        compute_scan_stamp_ns(2_000, -1)
    with pytest.raises(ValueError):
        compute_scan_stamp_ns(2_000, 2_001)
