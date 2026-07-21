"""Unit tests for battery and hardware-fault interpretation."""

from patrolbot_bridge.safety_telemetry import (
    battery_diagnostic,
    decode_fault_flags,
)


def test_every_documented_fault_flag_has_a_stable_name():
    flags = sum(1 << bit for bit in (0, 1, 2, 3, 4, 7, 8, 9, 15))
    assert decode_fault_flags(flags) == [
        'pdb_laser_status_read_failure',
        'high_temperature',
        'general_pdb_failure',
        'low_battery',
        'gyro_fault',
        'encoder_degraded_warning',
        'encoder_failure',
        'critical_driving_fault',
        'jodrive_safe_mode',
    ]


def test_reserved_and_unused_bits_are_never_silently_discarded():
    assert decode_fault_flags((1 << 5) | (1 << 10)) == [
        'unused_bit_5', 'reserved_bit_10']


def test_battery_thresholds_match_the_patrolbot_manual():
    assert battery_diagnostic(24.0, 0) == (0, 'battery voltage normal')
    assert battery_diagnostic(23.999, 0)[0] == 1
    assert battery_diagnostic(23.0, 0)[0] == 1
    assert battery_diagnostic(22.999, 0)[0] == 2
    assert battery_diagnostic(29.0, 1) == (0, 'charging')


def test_invalid_battery_voltage_is_an_error():
    assert battery_diagnostic(float('nan'), 0)[0] == 2
    assert battery_diagnostic(0.0, 0)[0] == 2
