"""Pure PatrolBot battery and FAULTFLAGS interpretation helpers."""

import math


BATTERY_RECHARGE_VOLTS = 24.0
BATTERY_MIN_OPERATING_VOLTS = 23.0

# Research PatrolBot 3rd Generation User Guide Rev. A, pages 60-61.
FAULT_FLAG_NAMES = {
    0: 'pdb_laser_status_read_failure',
    1: 'high_temperature',
    2: 'general_pdb_failure',
    3: 'low_battery',
    4: 'gyro_fault',
    5: 'unused_bit_5',
    6: 'unused_bit_6',
    7: 'encoder_degraded_warning',
    8: 'encoder_failure',
    9: 'critical_driving_fault',
    15: 'jodrive_safe_mode',
}


def decode_fault_flags(value):
    """Return stable names for every asserted 16-bit FAULTFLAGS bit."""
    value = int(value) & 0xFFFF
    active = []
    for bit in range(16):
        if value & (1 << bit):
            active.append(FAULT_FLAG_NAMES.get(bit, f'reserved_bit_{bit}'))
    return active


def battery_diagnostic(voltage, charge_state):
    """Return diagnostic level (OK/WARN/ERROR as 0/1/2) and message."""
    if not math.isfinite(voltage) or voltage <= 0.0:
        return 2, 'battery voltage unavailable'
    if voltage < BATTERY_MIN_OPERATING_VOLTS:
        return 2, 'battery below 23.0 V operating minimum'
    if voltage < BATTERY_RECHARGE_VOLTS:
        return 1, 'battery below 24.0 V recharge threshold'
    if int(charge_state) > 0:
        return 0, 'charging'
    return 0, 'battery voltage normal'
