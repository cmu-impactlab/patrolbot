"""Pure PatrolBot battery and FAULTFLAGS interpretation helpers."""

import math

BATTERY_RECHARGE_VOLTS = 24.0
BATTERY_MIN_OPERATING_VOLTS = 23.0

# sensor_msgs/BatteryState power_supply_status values. Kept here as integers
# so this pure helper remains unit-testable without importing ROS messages.
BATTERY_STATUS_UNKNOWN = 0
BATTERY_STATUS_CHARGING = 1
BATTERY_STATUS_DISCHARGING = 2
BATTERY_STATUS_NOT_CHARGING = 3

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


def battery_supply_status(charge_state, dock_state, dock_state_valid):
    """
    Interpret raw charge only through the fresh SBC dock observer.

    The µARCS charge state can re-latch to float after physical clearance.
    Therefore a positive raw value is charging only in DOCKED_CONFIRMED.
    """
    if not dock_state_valid:
        return BATTERY_STATUS_UNKNOWN
    state = str(dock_state).strip().upper()
    if state == 'DOCKED_CONFIRMED':
        return BATTERY_STATUS_CHARGING if int(charge_state) > 0 else BATTERY_STATUS_NOT_CHARGING
    if state in ('DEPARTING', 'CLEAR_CONFIRMED'):
        return BATTERY_STATUS_DISCHARGING
    return BATTERY_STATUS_UNKNOWN


def battery_diagnostic(voltage, charge_state, dock_state=None, dock_state_valid=False):
    """Return diagnostic level (OK/WARN/ERROR as 0/1/2) and message."""
    if not math.isfinite(voltage) or voltage <= 0.0:
        return 2, 'battery voltage unavailable'
    if voltage < BATTERY_MIN_OPERATING_VOLTS:
        return 2, 'battery below 23.0 V operating minimum'
    if voltage < BATTERY_RECHARGE_VOLTS:
        return 1, 'battery below 24.0 V recharge threshold'
    charging = (
        battery_supply_status(charge_state, dock_state, dock_state_valid)
        == BATTERY_STATUS_CHARGING
        if dock_state is not None
        else int(charge_state) > 0
    )
    if charging:
        return 0, 'charging'
    return 0, 'battery voltage normal'
