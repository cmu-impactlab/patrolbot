"""Pure fail-closed policy used by the PatrolBot priority-10 watchdog."""

import math


def _missing_or_stale(now, received_at, timeout):
    return received_at is None or (now - received_at) > timeout


def evaluate_safety(
        *, now, scan_at, odom_at, battery_at, battery_voltage,
        base_state_at, hardware_state_valid, fault_flags, estop_pressed,
        bumpers_enabled, bumpers_valid, front_bumper_pressed,
        rear_bumper_pressed, desired_linear, desired_angular,
        scan_timeout=0.5, odom_timeout=0.5, battery_timeout=1.0,
        base_state_timeout=0.75, battery_stop_voltage=23.0,
        stop_on_faults=True, stop_on_estop=True):
    """Return ordered reasons requiring a priority-10 zero-velocity hold."""
    reasons = []
    if _missing_or_stale(now, scan_at, scan_timeout):
        reasons.append('scan_stale')
    if _missing_or_stale(now, odom_at, odom_timeout):
        reasons.append('odom_stale')
    if _missing_or_stale(now, battery_at, battery_timeout):
        reasons.append('battery_stale')
    elif (not math.isfinite(battery_voltage) or
          battery_voltage < battery_stop_voltage):
        reasons.append('battery_below_operating_minimum')
    if _missing_or_stale(now, base_state_at, base_state_timeout):
        reasons.append('base_state_stale')
    elif not hardware_state_valid:
        reasons.append('hardware_state_invalid')
    if stop_on_faults and fault_flags:
        reasons.append(f'hardware_fault_0x{int(fault_flags) & 0xFFFF:04x}')
    if stop_on_estop and estop_pressed:
        reasons.append('estop_asserted')

    if bumpers_enabled:
        if not bumpers_valid:
            reasons.append('bumper_mapping_invalid')
        else:
            rotating = abs(desired_angular) > 0.01
            if front_bumper_pressed and (desired_linear > 0.005 or rotating):
                reasons.append('front_bumper_motion_block')
            if rear_bumper_pressed and (desired_linear < -0.005 or rotating):
                reasons.append('rear_bumper_motion_block')
    return reasons
