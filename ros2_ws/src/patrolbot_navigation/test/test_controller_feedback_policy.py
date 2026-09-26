"""Pure tests for controller blocker selection and finite pulse timing."""

from pathlib import Path
import sys


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from controller_feedback_policy import (  # noqa: E402
    PUBLISH_ZERO,
    PULSE_EVENT,
    PULSE_RETRY,
    PulseScheduler,
    feedback_blockers,
    joystick_drive_intent,
)


def safe_state(**overrides):
    state = {
        'now': 10.0,
        'scan_at': 9.9,
        'odom_at': 9.9,
        'battery_at': 9.9,
        'battery_voltage': 24.0,
        'base_state_at': 9.9,
        'hardware_state_valid': True,
        'fault_flags': 0,
        'estop_pressed': False,
        'bumpers_valid': True,
        'front_bumper_pressed': False,
        'rear_bumper_pressed': False,
        'desired_linear': 0.0,
        'desired_angular': 0.0,
        'motors_enabled': True,
        'dock_state_valid': True,
        'dock_state': 'CLEAR_CONFIRMED',
        'drive_attempt': False,
    }
    state.update(overrides)
    return state


def scheduler():
    return PulseScheduler(
        started_at=0.0,
        event_pulse=0.6,
        retry_pulse=0.35,
        retry_interval=1.5,
        startup_grace=2.0,
    )


def test_intentionally_docked_motor_off_is_not_an_error_until_drive_attempt():
    docked = safe_state(
        motors_enabled=False,
        dock_state='DOCKED_CONFIRMED',
    )
    assert feedback_blockers(**docked) == []
    reasons = feedback_blockers(**{
        **docked,
        'drive_attempt': True,
        'desired_linear': 0.1,
    })
    assert reasons == ['dock_state_DOCKED_CONFIRMED_blocks_drive']


def test_clear_robot_with_motors_disabled_is_always_a_blocker():
    assert feedback_blockers(**safe_state(motors_enabled=False)) == [
        'motors_disabled_while_clear']


def test_every_selected_telemetry_and_hardware_blocker_is_reported():
    cases = (
        ({'scan_at': None}, 'scan_stale'),
        ({'odom_at': None}, 'odom_stale'),
        ({'battery_at': None}, 'battery_stale'),
        ({'battery_voltage': 22.999},
         'battery_below_operating_minimum'),
        ({'base_state_at': None}, 'base_state_stale'),
        ({'hardware_state_valid': False}, 'hardware_state_invalid'),
        ({'fault_flags': 0x0108}, 'hardware_fault_0x0108'),
        ({'estop_pressed': True}, 'estop_asserted'),
    )
    for changes, expected in cases:
        assert expected in feedback_blockers(**safe_state(**changes))


def test_direction_aware_bumpers_only_report_motion_toward_contact():
    front = {
        'front_bumper_pressed': True,
        'desired_linear': 0.1,
    }
    assert 'front_bumper_motion_block' in feedback_blockers(
        **safe_state(**front))
    assert 'front_bumper_motion_block' not in feedback_blockers(
        **safe_state(front_bumper_pressed=True, desired_linear=-0.1))
    assert 'rear_bumper_motion_block' in feedback_blockers(**safe_state(
        rear_bumper_pressed=True, desired_linear=-0.1))
    assert 'rear_bumper_motion_block' not in feedback_blockers(**safe_state(
        rear_bumper_pressed=True, desired_linear=0.1))
    assert 'front_bumper_motion_block' in feedback_blockers(**safe_state(
        front_bumper_pressed=True, desired_angular=0.1))


def test_invalid_bumper_mapping_is_not_presented_as_physical_contact():
    assert feedback_blockers(**safe_state(bumpers_valid=False)) == []


def test_every_nonclear_or_invalid_dock_state_blocks_a_drive_attempt():
    for dock_state in (
            'UNKNOWN', 'DOCKED_CONFIRMED', 'DEPARTING', 'FAULT_STOPPED'):
        reasons = feedback_blockers(**safe_state(
            dock_state=dock_state,
            drive_attempt=True,
            desired_angular=0.1,
        ))
        assert f'dock_state_{dock_state}_blocks_drive' in reasons
    reasons = feedback_blockers(**safe_state(
        dock_state_valid=False,
        drive_attempt=True,
        desired_linear=0.1,
    ))
    assert reasons == ['dock_state_invalid_blocks_drive']


def test_fresh_rb_plus_stick_or_dpad_is_drive_intent():
    buttons = [0] * 6
    buttons[5] = 1
    assert joystick_drive_intent(
        now=10.0, joy_at=9.9, axes=[0.2, 0.3], buttons=buttons,
        timeout=0.4, deadzone=0.12) == (True, 0.3, 0.2)

    axes = [0.0] * 8
    axes[7] = -1.0
    assert joystick_drive_intent(
        now=10.0, joy_at=9.9, axes=axes, buttons=buttons,
        timeout=0.4, deadzone=0.12) == (True, 1.0, 0.0)


def test_released_rb_centered_controls_and_stale_joy_are_not_attempts():
    centered = [0.0] * 8
    buttons = [0] * 6
    assert joystick_drive_intent(
        now=10.0, joy_at=9.9, axes=centered, buttons=buttons,
        timeout=0.4, deadzone=0.12)[0] is False
    buttons[5] = 1
    assert joystick_drive_intent(
        now=10.0, joy_at=9.9, axes=centered, buttons=buttons,
        timeout=0.4, deadzone=0.12)[0] is False
    centered[1] = 1.0
    assert joystick_drive_intent(
        now=10.0, joy_at=9.5, axes=centered, buttons=buttons,
        timeout=0.4, deadzone=0.12) == (False, 0.0, 0.0)


def test_startup_blocker_is_suppressed_then_gets_one_event_pulse():
    policy = scheduler()
    assert policy.update(
        now=0.0, reasons=['scan_stale'], drive_attempt=False) == []
    assert policy.update(
        now=1.99, reasons=['scan_stale'], drive_attempt=False) == []
    assert policy.update(
        now=2.0, reasons=['scan_stale'], drive_attempt=False) == [
        (PULSE_EVENT, 0.6)]
    assert policy.update(
        now=2.1, reasons=['scan_stale'], drive_attempt=False) == []
    assert policy.update(
        now=2.6, reasons=['scan_stale'], drive_attempt=False) == [
        (PUBLISH_ZERO, 0.0)]
    assert policy.update(
        now=5.0, reasons=['scan_stale'], drive_attempt=False) == []


def test_blocker_appearing_after_grace_gets_immediate_event_pulse():
    policy = scheduler()
    assert policy.update(now=2.5, reasons=[], drive_attempt=False) == []
    assert policy.update(
        now=3.0, reasons=['estop_asserted'], drive_attempt=False) == [
        (PULSE_EVENT, 0.6)]


def test_held_drive_attempt_retries_no_more_than_every_1_5_seconds():
    policy = scheduler()
    reasons = ['motors_disabled_while_clear']
    assert policy.update(
        now=2.0, reasons=reasons, drive_attempt=True) == [
        (PULSE_EVENT, 0.6)]
    assert policy.update(
        now=2.6, reasons=reasons, drive_attempt=True) == [
        (PUBLISH_ZERO, 0.0)]
    assert policy.update(
        now=3.49, reasons=reasons, drive_attempt=True) == []
    assert policy.update(
        now=3.5, reasons=reasons, drive_attempt=True) == [
        (PULSE_RETRY, 0.35)]
    assert policy.update(
        now=3.85, reasons=reasons, drive_attempt=True) == [
        (PUBLISH_ZERO, 0.0)]
    assert policy.update(
        now=4.99, reasons=reasons, drive_attempt=True) == []
    assert policy.update(
        now=5.0, reasons=reasons, drive_attempt=True) == [
        (PULSE_RETRY, 0.35)]


def test_recovery_sends_zero_and_rearms_the_next_blocker_episode():
    policy = scheduler()
    assert policy.update(
        now=3.0, reasons=['battery_stale'], drive_attempt=False) == [
        (PULSE_EVENT, 0.6)]
    assert policy.update(now=3.1, reasons=[], drive_attempt=False) == [
        (PUBLISH_ZERO, 0.0)]
    assert policy.update(
        now=3.2, reasons=['odom_stale'], drive_attempt=False) == [
        (PULSE_EVENT, 0.6)]


def test_multiple_simultaneous_reasons_still_emit_one_event():
    policy = scheduler()
    reasons = ['scan_stale', 'odom_stale', 'base_state_stale']
    assert policy.update(
        now=3.0, reasons=reasons, drive_attempt=False) == [
        (PULSE_EVENT, 0.6)]
    assert policy.update(
        now=3.1, reasons=reasons + ['battery_stale'],
        drive_attempt=False) == []
