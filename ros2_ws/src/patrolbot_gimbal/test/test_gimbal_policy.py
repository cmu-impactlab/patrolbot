"""Gimbal policy tests: stick-to-rate mapping, limits, and the deadman.

Pure logic, no ROS and no hardware, mirroring test_joystick_policy.py in
patrolbot_navigation.

The deadman and joy-loss cases matter most: they are the behaviour that stops
the camera when the operator lets go or the controller disappears.
"""

import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from patrolbot_gimbal import gimbal_policy as gp  # noqa: E402


# ------------------------------------------------------------- axis reading --

def test_axis_value_is_bounded_and_missing_axes_are_zero():
    assert gp.axis_value([2.0, -2.0], 0) == 1.0
    assert gp.axis_value([2.0, -2.0], 1) == -1.0
    assert gp.axis_value([0.25], 7) == 0.0
    assert gp.axis_value([], 0) == 0.0


def test_button_is_fail_closed_when_absent():
    assert gp.button_pressed([0, 0, 0, 0, 1], gp.BTN_DEADMAN)
    assert not gp.button_pressed([0, 0], gp.BTN_DEADMAN)
    assert not gp.button_pressed([], gp.BTN_DEADMAN)


# ---------------------------------------------------------------- deadzone --

def test_deadzone_suppresses_noise_and_rescales_to_full_range():
    assert gp.apply_deadzone(0.10, 0.12) == 0.0
    assert gp.apply_deadzone(-0.10, 0.12) == 0.0
    # Full deflection must still reach 1.0 after rescaling, or the operator
    # loses top speed for no reason.
    assert gp.apply_deadzone(1.0, 0.12) == pytest.approx(1.0)
    assert gp.apply_deadzone(-1.0, 0.12) == pytest.approx(-1.0)


def test_deadzone_boundary_is_inclusive_of_zero():
    assert gp.apply_deadzone(0.12, 0.12) == 0.0


def test_deadzone_of_zero_is_a_passthrough():
    assert gp.apply_deadzone(0.5, 0.0) == pytest.approx(0.5)


# --------------------------------------------------------- stick to rate --

def test_stick_maps_to_rate_scaled_by_max_speed():
    assert gp.stick_to_rate(1.0, 45.0, invert=False) == pytest.approx(45.0)
    assert gp.stick_to_rate(0.5, 45.0, invert=False) == pytest.approx(22.5)
    assert gp.stick_to_rate(0.0, 45.0, invert=False) == 0.0


def test_inversion_flips_the_rate():
    assert gp.stick_to_rate(1.0, 45.0, invert=True) == pytest.approx(-45.0)


def test_triggers_combine_into_a_signed_roll_rate():
    """RT rolls clockwise, LT counter-clockwise; both held cancels out."""
    assert gp.triggers_to_roll(lt=0.0, rt=1.0, max_deg_s=40.0) == pytest.approx(40.0)
    assert gp.triggers_to_roll(lt=1.0, rt=0.0, max_deg_s=40.0) == pytest.approx(-40.0)
    assert gp.triggers_to_roll(lt=1.0, rt=1.0, max_deg_s=40.0) == 0.0
    assert gp.triggers_to_roll(lt=0.0, rt=0.0, max_deg_s=40.0) == 0.0


# ------------------------------------------------------------------ limits --

def test_clamp_reports_whether_it_clamped():
    assert gp.clamp(5.0, -10.0, 10.0) == (5.0, False)
    assert gp.clamp(15.0, -10.0, 10.0) == (10.0, True)
    assert gp.clamp(-15.0, -10.0, 10.0) == (-10.0, True)


def test_clamp_tolerates_inverted_limits():
    """min > max is a config error; behave predictably instead of crashing."""
    value, _ = gp.clamp(0.0, 10.0, -10.0)
    assert -10.0 <= value <= 10.0


def test_rate_is_gated_at_a_limit_only_in_the_offending_direction():
    """At the upper limit you may still come back down."""
    assert gp.gate_rate(position=10.0, rate=5.0, lo=-10.0, hi=10.0) == 0.0
    assert gp.gate_rate(position=10.0, rate=-5.0, lo=-10.0, hi=10.0) == -5.0
    assert gp.gate_rate(position=0.0, rate=5.0, lo=-10.0, hi=10.0) == 5.0


def test_rate_is_gated_at_the_lower_limit_too():
    assert gp.gate_rate(position=-10.0, rate=-5.0, lo=-10.0, hi=10.0) == 0.0
    assert gp.gate_rate(position=-10.0, rate=5.0, lo=-10.0, hi=10.0) == 5.0


# ----------------------------------------------------------------- deadman --

def test_command_is_invalid_without_the_deadman():
    buttons = [0] * 8                       # LB not held
    cmd = gp.joy_to_command(
        axes=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        buttons=buttons, cfg=gp.default_config(), joy_age_s=0.0)
    assert not cmd.valid
    assert cmd.pan == 0.0 and cmd.tilt == 0.0 and cmd.roll == 0.0


def test_full_deflection_with_deadman_produces_rate():
    buttons = [0] * 8
    buttons[gp.BTN_DEADMAN] = 1
    cfg = gp.default_config()
    cmd = gp.joy_to_command(
        axes=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        buttons=buttons, cfg=cfg, joy_age_s=0.0)
    assert cmd.valid
    assert cmd.pan == pytest.approx(cfg.max_pan_deg_s)


def test_stale_joy_drops_the_deadman_even_while_held():
    """An unplugged receiver must not leave the last command running."""
    buttons = [0] * 8
    buttons[gp.BTN_DEADMAN] = 1
    cmd = gp.joy_to_command(
        axes=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        buttons=buttons, cfg=gp.default_config(),
        joy_age_s=gp.JOY_TIMEOUT_S + 0.01)
    assert not cmd.valid
    assert cmd.pan == 0.0


def test_joy_age_exactly_at_timeout_is_treated_as_stale():
    buttons = [0] * 8
    buttons[gp.BTN_DEADMAN] = 1
    cmd = gp.joy_to_command(
        axes=[0.0] * 8, buttons=buttons, cfg=gp.default_config(),
        joy_age_s=gp.JOY_TIMEOUT_S)
    assert not cmd.valid


def test_empty_joy_message_does_not_crash_and_is_invalid():
    cmd = gp.joy_to_command(axes=[], buttons=[], cfg=gp.default_config(),
                            joy_age_s=0.0)
    assert not cmd.valid
    assert cmd.pan == 0.0 and cmd.tilt == 0.0 and cmd.roll == 0.0


# ---------------------------------------------------- controller isolation --

def test_gimbal_inputs_do_not_overlap_the_drive_teleop():
    """Guards against silently stealing an axis the drive teleop relies on.

    The authoritative cross-check against the live drive teleop source is in
    test_isolation_contract.py; this asserts the intent locally.
    """
    drive_axes = {0, 1, 6, 7}
    drive_buttons = {0, 1, 2, 3, 5}
    gimbal_axes = {gp.AX_PAN, gp.AX_TILT, gp.AX_ROLL_CCW, gp.AX_ROLL_CW}
    gimbal_buttons = {gp.BTN_DEADMAN, gp.BTN_HOME}

    assert not (gimbal_axes & drive_axes)
    assert not (gimbal_buttons & drive_buttons)


def test_gimbal_deadman_is_not_the_drive_deadman():
    """LB for looking, RB for driving — they must never be the same button."""
    assert gp.BTN_DEADMAN != 5
