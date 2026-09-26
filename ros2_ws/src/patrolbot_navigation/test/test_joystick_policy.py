"""Unit tests for joystick behavior that does not require a ROS graph."""

import sys
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from joystick_policy import (  # noqa: E402
    apply_deadzone,
    axis_value,
    button_pressed,
    drive_intent,
    ramp_toward,
    rising_edge,
)


def test_axis_value_is_bounded_and_missing_axes_are_zero():
    assert axis_value([2.0, -2.0], 0) == 1.0
    assert axis_value([2.0, -2.0], 1) == -1.0
    assert axis_value([0.25], 4) == 0.0


def test_deadzone_suppresses_center_noise_without_rescaling():
    assert apply_deadzone(0.119, 0.12) == 0.0
    assert apply_deadzone(-0.119, 0.12) == 0.0
    assert apply_deadzone(0.12, 0.12) == 0.12


def test_button_and_rising_edge_are_fail_closed():
    assert button_pressed([0, 1], 1)
    assert not button_pressed([0, 1], 4)
    assert rising_edge(True, False)
    assert not rising_edge(True, True)
    assert not rising_edge(False, True)


def test_ramp_toward_never_overshoots():
    assert ramp_toward(0.0, 0.35, 0.1) == 0.1
    assert ramp_toward(0.3, 0.35, 0.1) == 0.35
    assert ramp_toward(0.0, -0.35, 0.1) == -0.1
    assert ramp_toward(-0.3, -0.35, 0.1) == -0.35
    assert ramp_toward(0.2, 0.0, 0.3) == 0.0


def test_shared_f710_drive_intent_requires_rb_and_nonzero_direction():
    buttons = [0] * 6
    axes = [0.0] * 8
    axes[1] = 0.5
    assert drive_intent(axes, buttons, 0.12) == (False, 0.0, 0.0)
    buttons[5] = 1
    assert drive_intent(axes, buttons, 0.12) == (True, 0.5, 0.0)
