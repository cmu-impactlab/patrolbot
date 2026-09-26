"""Pure helpers for the PatrolBot joystick policy.

Kept free of ROS imports so the axis, deadzone, edge, and ramp behavior can be
unit-tested on a development host without a ROS installation.
"""


# Logitech F710 in XInput mode. Keep the motion and feedback nodes on one
# mapping so an alert cannot interpret a different button or direction from
# the teleop path it is describing.
BTN_DEADMAN = 5
AX_LIN = 1
AX_ANG = 0
AX_DPAD_LIN = 7
AX_DPAD_ANG = 6
LIN_SIGN = 1.0
ANG_SIGN = 1.0
DPAD_LIN_SIGN = -1.0
DPAD_ANG_SIGN = -1.0


def axis_value(axes, index):
    """Return a bounded controller axis, or zero when the axis is absent."""
    if index < 0 or index >= len(axes):
        return 0.0
    return max(-1.0, min(1.0, float(axes[index])))


def apply_deadzone(value, deadzone):
    """Suppress centered-stick noise without rescaling the remaining range."""
    value = float(value)
    return 0.0 if abs(value) < float(deadzone) else value


def button_pressed(buttons, index):
    """Treat a missing button as released."""
    return 0 <= index < len(buttons) and int(buttons[index]) == 1


def rising_edge(current, previous):
    """True once when a control changes from released to pressed."""
    return bool(current) and not bool(previous)


def ramp_toward(current, target, step):
    """Move current toward target by at most a non-negative step."""
    step = max(0.0, float(step))
    if target > current:
        return min(current + step, target)
    if target < current:
        return max(current - step, target)
    return current


def drive_intent(axes, buttons, deadzone):
    """Return ``(attempting, linear, angular)`` for an RB-held drive input."""
    if not button_pressed(buttons, BTN_DEADMAN):
        return False, 0.0, 0.0

    linear = apply_deadzone(axis_value(axes, AX_LIN), deadzone) * LIN_SIGN
    angular = apply_deadzone(axis_value(axes, AX_ANG), deadzone) * ANG_SIGN
    if linear == 0.0:
        linear = axis_value(axes, AX_DPAD_LIN) * DPAD_LIN_SIGN
    if angular == 0.0:
        angular = axis_value(axes, AX_DPAD_ANG) * DPAD_ANG_SIGN

    attempting = linear != 0.0 or angular != 0.0
    return attempting, linear, angular
