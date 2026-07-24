"""Pure helpers for the PatrolBot joystick policy.

Kept free of ROS imports so the axis, deadzone, edge, and ramp behavior can be
unit-tested on a development host without a ROS installation.
"""


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
