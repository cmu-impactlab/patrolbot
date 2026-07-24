"""Pure helpers for the PatrolBot gimbal joystick policy.

Kept free of ROS imports so the axis, deadzone, limit, and deadman behaviour
can be unit-tested on a development host without a ROS installation. Mirrors
patrolbot_navigation's joystick_policy.py, deliberately: the drive teleop's
safety behaviour is already commissioned, and the camera should fail the same
way for the same reasons.

Controller input indices are declared here rather than in the node so the
isolation contract test can parse them and prove they never collide with the
drive teleop's.
"""

from dataclasses import dataclass

# ---- Axis map (Logitech F710, XInput) ----
# The drive teleop owns axes 0, 1, 6, 7 (left stick and D-pad). These four are
# what it leaves free.
AX_ROLL_CCW = 2         # LT
AX_PAN = 3              # right stick X
AX_TILT = 4             # right stick Y
AX_ROLL_CW = 5          # RT

# ---- Button map ----
# The drive teleop owns buttons 0, 1, 2, 3 (A/B/X/Y) and 5 (RB deadman).
BTN_DEADMAN = 4         # LB — hold to move the camera
BTN_HOME = 6            # Back — recenter

# If /joy stops arriving for this long the deadman is dropped, whatever the
# last message said. Matches the drive teleop: joy_node autorepeats at ~20 Hz,
# so 0.4 s is several missed messages, not a transient.
JOY_TIMEOUT_S = 0.4


@dataclass
class GimbalConfig:
    """Rate limits, travel limits, and inversion for one mount."""

    max_pan_deg_s: float = 45.0
    max_tilt_deg_s: float = 45.0
    max_roll_deg_s: float = 30.0

    invert_pan: bool = False
    invert_tilt: bool = False
    invert_roll: bool = False

    pan_min_deg: float = -170.0
    pan_max_deg: float = 170.0
    tilt_min_deg: float = -90.0
    tilt_max_deg: float = 40.0
    roll_min_deg: float = -45.0
    roll_max_deg: float = 45.0

    deadzone: float = 0.12


@dataclass
class GimbalRates:
    """A rate command in degrees/second, plus whether it may be acted on."""

    pan: float = 0.0
    tilt: float = 0.0
    roll: float = 0.0
    valid: bool = False


def default_config() -> GimbalConfig:
    """A fresh config; never share one instance between callers."""
    return GimbalConfig()


# ------------------------------------------------------------ raw readings --

def axis_value(axes, index):
    """Return a bounded controller axis, or zero when the axis is absent."""
    if index < 0 or index >= len(axes):
        return 0.0
    return max(-1.0, min(1.0, float(axes[index])))


def trigger_value(axes, index):
    """Return a trigger as 0..1.

    joy_node reports F710 triggers as +1 released through -1 fully pressed, so
    an absent axis must read as released, not half-pressed.
    """
    if index < 0 or index >= len(axes):
        return 0.0
    raw = max(-1.0, min(1.0, float(axes[index])))
    return (1.0 - raw) / 2.0


def button_pressed(buttons, index):
    """Treat a missing button as released."""
    return 0 <= index < len(buttons) and int(buttons[index]) == 1


def rising_edge(current, previous):
    """True once when a control changes from released to pressed."""
    return bool(current) and not bool(previous)


# ---------------------------------------------------------------- shaping --

def apply_deadzone(value, deadzone):
    """Suppress centre noise, rescaling so full deflection still reaches 1.0.

    Rescaling matters: without it the operator loses the top of the range in
    proportion to the deadzone, which feels like the camera is slow rather
    than like the stick is filtered.
    """
    value = float(value)
    deadzone = float(deadzone)
    if deadzone <= 0.0:
        return value

    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0

    scaled = (magnitude - deadzone) / (1.0 - deadzone)
    return scaled if value > 0 else -scaled


def stick_to_rate(value, max_deg_s, invert=False):
    """Scale a normalised stick reading to a rate in degrees/second."""
    rate = float(value) * float(max_deg_s)
    return -rate if invert else rate


def triggers_to_roll(lt, rt, max_deg_s, invert=False):
    """Combine both triggers into one signed roll rate.

    RT rolls clockwise, LT counter-clockwise. Holding both cancels, which is
    the intuitive result and avoids one trigger silently winning.
    """
    rate = (float(rt) - float(lt)) * float(max_deg_s)
    return -rate if invert else rate


# ----------------------------------------------------------------- limits --

def clamp(value, low, high):
    """Clamp to [low, high], reporting whether clamping occurred.

    Inverted limits are a configuration error; swap them rather than returning
    something arbitrary.
    """
    low, high = float(low), float(high)
    if low > high:
        low, high = high, low
    clamped = max(low, min(high, float(value)))
    return clamped, clamped != float(value)


def gate_rate(position, rate, lo, hi):
    """Zero a rate that would drive further past a limit.

    Only the offending direction is blocked, so an axis sitting on its limit
    can always travel back out.
    """
    lo, hi = float(lo), float(hi)
    if lo > hi:
        lo, hi = hi, lo
    if position >= hi and rate > 0:
        return 0.0
    if position <= lo and rate < 0:
        return 0.0
    return rate


# ------------------------------------------------------------- top level --

def joy_to_command(axes, buttons, cfg, joy_age_s):
    """Turn one Joy message into a gimbal rate command.

    Fail-closed in three ways, all of which must hold for the command to be
    acted on: the deadman is held, the Joy message is fresh, and the axes it
    references actually exist.
    """
    stale = float(joy_age_s) >= JOY_TIMEOUT_S
    held = button_pressed(buttons, BTN_DEADMAN)

    if stale or not held:
        return GimbalRates(0.0, 0.0, 0.0, False)

    pan_in = apply_deadzone(axis_value(axes, AX_PAN), cfg.deadzone)
    tilt_in = apply_deadzone(axis_value(axes, AX_TILT), cfg.deadzone)
    lt = trigger_value(axes, AX_ROLL_CCW)
    rt = trigger_value(axes, AX_ROLL_CW)

    return GimbalRates(
        pan=stick_to_rate(pan_in, cfg.max_pan_deg_s, cfg.invert_pan),
        # Sticks report +1 up on joy_node; negate so positive tilt is up
        # unless the mount inverts it.
        tilt=stick_to_rate(-tilt_in, cfg.max_tilt_deg_s, cfg.invert_tilt),
        roll=triggers_to_roll(lt, rt, cfg.max_roll_deg_s, cfg.invert_roll),
        valid=True,
    )
