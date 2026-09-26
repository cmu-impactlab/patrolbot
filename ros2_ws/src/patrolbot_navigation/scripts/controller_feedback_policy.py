"""ROS-free blocker selection and pulse timing for F710 safety feedback."""

from joystick_policy import drive_intent
from safety_policy import evaluate_safety


DOCK_CLEAR_CONFIRMED = 'CLEAR_CONFIRMED'

PULSE_EVENT = 'event'
PULSE_RETRY = 'retry'
PUBLISH_ZERO = 'zero'

_FEEDBACK_SAFETY_REASONS = {
    'scan_stale',
    'odom_stale',
    'battery_stale',
    'battery_below_operating_minimum',
    'base_state_stale',
    'hardware_state_invalid',
    'estop_asserted',
    'front_bumper_motion_block',
    'rear_bumper_motion_block',
}


def joystick_drive_intent(*, now, joy_at, axes, buttons, timeout, deadzone):
    """Return fresh RB-plus-direction intent and its normalized direction."""
    if joy_at is None or (now - joy_at) > timeout:
        return False, 0.0, 0.0
    return drive_intent(axes, buttons, deadzone)


def feedback_blockers(
        *, now, scan_at, odom_at, battery_at, battery_voltage,
        base_state_at, hardware_state_valid, fault_flags, estop_pressed,
        bumpers_valid, front_bumper_pressed, rear_bumper_pressed,
        desired_linear, desired_angular, motors_enabled, dock_state_valid,
        dock_state, drive_attempt, scan_timeout=0.5, odom_timeout=0.5,
        battery_timeout=1.0, base_state_timeout=0.75,
        battery_stop_voltage=23.0):
    """Return ordered reasons that should be announced to the controller.

    The shared safety policy remains authoritative for sensor, battery,
    hardware, E-stop, fault, and direction-aware bumper decisions. Feedback
    deliberately omits the uncommissioned-bumper-mapping hold: it reports a
    real mapped bumper block, not the absence of a mapping.
    """
    safety_reasons = evaluate_safety(
        now=now,
        scan_at=scan_at,
        odom_at=odom_at,
        battery_at=battery_at,
        battery_voltage=battery_voltage,
        base_state_at=base_state_at,
        hardware_state_valid=hardware_state_valid,
        fault_flags=fault_flags,
        estop_pressed=estop_pressed,
        bumpers_enabled=True,
        bumpers_valid=bumpers_valid,
        front_bumper_pressed=front_bumper_pressed,
        rear_bumper_pressed=rear_bumper_pressed,
        desired_linear=desired_linear,
        desired_angular=desired_angular,
        scan_timeout=scan_timeout,
        odom_timeout=odom_timeout,
        battery_timeout=battery_timeout,
        base_state_timeout=base_state_timeout,
        battery_stop_voltage=battery_stop_voltage,
        stop_on_faults=True,
        stop_on_estop=True)

    reasons = [
        reason for reason in safety_reasons
        if reason in _FEEDBACK_SAFETY_REASONS
        or reason.startswith('hardware_fault_')
    ]

    base_fresh = (
        base_state_at is not None
        and (now - base_state_at) <= base_state_timeout
    )
    if base_fresh and dock_state_valid:
        if dock_state == DOCK_CLEAR_CONFIRMED and not motors_enabled:
            reasons.append('motors_disabled_while_clear')
        elif drive_attempt and dock_state != DOCK_CLEAR_CONFIRMED:
            reasons.append(f'dock_state_{dock_state}_blocks_drive')
    elif base_fresh and drive_attempt:
        reasons.append('dock_state_invalid_blocks_drive')

    return reasons


class PulseScheduler:
    """Turn blocker episodes and repeated drive attempts into finite pulses."""

    def __init__(
            self, *, started_at, event_pulse, retry_pulse, retry_interval,
            startup_grace):
        self.started_at = float(started_at)
        self.event_pulse = float(event_pulse)
        self.retry_pulse = float(retry_pulse)
        self.retry_interval = float(retry_interval)
        self.startup_grace = float(startup_grace)
        self._was_blocked = False
        self._event_sent = False
        self._pulse_until = None
        self._last_pulse_at = None

    @property
    def pulse_active(self):
        return self._pulse_until is not None

    def update(self, *, now, reasons, drive_attempt):
        """Return zero or one output actions for this clock tick."""
        now = float(now)
        blocked = bool(reasons)

        if not blocked:
            recovered = self._was_blocked
            active = self.pulse_active
            self._was_blocked = False
            self._event_sent = False
            self._pulse_until = None
            if recovered or active:
                return [(PUBLISH_ZERO, 0.0)]
            return []

        if not self._was_blocked:
            self._was_blocked = True
            self._event_sent = False

        if self.pulse_active and now >= self._pulse_until:
            self._pulse_until = None
            return [(PUBLISH_ZERO, 0.0)]
        if self.pulse_active:
            return []

        grace_complete = (now - self.started_at) >= self.startup_grace
        if not grace_complete:
            return []

        if not self._event_sent:
            self._event_sent = True
            self._last_pulse_at = now
            self._pulse_until = now + self.event_pulse
            return [(PULSE_EVENT, self.event_pulse)]

        retry_due = (
            self._last_pulse_at is None
            or (now - self._last_pulse_at) >= self.retry_interval
        )
        if drive_attempt and retry_due:
            self._last_pulse_at = now
            self._pulse_until = now + self.retry_pulse
            return [(PULSE_RETRY, self.retry_pulse)]
        return []
