"""Pure safety contracts shared by the dock manager and offline tests.

Physical separation from the dock is owned by the SBC. Everything here is a
Pi-side concern: whether it is sensible to ask for that operation at all, and
whether navigation may resume afterwards. There is deliberately no
`reverse_failures` any more — continuously monitoring a reverse the Pi does not
command was the split-ownership defect this design removes.
"""

from dataclasses import dataclass
from typing import List


HARD_MAX_DISTANCE = 0.70
# DO NOT RAISE. This mirrors the SBC's own hard cap; the SBC rejects anything
# larger outright rather than clamping it, so raising this only changes which
# side reports the failure.
#
# Measured 2026-07-28: at 0.10 m/s the robot reversed the full 0.53 m; raising
# this to 0.20 m/s made the base re-latch charge and cut the motors 0.24 m in,
# aborting the undock with "motors disabled". The reverse leg is slow because
# the hardware says so, not because the software is being cautious.
HARD_MAX_SPEED = 0.10
VALIDATION_MAX_DISTANCE = 0.10
VALIDATION_MAX_SPEED = 0.05
# ROS action fields are float32; exact decimal boundary values arrive a few
# nanounits above their source literals. This tolerance admits only that wire
# representation error and does not silently clamp an actual over-limit goal.
FLOAT32_BOUNDARY_EPSILON = 1e-6

# Nine states, down from sixteen: the six that used to command and monitor
# contact separation now live inside one SBC operation.
HAPPY_PATH = (
    'IDLE',
    'PRECHECK',
    'VERIFY_LOCALIZATION',
    'VERIFY_COMMAND_PATH',
    'HARDWARE_UNDOCK',
    'RESTORE_LOCALIZATION',
    'TURN_AWAY_FROM_DOCK',
    'STOPPED',
    'COMPLETE',
)
TERMINAL_STATES = frozenset(('ABORTED', 'FAULT'))
ALLOWED_TRANSITIONS = {
    state: frozenset((HAPPY_PATH[index + 1], *TERMINAL_STATES))
    for index, state in enumerate(HAPPY_PATH[:-1])
}

# Mirrors patrolbot::DockPhysicalState. The SBC permits ordinary motion only in
# CLEAR_CONFIRMED.
DOCK_UNKNOWN = 'UNKNOWN'
DOCK_DOCKED_CONFIRMED = 'DOCKED_CONFIRMED'
DOCK_DEPARTING = 'DEPARTING'
DOCK_CLEAR_CONFIRMED = 'CLEAR_CONFIRMED'
DOCK_FAULT_STOPPED = 'FAULT_STOPPED'


@dataclass(frozen=True)
class SafetySnapshot:
    """One fail-closed view of every input the Pi is responsible for."""

    link_connected: bool = False
    hardware_state_valid: bool = False
    session_generation: int = 0
    charge_state_valid: bool = False
    charge_state: int = -1
    motors_enabled: bool = False
    estop_pressed: bool = True
    fault_flags: int = 0
    bumpers_valid: bool = False
    front_bumper_pressed: bool = False
    rear_bumper_pressed: bool = False
    sonar_fresh: bool = False
    rear_clear: bool = False
    odom_fresh: bool = False
    stationary: bool = False
    localization_fresh: bool = False
    localization_valid: bool = False
    dock_pose_valid: bool = False
    command_path_valid: bool = False
    navigation_idle: bool = False
    # SBC-owned dock ownership. Absent or stale telemetry reads as UNKNOWN.
    dock_state: str = DOCK_UNKNOWN
    dock_state_valid: bool = False
    undock_active: bool = False
    physically_clear: bool = False
    # Advisory, published by the SBC and enforced here. It gates DOCKING and
    # never undocking.
    redock_inhibited: bool = True
    undock_profile_commissioned: bool = False


def transition_allowed(current: str, target: str) -> bool:
    """Return whether an undock state transition is explicitly authorized."""
    return target in ALLOWED_TRANSITIONS.get(current, ())


def validate_goal(distance: float, speed: float, validation_mode: bool) -> List[str]:
    """Return hard-cap violations without silently clamping a motion request."""
    failures = []
    if (distance <= 0.0 or
            distance > HARD_MAX_DISTANCE + FLOAT32_BOUNDARY_EPSILON):
        failures.append(
            f'reverse distance outside (0.0, {HARD_MAX_DISTANCE:.2f}] m')
    if speed <= 0.0 or speed > HARD_MAX_SPEED + FLOAT32_BOUNDARY_EPSILON:
        failures.append(
            f'reverse speed outside (0.0, {HARD_MAX_SPEED:.2f}] m/s')
    if (validation_mode and
            distance > VALIDATION_MAX_DISTANCE + FLOAT32_BOUNDARY_EPSILON):
        failures.append('validation reverse distance exceeds 0.10 m')
    if (validation_mode and
            speed > VALIDATION_MAX_SPEED + FLOAT32_BOUNDARY_EPSILON):
        failures.append('validation reverse speed exceeds 0.05 m/s')
    return failures


def precheck_failures(
        state: SafetySnapshot, validation_mode: bool = False) -> List[str]:
    """Validate the Pi-side gates before asking the SBC to undock.

    Physical readiness is the SBC's call: it owns the bumpers, the rear sonar,
    the charger and the motors, and refuses the operation itself if any of them
    is wrong. What is checked here is what only the Pi can see — localization,
    the velocity command path, and whether another navigation action is running
    — plus the SBC's own published verdict on whether the robot is docked and
    commissioned.

    E-stop and motor state are intentionally not required. A docked robot has
    been observed reporting estop=1 with motors=0, and the SBC's release
    sequence disables and confirms the motors itself before touching the
    charger.
    """
    failures = []
    if not state.link_connected or not state.hardware_state_valid:
        failures.append('hardware link/state invalid')
    if not state.dock_state_valid:
        failures.append('SBC dock state is stale or unavailable')
    elif state.dock_state == DOCK_FAULT_STOPPED:
        failures.append(
            'SBC dock state is FAULT_STOPPED; an explicit reset is required')
    elif state.dock_state != DOCK_DOCKED_CONFIRMED:
        failures.append(
            f'SBC does not report the robot docked '
            f'(dock_state={state.dock_state})')
    if state.undock_active:
        failures.append('an undock operation is already running')
    if not validation_mode and not state.undock_profile_commissioned:
        # Production undocking is refused by the SBC until rear-sonar indices,
        # E-stop semantics and fault masks have been physically validated.
        failures.append('SBC undock hardware profile is not commissioned')
    if not state.localization_fresh or not state.localization_valid:
        failures.append('localization invalid')
    if not state.command_path_valid:
        failures.append('velocity command path invalid')
    if not state.navigation_idle:
        failures.append('another navigation or behavior action is active')
    return failures


def resume_failures(state: SafetySnapshot, expected_session: int) -> List[str]:
    """Validate that Nav2 may take control after the SBC hands off.

    Called for a bounded convergence window after the hardware undock reports
    success. The SBC leaves the motors live at zero velocity precisely so this
    can succeed; a handoff that left them disabled would strand the robot,
    because the generic drive gate refuses motion with the motors down.
    """
    failures = []
    if not state.link_connected or not state.hardware_state_valid:
        failures.append('hardware link/state invalid')
    if state.session_generation != expected_session:
        failures.append('hardware session changed')
    if not state.dock_state_valid or state.dock_state != DOCK_CLEAR_CONFIRMED:
        failures.append(
            f'SBC has not confirmed physical clearance '
            f'(dock_state={state.dock_state})')
    if not state.motors_enabled:
        failures.append('motors are not live for the Nav2 handoff')
    if state.estop_pressed:
        failures.append('E-stop asserted')
    if not state.odom_fresh:
        failures.append('odometry stale')
    if not state.localization_fresh or not state.localization_valid:
        failures.append('localization invalid')
    if not state.command_path_valid:
        failures.append('velocity command path invalid')
    if not state.navigation_idle:
        failures.append('another navigation or behavior action became active')
    return failures


def automatic_docking_allowed(state: SafetySnapshot) -> bool:
    """Report whether a mission may send the robot back to the dock.

    The SBC publishes the inhibition but has no autonomous dock operation, so
    nothing on the hardware side enforces it. This is the check that does.
    Deliberately separate from undock readiness: after a successful undock the
    robot is not ready to undock (it is already clear) and not allowed to dock
    (the inhibition is running), and those are different facts.
    """
    if not state.dock_state_valid:
        return False
    if state.redock_inhibited:
        return False
    return not state.undock_active


def dock_pose_initialization_failures(state: SafetySnapshot) -> List[str]:
    """Validate a zero-motion, operator-requested dock-pose initialization.

    This deliberately does not require an existing localization estimate or a
    clear E-stop: those are the facts being recovered/physically clarified at
    the dock. It does require positive evidence that the robot is docked,
    stationary, fault-free, and not executing another action.
    """
    failures = []
    if not state.link_connected or not state.hardware_state_valid:
        failures.append('hardware link/state invalid')
    if not state.charge_state_valid or state.charge_state <= 0:
        failures.append('robot is not confirmed charging at the dock')
    # Motor state is deliberately NOT required here. This operation publishes a
    # pose estimate and moves nothing, so what makes it correct is that the
    # robot is at the dock (charging, required above) and not moving
    # (stationary, required below) — powered motors change neither. A robot
    # docked by hand, or left after a partial undock, sits charging with its
    # motors live, and refusing there left localization unrecoverable at
    # exactly the moment the dock pose was the only fix available.
    if state.fault_flags:
        failures.append('hardware fault flags are nonzero')
    if not state.odom_fresh or not state.stationary:
        failures.append('odometry stale or robot moving')
    if not state.navigation_idle:
        failures.append('another navigation or behavior action is active')
    return failures
