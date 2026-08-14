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


def localization_position_limit(
        limit: float, docked_limit: float,
        dock_contact_confirmed: bool, at_dock_pose: bool) -> float:
    """Which AMCL position-variance limit applies to the current pose.

    PB-AUDIT-026. Seated on the charger, AMCL's variance along the approach axis
    is permanently loose — measured 0.57-0.63 while the cross-axis stayed at
    0.001 and yaw at 0.0003 — because the dock fills the laser's near field and
    no scan return constrains that direction. The pose itself was accurate to
    ~6 cm against the surveyed dock pose, so refusing to undock on that variance
    was refusing on dock geometry rather than on any doubt about where the robot
    was.

    The wider `docked_limit` therefore needs *both* pieces of evidence, and each
    covers the other's blind spot:

    - `dock_contact_confirmed` (the SBC's own dock observer, mated plates) alone
      would accept a pose estimate that has drifted somewhere else on the map;
    - `at_dock_pose` (the estimate agrees with the surveyed dock pose) alone
      would accept a robot that has physically left the dock.

    `limit` is returned unchanged in every other case, including a `docked_limit`
    that is somehow tighter, so widening the docked allowance can never tighten
    the general one.
    """
    if dock_contact_confirmed and at_dock_pose:
        return max(float(limit), float(docked_limit))
    return float(limit)


def planar_covariance_usable(xx: float, yy: float, yaw_yaw: float) -> bool:
    """Whether AMCL's planar variances describe a covariance at all.

    Separate from `localization_position_limit`, and deliberately so: that
    function asks whether the filter is *confident enough*, this one asks
    whether its answer is *arithmetically meaningful*. A gate that only tests
    an upper bound conflates the two, and a degenerate filter then passes it
    the most easily of all — the smaller and more nonsensical the number, the
    more comfortably it sits under the limit.

    That is not hypothetical. Two independent captures from the docked robot,
    three days and a container restart apart:

        2026-08-11              2026-08-14
        cov[0]  (xx)  =  8.253264e-10    -7.861393e-10  <- negative
        cov[7]  (yy)  = -6.219768e-10     1.860600e-07
        cov[35] (yaw) = -6.128431e-14     4.660659e-07

    Note which entries are negative: they are not the same ones. This is a
    recurring degenerate condition rather than one stuck sample, and it is why
    all three entries are tested rather than whichever one happened to be
    caught first. On 2026-08-11 all 188 /amcl_pose samples over 60 s were
    byte-identical, so the filter was frozen rather than merely overconfident,
    and it had not self-recovered.
    `/patrolbot/dock_readiness` reported `localization_valid: true` and
    `ready_for_undock: true` against that — i.e. the robot would have been
    cleared to reverse off the charger on a pose estimate no filter was
    maintaining.

    Only the three planar entries are checked. A 2-D filter leaves z, roll and
    pitch at exactly zero as a matter of course, so testing those would refuse
    every healthy AMCL pose there has ever been.

    Zero is refused along with the negatives: a variance of exactly zero claims
    perfect certainty, which a particle filter cannot honestly produce, and it
    is the value a never-populated matrix carries.
    """
    for value in (xx, yy, yaw_yaw):
        value = float(value)
        # NaN fails both comparisons; the infinities are caught by the same
        # test that rejects them as a magnitude.
        if not value > 0.0 or value == float('inf'):
            return False
    return True


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
    the dock. The SBC dock observer is authoritative; raw charge state can
    re-latch after a proven departure and is never enough by itself.
    """
    failures = []
    if not state.link_connected or not state.hardware_state_valid:
        failures.append('hardware link/state invalid')
    if not state.dock_state_valid:
        failures.append('SBC dock state is stale or unavailable')
    elif state.dock_state != DOCK_DOCKED_CONFIRMED:
        failures.append(
            f'SBC does not report the robot docked '
            f'(dock_state={state.dock_state})')
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
