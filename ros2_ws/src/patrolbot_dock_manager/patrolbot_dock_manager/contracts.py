"""Pure safety contracts shared by the dock manager and offline tests."""

from dataclasses import dataclass
from typing import List


HARD_MAX_DISTANCE = 0.70
HARD_MAX_SPEED = 0.10
VALIDATION_MAX_DISTANCE = 0.10
VALIDATION_MAX_SPEED = 0.05
# ROS action fields are float32; exact decimal boundary values arrive a few
# nanounits above their source literals. This tolerance admits only that wire
# representation error and does not silently clamp an actual over-limit goal.
FLOAT32_BOUNDARY_EPSILON = 1e-6

HAPPY_PATH = (
    'IDLE',
    'PRECHECK',
    'REQUEST_DOCK_RELEASE',
    'WAIT_FOR_CHARGE_RELEASE',
    'VERIFY_ESTOP_AND_MOTOR',
    'VERIFY_REAR_SONAR',
    'VERIFY_LOCALIZATION',
    'VERIFY_COMMAND_PATH',
    'GUARDED_REVERSE',
    'CLEAR_OF_DOCK',
    'STOPPED',
    'COMPLETE',
)
TERMINAL_STATES = frozenset(('ABORTED', 'FAULT'))
ALLOWED_TRANSITIONS = {
    state: frozenset((HAPPY_PATH[index + 1], *TERMINAL_STATES))
    for index, state in enumerate(HAPPY_PATH[:-1])
}


@dataclass(frozen=True)
class SafetySnapshot:
    """One fail-closed view of every input required for automatic reverse."""

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


def transition_allowed(current: str, target: str) -> bool:
    """Return whether an undock state transition is explicitly authorized."""
    return target in ALLOWED_TRANSITIONS.get(current, ())


def validate_goal(distance: float, speed: float, validation_mode: bool) -> List[str]:
    """Return hard-cap violations without silently clamping a motion request."""
    failures = []
    if (distance <= 0.0 or
            distance > HARD_MAX_DISTANCE + FLOAT32_BOUNDARY_EPSILON):
        failures.append('reverse distance outside (0.0, 0.70] m')
    if speed <= 0.0 or speed > HARD_MAX_SPEED + FLOAT32_BOUNDARY_EPSILON:
        failures.append('reverse speed outside (0.0, 0.10] m/s')
    if (validation_mode and
            distance > VALIDATION_MAX_DISTANCE + FLOAT32_BOUNDARY_EPSILON):
        failures.append('validation reverse distance exceeds 0.10 m')
    if (validation_mode and
            speed > VALIDATION_MAX_SPEED + FLOAT32_BOUNDARY_EPSILON):
        failures.append('validation reverse speed exceeds 0.05 m/s')
    return failures


def precheck_failures(
        state: SafetySnapshot, allow_front_bumper_at_dock: bool = False) -> List[str]:
    """Validate docked, zero-motion gates before requesting charge release.

    E-stop clear and motor state are intentionally not required here. Manual
    charging may leave motors reported enabled; the SBC release operation first
    disables and confirms them before sending CHARGE 0. Some hardware may clear
    the dock-related stop indication only during the post-release operator PWR
    ON window (binding amendment A-1).
    """
    failures = []
    if not state.link_connected or not state.hardware_state_valid:
        failures.append('hardware link/state invalid')
    if not state.charge_state_valid or state.charge_state <= 0:
        failures.append('robot is not confirmed charging at the dock')
    if state.fault_flags:
        failures.append('hardware fault flags are nonzero')
    if not state.bumpers_valid:
        failures.append('bumper mapping/state invalid')
    if state.rear_bumper_pressed:
        failures.append('rear bumper pressed')
    if state.front_bumper_pressed and not allow_front_bumper_at_dock:
        failures.append('front bumper pressed without accepted dock baseline')
    if not state.sonar_fresh or not state.rear_clear:
        failures.append('rear sonar stale or obstructed')
    if not state.odom_fresh or not state.stationary:
        failures.append('odometry stale or robot moving')
    if not state.localization_fresh or not state.localization_valid:
        failures.append('localization invalid')
    if not state.dock_pose_valid:
        failures.append('robot is not within configured dock-pose tolerance')
    if not state.command_path_valid:
        failures.append('velocity command path invalid')
    if not state.navigation_idle:
        failures.append('another navigation or behavior action is active')
    return failures


def reverse_failures(
        state: SafetySnapshot, expected_session: int) -> List[str]:
    """Validate every continuously monitored reverse-motion gate."""
    failures = []
    if not state.link_connected or not state.hardware_state_valid:
        failures.append('hardware link/state invalid')
    if state.session_generation != expected_session:
        failures.append('hardware session changed')
    if not state.charge_state_valid or state.charge_state != 0:
        failures.append('charge release not confirmed')
    if state.estop_pressed:
        failures.append('E-stop asserted')
    if not state.motors_enabled:
        failures.append('motors disabled')
    if state.fault_flags:
        failures.append('hardware fault flags are nonzero')
    if not state.bumpers_valid:
        failures.append('bumper state invalid')
    if state.front_bumper_pressed or state.rear_bumper_pressed:
        failures.append('bumper pressed')
    if not state.sonar_fresh or not state.rear_clear:
        failures.append('rear sonar stale or obstructed')
    if not state.odom_fresh:
        failures.append('odometry stale')
    if not state.localization_fresh or not state.localization_valid:
        failures.append('localization invalid')
    if not state.command_path_valid:
        failures.append('velocity command path invalid')
    if not state.navigation_idle:
        failures.append('another navigation or behavior action became active')
    return failures


def dock_pose_initialization_failures(state: SafetySnapshot) -> List[str]:
    """Validate a zero-motion, operator-requested dock-pose initialization.

    This deliberately does not require an existing localization estimate or a
    clear E-stop: those are the facts being recovered/physically clarified at
    the dock. It does require positive evidence that the robot is docked,
    stationary, fault-free, motor-disabled, and not executing another action.
    """
    failures = []
    if not state.link_connected or not state.hardware_state_valid:
        failures.append('hardware link/state invalid')
    if not state.charge_state_valid or state.charge_state <= 0:
        failures.append('robot is not confirmed charging at the dock')
    if state.motors_enabled:
        failures.append('motors unexpectedly enabled while docked')
    if state.fault_flags:
        failures.append('hardware fault flags are nonzero')
    if not state.odom_fresh or not state.stationary:
        failures.append('odometry stale or robot moving')
    if not state.navigation_idle:
        failures.append('another navigation or behavior action is active')
    return failures
