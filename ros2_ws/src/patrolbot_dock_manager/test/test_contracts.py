"""Exhaustive fail-closed checks for the Pi-side undock safety contracts.

Physical separation is the SBC's job, so what is tested here is the Pi's own
responsibility: whether it is sensible to ask for the operation, whether Nav2
may take control afterwards, and whether a mission may dock again.
"""

from dataclasses import replace

from patrolbot_dock_manager.contracts import (
    automatic_docking_allowed,
    DOCK_CLEAR_CONFIRMED,
    DOCK_DEPARTING,
    DOCK_DOCKED_CONFIRMED,
    DOCK_FAULT_STOPPED,
    dock_pose_initialization_failures,
    DOCK_UNKNOWN,
    HAPPY_PATH,
    HARD_MAX_DISTANCE,
    HARD_MAX_SPEED,
    precheck_failures,
    resume_failures,
    SafetySnapshot,
    TERMINAL_STATES,
    transition_allowed,
    validate_goal,
)
import pytest


SAFE_DOCKED = SafetySnapshot(
    link_connected=True,
    hardware_state_valid=True,
    session_generation=7,
    charge_state_valid=True,
    charge_state=2,
    motors_enabled=False,
    estop_pressed=True,
    bumpers_valid=True,
    sonar_fresh=True,
    rear_clear=True,
    odom_fresh=True,
    stationary=True,
    localization_fresh=True,
    localization_valid=True,
    dock_pose_valid=True,
    command_path_valid=True,
    navigation_idle=True,
    dock_state=DOCK_DOCKED_CONFIRMED,
    dock_state_valid=True,
    undock_active=False,
    physically_clear=False,
    redock_inhibited=False,
    undock_profile_commissioned=True,
)

SAFE_HANDOFF = replace(
    SAFE_DOCKED,
    charge_state=0,
    motors_enabled=True,
    estop_pressed=False,
    dock_state=DOCK_CLEAR_CONFIRMED,
    physically_clear=True,
    redock_inhibited=True,
)


# ---------------------------------------------------------------------------
# Precheck
# ---------------------------------------------------------------------------

def test_docked_precheck_passes_with_estop_and_motors_unresolved():
    """A docked robot reporting estop=1, motors=0 is normal, not a failure."""
    assert precheck_failures(SAFE_DOCKED) == []
    assert precheck_failures(replace(SAFE_DOCKED, motors_enabled=True)) == []


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('link_connected', False),
        ('hardware_state_valid', False),
        ('dock_state_valid', False),
        ('undock_active', True),
        ('undock_profile_commissioned', False),
        ('localization_fresh', False),
        ('localization_valid', False),
        ('command_path_valid', False),
        ('navigation_idle', False),
    ],
)
def test_every_precheck_input_fails_closed(field, value):
    assert precheck_failures(replace(SAFE_DOCKED, **{field: value})) != []


@pytest.mark.parametrize(
    'dock_state',
    [DOCK_UNKNOWN, DOCK_DEPARTING, DOCK_CLEAR_CONFIRMED, DOCK_FAULT_STOPPED])
def test_precheck_requires_the_sbc_to_report_the_robot_docked(dock_state):
    failures = precheck_failures(replace(SAFE_DOCKED, dock_state=dock_state))
    assert failures != []


def test_fault_stopped_names_the_reset_it_needs():
    failures = precheck_failures(
        replace(SAFE_DOCKED, dock_state=DOCK_FAULT_STOPPED))
    assert any('reset' in failure for failure in failures)


def test_validation_mode_is_the_only_uncommissioned_path():
    uncommissioned = replace(SAFE_DOCKED, undock_profile_commissioned=False)
    assert precheck_failures(uncommissioned, validation_mode=False) != []
    assert precheck_failures(uncommissioned, validation_mode=True) == []


def test_redock_inhibition_never_blocks_undocking():
    """Inhibition gates docking. Conflating the two blocked recovery undocks."""
    assert precheck_failures(replace(SAFE_DOCKED, redock_inhibited=True)) == []


# ---------------------------------------------------------------------------
# Handoff back to Nav2
# ---------------------------------------------------------------------------

def test_handoff_requires_clearance_and_live_motors():
    assert resume_failures(SAFE_HANDOFF, 7) == []


def test_handoff_refuses_when_the_sbc_left_the_motors_down():
    # A handoff with the motors disabled would strand the robot: the generic
    # drive gate refuses motion without them.
    failures = resume_failures(replace(SAFE_HANDOFF, motors_enabled=False), 7)
    assert any('motors' in failure for failure in failures)


@pytest.mark.parametrize(
    'dock_state',
    [DOCK_UNKNOWN, DOCK_DOCKED_CONFIRMED, DOCK_DEPARTING, DOCK_FAULT_STOPPED])
def test_handoff_requires_confirmed_physical_clearance(dock_state):
    assert resume_failures(replace(SAFE_HANDOFF, dock_state=dock_state), 7) != []


def test_handoff_detects_a_session_change():
    assert resume_failures(SAFE_HANDOFF, 8) != []


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('link_connected', False),
        ('hardware_state_valid', False),
        ('estop_pressed', True),
        ('odom_fresh', False),
        ('localization_valid', False),
        ('command_path_valid', False),
        ('navigation_idle', False),
    ],
)
def test_every_handoff_input_fails_closed(field, value):
    assert resume_failures(replace(SAFE_HANDOFF, **{field: value}), 7) != []


# ---------------------------------------------------------------------------
# Redock inhibition
# ---------------------------------------------------------------------------

def test_docking_is_inhibited_immediately_after_an_undock():
    assert not automatic_docking_allowed(SAFE_HANDOFF)
    assert automatic_docking_allowed(
        replace(SAFE_HANDOFF, redock_inhibited=False))


def test_docking_is_refused_without_a_valid_dock_state():
    assert not automatic_docking_allowed(
        replace(SAFE_HANDOFF, redock_inhibited=False, dock_state_valid=False))


def test_docking_is_refused_while_an_undock_runs():
    assert not automatic_docking_allowed(
        replace(SAFE_HANDOFF, redock_inhibited=False, undock_active=True))


# ---------------------------------------------------------------------------
# State machine and goal bounds
# ---------------------------------------------------------------------------

def test_happy_path_is_strictly_ordered():
    for current, target in zip(HAPPY_PATH, HAPPY_PATH[1:]):
        assert transition_allowed(current, target)


def test_states_cannot_be_skipped_or_reentered():
    assert not transition_allowed('PRECHECK', 'HARDWARE_UNDOCK')
    assert not transition_allowed('HARDWARE_UNDOCK', 'PRECHECK')
    assert not transition_allowed('HARDWARE_UNDOCK', 'HARDWARE_UNDOCK')
    assert not transition_allowed('COMPLETE', 'IDLE')


def test_every_state_may_fail_closed_to_a_terminal_state():
    for state in HAPPY_PATH[:-1]:
        for terminal in TERMINAL_STATES:
            assert transition_allowed(state, terminal)


def test_contact_separation_is_no_longer_a_pi_state():
    """The six states that used to command and monitor motion are gone."""
    for removed in ('REQUEST_DOCK_RELEASE', 'WAIT_FOR_CHARGE_RELEASE',
                    'VERIFY_ESTOP_AND_MOTOR', 'GUARDED_REVERSE',
                    'CLEAR_CHARGE_LATCH', 'CLEAR_OF_DOCK', 'ENABLE_TURN',
                    'FINALIZE_CHARGE_LATCH'):
        assert removed not in HAPPY_PATH


@pytest.mark.parametrize(
    ('distance', 'speed'),
    [(0.0, 0.10), (-0.1, 0.10), (0.71, 0.10), (0.53, 0.0), (0.53, 0.11)])
def test_out_of_range_goals_are_rejected_not_clamped(distance, speed):
    assert validate_goal(distance, speed, False) != []


def test_boundary_goals_are_admitted():
    assert validate_goal(HARD_MAX_DISTANCE, HARD_MAX_SPEED, False) == []


def test_validation_mode_has_tighter_caps():
    assert validate_goal(0.53, 0.10, True) != []
    assert validate_goal(0.10, 0.05, True) == []


# ---------------------------------------------------------------------------
# Dock-pose initialization
# ---------------------------------------------------------------------------

def test_dock_pose_initialization_requires_charging_and_stillness():
    assert dock_pose_initialization_failures(SAFE_DOCKED) == []
    assert dock_pose_initialization_failures(
        replace(SAFE_DOCKED, charge_state=0)) != []
    assert dock_pose_initialization_failures(
        replace(SAFE_DOCKED, stationary=False)) != []


def test_dock_pose_initialization_ignores_motor_state():
    assert dock_pose_initialization_failures(
        replace(SAFE_DOCKED, motors_enabled=True)) == []
