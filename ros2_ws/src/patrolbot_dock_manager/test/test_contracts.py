"""Exhaustive fail-closed checks for guarded-undock safety contracts."""

from dataclasses import replace

from patrolbot_dock_manager.contracts import (
    dock_pose_initialization_failures,
    HAPPY_PATH,
    precheck_failures,
    reverse_failures,
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
)

SAFE_REVERSE = replace(
    SAFE_DOCKED, charge_state=0, motors_enabled=True, estop_pressed=False)


def test_docked_precheck_allows_estop_until_shared_operator_window():
    assert precheck_failures(SAFE_DOCKED) == []


def test_docked_precheck_allows_release_to_disable_enabled_motors():
    assert precheck_failures(replace(SAFE_DOCKED, motors_enabled=True)) == []


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('link_connected', False),
        ('hardware_state_valid', False),
        ('charge_state_valid', False),
        ('charge_state', 0),
        ('bumpers_valid', False),
        ('rear_bumper_pressed', True),
        ('sonar_fresh', False),
        ('rear_clear', False),
        ('odom_fresh', False),
        ('stationary', False),
        ('localization_valid', False),
        ('dock_pose_valid', False),
        ('command_path_valid', False),
        ('navigation_idle', False),
    ],
)
def test_each_precheck_input_fails_closed(field, value):
    assert precheck_failures(replace(SAFE_DOCKED, **{field: value}))


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('link_connected', False),
        ('hardware_state_valid', False),
        ('charge_state_valid', False),
        ('charge_state', 1),
        ('motors_enabled', False),
        ('estop_pressed', True),
        ('bumpers_valid', False),
        ('front_bumper_pressed', True),
        ('rear_bumper_pressed', True),
        ('sonar_fresh', False),
        ('rear_clear', False),
        ('odom_fresh', False),
        ('localization_fresh', False),
        ('localization_valid', False),
        ('command_path_valid', False),
        ('navigation_idle', False),
    ],
)
def test_each_reverse_input_aborts(field, value):
    assert reverse_failures(replace(SAFE_REVERSE, **{field: value}), 7)


def test_hardware_reconnect_aborts_reverse():
    assert reverse_failures(replace(SAFE_REVERSE, session_generation=8), 7)


def test_dock_pose_initialization_is_zero_motion_and_dock_evidence_gated():
    assert dock_pose_initialization_failures(SAFE_DOCKED) == []


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('link_connected', False),
        ('hardware_state_valid', False),
        ('charge_state_valid', False),
        ('charge_state', 0),
        ('motors_enabled', True),
        ('fault_flags', 1),
        ('odom_fresh', False),
        ('stationary', False),
        ('navigation_idle', False),
    ],
)
def test_each_dock_pose_initialization_input_fails_closed(field, value):
    assert dock_pose_initialization_failures(
        replace(SAFE_DOCKED, **{field: value}))


def test_dock_pose_initialization_does_not_require_old_localization_or_clear_estop():
    recovering = replace(
        SAFE_DOCKED,
        localization_fresh=False,
        localization_valid=False,
        dock_pose_valid=False,
        estop_pressed=True,
        bumpers_valid=False,
    )
    assert dock_pose_initialization_failures(recovering) == []


def test_happy_path_transitions_are_explicit_and_ordered():
    for current, target in zip(HAPPY_PATH, HAPPY_PATH[1:]):
        assert transition_allowed(current, target)


def test_every_active_state_can_abort_or_fault():
    for state in HAPPY_PATH[:-1]:
        for terminal in TERMINAL_STATES:
            assert transition_allowed(state, terminal)


def test_state_machine_rejects_skips_reentry_and_unknown_states():
    assert not transition_allowed('PRECHECK', 'GUARDED_REVERSE')
    assert not transition_allowed('GUARDED_REVERSE', 'PRECHECK')
    assert not transition_allowed('PRECHECK', 'PRECHECK')
    assert not transition_allowed('UNKNOWN', 'PRECHECK')


def test_validation_and_hard_motion_caps_are_rejected_not_clamped():
    assert validate_goal(0.10, 0.05, True) == []
    assert validate_goal(0.10000000149, 0.050000000745, True) == []
    assert validate_goal(0.1001, 0.05, True)
    assert validate_goal(0.10, 0.0501, True)
    assert validate_goal(0.7001, 0.05, False)
    assert validate_goal(0.10, 0.1001, False)
