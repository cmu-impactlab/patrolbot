"""Exhaustive fail-closed tests for priority-10 safety policy."""

from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from safety_policy import evaluate_safety  # noqa: E402


def safe_state(**overrides):
    state = {
        'now': 10.0,
        'scan_at': 9.9,
        'odom_at': 9.9,
        'battery_at': 9.9,
        'battery_voltage': 24.0,
        'base_state_at': 9.9,
        'hardware_state_valid': True,
        'fault_flags': 0,
        'estop_pressed': False,
        'bumpers_enabled': False,
        'bumpers_valid': False,
        'front_bumper_pressed': False,
        'rear_bumper_pressed': False,
        'desired_linear': 0.0,
        'desired_angular': 0.0,
    }
    state.update(overrides)
    return state


def test_nominal_state_releases_priority_hold():
    assert evaluate_safety(**safe_state()) == []


def test_every_missing_or_stale_required_stream_holds():
    cases = {
        'scan_at': 'scan_stale',
        'odom_at': 'odom_stale',
        'battery_at': 'battery_stale',
        'base_state_at': 'base_state_stale',
    }
    for field, reason in cases.items():
        assert reason in evaluate_safety(**safe_state(**{field: None}))
        assert reason in evaluate_safety(**safe_state(**{field: 0.0}))


def test_battery_boundary_matches_manual_operating_minimum():
    assert evaluate_safety(**safe_state(battery_voltage=23.0)) == []
    assert 'battery_below_operating_minimum' in evaluate_safety(
        **safe_state(battery_voltage=22.999))
    assert 'battery_below_operating_minimum' in evaluate_safety(
        **safe_state(battery_voltage=float('nan')))


def test_invalid_hardware_fault_and_estop_each_hold():
    assert 'hardware_state_invalid' in evaluate_safety(
        **safe_state(hardware_state_valid=False))
    assert 'hardware_fault_0x0108' in evaluate_safety(
        **safe_state(fault_flags=0x0108))
    assert 'estop_asserted' in evaluate_safety(
        **safe_state(estop_pressed=True))


def test_bumper_enforcement_is_explicitly_uncommissioned_by_default():
    # Disabled bumper enforcement does not pretend unmapped bits are valid.
    assert evaluate_safety(**safe_state(bumpers_enabled=False)) == []
    assert 'bumper_mapping_invalid' in evaluate_safety(
        **safe_state(bumpers_enabled=True, bumpers_valid=False))


def test_bumpers_block_only_motion_toward_contact_or_rotation():
    common = {'bumpers_enabled': True, 'bumpers_valid': True}
    assert 'front_bumper_motion_block' in evaluate_safety(**safe_state(
        **common, front_bumper_pressed=True, desired_linear=0.1))
    assert 'front_bumper_motion_block' not in evaluate_safety(**safe_state(
        **common, front_bumper_pressed=True, desired_linear=-0.1))
    assert 'rear_bumper_motion_block' in evaluate_safety(**safe_state(
        **common, rear_bumper_pressed=True, desired_linear=-0.1))
    assert 'rear_bumper_motion_block' not in evaluate_safety(**safe_state(
        **common, rear_bumper_pressed=True, desired_linear=0.1))
    for front, rear in ((True, False), (False, True)):
        reasons = evaluate_safety(**safe_state(
            **common,
            front_bumper_pressed=front,
            rear_bumper_pressed=rear,
            desired_angular=0.1))
        assert any(reason.endswith('bumper_motion_block') for reason in reasons)


def test_explicit_policy_switches_can_only_disable_the_named_gate():
    assert 'hardware_fault_0x0001' not in evaluate_safety(
        **safe_state(fault_flags=1, stop_on_faults=False))
    assert 'estop_asserted' not in evaluate_safety(
        **safe_state(estop_pressed=True, stop_on_estop=False))
