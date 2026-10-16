"""Scanner acquisition and transport freshness are independent of commissioning."""

from patrolbot_bridge.control_protocol import format_request, parse_reply
from patrolbot_bridge.lidar_state import LidarState
import pytest

ID = '123e4567-e89b-12d3-a456-426614174000'


def test_new_reset_and_stale_state_fail_closed():
    state = LidarState()
    assert state.snapshot(10) == ('UNKNOWN', False, -1.0)
    state.update('READY,1,100', 10)
    assert state.ready(10.39)
    assert not state.ready(10.41)
    assert state.snapshot(10.6) == ('UNKNOWN', False, -1.0)
    state.reset()
    assert not state.ready(10)


def test_receipt_queue_age_is_preserved():
    state = LidarState()
    state.update('READY,1,100', 10)
    value, supported, age = state.snapshot(10.2)
    assert (value, supported) == ('READY', True)
    assert age == pytest.approx(0.3)
    assert not state.ready(9.9)


@pytest.mark.parametrize(
    'wire', ['OFF,1,-1', 'STARTING,1,-1', 'FAULT,1,-1', 'UNKNOWN,0,-1', 'READY,1,501']
)
def test_only_fresh_ready_allows_pose_and_motion(wire):
    state = LidarState()
    state.update(wire, 10)
    assert not state.ready(10)


@pytest.mark.parametrize(
    'wire',
    [
        'READY,1,nan',
        'READY,1,-2',
        'BOGUS,1,1',
        'READY,true,1',
        'READY,1,1,0',
        'READY,1,-1',
        'READY,1,1.5',
        'READY,1,+1',
    ],
)
def test_malformed_report_invalidates_previous_ready(wire):
    state = LidarState()
    state.update('READY,1,20', 10)
    with pytest.raises(ValueError):
        state.update(wire, 10.1)
    assert not state.ready(10.1)


def test_old_queued_report_cannot_overwrite_newer_off():
    state = LidarState()
    state.update('OFF,1,-1', 11)
    state.update('READY,1,1', 10)
    assert state.snapshot(11)[0] == 'OFF'


def test_uncommissioned_scanner_can_be_ready():
    state = LidarState()
    state.update('READY,0,20', 10)
    assert state.ready(10)
    assert not state.snapshot(10)[1]


@pytest.mark.parametrize('command', ['LIDAR_ON', 'LIDAR_OFF'])
def test_power_commands_use_existing_idempotent_protocol(command):
    assert format_request(command, ID) == f'{command}:{ID}\n'.encode()
    assert parse_reply(f'ACK:{command}:{ID}:ACCEPTED').accepted
    assert parse_reply(f'RESULT:{command}:{ID}:OK:0:commanded:2:0:1').success


def test_power_transition_invalidates_old_seed_without_changing_odometry_origin():
    from patrolbot_bridge.epoch_recovery import EpochRecovery

    recovery = EpochRecovery()
    old = recovery.observe_origin(ID)
    recovery.reset_complete(old, 100)
    recovery.operator_seed(old, 110, 110)
    recovery.amcl_pose(old, 120, 120)
    assert not recovery.required
    recovery.require_seed(200)
    assert recovery.required and recovery.stage == recovery.WAIT_SEED
    assert recovery.token.epoch == old.epoch and recovery.token != old
    assert not recovery.amcl_pose(old, 210, 210)
    assert not recovery.operator_seed(recovery.token, 190, 210)
    assert recovery.operator_seed(recovery.token, 220, 220)
    assert recovery.amcl_pose(recovery.token, 230, 230)
