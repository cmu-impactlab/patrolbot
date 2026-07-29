"""Offline tests for the SBC hardware-control wire contract."""

from patrolbot_bridge.control_protocol import (
    COMMAND_CHARGE_RELEASE,
    COMMAND_DOCK_RESET,
    COMMAND_MOTOR_ENABLE,
    COMMAND_UNDOCK,
    format_request,
    motor_enable_allowed,
    parse_reply,
    valid_request_id,
)
import pytest


REQUEST_ID = '123e4567-e89b-12d3-a456-426614174000'


@pytest.mark.parametrize(
    'command',
    [COMMAND_CHARGE_RELEASE, COMMAND_MOTOR_ENABLE, COMMAND_DOCK_RESET])
def test_request_is_exactly_one_zero_motion_line(command):
    payload = format_request(command, REQUEST_ID)
    assert payload == f'{command}:{REQUEST_ID}\n'.encode('ascii')
    assert b'DRIVE' not in payload


def test_duplicate_ack_and_terminal_result_are_parseable():
    ack = parse_reply(f'ACK:CHARGE_RELEASE:{REQUEST_ID}:REPLAYED')
    assert ack.accepted
    result = parse_reply(
        f'RESULT:CHARGE_RELEASE:{REQUEST_ID}:OK:0:released:0:0:0')
    assert result.success
    assert result.charge_state == 0
    assert not result.motors_enabled


def test_motor_enable_is_decoupled_from_charging():
    """Permitted only where dock contact is impossible, never by a timer."""
    assert motor_enable_allowed('CLEAR_CONFIRMED', False)
    assert motor_enable_allowed('UNKNOWN', False)
    assert not motor_enable_allowed('DOCKED_CONFIRMED', False)
    assert not motor_enable_allowed('DEPARTING', False)
    assert not motor_enable_allowed('FAULT_STOPPED', False)
    # E-stop always refuses, and there is no post-release back door left.
    assert not motor_enable_allowed('CLEAR_CONFIRMED', True)


def test_undock_is_not_a_simple_request_command():
    """UNDOCK carries parameters, so it must not go through format_request."""
    with pytest.raises(ValueError):
        format_request(COMMAND_UNDOCK, REQUEST_ID)


@pytest.mark.parametrize(
    'value', ['', 'not-a-uuid', REQUEST_ID.upper(), REQUEST_ID + ':extra'])
def test_invalid_request_ids_are_rejected(value):
    assert not valid_request_id(value)
    with pytest.raises(ValueError):
        format_request(COMMAND_MOTOR_ENABLE, value)


@pytest.mark.parametrize(
    'line', [
        f'ACK:BAD:{REQUEST_ID}:ACCEPTED',
        f'RESULT:MOTOR_ENABLE:{REQUEST_ID}:OK:not-int:enabled:0:1:0',
        f'RESULT:MOTOR_ENABLE:{REQUEST_ID}:MAYBE:0:enabled:0:1:0',
    ],
)
def test_malformed_replies_fail_closed(line):
    with pytest.raises(ValueError):
        parse_reply(line)


def test_telemetry_is_not_misclassified_as_control():
    assert parse_reply('AUX:SONAR=|BATT=24.0,80,0,25|FLAGS=0,0,0,0,0') is None
