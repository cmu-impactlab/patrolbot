"""Strict parser and formatter for zero-motion PatrolBot hardware controls."""

from dataclasses import dataclass
from typing import Optional
from uuid import UUID


COMMAND_CHARGE_RELEASE = 'CHARGE_RELEASE'
COMMAND_MOTOR_ENABLE = 'MOTOR_ENABLE'
COMMAND_DOCK_RESET = 'DOCK_RESET'
# UNDOCK and UNDOCK_CANCEL are acknowledged with the same ACK frame as the
# zero-motion controls, so they belong here even though their progress and
# terminal frames live in undock_protocol.
COMMAND_UNDOCK = 'UNDOCK'
COMMAND_UNDOCK_CANCEL = 'UNDOCK_CANCEL'
COMMANDS = frozenset((
    COMMAND_CHARGE_RELEASE, COMMAND_MOTOR_ENABLE, COMMAND_DOCK_RESET,
    COMMAND_UNDOCK, COMMAND_UNDOCK_CANCEL,
))
# Commands whose payload is a bare request id.
SIMPLE_COMMANDS = frozenset((
    COMMAND_CHARGE_RELEASE, COMMAND_MOTOR_ENABLE, COMMAND_DOCK_RESET,
))


@dataclass(frozen=True)
class ControlReply:
    """One parsed ACK or terminal RESULT from the SBC."""

    kind: str
    command: str
    request_id: str
    accepted: bool = False
    success: bool = False
    code: int = 0
    message: str = ''
    charge_state: int = -1
    motors_enabled: bool = False
    estop_pressed: bool = False


def valid_request_id(value: str) -> bool:
    """Accept canonical UUIDs only, preventing ambiguous protocol tokens."""
    try:
        return str(UUID(value)) == value
    except (AttributeError, TypeError, ValueError):
        return False


def format_request(command: str, request_id: str) -> bytes:
    """Format an idempotent, newline-terminated hardware control request."""
    if command not in SIMPLE_COMMANDS:
        raise ValueError(f'unsupported command: {command}')
    if not valid_request_id(request_id):
        raise ValueError('request_id must be a canonical UUID')
    return f'{command}:{request_id}\n'.encode('ascii')


def motor_enable_allowed(dock_state: str, estop_pressed: bool) -> bool:
    """Permit a motor enable only where dock contact is impossible.

    MOTOR_ENABLE is a decoupled maintenance command: it never releases charging
    and never authorizes motion. There is no post-release window and no
    charge-state exception, because enabling the motors in order to leave the
    dock is the UNDOCK operation's job. The SBC independently enforces the same
    rule against its own dock observer.
    """
    if estop_pressed:
        return False
    return dock_state in ('CLEAR_CONFIRMED', 'UNKNOWN')


def parse_reply(line: str) -> Optional[ControlReply]:
    """Parse an SBC ACK/RESULT, returning None for non-control telemetry."""
    fields = line.strip().split(':')
    if not fields or fields[0] not in ('ACK', 'RESULT'):
        return None
    if fields[0] == 'ACK':
        if len(fields) != 4:
            raise ValueError('malformed ACK')
        _, command, request_id, status = fields
        if command not in COMMANDS or not valid_request_id(request_id):
            raise ValueError('invalid ACK identity')
        if status not in ('ACCEPTED', 'REPLAYED'):
            raise ValueError('invalid ACK status')
        return ControlReply(
            kind='ACK', command=command, request_id=request_id,
            accepted=True, message=status.lower())

    if len(fields) != 9:
        raise ValueError('malformed RESULT')
    _, command, request_id, status, code, message, charge, motors, estop = fields
    if command not in COMMANDS or not valid_request_id(request_id):
        raise ValueError('invalid RESULT identity')
    if status not in ('OK', 'ERROR'):
        raise ValueError('invalid RESULT status')
    try:
        code_value = int(code)
        charge_value = int(charge)
        motor_value = bool(int(motors))
        estop_value = bool(int(estop))
    except ValueError as exc:
        raise ValueError('non-numeric RESULT state') from exc
    return ControlReply(
        kind='RESULT', command=command, request_id=request_id,
        accepted=True, success=status == 'OK', code=code_value,
        message=message.replace('_', ' '), charge_state=charge_value,
        motors_enabled=motor_value, estop_pressed=estop_value)
