"""
Strict parser and formatter for the SBC-owned UNDOCK operation.

Mirrors hardware/patrolbot_hw_server/protocol.cpp. Pure and ROS-free so it can
be unit tested without a robot, a socket or an rclpy context.

UNDOCK_EVENT and UNDOCK_RESULT use distinct leading verbs rather than
overloading RESULT. control_protocol.parse_reply returns None for an unknown
leading verb instead of raising, so a bridge that predates these frames ignores
them rather than logging a parse error on every one.
"""

from dataclasses import dataclass, field
from typing import Optional
from uuid import UUID

COMMAND_UNDOCK = 'UNDOCK'
COMMAND_UNDOCK_CANCEL = 'UNDOCK_CANCEL'
COMMAND_DOCK_RESET = 'DOCK_RESET'

EVENT_PREFIX = 'UNDOCK_EVENT'
RESULT_PREFIX = 'UNDOCK_RESULT'
HELLO_PREFIX = 'HELLO'

# Field counts are fixed. Anything else is a malformed frame, not a frame with
# optional trailing fields.
EVENT_FIELDS = 12
RESULT_FIELDS = 17

# Mirrors patrolbot::UndockCode.
OK = 0
INVALID_REQUEST = 1
LINK_UNAVAILABLE = 2
PRECONDITION_FAILED = 3
BUSY = 4
TIMEOUT = 5
SESSION_CHANGED = 6
PROTOCOL_ERROR = 7
RELEASE_FAILED = 8
MOTOR_STATE_FAILED = 9
MOTION_FAILED = 10
SENSOR_BLOCKED = 11
SAFETY_FAULT = 12
CANCELED = 13
HARDWARE_PROFILE_UNCOMMISSIONED = 14

DOCK_STATES = frozenset(
    (
        'UNKNOWN',
        'DOCKED_CONFIRMED',
        'DEPARTING',
        'CLEAR_CONFIRMED',
        'FAULT_STOPPED',
    )
)
UNDOCK_PHASES = frozenset(
    (
        'IDLE',
        'PREPARING_UNDOCK',
        'RELEASING',
        'ENABLING_MOTORS',
        'BACKING_OUT',
        'VERIFYING_CLEARANCE',
        'HANDOFF_READY',
        'STOPPING',
        'MANUAL_TURN',
        'MANUAL_SETTLING',
    )
)

# Feature advertised in the HELLO banner. Without it the bridge refuses undock
# goals rather than sending a command the SBC will never answer.
UNDOCK_FEATURE = 'UNDOCK'


@dataclass(frozen=True)
class UndockEvent:
    """One phase-transition frame from the SBC."""

    request_id: str
    phase: int
    phase_name: str
    distance_traveled: float
    lateral_error: float
    heading_error: float
    charge_state: int
    motors_enabled: bool
    minimum_rear_range: float
    release_attempts: int
    dock_state: str


@dataclass(frozen=True)
class UndockResult:
    """The terminal frame for one undock operation."""

    request_id: str
    success: bool
    code: int
    message: str
    phase: int
    phase_name: str
    distance_traveled: float
    lateral_error: float
    heading_error: float
    charge_state: int
    motors_enabled: bool
    estop_pressed: bool
    minimum_rear_range: float
    release_attempts: int
    dock_state: str
    physically_clear: bool


@dataclass(frozen=True)
class ServerHello:
    """Capability banner sent when the SBC accepts a new client."""

    protocol: int
    features: frozenset = field(default_factory=frozenset)
    odom_epoch: Optional[str] = None

    def supports_undock(self) -> bool:
        """Report whether this SBC can execute the hardware undock at all."""
        return UNDOCK_FEATURE in self.features


def valid_request_id(value: str) -> bool:
    """Accept canonical UUIDs only, preventing ambiguous protocol tokens."""
    try:
        return str(UUID(value)) == value
    except (AttributeError, TypeError, ValueError):
        return False


def format_undock_request(
    request_id: str, clearance_m: float, speed_mps: float, timeout_s: float, validation_mode: bool
) -> bytes:
    """
    Format an idempotent UNDOCK request.

    Values are sent as given. The SBC rejects anything outside its hard caps
    rather than clamping it, so a bad goal fails loudly instead of quietly
    becoming a different goal.
    """
    if not valid_request_id(request_id):
        raise ValueError('request_id must be a canonical UUID')
    for name, value in (('clearance', clearance_m), ('speed', speed_mps), ('timeout', timeout_s)):
        if value != value or value in (float('inf'), float('-inf')):
            raise ValueError(f'{name} must be finite')
        if value <= 0.0:
            raise ValueError(f'{name} must be positive')
    return (
        f'{COMMAND_UNDOCK}:{request_id}:{clearance_m:.4f}:{speed_mps:.4f}:'
        f'{timeout_s:.3f}:{1 if validation_mode else 0}\n'
    ).encode('ascii')


def format_undock_cancel(target_request_id: str) -> bytes:
    """
    Format a cancellation naming the operation being cancelled.

    The payload is the target operation's id, not a new request id. The SBC
    dispatches this outside its request tracker, because the tracker keys on the
    command name and would otherwise call a cancel of a pending UNDOCK "busy".
    """
    if not valid_request_id(target_request_id):
        raise ValueError('target_request_id must be a canonical UUID')
    return f'{COMMAND_UNDOCK_CANCEL}:{target_request_id}\n'.encode('ascii')


def format_manual_undock_request(request_id: str) -> bytes:
    """Fixed local-controller profile; requires MANUAL_UNDOCK_V1 in HELLO."""
    return format_undock_request(request_id, .53, .10, 45., False).replace(
        b'UNDOCK:', b'MANUAL_UNDOCK:', 1)


def format_dock_reset(request_id: str) -> bytes:
    """Format an explicit recovery from the SBC's FAULT_STOPPED dock state."""
    if not valid_request_id(request_id):
        raise ValueError('request_id must be a canonical UUID')
    return f'{COMMAND_DOCK_RESET}:{request_id}\n'.encode('ascii')


def parse_hello(line: str) -> Optional[ServerHello]:
    """Parse the capability banner, returning None for any other line."""
    fields = line.strip().split(':')
    if not fields or fields[0] != HELLO_PREFIX:
        return None
    protocol = 0
    features = set()
    odom_epoch = None
    for item in fields[1:]:
        if '=' not in item:
            raise ValueError('malformed HELLO field')
        key, value = item.split('=', 1)
        if key == 'PROTOCOL':
            try:
                protocol = int(value)
            except ValueError as exc:
                raise ValueError('non-numeric HELLO protocol') from exc
        elif key == 'ODOM_EPOCH':
            if odom_epoch is not None or not valid_request_id(value):
                raise ValueError('invalid or duplicate odometry epoch')
            odom_epoch = value
        elif key == 'FEATURES':
            features = {token for token in value.split(',') if token}
    return ServerHello(protocol=protocol, features=frozenset(features), odom_epoch=odom_epoch)


def _float(value: str, name: str) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise ValueError(f'non-numeric {name}') from exc
    if result != result or result in (float('inf'), float('-inf')):
        raise ValueError(f'non-finite {name}')
    return result


def _flag(value: str, name: str) -> bool:
    if value not in ('0', '1'):
        raise ValueError(f'malformed {name} flag')
    return value == '1'


def parse_undock_event(line: str) -> Optional[UndockEvent]:
    """Parse one UNDOCK_EVENT, returning None for any other line."""
    fields = line.strip().split(':')
    if not fields or fields[0] != EVENT_PREFIX:
        return None
    if len(fields) != EVENT_FIELDS:
        raise ValueError('malformed UNDOCK_EVENT')
    if not valid_request_id(fields[1]):
        raise ValueError('invalid UNDOCK_EVENT identity')
    if fields[3] not in UNDOCK_PHASES:
        raise ValueError('unknown undock phase')
    if fields[11] not in DOCK_STATES:
        raise ValueError('unknown dock state')
    try:
        phase = int(fields[2])
        attempts = int(fields[10])
        charge = int(fields[7])
    except ValueError as exc:
        raise ValueError('non-numeric UNDOCK_EVENT field') from exc
    return UndockEvent(
        request_id=fields[1],
        phase=phase,
        phase_name=fields[3],
        distance_traveled=_float(fields[4], 'distance'),
        lateral_error=_float(fields[5], 'lateral error'),
        heading_error=_float(fields[6], 'heading error'),
        charge_state=charge,
        motors_enabled=_flag(fields[8], 'motors'),
        minimum_rear_range=_float(fields[9], 'rear range'),
        release_attempts=attempts,
        dock_state=fields[11],
    )


def parse_undock_result(line: str) -> Optional[UndockResult]:
    """Parse one terminal UNDOCK_RESULT, returning None for any other line."""
    fields = line.strip().split(':')
    if not fields or fields[0] != RESULT_PREFIX:
        return None
    if len(fields) != RESULT_FIELDS:
        raise ValueError('malformed UNDOCK_RESULT')
    if not valid_request_id(fields[1]):
        raise ValueError('invalid UNDOCK_RESULT identity')
    if fields[2] not in ('OK', 'ERROR'):
        raise ValueError('invalid UNDOCK_RESULT status')
    if fields[6] not in UNDOCK_PHASES:
        raise ValueError('unknown undock phase')
    if fields[15] not in DOCK_STATES:
        raise ValueError('unknown dock state')
    try:
        code = int(fields[3])
        phase = int(fields[5])
        charge = int(fields[10])
        attempts = int(fields[14])
    except ValueError as exc:
        raise ValueError('non-numeric UNDOCK_RESULT field') from exc
    return UndockResult(
        request_id=fields[1],
        success=fields[2] == 'OK',
        code=code,
        # The SBC sanitises messages to a single token; restore readability.
        message=fields[4].replace('_', ' '),
        phase=phase,
        phase_name=fields[6],
        distance_traveled=_float(fields[7], 'distance'),
        lateral_error=_float(fields[8], 'lateral error'),
        heading_error=_float(fields[9], 'heading error'),
        charge_state=charge,
        motors_enabled=_flag(fields[11], 'motors'),
        estop_pressed=_flag(fields[12], 'estop'),
        minimum_rear_range=_float(fields[13], 'rear range'),
        release_attempts=attempts,
        dock_state=fields[15],
        physically_clear=_flag(fields[16], 'physically clear'),
    )
