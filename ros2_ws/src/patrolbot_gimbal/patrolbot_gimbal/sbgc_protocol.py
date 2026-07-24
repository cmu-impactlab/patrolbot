"""SimpleBGC 32-bit (BaseCam / AlexMos) serial protocol codec.

Pure encoding and decoding: no serial port, no ROS. That separation is what
lets the frame layout be tested byte-for-byte against the vendor's published
hex dumps without any hardware present.

Protocol version 1 is used deliberately. Version 2 (start byte '$', CRC16)
needs firmware 2.68b0+, and the board locks to v2 permanently once it sees a
single v2 message; v1 is understood by every 32-bit firmware.

Reference: "SBGC32 Serial API Protocol Specification" v2.6, BaseCam
Electronics. Frame layout and unit scaling verified against the vendor's
"SBGC32 API cmd examples" worked hex dumps.
"""

from dataclasses import dataclass, field
from typing import List, Tuple

START_BYTE = 0x3E          # '>'
MAX_PAYLOAD = 255

# --- command IDs ---
CMD_REALTIME_DATA_4 = 25
CMD_CONTROL = 67
CMD_EXECUTE_MENU = 69
CMD_GET_ANGLES = 73
CMD_MOTORS_ON = 77
CMD_BOARD_INFO = 86
CMD_MOTORS_OFF = 109
CMD_ERROR = 255

# --- per-axis control modes (low bits) ---
MODE_NO_CONTROL = 0
MODE_SPEED = 1
MODE_ANGLE = 2
MODE_SPEED_ANGLE = 3
MODE_RC = 4
MODE_ANGLE_REL_FRAME = 5

# --- control mode flags (high bits, OR'd onto a mode) ---
CTRL_FLAG_AUTO_TASK = 0x40
CTRL_FLAG_FORCE_RC_SPEED = 0x80

# --- menu actions ---
MENU_MOTOR_TOGGLE = 10
MENU_HOME_POSITION = 18

# --- axis order used by every 3-element sequence here ---
ROLL, PITCH, YAW = 0, 1, 2
NUM_AXES = 3

# --- wire units, confirmed against the vendor worked values ---
#   45 deg   -> 2048     (45 / 0.02197265625)
#    5 deg/s -> 41       (5  / 0.1220740379 = 40.96)
#   90 deg/s -> 737      (90 / 0.1220740379 = 737.3)
ANGLE_UNIT_DEG = 0.02197265625      # 360 / 16384
SPEED_UNIT_DEG_S = 0.1220740379

_INT16_MIN = -32768
_INT16_MAX = 32767


# ------------------------------------------------------------------- units --

def _round_half_away_from_zero(value: float) -> float:
    """Match the vendor's rounding: 40.96 -> 41, -40.96 -> -41."""
    if value < 0:
        return -int(-value + 0.5)
    return int(value + 0.5)


def _saturate_int16(value: float) -> int:
    """Clamp rather than wrap.

    A wrapped int16 would command the opposite direction at full speed, which
    is the worst possible failure mode for a value that drives a motor.
    """
    return int(max(_INT16_MIN, min(_INT16_MAX, value)))


def deg_to_units(degrees: float) -> int:
    """Convert degrees to SBGC angle units."""
    return _saturate_int16(_round_half_away_from_zero(degrees / ANGLE_UNIT_DEG))


def degs_to_units(deg_per_s: float) -> int:
    """Convert degrees/second to SBGC speed units."""
    return _saturate_int16(
        _round_half_away_from_zero(deg_per_s / SPEED_UNIT_DEG_S))


def units_to_deg(units: int) -> float:
    """Convert SBGC angle units to degrees."""
    return units * ANGLE_UNIT_DEG


def units_to_degs(units: int) -> float:
    """Convert SBGC speed units to degrees/second."""
    return units * SPEED_UNIT_DEG_S


# ----------------------------------------------------------------- framing --

def _checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def encode_frame(command_id: int, payload: bytes = b'') -> bytes:
    """Build a protocol-v1 frame.

    Layout: start, command, length, header checksum, payload, body checksum.
    Header checksum is (command + length) mod 256; body checksum is the sum of
    the payload bytes mod 256.
    """
    if not 0 <= command_id <= 255:
        raise ValueError(f'command id out of range: {command_id}')
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(
            f'payload too long: {len(payload)} > {MAX_PAYLOAD}')

    header = bytes([
        START_BYTE,
        command_id,
        len(payload),
        (command_id + len(payload)) & 0xFF,
    ])
    return header + payload + bytes([_checksum(payload)])


def _pack_int16(value: int) -> bytes:
    return int(value).to_bytes(2, 'little', signed=True)


def build_control_payload(mode, speed, angle) -> bytes:
    """Build the 15-byte CMD_CONTROL payload.

    Each sequence is [ROLL, PITCH, YAW]; speed and angle are already in wire
    units. Layout is three mode bytes, then speed/angle interleaved per axis.
    """
    for name, seq in (('mode', mode), ('speed', speed), ('angle', angle)):
        if len(seq) != NUM_AXES:
            raise ValueError(f'{name} must have {NUM_AXES} entries')

    out = bytearray(bytes(int(m) & 0xFF for m in mode))
    for axis in range(NUM_AXES):
        out += _pack_int16(speed[axis])
        out += _pack_int16(angle[axis])
    return bytes(out)


def build_home_payload() -> bytes:
    """Vendor home position: roll to horizon, pitch and yaw neutral to frame.

    Reproduces the documented mode bytes 0x42 0x45 0x45 with all values zero.
    """
    return build_control_payload(
        [MODE_ANGLE | CTRL_FLAG_AUTO_TASK,
         MODE_ANGLE_REL_FRAME | CTRL_FLAG_AUTO_TASK,
         MODE_ANGLE_REL_FRAME | CTRL_FLAG_AUTO_TASK],
        [0, 0, 0], [0, 0, 0])


def build_level_payload() -> bytes:
    """Vendor level position: roll and pitch to horizon, yaw neutral to frame.

    Reproduces the documented mode bytes 0x42 0x42 0x45.
    """
    return build_control_payload(
        [MODE_ANGLE | CTRL_FLAG_AUTO_TASK,
         MODE_ANGLE | CTRL_FLAG_AUTO_TASK,
         MODE_ANGLE_REL_FRAME | CTRL_FLAG_AUTO_TASK],
        [0, 0, 0], [0, 0, 0])


def build_rate_payload(roll_deg_s: float, pitch_deg_s: float,
                       yaw_deg_s: float) -> bytes:
    """Build a speed-mode payload from rates in degrees/second.

    Zero rates are sent in SPEED mode rather than NO_CONTROL: SPEED with rate
    zero is an active hold, whereas NO_CONTROL relinquishes the axis and lets
    whatever was previously commanded persist.
    """
    return build_control_payload(
        [MODE_SPEED] * NUM_AXES,
        [degs_to_units(roll_deg_s),
         degs_to_units(pitch_deg_s),
         degs_to_units(yaw_deg_s)],
        [0, 0, 0])


def build_angle_payload(roll_deg: float, pitch_deg: float, yaw_deg: float,
                        slew_deg_s: float = 0.0,
                        relative_to_frame: bool = True) -> bytes:
    """Build an absolute-angle payload.

    A slew of zero tells the board to use its own configured speed.
    """
    mode = MODE_ANGLE_REL_FRAME if relative_to_frame else MODE_ANGLE
    slew = degs_to_units(slew_deg_s)
    return build_control_payload(
        [mode] * NUM_AXES,
        [slew] * NUM_AXES,
        [deg_to_units(roll_deg),
         deg_to_units(pitch_deg),
         deg_to_units(yaw_deg)])


# ------------------------------------------------------------------ parser --

class Parser:
    """Incremental frame parser.

    Feed it bytes as they arrive. It resynchronises on the start byte and
    validates both checksums, discarding anything malformed rather than
    surfacing a half-decoded frame.
    """

    def __init__(self):
        self._buf = bytearray()

    def reset(self):
        self._buf.clear()

    def push(self, data: bytes) -> List[Tuple[int, bytes]]:
        """Consume bytes, returning every complete valid frame found.

        Each frame is (command_id, payload).
        """
        self._buf.extend(data)
        frames = []

        while True:
            # Resynchronise: drop anything before the next start byte.
            start = self._buf.find(START_BYTE)
            if start < 0:
                self._buf.clear()
                break
            if start > 0:
                del self._buf[:start]

            if len(self._buf) < 4:
                break                       # header incomplete

            command_id = self._buf[1]
            length = self._buf[2]

            if self._buf[3] != (command_id + length) & 0xFF:
                del self._buf[0]            # bad header; rescan from next byte
                continue

            total = 4 + length + 1
            if len(self._buf) < total:
                break                       # body incomplete

            payload = bytes(self._buf[4:4 + length])
            if self._buf[4 + length] == _checksum(payload):
                frames.append((command_id, payload))
            # Consume the frame either way; a bad body checksum is dropped.
            del self._buf[:total]

        return frames


# ------------------------------------------------------------ angle decode --

@dataclass
class Angles:
    """Decoded CMD_GET_ANGLES response, in degrees and degrees/second."""

    imu_deg: List[float] = field(default_factory=lambda: [0.0] * NUM_AXES)
    target_deg: List[float] = field(default_factory=lambda: [0.0] * NUM_AXES)
    target_deg_s: List[float] = field(default_factory=lambda: [0.0] * NUM_AXES)


def parse_angles(payload: bytes) -> Angles:
    """Decode an 18-byte CMD_GET_ANGLES payload.

    The length is checked rather than assumed. Only the request side of this
    command could be byte-verified against the vendor examples, so a payload
    of unexpected length is reported as an error instead of being misparsed
    into plausible-looking nonsense.
    """
    if len(payload) != 18:
        raise ValueError(
            f'CMD_GET_ANGLES payload must be 18 bytes, got {len(payload)}')

    out = Angles()
    for axis in range(NUM_AXES):
        base = axis * 6
        imu = int.from_bytes(payload[base:base + 2], 'little', signed=True)
        target = int.from_bytes(payload[base + 2:base + 4], 'little', signed=True)
        speed = int.from_bytes(payload[base + 4:base + 6], 'little', signed=True)
        out.imu_deg[axis] = units_to_deg(imu)
        out.target_deg[axis] = units_to_deg(target)
        out.target_deg_s[axis] = units_to_degs(speed)
    return out
