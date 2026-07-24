"""SimpleBGC 32-bit serial protocol tests.

The CMD_CONTROL cases assert byte-for-byte equality with the payload hex dumps
published by BaseCam in "SBGC32 API cmd examples", and the framing case uses
the worked example from the protocol specification itself. If these pass, the
frames this package puts on the wire are the frames the vendor documents.

Pure logic: no ROS, no serial port, no hardware.
"""

import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from patrolbot_gimbal import sbgc_protocol as p  # noqa: E402


# --------------------------------------------------------------- unit scaling --

def test_angle_units_match_vendor_worked_values():
    # 45 deg / 0.02197265625 = 2048 exactly
    assert p.deg_to_units(45.0) == 2048
    assert p.deg_to_units(-90.0) == -4096
    assert p.deg_to_units(-45.0) == -2048
    assert p.deg_to_units(0.0) == 0


def test_speed_units_match_vendor_worked_values():
    # 5 deg/s -> 40.96 -> 41;  90 deg/s -> 737.3 -> 737
    assert p.degs_to_units(5.0) == 41
    assert p.degs_to_units(90.0) == 737
    assert p.degs_to_units(0.0) == 0


def test_unit_conversion_round_trips():
    assert p.units_to_deg(2048) == pytest.approx(45.0)
    assert p.units_to_degs(737) == pytest.approx(90.0, abs=0.1)


def test_units_saturate_instead_of_wrapping():
    """A wrapped int16 would silently command the opposite direction."""
    assert p.deg_to_units(1e9) == 32767
    assert p.deg_to_units(-1e9) == -32768
    assert p.degs_to_units(1e9) == 32767


def test_rounding_is_half_away_from_zero():
    assert p.degs_to_units(-5.0) == -41


# ------------------------------------------------------ CMD_CONTROL payloads --

def _control(mode, speed, angle):
    return p.build_control_payload(mode, speed, angle)


def test_vendor_yaw_45_degrees_default_speed():
    """Vendor: rotate YAW 45 deg right with the default speed."""
    want = bytes([
        0x00, 0x00, 0x02,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x08,
    ])
    got = _control(
        [p.MODE_NO_CONTROL, p.MODE_NO_CONTROL, p.MODE_ANGLE],
        [0, 0, 0],
        [0, 0, p.deg_to_units(45.0)],
    )
    assert got == want


def test_vendor_pitch_90_degrees_at_5_deg_per_second():
    """Vendor: rotate PITCH 90 deg up at 5 deg/sec."""
    want = bytes([
        0x00, 0x02, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x29, 0x00, 0x00, 0xF0,
        0x00, 0x00, 0x00, 0x00,
    ])
    got = _control(
        [p.MODE_NO_CONTROL, p.MODE_ANGLE, p.MODE_NO_CONTROL],
        [0, p.degs_to_units(5.0), 0],
        [0, p.deg_to_units(-90.0), 0],
    )
    assert got == want


def test_vendor_pitch_relative_to_frame_as_auto_task():
    """Vendor: PITCH 45 deg up relative to frame at 90 deg/sec, auto task."""
    want = bytes([
        0x00, 0x45, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0xE1, 0x02, 0x00, 0xF8,
        0x00, 0x00, 0x00, 0x00,
    ])
    got = _control(
        [p.MODE_NO_CONTROL,
         p.MODE_ANGLE_REL_FRAME | p.CTRL_FLAG_AUTO_TASK,
         p.MODE_NO_CONTROL],
        [0, p.degs_to_units(90.0), 0],
        [0, p.deg_to_units(-45.0), 0],
    )
    assert got == want


def test_vendor_home_position_mode_bytes():
    """Vendor home: roll to horizon, pitch and yaw neutral to frame."""
    assert p.build_home_payload() == bytes([0x42, 0x45, 0x45] + [0] * 12)


def test_vendor_level_position_mode_bytes():
    """Vendor level: roll and pitch to horizon, yaw neutral to frame."""
    assert p.build_level_payload() == bytes([0x42, 0x42, 0x45] + [0] * 12)


def test_control_payload_is_always_15_bytes():
    assert len(_control([0, 0, 0], [0, 0, 0], [0, 0, 0])) == 15


# ------------------------------------------------------------------- framing --

def test_spec_example_read_profile2_frame():
    """Protocol spec worked example: 3E 52 01 53 01 01."""
    assert p.encode_frame(0x52, bytes([0x01])) == bytes(
        [0x3E, 0x52, 0x01, 0x53, 0x01, 0x01])


def test_zero_payload_frame():
    """CMD_GET_ANGLES (#73 = 0x49) with no payload."""
    assert p.encode_frame(p.CMD_GET_ANGLES, b'') == bytes(
        [0x3E, 0x49, 0x00, 0x49, 0x00])


def test_checksums_wrap_modulo_256():
    payload = bytes([0xFF] * 200)
    frame = p.encode_frame(200, payload)
    assert frame[3] == (200 + 200) & 0xFF          # header checksum
    assert frame[-1] == (200 * 0xFF) & 0xFF        # body checksum


def test_oversized_payload_is_rejected():
    with pytest.raises(ValueError):
        p.encode_frame(1, bytes(256))


# -------------------------------------------------------------------- parser --

def test_parser_round_trips_a_frame():
    frame = p.encode_frame(42, bytes([0xAA, 0xBB, 0xCC]))
    parser = p.Parser()
    out = [f for b in frame for f in parser.push(bytes([b]))]
    assert out == [(42, bytes([0xAA, 0xBB, 0xCC]))]


def test_parser_resynchronises_after_garbage():
    frame = p.encode_frame(7, bytes([0x10, 0x20]))
    parser = p.Parser()
    parser.push(bytes([0x00, 0x99, 0x12, 0xFE, 0x77]))
    assert parser.push(frame) == [(7, bytes([0x10, 0x20]))]


def test_parser_rejects_a_corrupted_body_checksum():
    frame = bytearray(p.encode_frame(7, bytes([0x10, 0x20])))
    frame[-1] ^= 0xFF
    parser = p.Parser()
    assert parser.push(bytes(frame)) == []


def test_parser_rejects_a_corrupted_header_checksum():
    frame = bytearray(p.encode_frame(7, bytes([0x10, 0x20])))
    frame[3] ^= 0xFF
    parser = p.Parser()
    assert parser.push(bytes(frame)) == []


def test_parser_handles_two_frames_in_one_chunk():
    a = p.encode_frame(1, bytes([0x01]))
    b = p.encode_frame(2, bytes([0x02]))
    parser = p.Parser()
    assert parser.push(a + b) == [(1, bytes([0x01])), (2, bytes([0x02]))]


# ------------------------------------------------------------- angle decode --

def test_get_angles_payload_decodes():
    payload = bytearray(18)
    payload[0:2] = (2048).to_bytes(2, 'little', signed=True)     # roll imu 45
    payload[6:8] = (-4096).to_bytes(2, 'little', signed=True)    # pitch imu -90
    payload[16:18] = (737).to_bytes(2, 'little', signed=True)    # yaw tgt speed

    a = p.parse_angles(bytes(payload))
    assert a.imu_deg[p.ROLL] == pytest.approx(45.0)
    assert a.imu_deg[p.PITCH] == pytest.approx(-90.0)
    assert a.target_deg_s[p.YAW] == pytest.approx(90.0, abs=0.1)


def test_get_angles_wrong_length_is_rejected_not_misparsed():
    """The response layout could not be byte-verified, so length is checked."""
    with pytest.raises(ValueError):
        p.parse_angles(bytes(17))
