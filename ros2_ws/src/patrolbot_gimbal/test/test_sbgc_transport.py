"""Serial transport tests, exercised against a real pty pair.

A pty gives genuine file-descriptor reads and writes without any gimbal
attached, so framing, reconnection and error handling are tested for real
rather than against a mock that agrees with whatever the code does.

No ROS.
"""

import os
import pty
import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from patrolbot_gimbal import sbgc_protocol as p          # noqa: E402
from patrolbot_gimbal.sbgc_transport import SbgcTransport  # noqa: E402


@pytest.fixture
def link():
    """A transport bound to one end of a pty, and the raw other end."""
    master, slave = pty.openpty()
    transport = SbgcTransport()
    transport.open(os.ttyname(slave), baud=115200)
    yield transport, master
    transport.close()
    for fd in (master, slave):
        try:
            os.close(fd)
        except OSError:
            pass


def _read_available(fd, expected):
    return os.read(fd, expected)


# ------------------------------------------------------------------- basics --

def test_open_reports_connected(link):
    transport, _ = link
    assert transport.connected


def test_opening_a_missing_device_fails_cleanly():
    transport = SbgcTransport()
    with pytest.raises(IOError):
        transport.open('/dev/tty-patrolbot-does-not-exist', baud=115200)
    assert not transport.connected


def test_send_puts_the_exact_frame_on_the_wire(link):
    transport, master = link
    transport.send(p.CMD_GET_ANGLES, b'')
    assert _read_available(master, 5) == bytes([0x3E, 0x49, 0x00, 0x49, 0x00])


def test_rate_command_frame_matches_the_protocol(link):
    transport, master = link
    payload = p.build_rate_payload(0.0, 0.0, 30.0)
    transport.send(p.CMD_CONTROL, payload)

    frame = _read_available(master, 20)
    assert frame == p.encode_frame(p.CMD_CONTROL, payload)
    # 30 deg/s -> 246 units, on the YAW axis
    assert frame[4:7] == bytes([1, 1, 1])                 # all axes SPEED mode
    assert int.from_bytes(frame[15:17], 'little', signed=True) == 246


def test_last_tx_is_recorded_for_diagnostics(link):
    transport, master = link
    transport.send(p.CMD_MOTORS_ON, b'')
    _read_available(master, 5)
    assert transport.last_tx == bytes([0x3E, 0x4D, 0x00, 0x4D, 0x00])


# ----------------------------------------------------------------- receive --

def test_poll_decodes_a_frame_written_by_the_peer(link):
    transport, master = link
    payload = bytearray(18)
    payload[0:2] = (2048).to_bytes(2, 'little', signed=True)
    os.write(master, p.encode_frame(p.CMD_GET_ANGLES, bytes(payload)))

    frames = transport.poll(timeout_s=0.5)
    assert len(frames) == 1
    command_id, body = frames[0]
    assert command_id == p.CMD_GET_ANGLES
    assert p.parse_angles(body).imu_deg[p.ROLL] == pytest.approx(45.0)


def test_poll_returns_empty_when_nothing_arrives(link):
    transport, _ = link
    assert transport.poll(timeout_s=0.05) == []


def test_leading_garbage_does_not_prevent_decoding(link):
    transport, master = link
    os.write(master, b'\x00\x99\xfe')
    os.write(master, p.encode_frame(p.CMD_MOTORS_ON, b''))
    assert transport.poll(timeout_s=0.5) == [(p.CMD_MOTORS_ON, b'')]


def test_a_corrupted_frame_is_dropped_not_surfaced(link):
    transport, master = link
    frame = bytearray(p.encode_frame(p.CMD_MOTORS_ON, b'\x01'))
    frame[-1] ^= 0xFF
    os.write(master, bytes(frame))
    assert transport.poll(timeout_s=0.2) == []


# -------------------------------------------------------------- disconnect --

def test_send_after_close_raises_rather_than_silently_succeeding(link):
    transport, _ = link
    transport.close()
    assert not transport.connected
    with pytest.raises(IOError):
        transport.send(p.CMD_MOTORS_ON, b'')


def test_peer_hangup_is_reported_as_disconnected(link):
    """Unplugging the USB adapter must surface, not hang."""
    transport, master = link
    os.close(master)
    transport.poll(timeout_s=0.1)          # notices the hangup
    assert not transport.connected
