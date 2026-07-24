"""Serial transport for the SimpleBGC controller.

Isolated from both ROS and the frame codec so it can be exercised against a
pty pair in tests: real file-descriptor reads and writes, no hardware, no
mock that agrees with whatever the code happens to do.

Only this module and the node that owns it touch the serial port.
"""

import select
from typing import List, Optional, Tuple

import serial

from . import sbgc_protocol as protocol


class SbgcTransport:
    """A framed, non-blocking link to a SimpleBGC board."""

    def __init__(self):
        self._port: Optional[serial.Serial] = None
        self._parser = protocol.Parser()
        self.last_tx: bytes = b''
        self.last_error: str = ''

    # ------------------------------------------------------------ lifecycle --

    @property
    def connected(self) -> bool:
        return self._port is not None and self._port.is_open

    def open(self, device: str, baud: int = 115200, timeout_s: float = 0.0):
        """Open the port in raw 8N1 with no flow control.

        Raises IOError if the device is missing or cannot be configured, so a
        typo in the port name fails loudly instead of leaving a dead link that
        silently swallows every command.
        """
        self.close()
        try:
            self._port = serial.Serial(
                port=device,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout_s,
                write_timeout=1.0,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False,
            )
        except (serial.SerialException, OSError) as exc:
            self._port = None
            self.last_error = f'open({device}): {exc}'
            raise IOError(self.last_error) from exc

        self._parser.reset()
        self.last_error = ''
        try:
            self._port.reset_input_buffer()
            self._port.reset_output_buffer()
        except (serial.SerialException, OSError):
            pass    # a pty may not support flushing; not fatal

    def close(self):
        if self._port is not None:
            try:
                self._port.close()
            except (serial.SerialException, OSError):
                pass
        self._port = None

    def _disconnect(self, reason: str):
        self.last_error = reason
        self.close()

    # -------------------------------------------------------------- send --

    def send(self, command_id: int, payload: bytes = b'') -> None:
        """Encode and write one frame.

        Raises IOError when the link is down, rather than returning a status
        that a caller might not check. A command that did not reach the board
        must never look like one that did.
        """
        if not self.connected:
            raise IOError(self.last_error or 'serial port is not open')

        frame = protocol.encode_frame(command_id, payload)
        self.last_tx = frame
        try:
            self._port.write(frame)
        except (serial.SerialException, OSError) as exc:
            self._disconnect(f'write failed: {exc}')
            raise IOError(self.last_error) from exc

    # ------------------------------------------------------------ receive --

    def poll(self, timeout_s: float = 0.0) -> List[Tuple[int, bytes]]:
        """Read whatever is available and return complete valid frames.

        Blocks up to timeout_s for the first byte, then drains without
        blocking again. A peer hangup marks the link disconnected instead of
        spinning on a dead descriptor.
        """
        if not self.connected:
            return []

        frames: List[Tuple[int, bytes]] = []
        try:
            readable, _, errored = select.select(
                [self._port.fileno()], [], [self._port.fileno()], timeout_s)
            if errored:
                self._disconnect('serial device reported an error')
                return frames
            if not readable:
                return frames

            chunk = self._port.read(4096)
            if not chunk:
                # readable but empty means EOF: the peer went away.
                self._disconnect('serial device disconnected')
                return frames

            frames.extend(self._parser.push(chunk))
        except (serial.SerialException, OSError) as exc:
            self._disconnect(f'read failed: {exc}')

        return frames
