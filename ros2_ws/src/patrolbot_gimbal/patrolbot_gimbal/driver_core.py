"""Driver decision logic: what actually reaches the gimbal, and when it stops.

Free of ROS and of serial I/O, with an injected clock, so every safety rule
here is testable deterministically.

Three rules define this layer:

1. **A command expires.** If nothing valid arrives within the timeout, the
   driver holds. A publisher that dies must not leave the camera slewing.
2. **The client is not trusted.** Limits and rate caps are enforced here, on
   the robot. A browser or a buggy node can ask for anything; it gets the
   limit.
3. **Non-finite values are a hold.** NaN and infinity would encode as garbage
   on the wire, so they are treated as no command at all.
"""

import math
import time
from dataclasses import dataclass
from typing import Optional

from . import gimbal_policy as policy
from . import sbgc_protocol as protocol


@dataclass
class GimbalStateSnapshot:
    """Plain snapshot; the node converts this into the ROS message."""

    connected: bool = False
    motors_on: bool = False
    imu_pan: float = 0.0
    imu_tilt: float = 0.0
    imu_roll: float = 0.0
    target_pan: float = 0.0
    target_tilt: float = 0.0
    target_roll: float = 0.0
    pan_min: float = 0.0
    pan_max: float = 0.0
    tilt_min: float = 0.0
    tilt_max: float = 0.0
    roll_min: float = 0.0
    roll_max: float = 0.0
    command_timeout: bool = True
    fault: str = ''


def _finite(*values) -> bool:
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


class DriverCore:
    """Turns commands into SBGC payloads, subject to the safety rules above."""

    MODE_HOLD = 0
    MODE_RATE = 1
    MODE_ANGLE = 2

    def __init__(self, cfg: Optional[policy.GimbalConfig] = None,
                 clock=time.monotonic, command_timeout_s: float = 0.5):
        self.cfg = cfg if cfg is not None else policy.default_config()
        self._clock = clock
        self._timeout = float(command_timeout_s)

        self._mode = self.MODE_HOLD
        self._pan = 0.0
        self._tilt = 0.0
        self._roll = 0.0
        self._received_at: Optional[float] = None
        self._last_tick: Optional[float] = None

        # Commanded position estimate, used to gate rates between board
        # updates. Overwritten by the board's IMU whenever a frame arrives:
        # this is a stand-in for ground truth, never a replacement.
        self._pos_pan = 0.0
        self._pos_tilt = 0.0
        self._pos_roll = 0.0

        self._state = GimbalStateSnapshot()
        self._apply_limits_to_state()
        self.command_timed_out = True

    # ---------------------------------------------------------- submission --

    def submit_rate(self, pan: float, tilt: float, roll: float):
        """Accept a rate command in degrees/second."""
        if not _finite(pan, tilt, roll):
            self.submit_invalid()
            return
        self._mode = self.MODE_RATE
        self._pan, self._tilt, self._roll = float(pan), float(tilt), float(roll)
        self._received_at = self._clock()

    def submit_angle(self, pan: float, tilt: float, roll: float):
        """Accept an absolute angle command in degrees."""
        if not _finite(pan, tilt, roll):
            self.submit_invalid()
            return
        self._mode = self.MODE_ANGLE
        self._pan, self._tilt, self._roll = float(pan), float(tilt), float(roll)
        self._received_at = self._clock()

    def submit_invalid(self):
        """Deadman released, or an unusable command: hold immediately."""
        self._mode = self.MODE_HOLD
        self._pan = self._tilt = self._roll = 0.0
        self._received_at = self._clock()

    def set_position(self, pan: float, tilt: float, roll: float):
        """Seed the position estimate, e.g. after a home."""
        self._pos_pan, self._pos_tilt, self._pos_roll = pan, tilt, roll

    # --------------------------------------------------------------- tick --

    def hold_payload(self) -> bytes:
        """An active hold: speed mode, zero rate on every axis."""
        return protocol.build_rate_payload(0.0, 0.0, 0.0)

    def _expired(self) -> bool:
        if self._received_at is None:
            return True
        return (self._clock() - self._received_at) > self._timeout

    def tick(self) -> bytes:
        """Produce the payload to send this cycle."""
        now = self._clock()
        dt = 0.0 if self._last_tick is None else max(0.0, now - self._last_tick)
        self._last_tick = now

        self.command_timed_out = self._expired()

        if self.command_timed_out or self._mode == self.MODE_HOLD:
            self._state.command_timeout = self.command_timed_out
            self._sync_state()
            return self.hold_payload()

        self._state.command_timeout = False

        if self._mode == self.MODE_RATE:
            payload = self._rate_payload(dt)
        else:
            payload = self._angle_payload()

        self._sync_state()
        return payload

    def _rate_payload(self, dt: float) -> bytes:
        c = self.cfg
        # Cap first: a client asking for 1e6 deg/s gets the configured max.
        pan = max(-c.max_pan_deg_s, min(c.max_pan_deg_s, self._pan))
        tilt = max(-c.max_tilt_deg_s, min(c.max_tilt_deg_s, self._tilt))
        roll = max(-c.max_roll_deg_s, min(c.max_roll_deg_s, self._roll))

        # Then gate against travel limits, blocking only the direction that
        # would go further out of range.
        pan = policy.gate_rate(self._pos_pan, pan, c.pan_min_deg, c.pan_max_deg)
        tilt = policy.gate_rate(self._pos_tilt, tilt, c.tilt_min_deg, c.tilt_max_deg)
        roll = policy.gate_rate(self._pos_roll, roll, c.roll_min_deg, c.roll_max_deg)

        self._pos_pan, _ = policy.clamp(
            self._pos_pan + pan * dt, c.pan_min_deg, c.pan_max_deg)
        self._pos_tilt, _ = policy.clamp(
            self._pos_tilt + tilt * dt, c.tilt_min_deg, c.tilt_max_deg)
        self._pos_roll, _ = policy.clamp(
            self._pos_roll + roll * dt, c.roll_min_deg, c.roll_max_deg)

        # Axis order on the wire is ROLL, PITCH, YAW; pan is yaw, tilt is pitch.
        return protocol.build_rate_payload(roll, tilt, pan)

    def _angle_payload(self) -> bytes:
        c = self.cfg
        pan, _ = policy.clamp(self._pan, c.pan_min_deg, c.pan_max_deg)
        tilt, _ = policy.clamp(self._tilt, c.tilt_min_deg, c.tilt_max_deg)
        roll, _ = policy.clamp(self._roll, c.roll_min_deg, c.roll_max_deg)

        self._pos_pan, self._pos_tilt, self._pos_roll = pan, tilt, roll
        return protocol.build_angle_payload(
            roll, tilt, pan, slew_deg_s=c.max_pan_deg_s,
            relative_to_frame=True)

    # ------------------------------------------------------ inbound frames --

    def on_frame(self, command_id: int, payload: bytes):
        """Consume a decoded frame from the board."""
        if command_id == protocol.CMD_GET_ANGLES:
            try:
                angles = protocol.parse_angles(payload)
            except ValueError as exc:
                # Length mismatch: report it rather than misparse. The
                # response layout is the one part of the protocol that could
                # not be byte-verified against the vendor examples.
                self._state.fault = f'bad angle frame: {exc}'
                return
            self._state.imu_roll = angles.imu_deg[protocol.ROLL]
            self._state.imu_tilt = angles.imu_deg[protocol.PITCH]
            self._state.imu_pan = angles.imu_deg[protocol.YAW]
            # Ground truth replaces the estimate.
            self._pos_roll = self._state.imu_roll
            self._pos_tilt = self._state.imu_tilt
            self._pos_pan = self._state.imu_pan
            self._state.fault = ''
        elif command_id == protocol.CMD_ERROR:
            self._state.fault = 'board reported CMD_ERROR'

    def set_connected(self, connected: bool, fault: str = ''):
        self._state.connected = connected
        if fault:
            self._state.fault = fault

    def set_motors(self, on: bool):
        self._state.motors_on = bool(on)

    # -------------------------------------------------------------- state --

    def _apply_limits_to_state(self):
        c = self.cfg
        s = self._state
        s.pan_min, s.pan_max = c.pan_min_deg, c.pan_max_deg
        s.tilt_min, s.tilt_max = c.tilt_min_deg, c.tilt_max_deg
        s.roll_min, s.roll_max = c.roll_min_deg, c.roll_max_deg

    def _sync_state(self):
        self._state.target_pan = self._pos_pan
        self._state.target_tilt = self._pos_tilt
        self._state.target_roll = self._pos_roll
        self._apply_limits_to_state()

    def state(self) -> GimbalStateSnapshot:
        self._sync_state()
        self._state.command_timeout = self.command_timed_out
        return self._state
