"""
Bounded, offline raw-wheel versus firmware-fused SIP comparisons.

No ROS, ARIA connection, motor commands, or inferred wheel calibration.
Capture timestamps must be from one SBC monotonic clock and origin must identify
one controller/ARIA odometry epoch, not a TCP session. Counter modulus comes
from the confirmed encoder packet format, never a guessed hardware value.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class EncoderSample:
    captured_at: float
    origin: str
    left_counts: int
    right_counts: int
    sip_x_m: float
    sip_y_m: float
    sip_yaw_rad: float


@dataclass(frozen=True)
class EncoderDelta:
    elapsed_s: float
    left_counts: int
    right_counts: int
    sip_distance_m: float
    sip_yaw_rad: float


class EncoderDiagnostic:
    def __init__(self, *, counter_modulus, max_samples):
        if (
            type(counter_modulus) is not int
            or counter_modulus < 2
            or counter_modulus % 2
            or type(max_samples) is not int
            or max_samples < 2
        ):
            raise ValueError('even counter modulus and sample budget >= 2 required')
        self._modulus = counter_modulus
        self._max_samples = max_samples
        self._samples = 0
        self._previous = None

    def observe(self, sample):
        if self._samples >= self._max_samples:
            raise StopIteration('diagnostic sample budget exhausted')
        if (
            not isinstance(sample.origin, str)
            or not sample.origin
            or not all(
                math.isfinite(value)
                for value in (
                    sample.captured_at,
                    sample.sip_x_m,
                    sample.sip_y_m,
                    sample.sip_yaw_rad,
                )
            )
            or any(
                type(value) is not int or not 0 <= value < self._modulus
                for value in (sample.left_counts, sample.right_counts)
            )
        ):
            raise ValueError('invalid encoder/SIP sample')
        previous = self._previous
        delta = None
        if previous is not None:
            elapsed = sample.captured_at - previous.captured_at
            if sample.origin != previous.origin or elapsed <= 0:
                raise ValueError('origin changed or capture time did not advance')
            left = self._counter_delta(sample.left_counts - previous.left_counts)
            right = self._counter_delta(sample.right_counts - previous.right_counts)
            delta = EncoderDelta(
                elapsed,
                left,
                right,
                math.hypot(sample.sip_x_m - previous.sip_x_m, sample.sip_y_m - previous.sip_y_m),
                (sample.sip_yaw_rad - previous.sip_yaw_rad + math.pi) % (2 * math.pi) - math.pi,
            )
        self._previous = sample
        self._samples += 1
        return delta

    def _counter_delta(self, difference):
        # shortcut: shortest modular delta requires < half a wrap per sample;
        # use a confirmed count-rate bound before interpreting a live capture.
        half = self._modulus // 2
        delta = (difference + half) % self._modulus - half
        if delta == -half:
            raise ValueError('ambiguous half-wrap encoder delta')
        return delta
