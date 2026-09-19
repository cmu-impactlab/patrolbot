"""Validation helpers for acquisition age carried by the SBC scan frame.

The age is an elapsed duration measured by the SBC's monotonic ARIA clock.  It
is deliberately not an SBC wall-clock timestamp: the two machines need not
share an epoch.  Transport and queueing delay after the SBC measured the age
remain unmeasured, so the corrected ROS stamp is an estimate of acquisition
time, not an exact timestamp.
"""

from dataclasses import dataclass
from typing import Optional


MAX_SCAN_AGE_MS = 500
_AGE_FIELD = 'LASER_AGE_MS'


@dataclass(frozen=True)
class LaserAgeResult:
    """Result of inspecting the optional laser age section.

    ``legacy`` distinguishes a valid old server frame from a new frame with
    invalid metadata.  Callers can warn for legacy frames while rejecting a
    malformed or unsafe age instead of silently treating it as fresh.
    """

    accepted: bool
    legacy: bool
    age_ms: Optional[int]
    reason: str


def parse_laser_age(line: str) -> LaserAgeResult:
    """Parse ``LASER_AGE_MS:<integer>`` from a complete telemetry line.

    Absence is an explicitly accepted legacy frame.  Once the field is
    present, its value must be exactly decimal digits in the inclusive range
    0..500; duplicate or malformed fields are rejected.
    """

    if not isinstance(line, str):
        return LaserAgeResult(False, False, None, 'non-text-frame')

    fields = [field for field in line.split('|')
              if field.startswith(_AGE_FIELD)]
    if not fields:
        return LaserAgeResult(True, True, None, 'legacy-no-age')
    if len(fields) != 1:
        return LaserAgeResult(False, False, None, 'duplicate-age-field')

    field = fields[0]
    prefix = _AGE_FIELD + ':'
    if not field.startswith(prefix):
        return LaserAgeResult(False, False, None, 'malformed-age-field')
    value = field[len(prefix):]
    if not value or not value.isascii() or not value.isdecimal():
        return LaserAgeResult(False, False, None, 'malformed-age-value')
    age_ms = int(value)
    if age_ms > MAX_SCAN_AGE_MS:
        return LaserAgeResult(False, False, None, 'age-exceeds-freshness-limit')
    return LaserAgeResult(True, False, age_ms, 'age-valid')


def compute_scan_stamp_ns(receipt_ns: int, age_ns: int) -> int:
    """Return a receive-clock stamp corrected by a nonnegative elapsed age."""

    if isinstance(receipt_ns, bool) or not isinstance(receipt_ns, int):
        raise ValueError('receipt timestamp must be an integer nanosecond value')
    if isinstance(age_ns, bool) or not isinstance(age_ns, int):
        raise ValueError('scan age must be an integer nanosecond value')
    if receipt_ns < 0:
        raise ValueError('receipt timestamp must be nonnegative')
    if age_ns < 0:
        raise ValueError('scan age must be nonnegative')
    stamp_ns = receipt_ns - age_ns
    if stamp_ns < 0:
        raise ValueError('corrected scan timestamp is negative')
    return stamp_ns
