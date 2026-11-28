"""Fail-closed semantic validation for SBC odometry samples."""

from dataclasses import dataclass
import math
import re
from typing import Optional


SOURCE_TIMING_FEATURE = 'ODOM_SOURCE_TIMING_V1'
_SOURCE_PREFIX = 'ODOM_SOURCE_MS'
_UNSIGNED_MS = re.compile(r'(?:0|[1-9][0-9]{0,15})')


@dataclass(frozen=True)
class OdomSource:
    stamp_ms: int
    age_ms: int


def parse_odom_source(parts, *, required):
    """Parse exactly one source extension when advertised; never fall back on errors."""
    fields = [part for part in parts if part.startswith(_SOURCE_PREFIX)]
    if not fields:
        if required:
            raise ValueError('missing-odometry-source')
        return None
    if not required:
        raise ValueError('unadvertised-odometry-source')
    if len(fields) != 1 or not fields[0].startswith(_SOURCE_PREFIX + ':'):
        raise ValueError('malformed-odometry-source')
    value = fields[0][len(_SOURCE_PREFIX) + 1:]
    if value == '-1,-1':
        raise ValueError('unavailable-odometry-source')
    tokens = value.split(',')
    if len(tokens) != 2 or not all(_UNSIGNED_MS.fullmatch(token) for token in tokens):
        raise ValueError('malformed-odometry-source')
    return OdomSource(*(int(token) for token in tokens))


@dataclass(frozen=True)
class OdomSample:
    x: float
    y: float
    yaw: float
    linear_x: float
    angular_z: float

    def values(self):
        return (self.x, self.y, self.yaw, self.linear_x, self.angular_z)


@dataclass(frozen=True)
class OdomValidation:
    accepted: bool
    reason: str
    detail: str = ''
    repeated: bool = False
    fresh_at: Optional[float] = None


class OdomValidator:
    """
    Validate numeric values and pose continuity before state is mutated.

    Continuity limits come from two existing contracts rather than invented
    distances: the SBC accepts at most the configured linear/angular speeds,
    and it emits odometry at the configured period. One period of timing margin
    covers sampling phase. With source timing, elapsed SIP time is capped by
    Pi receipt elapsed, both from the last accepted distinct observation.
    """

    def __init__(self, *, max_linear_speed, max_angular_speed, expected_period):
        self._max_linear_speed = self._positive(max_linear_speed, 'max_linear_speed')
        self._max_angular_speed = self._positive(max_angular_speed, 'max_angular_speed')
        self._expected_period = self._positive(expected_period, 'expected_period')
        self.reset_epoch()

    @staticmethod
    def _positive(value, name):
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def check_and_accept(self, sample, received_at, *, source=None, queue_age=0.0,
                         max_age=0.5):
        sample = OdomSample(*(float(value) for value in sample.values()))
        received_at = float(received_at)
        if not math.isfinite(received_at):
            return OdomValidation(False, 'invalid-receipt-time')
        if self._previous_at is not None and received_at < self._previous_at:
            return OdomValidation(False, 'receipt-time-regressed')
        if self._previous_sample is not None and (
            (source is None) != (self._previous_source is None)
        ):
            return OdomValidation(False, 'odometry-source-mode-changed')

        fresh_at = received_at
        if source is not None:
            repeated = self._observe_source(sample, source, received_at, queue_age, max_age)
            if repeated is not None and not repeated.accepted:
                return repeated
            age_s = source.age_ms / 1000.0
            if not (0.0 <= queue_age and 0.0 <= age_s <= max_age - queue_age):
                return OdomValidation(False, 'stale-or-future-odometry-source',
                                      f'source_age_s={age_s:.3f} queue_age_s={queue_age:.3f}')
            fresh_at -= age_s
            if repeated is not None:
                return repeated
            # Do not revive an older projected observation by treating its receipt
            # as freshness, even if its remote stamp is numerically newer.
            if self._fresh_at is not None and fresh_at < self._fresh_at:
                return OdomValidation(False, 'odometry-source-freshness-regressed')

        if not all(math.isfinite(value) for value in sample.values()):
            return OdomValidation(False, 'non-finite')
        if self._exceeds(abs(sample.linear_x), self._max_linear_speed):
            return OdomValidation(False, 'linear-speed-limit',
                                  f'speed_m_s={sample.linear_x:.6g} limit={self._max_linear_speed:g}')
        if self._exceeds(abs(sample.angular_z), self._max_angular_speed):
            return OdomValidation(False, 'angular-speed-limit',
                                  f'speed_rad_s={sample.angular_z:.6g} limit={self._max_angular_speed:g}')

        if self._previous_sample is not None:
            received_elapsed = elapsed = received_at - self._previous_at
            source_detail = ''
            if source is not None:
                source_elapsed = (source.stamp_ms - self._previous_source.stamp_ms) / 1000.0
                elapsed = min(received_elapsed, source_elapsed)
                source_detail = (f' source_elapsed_s={source_elapsed:.6g}'
                                 f' source_age_ms={source.age_ms}')
            allowance_s = elapsed + self._expected_period
            step_m = math.hypot(
                sample.x - self._previous_sample.x, sample.y - self._previous_sample.y
            )
            step_rad = abs(self._wrapped_delta(sample.yaw - self._previous_sample.yaw))
            timing = (f'elapsed_s={elapsed:.6g} receipt_elapsed_s={received_elapsed:.6g}'
                      f'{source_detail}')
            position_limit = self._max_linear_speed * allowance_s
            heading_limit = self._max_angular_speed * allowance_s
            if self._exceeds(step_m, position_limit):
                return OdomValidation(False, 'position-discontinuity',
                                      f'step_m={step_m:.6g} limit_m={position_limit:.6g} {timing}')
            if self._exceeds(step_rad, heading_limit):
                return OdomValidation(False, 'heading-discontinuity',
                                      f'step_rad={step_rad:.6g} limit_rad={heading_limit:.6g} {timing}')

        self._previous_sample = sample
        self._previous_at = received_at
        self._previous_source = source
        self._fresh_at = fresh_at
        return OdomValidation(True, 'accepted', fresh_at=fresh_at)

    def _observe_source(self, sample, source, received_at, queue_age, max_age):
        """Keep a wire high-water mark separate from the accepted pose baseline."""
        # An out-of-bounds age must not set the send-clock watermark minutes
        # into the future, but the rejected source stamp still cannot be reused.
        emitted_ms = (source.stamp_ms + source.age_ms
                      if 0.0 <= source.age_ms / 1000.0 <= max_age else None)
        if self._observed_source is not None:
            if received_at < self._observed_at:
                return OdomValidation(False, 'receipt-time-regressed')
            if source.stamp_ms < self._observed_source or (
                emitted_ms is not None and self._emitted_ms is not None
                and emitted_ms < self._emitted_ms
            ):
                return OdomValidation(False, 'odometry-source-time-regressed')
            if source.stamp_ms == self._observed_source:
                if sample != self._observed_sample:
                    self._source_conflict = True
                if self._source_conflict:
                    return OdomValidation(False, 'conflicting-odometry-source')
                if (self._previous_source is None
                        or source.stamp_ms != self._previous_source.stamp_ms):
                    return OdomValidation(False, 'unaccepted-odometry-source')
                if emitted_ms is not None:
                    self._emitted_ms = emitted_ms
                self._observed_at = received_at
                if not 0.0 <= (received_at - self._fresh_at) + queue_age <= max_age:
                    return OdomValidation(False, 'stale-repeated-odometry-source')
                return OdomValidation(True, 'repeated-odometry-source', repeated=True,
                                      fresh_at=self._fresh_at)
        self._observed_source = source.stamp_ms
        if emitted_ms is not None:
            self._emitted_ms = emitted_ms
        self._observed_at = received_at
        self._observed_sample = sample
        self._source_conflict = False
        return None

    def reset_epoch(self):
        """Accept a new absolute origin only after an explicit epoch change."""
        self._previous_sample = None
        self._previous_at = None
        self._previous_source = None
        self._fresh_at = None
        self._observed_source = None
        self._observed_at = None
        self._emitted_ms = None
        self._observed_sample = None
        self._source_conflict = False

    @staticmethod
    def _exceeds(value, limit):
        # Admit only binary floating-point representation error at a contract
        # boundary; this does not add physical motion allowance.
        return value > limit and not math.isclose(value, limit, rel_tol=1.0e-9, abs_tol=1.0e-12)

    @staticmethod
    def _wrapped_delta(delta):
        if not math.isfinite(delta):
            return math.inf
        return (delta + math.pi) % (2.0 * math.pi) - math.pi
