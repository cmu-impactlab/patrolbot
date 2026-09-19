"""Fail-closed semantic validation for SBC odometry samples."""

from dataclasses import dataclass
import math


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


class OdomValidator:
    """Validate numeric values and pose continuity before state is mutated.

    Continuity limits come from two existing contracts rather than invented
    distances: the SBC accepts at most the configured linear/angular speeds,
    and it emits odometry at the configured period. One period of timing margin
    covers sampling phase; elapsed monotonic time covers delayed frames.
    """

    def __init__(
            self, *, max_linear_speed, max_angular_speed, expected_period):
        self._max_linear_speed = self._positive(
            max_linear_speed, "max_linear_speed")
        self._max_angular_speed = self._positive(
            max_angular_speed, "max_angular_speed")
        self._expected_period = self._positive(
            expected_period, "expected_period")
        self._previous_sample = None
        self._previous_at = None

    @staticmethod
    def _positive(value, name):
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def check_and_accept(self, sample, received_at):
        sample = OdomSample(*(float(value) for value in sample.values()))
        received_at = float(received_at)
        if not all(math.isfinite(value) for value in sample.values()):
            return OdomValidation(False, "non-finite")
        if not math.isfinite(received_at):
            return OdomValidation(False, "invalid-receipt-time")
        if self._exceeds(abs(sample.linear_x), self._max_linear_speed):
            return OdomValidation(False, "linear-speed-limit")
        if self._exceeds(abs(sample.angular_z), self._max_angular_speed):
            return OdomValidation(False, "angular-speed-limit")

        if self._previous_sample is not None:
            elapsed = received_at - self._previous_at
            if elapsed < 0.0:
                return OdomValidation(False, "receipt-time-regressed")
            allowance_s = elapsed + self._expected_period
            step_m = math.hypot(
                sample.x - self._previous_sample.x,
                sample.y - self._previous_sample.y)
            step_rad = abs(self._wrapped_delta(
                sample.yaw - self._previous_sample.yaw))
            if self._exceeds(
                    step_m, self._max_linear_speed * allowance_s):
                return OdomValidation(False, "position-discontinuity")
            if self._exceeds(
                    step_rad, self._max_angular_speed * allowance_s):
                return OdomValidation(False, "heading-discontinuity")

        self._previous_sample = sample
        self._previous_at = received_at
        return OdomValidation(True, "accepted")

    def reset_epoch(self):
        """Accept a new absolute origin only after an explicit epoch change."""
        self._previous_sample = None
        self._previous_at = None

    @staticmethod
    def _exceeds(value, limit):
        # Admit only binary floating-point representation error at a contract
        # boundary; this does not add physical motion allowance.
        return value > limit and not math.isclose(
            value, limit, rel_tol=1.0e-9, abs_tol=1.0e-12)

    @staticmethod
    def _wrapped_delta(delta):
        return (delta + math.pi) % (2.0 * math.pi) - math.pi
