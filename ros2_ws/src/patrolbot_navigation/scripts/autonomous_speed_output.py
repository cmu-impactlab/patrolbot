"""Pure offline guard model for ordinary autonomous path-following output."""

from dataclasses import dataclass
from math import isfinite, nextafter

from autonomous_speed_policy import resolve_autonomous_speed


_STOP = (0.0, 0.0)
_MAX_LINEAR_MPS = 0.26
_MAX_ANGULAR_RADPS = 0.70
_MAX_SAMPLE_AGE_S = 0.25  # Matches the upstream controller mux timeout.


class CommandEpoch:
    """Opaque identity, never reused across fence changes or restart."""

    __slots__ = ()


@dataclass(frozen=True)
class ActiveOutputFence:
    """Independent gate snapshot; caps are the pair from the speed policy."""

    epoch: CommandEpoch
    caps: tuple[float, float]


@dataclass(frozen=True)
class ControllerSample:
    """Planar SI sample stamped at trusted ordinary-controller ingress only."""

    epoch: CommandEpoch
    linear_x: float
    angular_z: float
    received_monotonic_s: float


def _valid_age(received, now):
    if type(received) not in (int, float) or type(now) not in (int, float):
        return False
    try:
        first, last = float(received), float(now)
    except OverflowError:
        return False
    return (isfinite(first) and isfinite(last)
            and first >= 0.0 and 0.0 <= last - first < _MAX_SAMPLE_AGE_S)


def guard_autonomous_speed_output(
    sample: ControllerSample | None,
    current: ActiveOutputFence | None,
    *,
    now_monotonic_s: float,
) -> tuple[float, float]:
    """Return guarded (linear m/s, angular rad/s), or an explicit zero pair.

    The offline model compares the captured sample epoch against the independent
    current fence. The caller cannot substitute a `fresh=True` flag. In a real
    system, trusted ingress must bind origin and epoch before queuing, never
    restamp a delayed sample, and compare against a live fence at output. Both
    times must be sampled by that trusted adapter in the same clock domain;
    this checks only time since ingress, not freshness before receipt.
    This does not gate, convert, or authorize manual, recovery, or undock motion.
    """
    if (
        type(sample) is not ControllerSample
        or type(current) is not ActiveOutputFence
        or type(current.epoch) is not CommandEpoch
        or sample.epoch is not current.epoch
        or not _valid_age(sample.received_monotonic_s, now_monotonic_s)
    ):
        return _STOP

    linear, angular = sample.linear_x, sample.angular_z
    # Range checks precede arithmetic or conversion, including for huge ints.
    if (
        type(linear) not in (int, float)
        or type(angular) not in (int, float)
        or not 0.0 <= linear <= _MAX_LINEAR_MPS
        or not -_MAX_ANGULAR_RADPS <= angular <= _MAX_ANGULAR_RADPS
    ):
        return _STOP
    if type(current.caps) is not tuple or len(current.caps) != 2:
        return _STOP
    try:
        linear_cap, angular_cap = resolve_autonomous_speed(
            'Custom',
            custom_linear_mps=current.caps[0],
            custom_angular_radps=current.caps[1],
            commissioned_linear_mps=_MAX_LINEAR_MPS,
            commissioned_angular_radps=_MAX_ANGULAR_RADPS,
        )
    except ValueError:
        return _STOP
    if (linear and not linear_cap) or (angular and not angular_cap):
        return _STOP

    factor = 1.0
    # Divide only on a violated bound: ratios are <= 1 and denominators > 0.
    if linear > linear_cap:
        factor = linear_cap / linear
    if abs(angular) > angular_cap:
        factor = min(factor, angular_cap / abs(angular))
    x, z = linear * factor, angular * factor
    if x > linear_cap or abs(z) > angular_cap:
        # A rounded quotient can overshoot. Round the common factor down,
        # not the individual outputs (which would change curvature).
        factor = nextafter(factor, 0.0)
        x, z = linear * factor, angular * factor
    if x > linear_cap or abs(z) > angular_cap or (linear and not x) or (angular and not z):
        return _STOP
    return x, z
