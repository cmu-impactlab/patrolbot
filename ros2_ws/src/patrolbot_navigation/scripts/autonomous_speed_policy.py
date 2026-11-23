"""Offline policy for autonomous speed profile caps; never applies commands."""

# Planning target only. No input can make this profile available.
REALLY_FAST_TARGET = (2.00, 0.70)  # m/s, rad/s

_MAX_LINEAR_MPS = 0.26
_MAX_ANGULAR_RADPS = 0.70


def _validate_cap(value, maximum, name):
    # This inclusive range also excludes NaN and infinities. Check before
    # converting so even huge integers fail without a float overflow.
    if type(value) not in (int, float) or not 0.0 <= value <= maximum:
        raise ValueError(f'{name} must be a finite number in [0, {maximum}]')
    return float(value) if value else 0.0


def resolve_autonomous_speed(
    profile: str = 'Fast',
    *,
    custom_linear_mps: float | None = None,
    custom_angular_radps: float | None = None,
    commissioned_linear_mps: float = _MAX_LINEAR_MPS,
    commissioned_angular_radps: float = _MAX_ANGULAR_RADPS,
) -> tuple[float, float]:
    """Return the requested ``(linear m/s, angular rad/s)`` caps, not applied speeds.

    Names are exact: ``Slow`` (0.12, 0.40), ``Fast`` (0.22, 0.70, the
    default), ``Custom`` (both custom values required), and ``Really Fast``
    (uncommissioned and always unavailable). Presets do not take custom values.

    Commissioned bounds may only narrow the absolute limits and do not attest
    to commissioning. Caps and bounds accept only plain ints or floats.
    Zero means a zero cap on that axis, not a request to move or a proven
    operating point. No minimum usable nonzero speed is asserted.

    Raises ValueError for unknown or unavailable profiles, invalid inputs,
    or a requested cap above its bound. No clamping or fallback is performed.
    """
    if type(profile) is not str or profile not in ('Slow', 'Fast', 'Custom', 'Really Fast'):
        raise ValueError('unknown autonomous speed profile')
    if profile == 'Really Fast':
        raise ValueError('Really Fast is uncommissioned and unavailable')

    linear_limit = _validate_cap(
        commissioned_linear_mps, _MAX_LINEAR_MPS, 'commissioned_linear_mps',
    )
    angular_limit = _validate_cap(
        commissioned_angular_radps, _MAX_ANGULAR_RADPS, 'commissioned_angular_radps',
    )

    if profile == 'Custom':
        linear, angular = custom_linear_mps, custom_angular_radps
    else:
        if custom_linear_mps is not None or custom_angular_radps is not None:
            raise ValueError('custom caps may only be supplied for Custom')
        linear, angular = (0.12, 0.40) if profile == 'Slow' else (0.22, 0.70)

    return (
        _validate_cap(linear, linear_limit, 'linear cap (m/s)'),
        _validate_cap(angular, angular_limit, 'angular cap (rad/s)'),
    )
