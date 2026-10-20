"""
Trusted-side ordinary-DRIVE speed backstop.

Defence in depth for the final command path. Even if an upstream producer (the
F710 teleop with a cached trim, the dashboard web bridge, Nav2, a future helper)
emits a value above the active policy, the core bridge clamps *ordinary DRIVE*
to the active effective ceiling before the bytes reach the SBC.

Properties this module guarantees (unit-tested without ROS):

* The clamp never *raises* a bound. When no active policy is readable it falls
  back to the fixed commissioned hard envelope (0.30 m/s, 0.70 rad/s) — the
  same values the final smoother already enforces, never an old higher default.
* Reverse is symmetric and lateral speed is untouched (only x and yaw are
  clamped).
* The undock exclusive profile is *not* subject to this backstop; callers must
  skip it for the owner's turn/stop phases. This module does not decide that.
* The clamp is a pure function with no lifecycle, ROS or velocity side effects.
"""

from __future__ import annotations

import math

# Fixed, source-backed commissioned envelope. These are ceilings, never
# commanded speeds, and are the fallback when no tuning policy is active.
HARD_ENVELOPE = {'max_linear_m_s': 0.30, 'max_angular_rad_s': 0.70}
POLICY_KEYS = ('max_linear_m_s', 'max_angular_rad_s')


class PolicyCache:
    """
    Caches the active effective limits and detects generation changes.

    ``probe`` is a zero-argument callable returning ``(generation, limits)`` or
    raising when unavailable. A generation change (a new activation) updates the
    cached limits but never arms or drives anything by itself.
    """

    def __init__(self, probe=None, *, hard_envelope=None, required=False):
        self.probe = probe
        self.hard = dict(hard_envelope or HARD_ENVELOPE)
        self._generation = None
        self._limits = None
        self.required = required

    @property
    def generation(self):
        return self._generation

    def refresh(self):
        if self.probe is None:
            return False
        try:
            generation, limits = self.probe()
        except Exception:
            self._limits = None
            self._generation = None
            return False
        new_limits = self._sanitize(limits)
        if new_limits is None or not isinstance(generation, str) or not generation:
            self._limits = None
            self._generation = None
            return False
        changed = generation != self._generation or new_limits != self._limits
        self._generation = generation
        self._limits = new_limits
        return changed

    def limits(self) -> dict:
        if self._limits is not None:
            return dict(self._limits)
        if self.required:
            return dict.fromkeys(POLICY_KEYS, 0.0)
        return dict(self.hard)

    def _sanitize(self, limits):
        if not isinstance(limits, dict):
            return None
        result = {}
        for key in POLICY_KEYS:
            value = limits.get(key)
            if not isinstance(value, (int, float)) or isinstance(value, bool) \
                    or abs(value) > 10 or not math.isfinite(value) or value < 0:
                return None
            # Never allow the active policy to exceed the fixed hard envelope.
            result[key] = min(float(value), float(self.hard[key]))
        return result

    def clamp(self, linear, angular):
        """Return ``(linear, angular, clamped)`` for ordinary DRIVE."""
        limits = self.limits()
        linear_value = linear if isinstance(linear, (int, float)) else 0.0
        angular_value = angular if isinstance(angular, (int, float)) else 0.0
        if not math.isfinite(linear_value):
            linear_value = 0.0
        if not math.isfinite(angular_value):
            angular_value = 0.0
        lin_cap = limits['max_linear_m_s']
        ang_cap = limits['max_angular_rad_s']
        clamped_lin = max(-lin_cap, min(lin_cap, linear_value))
        clamped_ang = max(-ang_cap, min(ang_cap, angular_value))
        clamped = clamped_lin != linear_value or clamped_ang != angular_value
        return clamped_lin, clamped_ang, clamped


def limits_from_store(store):
    """Return ``(generation, limits)`` for an initialized store, else raise."""
    with store.writer():
        return _limits_from_locked_store(store)


def _limits_from_locked_store(store):
    state = store.snapshot_state()
    startup = store.read_startup()
    if startup['revision'] != state.get('active_revision'):
        raise ValueError('active startup/manifest mismatch')
    active = store.active_overrides()
    if active is None:
        raise ValueError('no active tuning revision')
    from . import tuning_spec
    generation = f"{state.get('generation')}:{state.get('active_revision')}"
    return generation, tuning_spec.effective_limits(active, store.index)


def clamp_drive(linear, angular, limits=None):
    """Clamp ``linear`` and ``angular`` against ``limits`` or the hard envelope."""
    return PolicyCache(probe=None, hard_envelope=limits or HARD_ENVELOPE).clamp(
        linear, angular)


__all__ = [
    'HARD_ENVELOPE',
    'POLICY_KEYS',
    'PolicyCache',
    'clamp_drive',
    'limits_from_store',
]
