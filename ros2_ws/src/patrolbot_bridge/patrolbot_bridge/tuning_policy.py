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
import threading

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
        self._lock = threading.RLock()
        self._staged = None
        self._refresh_epoch = 0

    @property
    def generation(self):
        return self._generation

    def refresh(self):
        with self._lock:
            # A retained candidate is never replaced by an ordinary background
            # ACTIVE read while an exact-job commit is in progress. Commands
            # remain zero-clamped until explicit committed binding below.
            if self._staged is not None:
                return False
            epoch = self._refresh_epoch
        if self.probe is None:
            return False
        try:
            # The probe takes the revision writer. Never hold the cache mutex
            # across it: reviewed commit holds writer -> cache, and the worker
            # must not establish the inverse cache -> writer ordering.
            generation, limits = self.probe()
        except Exception:
            with self._lock:
                if self._staged is None and self._refresh_epoch == epoch:
                    self._limits = None
                    self._generation = None
            return False
        with self._lock:
            if self._staged is not None or self._refresh_epoch != epoch:
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
        with self._lock:
            if self._staged is not None:
                return dict.fromkeys(POLICY_KEYS, 0.0)
            if self._limits is not None:
                return dict(self._limits)
            if self.required:
                return dict.fromkeys(POLICY_KEYS, 0.0)
            return dict(self.hard)

    def stage(self, context, limits, *, maintenance_owned):
        """Stage only under exact acknowledged ownership; it grants no DRIVE."""
        from .tuning_lease import context as validate_context
        context = validate_context(context)
        value = self._sanitize(limits)
        if maintenance_owned is not True or value is None or value != limits:
            raise ValueError('candidate policy requires owned maintenance and exact safe limits')
        with self._lock:
            candidate = {'context': context, 'limits': value}
            if self._staged is not None and self._staged != candidate:
                raise ValueError('another staged policy owns maintenance')
            self._staged = candidate
            self._refresh_epoch += 1
            return self.staged_evidence(context)

    def staged_evidence(self, context):
        from .tuning_lease import context as validate_context
        context = validate_context(context)
        with self._lock:
            if self._staged is None or self._staged['context'] != context:
                raise ValueError('exact candidate policy is not staged')
            return {'context': dict(context), 'limits': dict(self._staged['limits']), 'drive_blocked': True}

    def commit_staged(self, context, *, generation, committed_limits, maintenance_owned):
        """Bind to already-committed ACTIVE, never DESIRED or a staged file."""
        # Same writer/cache order as refresh: no store probe under cache mutex.
        actual_generation, actual_limits = self.probe()
        with self._lock:
            evidence = self.staged_evidence(context)
            if (maintenance_owned is not True or not isinstance(generation, str) or not generation
                    or committed_limits != evidence['limits']):
                raise ValueError('committed candidate policy binding differs')
            # Probe reads committed ACTIVE through the existing store/lock path.
            if actual_generation != generation or actual_limits != committed_limits:
                raise ValueError('ACTIVE policy readback did not confirm candidate')
            self._generation, self._limits = generation, dict(committed_limits)
            self._staged = None
            self._refresh_epoch += 1
            return {'generation': self._generation, 'limits': dict(self._limits)}

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
