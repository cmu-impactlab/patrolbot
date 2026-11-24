"""Offline forward-speed policy model. It has no ROS or hardware adapter."""

from dataclasses import dataclass
from enum import Enum


class DriveMode(Enum):
    NAVIGATION = 'navigation'
    DASHBOARD = 'dashboard'
    CONTROLLER = 'controller'


# Scenario limits, not a production configuration or a braking calculation.
STUDY_MAX_M_S = 1.8
STUDY_LONG_ZONE_M = 2.8  # Distance ahead of the leading edge, not base_link.
DEFAULTS_M_S = {
    DriveMode.NAVIGATION: 0.22,
    DriveMode.DASHBOARD: 0.30,
    DriveMode.CONTROLLER: 0.30,
}


@dataclass(frozen=True)
class Sample:
    value: float
    received_at: float  # Monotonic time on the simulator's own clock.


@dataclass(frozen=True)
class Decision:
    requested_m_s: float
    bounded_intent_m_s: float
    required_zone_m: float
    reason: str


def _in_range(value, maximum):
    # Comparison before arithmetic also rejects bool, NaN, infinity and huge ints.
    return type(value) in (int, float) and 0 <= value <= maximum


def _fresh(sample, now, max_age):
    # These are simulator timestamps; reject bad/future samples without restamping.
    return (
        type(sample) is Sample
        and _in_range(now, 1e12)
        and _in_range(sample.received_at, now)
        and now - sample.received_at < max_age
    )


class ForwardSpeedStudy:
    """Exercise only forward intent, not braking, geometry, or approval."""

    def __init__(self, *, short_zone_m, max_age_s, nav_resume_permitted=False):
        if not (
            _in_range(short_zone_m, STUDY_LONG_ZONE_M) and 0 < short_zone_m < STUDY_LONG_ZONE_M
            and _in_range(max_age_s, 1.0) and max_age_s > 0
            and type(nav_resume_permitted) is bool
        ):
            raise ValueError('expected explicit short-zone reach and freshness bound for the study')
        self._short_zone = short_zone_m
        self._max_age = max_age_s
        self._nav_resume_permitted = nav_resume_permitted
        self._selected = dict(DEFAULTS_M_S)
        self._owner = None
        self._lease = None

    def take_control(self, mode):
        """Issue a new opaque lease; a previous owner cannot restore its intent."""
        if type(mode) is not DriveMode:
            raise ValueError('mode must own an ordinary forward drive')
        self._owner, self._lease = mode, object()
        return self._lease

    def reset(self):
        """Model a restart or connection reset: clear selections and motion authority."""
        self._selected = dict(DEFAULTS_M_S)
        self._owner = self._lease = None

    def select(self, lease, maximum_m_s):
        """Change only the active mode's ceiling; does not start motion."""
        if self._lease is None or lease is not self._lease:
            return False
        if not _in_range(maximum_m_s, STUDY_MAX_M_S):
            return False
        self._selected[self._owner] = maximum_m_s
        return True

    def _stop(self, request, zone, reason):
        self._owner = self._lease = None
        return Decision(request, 0.0, zone, reason)

    def evaluate(self, lease, *, now, command, measured, clearance,
                 route_cap, turn_cap, robot_cap):
        """Return a bounded intent and required sensor reach for a simulated frame.

        Clearance is a synthetic observation of available forward reach. The
        supplied caps are independent simulator inputs, not approval flags.
        None for a cap means unknown. The caller must normalize Nav2's
        no-restriction sentinel before passing a route cap (zero here is a stop).
        """
        request = self._selected[self._owner] if self._owner is not None else 0.0
        if not _fresh(measured, now, self._max_age) or not _in_range(measured.value, STUDY_MAX_M_S):
            return self._stop(request, STUDY_LONG_ZONE_M, 'unknown_measured_speed')

        # Use the lower crossover through handoffs. A Nav2 cap cannot suddenly
        # become a short-zone command just because manual control took over.
        low_speed = min(DEFAULTS_M_S.values())
        occupied_zone = STUDY_LONG_ZONE_M if measured.value > low_speed else self._short_zone
        if self._lease is None or lease is not self._lease:
            return Decision(request, 0.0, occupied_zone, 'no_current_owner')
        if not _fresh(command, now, self._max_age) or not _in_range(command.value, STUDY_MAX_M_S):
            return self._stop(request, occupied_zone, 'no_fresh_forward_intent')
        caps = (route_cap, turn_cap, robot_cap)
        if not all(_in_range(cap, STUDY_MAX_M_S) for cap in caps):
            return self._stop(request, occupied_zone, 'unknown_cap')

        target = min(request, command.value, *caps)
        zone = STUDY_LONG_ZONE_M if max(target, measured.value) > low_speed else self._short_zone
        if not _fresh(clearance, now, self._max_age) or not _in_range(clearance.value, 1e6):
            return self._stop(request, zone, 'unknown_clearance')
        if clearance.value <= zone:
            # Do not downshift into the short field while still moving fast.
            if (zone == self._short_zone or occupied_zone == STUDY_LONG_ZONE_M
                    or clearance.value <= self._short_zone):
                if self._owner is DriveMode.NAVIGATION and self._nav_resume_permitted:
                    return Decision(request, 0.0, zone, 'blocked')
                return self._stop(request, zone, 'blocked')
            return Decision(request, min(target, low_speed), self._short_zone, 'clearance_limits_speed')
        return Decision(request, target, zone, 'within_study_caps')
