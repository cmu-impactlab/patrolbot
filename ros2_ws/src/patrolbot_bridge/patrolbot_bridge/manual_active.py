"""Manual recovery admission and an observed zero-drain barrier (no ROS IO).

Bool is a permission heartbeat, NOT Twist provenance. The bridge additionally
requires exclusive, stable joy/mux/final endpoints, a fresh continuous
joy lease, no other action/input owner, and observed zero through BOTH buffers
before opening that lease. The teleop keeps its existing priority-8 output at
zero until cancellation and this drain complete. No second velocity path.
"""
import math


# Existing joy mux / final smoother command timeout, not a physical estimate.
MANUAL_ACTIVE_TIMEOUT = 0.5
# Readiness replies are useful for at most one default teleop tick. This does
# not extend the command/zero timeout or any physical safety threshold.
MANUAL_HANDOFF_TIMEOUT = 1. / 30.
MANUAL_READY = 'manual-ready-v1'
MANUAL_OWNED = 'manual-owned-v1'


def finite_planar(message):
    values = (message.linear.x, message.linear.y, message.linear.z,
              message.angular.x, message.angular.y, message.angular.z)
    return (all(math.isfinite(v) for v in values)
            and all(values[i] == 0. for i in (1, 2, 3, 4)))


def zero_twist(message):
    return finite_planar(message) and message.linear.x == 0. and message.angular.z == 0.


def source_stamp(info):
    return info['source_timestamp'] if isinstance(info, dict) else info.source_timestamp


def trusted_message(node, topic, name, kind, info):
    """Check sole stable endpoint topology and fresh DDS source metadata.

    Jazzy rclpy exposes timestamp/sequence but no publisher GID in its callback
    dictionary. Do not invent that capability: endpoint GIDs are separately
    bound to the drain lease. This is not DDS security/cryptographic identity.
    """
    try:
        endpoints = node.get_publishers_info_by_topic(topic)
        if len(endpoints) != 1:
            return False
        endpoint = endpoints[0]
        stamp = source_stamp(info)
        age = (node.get_clock().now().nanoseconds - stamp) / 1e9
        return (endpoint.node_name == name and endpoint.node_namespace == '/'
                and endpoint.topic_type == kind
                and stamp > 0 and math.isfinite(age)
                and 0. <= age <= MANUAL_ACTIVE_TIMEOUT)
    except (AttributeError, KeyError, TypeError, ValueError, RuntimeError):
        return False


class ManualDrain:
    """Require fresh zero at joy, mux and final for one full buffer timeout.

    Every input carries checked DDS source time. Old queued samples cannot
    establish a new drain, and a lost/foreign source cannot renew its lease.
    Once armed, continuous nonzero manual output keeps the lease alive; any
    foreign command, False, context change or missing report revokes it.
    """
    def __init__(self):
        self.revoke()

    def revoke(self):
        self.generation = getattr(self, 'generation', 0) + 1
        self.owned = False
        self.start_stamp = None
        self.zero_since = {}
        self.at = {}
        self.stamp = {}

    def observe(self, stage, is_zero, now, stamp):
        # Latch the observed drain BEFORE the first nonzero joy sample clears
        # its zero history. Bool may arrive on a different DDS reader afterward;
        # this latch is only provenance, never permission to forward by itself.
        if self.start_stamp is not None and self.ready(now):
            self.owned = True
        if stage in self.stamp and stamp <= self.stamp[stage]:
            self.revoke()
            return
        if self.owned and stage in self.at and now-self.at[stage] > MANUAL_ACTIVE_TIMEOUT:
            self.revoke()
        if stage == 'joy' and self.start_stamp is None:
            if not is_zero:
                return
            self.start_stamp = stamp
        if self.start_stamp is None or stamp < self.start_stamp:
            return
        self.at[stage] = now
        self.stamp[stage] = stamp
        if not self.owned:
            if is_zero:
                self.zero_since.setdefault(stage, now)
            else:
                self.zero_since.pop(stage, None)

    def ready(self, now):
        if not all(stage in self.at and 0. <= now-self.at[stage] <= MANUAL_ACTIVE_TIMEOUT
                   for stage in ('joy', 'mux', 'final')):
            if self.owned:
                self.revoke()
            return False
        return self.owned or all(
            stage in self.zero_since and now-self.zero_since[stage] >= MANUAL_ACTIVE_TIMEOUT
            for stage in ('joy', 'mux', 'final'))


def manual_drive_bypass(manual_age_s, *, undock_hold, lidar_ready,
                        map_hold, guard_ready, recovery_required,
                        manual_asserted=False, mutations_settled=False,
                        navigation_quiet=False, raw_ready=False,
                        manual_owned=False, settled_map_failed=False):
    """Bypass only localization/settled map failure, never origin/owner safety."""
    try:
        fresh = manual_age_s is not None and 0. <= manual_age_s <= MANUAL_ACTIVE_TIMEOUT
    except TypeError:
        return False
    return bool(fresh and manual_asserted and not undock_hold and lidar_ready
                and guard_ready and raw_ready and mutations_settled
                and navigation_quiet and manual_owned
                and (recovery_required or (map_hold and settled_map_failed)))
