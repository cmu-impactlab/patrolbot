"""Fail-closed ordering contract for coordinated odometry-origin recovery.

The runtime coordinator must reset AMCL through its lifecycle before reporting
reset completion. It must serialize these methods with epoch publication. This
module never sends a pose, clears a watchdog, or decides localization quality.
"""

from dataclasses import dataclass
from uuid import uuid4

from .odom_epoch import valid_epoch


@dataclass(frozen=True)
class RecoveryToken:
    bridge_instance: str
    generation: int
    epoch: str


class EpochRecovery:
    WAIT_ORIGIN = 'wait-origin'
    WAIT_RESET = 'wait-amcl-reset'
    WAIT_SEED = 'wait-operator-seed'
    WAIT_POSE = 'wait-amcl-pose'
    READY = 'ready'

    def __init__(self):
        self.instance = str(uuid4())
        self.token = None
        self.stage = self.WAIT_ORIGIN
        self._generation = 0
        self._last_clock_ns = None
        self._reset_ns = None
        self._seed_ns = None

    @property
    def required(self):
        return self.stage != self.READY

    def observe_origin(self, epoch):
        """Called only after the transport guard accepts an explicit identity."""
        if not valid_epoch(epoch):
            raise ValueError('origin must be a canonical UUID')
        if self.token is None or self.token.epoch != epoch:
            self._start_reset(epoch)
        return self.token

    def _start_reset(self, epoch):
        self._generation += 1
        self.token = RecoveryToken(self.instance, self._generation, epoch)
        self.stage = self.WAIT_RESET
        self._reset_ns = None
        self._seed_ns = None

    def require_seed(self, now_ns):
        """Discard pre-power localization without resetting the odometry origin."""
        if self.token is not None:
            self._start_reset(self.token.epoch)
            self.reset_complete(self.token, now_ns)

    def observe_clock(self, now_ns):
        """Clock regression invalidates ordering and all in-flight reset ACKs."""
        if isinstance(now_ns, bool) or not isinstance(now_ns, int) or now_ns < 0:
            raise ValueError('clock must be nonnegative integer nanoseconds')
        regressed = (self._last_clock_ns is not None and
                     now_ns < self._last_clock_ns)
        self._last_clock_ns = now_ns
        if regressed and self.token is not None:
            self._start_reset(self.token.epoch)
        return not regressed

    def reset_complete(self, token, now_ns):
        """ACK from the coordinator after cleanup/configure/activate succeed.

        Failure/timeout must not call this method. A token captured before a
        newer origin, bridge restart or clock regression cannot acknowledge it.
        """
        if not self.observe_clock(now_ns):
            return False
        if token != self.token or self.stage != self.WAIT_RESET:
            return False
        self._reset_ns = now_ns
        self.stage = self.WAIT_SEED
        return True

    def operator_seed(self, token, stamp_ns, now_ns):
        """Observe a valid operator seed after the reset barrier.

        This only records an already-issued seed. It never publishes one.
        The caller validates frame, numeric pose and covariance first.
        """
        if not self.observe_clock(now_ns):
            return False
        if token != self.token or self.stage not in (
                self.WAIT_SEED, self.WAIT_POSE, self.READY):
            return False
        if not self._ordered_stamp(stamp_ns, self._reset_ns, now_ns):
            return False
        # Never let a delayed older seed supersede a newer one.
        if self._seed_ns is not None and stamp_ns <= self._seed_ns:
            return False
        self._seed_ns = stamp_ns
        self.stage = self.WAIT_POSE
        return True

    def amcl_pose(self, token, stamp_ns, now_ns):
        """Observe output from the reset AMCL instance after its new seed.

        Header time, not callback receipt order, rejects old DDS samples. This
        establishes origin association only; existing covariance/freshness
        gates still decide whether the pose is usable for navigation.
        """
        if not self.observe_clock(now_ns):
            return False
        if token != self.token or self.stage != self.WAIT_POSE:
            return False
        if not self._ordered_stamp(stamp_ns, self._seed_ns, now_ns):
            return False
        self.stage = self.READY
        return True

    @staticmethod
    def _ordered_stamp(stamp_ns, lower_ns, now_ns):
        return (not isinstance(stamp_ns, bool) and isinstance(stamp_ns, int)
                and lower_ns is not None and lower_ns < stamp_ns <= now_ns)
