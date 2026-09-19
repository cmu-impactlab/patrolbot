"""ROS-free identity checks for odometry origins and asynchronous wire frames.

The caller must serialize these operations with pose mutation and publication.
This helper does not acknowledge localization recovery, publish TF, or reset an
estimator. A changed identity returns an explicit reset decision to the caller.
"""

from dataclasses import dataclass
from typing import Optional
from uuid import UUID


def valid_epoch(value):
    if not isinstance(value, str) or len(value) != 36:
        return False
    try:
        return str(UUID(value)) == value
    except ValueError:
        return False


def parse_frame_epoch(line):
    """Return exactly one canonical origin ID; missing metadata is unknown."""
    fields = [part for part in line.split('|')
              if part.startswith('ODOM_EPOCH')]
    if len(fields) != 1 or not fields[0].startswith('ODOM_EPOCH:'):
        raise ValueError('missing, duplicate or malformed odometry epoch')
    value = fields[0][len('ODOM_EPOCH:'):]
    if not valid_epoch(value):
        raise ValueError('invalid odometry epoch UUID')
    return value


@dataclass(frozen=True)
class EpochDecision:
    accepted: bool
    reset_required: bool
    reason: str


class OdomEpochGuard:
    """Keep transport generations distinct from origin identity.

    No caller-supplied numeric pose can establish a new epoch. Each transport
    must first announce an identity. Prior-transport queued frames are rejected
    even when their origin ID matches, since their receipt order is obsolete.
    A mismatch poisons that transport until a new connection is established.
    """

    def __init__(self):
        self.epoch: Optional[str] = None
        self.generation = 0
        self.session = 0
        self.ready = False
        self._announced = None
        self._retired = set()
        self._poisoned = False

    def begin_session(self):
        self.session += 1
        self.ready = False
        self._announced = None
        self._poisoned = False
        return self.session

    def disconnect(self, session):
        if session == self.session:
            self.ready = False
            self._poisoned = True

    def announce(self, session, epoch):
        if session != self.session or session == 0:
            return EpochDecision(False, False, 'obsolete-session')
        if self._poisoned:
            return EpochDecision(False, False, 'transport-invalid')
        if not valid_epoch(epoch):
            self.ready = False
            self._poisoned = True
            return EpochDecision(False, False, 'unknown-epoch')
        if self._announced is not None and epoch != self._announced:
            self.ready = False
            self._poisoned = True
            return EpochDecision(False, False, 'epoch-changed-within-session')
        if epoch in self._retired:
            self.ready = False
            self._poisoned = True
            return EpochDecision(False, False, 'retired-epoch')
        changed = self.epoch is not None and epoch != self.epoch
        if epoch != self.epoch:
            if self.epoch is not None:
                self._retired.add(self.epoch)
            self.epoch = epoch
            self.generation += 1
        self._announced = epoch
        self.ready = True
        return EpochDecision(True, changed, 'new-epoch' if changed else 'accepted')

    def check_frame(self, session, line):
        if session != self.session or session == 0:
            return EpochDecision(False, False, 'obsolete-session')
        if not self.ready:
            return EpochDecision(False, False, 'epoch-not-established')
        try:
            epoch = parse_frame_epoch(line)
        except ValueError:
            epoch = None
        if epoch != self.epoch:
            self.ready = False
            self._poisoned = True
            return EpochDecision(False, False, 'frame-epoch-mismatch')
        return EpochDecision(True, False, 'accepted')
