"""
Fail-closed map-localization ordering, owned by EpochRuntime.

All acknowledgements are scoped to one request and origin/transport generation.
A failed transaction remains held even if a late success arrives. This policy
never publishes velocity, seeds, changes lifecycle state, or guesses rollback.
"""

from dataclasses import dataclass
import math
from uuid import UUID


@dataclass(frozen=True)
class MapToken:
    request_id: str
    generation: int
    origin: object
    session: int


class MapTransaction:
    STAGES = (
        'hold',
        'pause-navigation',
        'reset-amcl',
        'load-map',
        'verify-map',
        'persist-map',
        'wait-tf',
        'seed',
        'wait-localization',
        'refresh-costmaps',
        'verify-costmaps',
        'ready',
    )

    def __init__(self, catalog):
        self.catalog = catalog
        self.token = None
        self.stage = 'idle'
        self.generation = 0
        self.target = None
        self.active = None
        self.hold = False
        self.activated = False
        self.error = ''
        self.seed_stamp = None
        self._seen = set()

    def begin(
        self,
        request_id,
        map_id,
        revision,
        pose,
        authorized,
        origin,
        session,
        telemetry_age,
        linear,
        angular,
        navigation,
        undock,
        hardware_busy,
    ):
        try:
            if str(UUID(request_id)) != request_id:
                raise ValueError()
        except (ValueError, TypeError, AttributeError):
            raise ValueError('request_id must be a canonical UUID') from None
        if not authorized:
            raise ValueError('operator authorization required')
        if self.stage not in ('idle', 'ready') or request_id in self._seen:
            raise ValueError('map recovery busy or request already consumed')
        if origin is None or not all(math.isfinite(v) for v in (telemetry_age, linear, angular)):
            raise ValueError('base telemetry unavailable')
        if not 0 <= telemetry_age <= 0.5 or abs(linear) > 0.005 or abs(angular) > 0.01:
            raise ValueError('fresh stationary telemetry required')
        if navigation or undock or hardware_busy:
            raise ValueError('another operation owns the robot')
        target = self.catalog.require(map_id, revision)
        if not self.catalog.contains(target, pose[0], pose[1]) or not math.isfinite(pose[2]):
            raise ValueError('pose outside target map or non-finite')
        self._seen.add(request_id)
        self.generation += 1
        self.token = MapToken(request_id, self.generation, origin, session)
        self.target = target
        self.hold = True
        self.activated = False
        self.stage = 'hold'
        self.seed_stamp = None
        self.error = ''
        return self.token

    def advance(self, token, completed):
        if (
            token != self.token
            or completed != self.stage
            or self.stage in ('failed', 'ready', 'idle')
        ):
            return False
        if completed == 'verify-map':
            self.active = self.target
            self.activated = True
        if completed == 'verify-costmaps':
            self.hold = False
        self.stage = self.STAGES[self.STAGES.index(completed) + 1]
        return True

    def seeded(self, token, stamp):
        if token != self.token or self.stage != 'seed' or self.seed_stamp is not None:
            return False
        if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0:
            return False
        self.seed_stamp = stamp
        return self.advance(token, 'seed')

    def fail(self, reason):
        if self.token is not None:
            self.hold = True
            self.stage = 'failed'
            self.error = reason
