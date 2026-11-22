"""Runtime-only consumer reservation. Call under the consumer's admission lock.

An uncertain operation never expires into permission to move. No velocity or
motor operation belongs here. A consumer restart creates a new incarnation.
"""
from uuid import UUID, uuid4
import math
import time
from .runtime_consumer_authorization import verify, claims


def canonical(value):
    try:
        return isinstance(value, str) and str(UUID(value)) == value
    except (ValueError, TypeError, AttributeError):
        return False


class ManualRuntime:
    def __init__(self, observe, idle, apply, *, key=None, robot_id="patrolbot-01", consumer="", clock=time.monotonic):
        self.key, self.robot_id, self.consumer = key, robot_id, consumer
        # shortcut: 4096 edits per process; add expiry-aware eviction if this bound is approached.
        self.seen = set()
        self.committed = False
        self.observe, self.idle, self.apply = observe, idle, apply
        self.generation = str(uuid4())
        self.signature = None
        self.reservation = ''
        self.bound_generation = None
        self.require_release = False
        self.clock = clock
        self.release_cutoff = float("inf")

    @property
    def held(self):
        return bool(self.reservation)

    def state(self):
        state = self.observe()
        signature = tuple(sorted(state.items()))
        if signature != self.signature:
            self.generation, self.signature = str(uuid4()), signature
        state.pop('_identity', None)
        return dict(state, generation=self.generation, reservation=self.reservation, available=self.key is not None)

    def handle(self, request, response):
        state = self.state()
        try:
            if request.phase not in ('read', 'prepare', 'commit', 'release', 'reconcile') or not canonical(request.request_id):
                raise ValueError('Invalid runtime consumer request')
            if request.phase != 'read':
                verify(request.authorization_json, claims(request), self.key, self.robot_id, self.consumer)
                if (request.expected_generation != state['generation']
                        or request.hardware_generation != state['hardware_generation']):
                    raise ValueError('Consumer or hardware generation changed')
                if request.phase in ('prepare', 'commit'):
                    for value, minimum, upper in ((request.linear_m_s, .05, state['upper_linear_m_s']),
                                                   (request.angular_rad_s, .1, state['upper_angular_rad_s'])):
                        if type(value) not in (int, float) or not math.isfinite(value) or not minimum <= value <= upper:
                            raise ValueError('Runtime ceiling outside commissioned policy')
                if request.phase == 'reconcile':
                    if (self.held and self.reservation != request.request_id) or not self.idle():
                        raise ValueError('Recovery requires current reservation identity and fresh idle input')
                    self.reservation = ''
                    self.bound_generation = None
                    self.require_release = True
                    self.release_cutoff = self.clock()
                    self.generation = str(uuid4())
                elif request.phase == 'prepare':
                    if self.held or request.request_id in self.seen or len(self.seen) >= 4096 or not self.idle():
                        raise ValueError('Consumer busy, stale or input held')
                    self.seen.add(request.request_id)
                    self.committed = False
                    self.reservation = request.request_id
                    self.bound_generation = state['generation']
                    self.require_release = True
                else:
                    if self.reservation != request.request_id:
                        raise ValueError('Runtime reservation identity mismatch')
                    if request.phase == 'commit':
                        if self.committed or not self.idle() or self.bound_generation != state['generation']:
                            raise ValueError('Input or consumer state changed during reservation')
                        self.committed = True
                        self.apply(request.linear_m_s, request.angular_rad_s)
                        self.bound_generation = self.state()['generation']
                    else:
                        self.release_cutoff = self.clock()
                        self.reservation = ''
                        self.bound_generation = None
            response.accepted, response.detail = True, ''
        except (ValueError, RuntimeError) as exc:
            response.accepted, response.detail = False, str(exc)[:256]
        for key, value in self.state().items():
            setattr(response, key, value)
        return response
