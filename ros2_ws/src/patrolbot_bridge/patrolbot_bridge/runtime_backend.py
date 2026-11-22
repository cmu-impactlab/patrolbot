"""Fixed native services and core admission, without tuning/localization effects."""
from uuid import uuid4
import threading
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup
from patrolbot_interfaces.srv import RuntimeManual
from std_srvs.srv import SetBool
from .tuning_activation import evaluate_evidence
from .epoch_recovery import EpochRecovery
from .runtime_contract import number, uuid


FIELDS = ('available', 'generation', 'hardware_generation', 'reservation', 'linear_m_s', 'angular_rad_s',
          'effective_linear_m_s', 'effective_angular_rad_s', 'upper_linear_m_s', 'upper_angular_rad_s')


class RuntimeBackend:
    def __init__(self, node, key=None, *, enabled=False):
        self.enabled = enabled
        self.key = key
        self.actor = None
        self.node = node
        self.group = ReentrantCallbackGroup()
        self.clients = {name: node.create_client(RuntimeManual, '/patrolbot/' + path + '_runtime_manual',
                                                callback_group=self.group)
                        for name, path in (('web', 'web'), ('f710', 'joy'))}
        # Existing mutex-backed navigator gate only; never EpochRuntime's tuning hold.
        self.nav = node.create_client(SetBool, '/bt_navigator/tuning_hold', callback_group=self.group)
        self.reservation = ''
        self.ownership_generation = 0
        # Discover surviving reservations before admitting any new motion.
        # This startup task never reapplies settings or releases a remote hold.
        self.initializing = enabled
        self.initializing_busy = False
        node._epoch.runtime_hold = enabled
        node._epoch.runtime_block_drive = enabled
        self.startup_timer = node.create_timer(.5, self.initialize, callback_group=self.group)

    async def initialize(self):
        if not self.enabled or self.initializing_busy:
            return
        if not self.initializing and self.reservation:
            return
        self.initializing_busy = True
        ownership = self.ownership_generation
        try:
            states = [await self.consumer(name, 'read', str(uuid4())) for name in ('web', 'f710')]
            with self.node._epoch.lock:
                if ownership != self.ownership_generation:
                    return  # A transaction retired this read, including a reserve/release ABA.
                if not self.initializing:
                    if any(state['reservation'] for state in states) and not self.reservation:
                        self.reservation = next(state['reservation'] for state in states if state['reservation'])
                        self.uncertain()
                    return
                held = [state['reservation'] for state in states if state['reservation']]
                if held:
                    self.reservation = held[0]
                else:
                    self.node._epoch.runtime_hold = False
                    self.node._epoch.runtime_block_drive = False
                self.initializing = False
                # Keep monitoring enabled epochs; a lost consumer never disables policy.
        except (ValueError, RuntimeError, TimeoutError):
            if ownership == self.ownership_generation:
                self.uncertain()
        finally:
            self.initializing_busy = False

    @property
    def block_drive(self):
        return self.node._epoch.runtime_block_drive

    def identity(self):
        n = self.node
        policy = n._tuning_policy
        upper = policy.limits()
        return n._session_generation, (upper['max_linear_m_s'], upper['max_angular_rad_s']), policy.generation

    def check(self, identity):
        if self.identity() != identity:
            raise RuntimeError('Hardware or active policy generation changed')
        allowed, reason = evaluate_evidence(self.node._tuning_evidence())
        epoch = self.node._epoch
        if (epoch._pending_pose is not None
                or epoch.recovery.stage not in (EpochRecovery.READY, EpochRecovery.WAIT_SEED)
                or (epoch.recovery.stage == EpochRecovery.READY and not epoch._navigation_ready)):
            raise ValueError('Localization or navigation readiness transition is pending')
        if not allowed or epoch.tuning_hold:
            raise ValueError(reason or 'Tuning owns admission')

    def reserve(self, request_id, actor):
        n = self.node
        with n._epoch.lock:
            if not self.enabled:
                raise ValueError('Runtime controls are statically disabled')
            if self.reservation or getattr(n._epoch, 'runtime_hold', False):
                raise ValueError('Runtime reservation is pending or unknown')
            self.check(self.identity())
            with n._control_lock:
                if n._pending_controls:
                    raise ValueError('Hardware control pending or unknown')
            if (n._manual_undock.busy or n._manual_undock.unknown
                    or n._undock_turn.busy or getattr(n._undock_turn, 'unknown', False)
                    or n._undock_goal_reserved):
                raise ValueError('Exclusive motion owner pending or unknown')
            if not self.nav.service_is_ready() or not all(c.service_is_ready() for c in self.clients.values()):
                raise ValueError('Runtime consumers unavailable')
            self.ownership_generation += 1
            self.actor = actor
            self.reservation = request_id
            n._epoch.runtime_hold = True

    def recover(self, request_id, actor):
        # Current operator adopts ownership only; never reapply saved ceilings.
        with self.node._epoch.lock:
            if not self.enabled or self.initializing:
                raise ValueError('Runtime mode unavailable or discovery pending')
            self.check(self.identity())
            n = self.node
            with n._control_lock:
                if n._pending_controls:
                    raise ValueError('Hardware control pending or unknown')
            if (n._manual_undock.busy or n._manual_undock.unknown
                    or n._undock_turn.busy or getattr(n._undock_turn, 'unknown', False)
                    or n._undock_goal_reserved):
                raise ValueError('Exclusive owner pending or unknown')
            self.ownership_generation += 1
            self.actor = actor
            self.reservation = request_id
            n._epoch.runtime_hold = True
            n._epoch.runtime_block_drive = True

    def begin_commit(self, identity):
        with self.node._epoch.lock:
            self.check(identity)
            if not self.reservation:
                raise RuntimeError('Core runtime reservation was lost')
            self.node._epoch.runtime_block_drive = True

    def uncertain(self):
        with self.node._epoch.lock:
            self.ownership_generation += 1
            self.node._epoch.runtime_block_drive = True
            self.node._epoch.runtime_hold = True

    def release(self, request_id):
        with self.node._epoch.lock:
            if self.reservation != request_id:
                raise RuntimeError('Core reservation identity changed')
            self.ownership_generation += 1
            self.node._epoch.runtime_hold = False
            self.node._epoch.runtime_block_drive = False
            self.reservation = ''

    async def call(self, client, request):
        if not client.service_is_ready():
            raise RuntimeError('Required runtime consumer is unreachable')
        result = Future()
        completion = threading.Lock()
        future = client.call_async(request)
        def expired():
            with completion:
                if not result.done():
                    result.set_exception(TimeoutError('Runtime consumer outcome unknown; reservation retained'))
        timer = self.node.create_timer(.6, expired, callback_group=self.group)
        def done(value):
            with completion:
                if not result.done():
                    try:
                        result.set_result(value.result())
                    except Exception as exc:
                        result.set_exception(RuntimeError(str(exc)))
        future.add_done_callback(done)
        try:
            return await result
        finally:
            self.node.destroy_timer(timer)
            if not future.done():
                client.remove_pending_request(future)
                future.cancel()

    async def navigation(self, held):
        request = SetBool.Request()
        request.data = held
        response = await self.call(self.nav, request)
        if not response.success:
            raise ValueError('Navigation owns motion; settings refused')

    async def consumer(self, name, phase, request_id, evidence=None, values=None):
        request = RuntimeManual.Request()
        request.phase, request.request_id = phase, request_id
        if evidence:
            request.expected_generation = evidence['generation']
            request.hardware_generation = evidence['hardware_generation']
        if values:
            request.linear_m_s = float(values['linear_m_s'])
            request.angular_rad_s = float(values['angular_rad_s'])
        if phase != 'read':
            from .runtime_consumer_authorization import sign, claims
            request.authorization_json = sign(claims(request), self.actor, self.key,
                self.node._tuning_robot_id, name)
        response = await self.call(self.clients[name], request)
        if not response.accepted:
            raise ValueError(response.detail)
        value = {key: getattr(response, key) for key in FIELDS}
        if (not uuid(value['generation']) or not isinstance(value['reservation'], str)
                or (value['reservation'] and not uuid(value['reservation']))
                or type(value['available']) is not bool
                or type(value['hardware_generation']) is not int
                or not 0 <= value['hardware_generation'] < 2**53):
            raise RuntimeError('Invalid runtime consumer identity')
        for key in FIELDS[4:]:
            if not number(value[key], 0, .3 if 'linear' in key else .7):
                raise RuntimeError('Invalid consumer speed evidence')
        if (value['effective_linear_m_s'] > value['linear_m_s']
                or value['effective_angular_rad_s'] > value['angular_rad_s']):
            raise RuntimeError('Effective trim exceeds selected manual ceiling')
        return value
