"""Runtime manual transaction; effects require producer and core reservations."""
import json
from uuid import uuid4, uuid5, NAMESPACE_URL
from . import runtime_contract as contract
from .runtime_authorization import verify


class RuntimeControls:
    def __init__(self, robot_id, observe, backend=None, key=None):
        self.robot_id, self.observe, self.backend, self.key = robot_id, observe, backend, key
        self.generation = str(uuid4())
        # shortcut: 4096 edits per process; add expiry-aware eviction if this bound is approached.
        self.seen = set()
        self.status = 'default'
        self.requested = None
        self.busy = False
        self.confirmation = None

    async def inspect(self):
        state = contract.snapshot(self.robot_id, self.generation, self.observe())
        if self.backend is None or not getattr(self.backend, 'enabled', True):
            return state
        consumers = {name: await self.backend.consumer(name, 'read', str(uuid4()))
                     for name in ('web', 'f710')}
        hardware, upper, policy_generation = self.backend.identity()
        for value in consumers.values():
            if value['hardware_generation'] != hardware:
                raise ValueError('Consumer hardware generation mismatch')
        consumer_generation = str(uuid5(NAMESPACE_URL, json.dumps(consumers, sort_keys=True)))
        state['consumer_generation'] = consumer_generation
        state['state_generation'] = str(uuid5(NAMESPACE_URL,
            f'{self.generation}:{hardware}:{policy_generation}:{consumer_generation}:'
            f'{self.backend.reservation}:{getattr(self.backend, "block_drive", False)}'))
        incarnations = {name: value['generation'] for name, value in consumers.items()}
        if self.status == 'confirmed' and self.confirmation is not None and self.confirmation != incarnations:
            self.status, self.requested = 'default', None
        self.confirmation = incarnations
        state['status'] = 'unknown' if (self.backend.reservation or getattr(self.backend, 'block_drive', False)) and self.status in ('default', 'confirmed') else self.status
        state['pending'] = self.backend.reservation or None
        state['manual'].update(requested=self.requested, confirmed=consumers, reason='' if self.key and all(c['available'] for c in consumers.values()) else 'Runtime mutation authorization unavailable')
        state['manual']['linear_m_s']['max'] = min(upper[0], *(c['upper_linear_m_s'] for c in consumers.values()))
        state['manual']['angular_rad_s']['max'] = min(upper[1], *(c['upper_angular_rad_s'] for c in consumers.values()))
        if any(state['manual'][axis]['max'] < state['manual'][axis]['min']
               for axis in ('linear_m_s', 'angular_rad_s')):
            state['manual']['reason'] = 'Current policy has no admissible positive manual speed range.'
        if getattr(self.backend, 'initializing', False):
            state['manual']['reason'] = 'Runtime startup reconciliation is pending.'
        return state

    async def run(self, raw):
        value = contract.loads(raw)
        signed = isinstance(value, dict) and 'request' in value
        authorized = verify(value, self.key, self.robot_id) if signed else None
        payload = contract.request(authorized['request'] if authorized else value)
        request_id = payload['request_id']
        if payload['operation'] == 'read':
            try:
                state = await self.inspect()
                return dict(request_id=request_id, outcome='ok', detail='', snapshot=state)
            except (ValueError, RuntimeError, TimeoutError):
                if self.backend is not None and getattr(self.backend, 'enabled', True):
                    self.backend.uncertain()
                state = contract.snapshot(self.robot_id, self.generation, self.observe())
                state['manual']['reason'] = 'Fresh runtime consumer evidence is unavailable; refresh after recovery.'
                return dict(request_id=request_id, outcome='ok', detail='', snapshot=state)
        if not signed:
            raise ValueError('Signed runtime authorization required')
        if payload['target'] != 'manual' or self.backend is None or self.key is None:
            return dict(request_id=request_id, outcome='unavailable', detail=contract.NAV_REASON
                        if payload['target'] != 'manual' else contract.MANUAL_REASON, snapshot=None)
        if self.busy or request_id in self.seen or len(self.seen) >= 4096:
            return dict(request_id=request_id, outcome='rejected', detail='Runtime busy or request already used; refresh.', snapshot=None)
        self.seen.add(request_id)
        self.busy = True
        previous_status, previous_requested = self.status, self.requested
        reserved = False
        changed = False
        prepared = {}
        outcome, detail, state = 'rejected', '', None
        try:
            state = await self.inspect()
            previous_status, previous_requested = self.status, self.requested
            if state['state_generation'] != payload['state_generation']:
                raise ValueError('State generation changed; refresh before submitting')
            if payload['operation'] == 'reconcile':
                hardware = self.backend.identity()
                self.backend.recover(request_id, authorized['actor'])
                reserved = True
                await self.backend.navigation(True)
                # Fresh read binds each existing reservation, not a historical request.
                consumers = state['manual']['confirmed']
                for name, evidence in consumers.items():
                    self.backend.check(hardware)
                    current = await self.backend.consumer(name, 'read', str(uuid4()))
                    if current != evidence:
                        raise RuntimeError('Recovery evidence changed; refresh')
                    prepared[name] = await self.backend.consumer(name, 'reconcile',
                        evidence['reservation'] or request_id, evidence)
                self.backend.check(hardware)
                for name, evidence in prepared.items():
                    if evidence['reservation'] or await self.backend.consumer(name, 'read', str(uuid4())) != evidence:
                        raise RuntimeError('Recovery readback changed')
                await self.backend.navigation(False)
                self.backend.release(request_id)
                reserved = False
                self.status, self.requested, self.confirmation = 'default', None, None
                state = await self.inspect()
                return dict(request_id=request_id, outcome='ok',
                    detail='Ownership reconciled; current per-consumer ceilings retained. No setting reapplied; arm explicitly.', snapshot=state)
            if state['manual']['reason']:
                raise ValueError(state['manual']['reason'])
            values = payload['values']
            if (values['linear_m_s'] > state['manual']['linear_m_s']['max']
                    or values['angular_rad_s'] > state['manual']['angular_rad_s']['max']):
                raise ValueError('Requested speed exceeds current commissioned policy')
            hardware = self.backend.identity()
            self.backend.reserve(request_id, authorized['actor'])
            reserved = True
            self.status, self.requested = 'pending', values
            # The native gate is atomic with navigator goal admission. Status polling
            # is supplementary evidence, never the lock.
            await self.backend.navigation(True)
            for name in ('web', 'f710'):
                self.backend.check(hardware)
                prepared[name] = await self.backend.consumer(name, 'prepare', request_id,
                    state['manual']['confirmed'][name], values)
            self.backend.begin_commit(hardware)
            for name in ('web', 'f710'):
                self.backend.check(hardware)
                for peer, evidence in prepared.items():
                    if await self.backend.consumer(peer, 'read', str(uuid4())) != evidence:
                        raise RuntimeError('Prepared consumer or transport generation changed')
                await self.backend.navigation(True)
                # From here a timeout is uncertain; never rollback a reduction.
                changed = True
                prepared[name] = await self.backend.consumer(name, 'commit', request_id, prepared[name], values)
                actual = await self.backend.consumer(name, 'read', str(uuid4()))
                if (actual != prepared[name] or actual['linear_m_s'] != values['linear_m_s']
                        or actual['angular_rad_s'] != values['angular_rad_s']
                        or actual['effective_linear_m_s'] > values['linear_m_s']
                        or actual['effective_angular_rad_s'] > values['angular_rad_s']):
                    raise RuntimeError('Effective consumer readback mismatch')
            self.backend.check(hardware)
            for name in ('web', 'f710'):
                prepared[name] = await self.backend.consumer(name, 'release', request_id, prepared[name])
            await self.backend.navigation(False)
            self.backend.release(request_id)
            reserved = False
            outcome, self.status = 'confirmed', 'confirmed'
            self.confirmation = None
            state = await self.inspect()
        except ValueError as exc:
            detail = str(exc)[:512]
            if reserved:
                # A definite refusal can release only before any possible write.
                if not changed and payload['operation'] != 'reconcile':
                    try:
                        for name, evidence in prepared.items():
                            await self.backend.consumer(name, 'release', request_id, evidence)
                        for name in ('web', 'f710'):
                            current = await self.backend.consumer(name, 'read', str(uuid4()))
                            if current['reservation']:
                                raise RuntimeError('Cleanup reservation is unresolved')
                        await self.backend.navigation(False)
                        self.backend.release(request_id)
                        reserved = False
                    except (ValueError, RuntimeError, TimeoutError):
                        pass
                outcome = 'partial' if changed else ('unknown' if reserved else 'rejected')
                if reserved:
                    self.status = outcome
                else:
                    self.status, self.requested = previous_status, previous_requested
            else:
                outcome = 'unknown' if outcome == 'confirmed' else 'rejected'
                state = None
        except (RuntimeError, TimeoutError) as exc:
            detail = str(exc)[:512]
            if reserved:
                self.status = outcome = 'partial' if changed else 'unknown'
            else:
                outcome = 'unknown' if outcome == 'confirmed' else 'rejected'
                state = None
        finally:
            self.busy = False
        if reserved:
            self.backend.uncertain()
            state = None
        elif outcome == 'rejected':
            try:
                state = await self.inspect()
            except (ValueError, RuntimeError, TimeoutError):
                state = None
        return dict(request_id=request_id, outcome=outcome, detail=detail, snapshot=state)

    async def callback(self, request, response):
        try:
            result = await self.run(request.payload_json)
        except (ValueError, TypeError, KeyError, OverflowError, UnicodeError):
            result = {'request_id': None, 'outcome': 'unavailable',
                      'detail': 'Invalid or unauthorized runtime request', 'snapshot': None}
        response.result_json = json.dumps(result, allow_nan=False)
        return response
