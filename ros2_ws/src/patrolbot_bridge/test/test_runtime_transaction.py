"""Real coordinator and real consumer state machines, no DDS, devices or motors."""
import asyncio
import copy
import json
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from uuid import uuid4
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from patrolbot_bridge.runtime_controls import RuntimeControls
from patrolbot_bridge.runtime_manual import ManualRuntime
from patrolbot_bridge.runtime_authorization import sign, verify
from patrolbot_bridge.runtime_consumer_authorization import sign as delegate, claims
from patrolbot_bridge import runtime_contract as c

KEY = b'offline-test-only-key-material-32bytes'
ACTOR = dict(id=7, username='operator', role='operator')


class Backend:
    def __init__(self):
        self.reservation = ''
        self.motion = False
        self.block_drive = False
        self.navigation_held = False
        self.events = []
        self.hardware = 2
        self.policy = 'policy-1'
        self.failure = None
        self.hook = None
        self.states = {name: dict(hardware_generation=2, linear_m_s=.3, angular_rad_s=.7,
            effective_linear_m_s=.25 if name == 'f710' else .3,
            effective_angular_rad_s=.6 if name == 'f710' else .7,
            upper_linear_m_s=.3, upper_angular_rad_s=.7) for name in ('web', 'f710')}
        self.idle = {name: True for name in self.states}
        def apply(name, linear, angular):
            self.events.append(('write', name, linear, angular))
            state = self.states[name]
            state.update(linear_m_s=linear, angular_rad_s=angular,
                effective_linear_m_s=min(state['effective_linear_m_s'], linear),
                effective_angular_rad_s=min(state['effective_angular_rad_s'], angular))
        self.consumers = {name: ManualRuntime(lambda name=name: self.states[name].copy(),
            lambda name=name: self.idle[name] and not self.motion,
            lambda linear, angular, name=name: apply(name, linear, angular), key=KEY, robot_id="fixture", consumer=name) for name in self.states}

    def identity(self):
        return self.hardware, (.3, .7), self.policy

    def check(self, identity):
        if identity != self.identity():
            raise RuntimeError('Generation changed')
        if self.motion:
            raise ValueError('Motion owns admission')

    def reserve(self, request_id, actor):
        self.check(self.identity())
        if self.reservation:
            raise ValueError('Reservation pending or unknown')
        self.reservation = request_id

    def recover(self, request_id, actor):
        self.check(self.identity())
        self.reservation = request_id
        self.block_drive = True

    def begin_commit(self, identity):
        self.check(identity)
        self.block_drive = True

    def uncertain(self):
        self.block_drive = True

    def release(self, request_id):
        assert self.reservation == request_id
        self.reservation = ''
        self.block_drive = False

    def admit_motion(self):
        if self.reservation or self.navigation_held or any(c.held for c in self.consumers.values()):
            return False
        self.motion = True
        return True

    async def navigation(self, held):
        if held and self.motion:
            raise ValueError('Native navigation gate refused')
        self.navigation_held = held

    async def consumer(self, name, phase, request_id, evidence=None, values=None):
        if self.hook:
            await self.hook(name, phase)
        if self.failure == (name, phase, 'before'):
            raise TimeoutError('Lost consumer response')
        request = NS(phase=phase, request_id=request_id,
            expected_generation=(evidence or {}).get('generation', ''),
            hardware_generation=(evidence or {}).get('hardware_generation', 0),
            linear_m_s=(values or {}).get('linear_m_s', 0), angular_rad_s=(values or {}).get('angular_rad_s', 0))
        request.authorization_json = delegate(claims(request), ACTOR, KEY, "fixture", name)
        response = self.consumers[name].handle(request, NS())
        if self.failure == (name, phase, 'after'):
            raise TimeoutError('Lost consumer response after effect')
        if not response.accepted:
            raise ValueError(response.detail)
        return {k: v for k, v in vars(response).items() if k not in ('accepted', 'detail')}


class Transactions(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.backend = Backend()
        self.service = RuntimeControls('fixture', lambda: None, self.backend, KEY)

    async def mutation(self, state=None, **changes):
        state = state or await self.service.inspect()
        payload = dict(contract=c.CONTRACT, version=1, operation='set', request_id=str(uuid4()),
            state_generation=state['state_generation'], target='manual', values=dict(linear_m_s=.1, angular_rad_s=.3))
        payload.update(changes)
        return json.dumps(sign(dict(request=payload, actor=ACTOR), KEY, 'fixture'))

    async def test_real_prepare_atomic_apply_readback_commit_and_no_motion(self):
        result = await self.service.run(await self.mutation())
        self.assertEqual(result['outcome'], 'confirmed')
        self.assertEqual(c.response(result), result)
        self.assertEqual(self.backend.events, [('write', 'web', .1, .3), ('write', 'f710', .1, .3)])
        self.assertFalse(self.backend.motion)
        self.assertFalse(self.backend.reservation)
        for consumer in self.backend.consumers.values():
            self.assertFalse(consumer.held)
            self.assertTrue(consumer.require_release)
        self.assertEqual(result['snapshot']['navigation']['confirmed'], None)

    async def test_motion_first_rejects_without_effect(self):
        raw = await self.mutation()
        self.assertTrue(self.backend.admit_motion())
        result = await self.service.run(raw)
        self.assertEqual(result['outcome'], 'rejected')
        self.assertEqual(self.backend.events, [])

    async def test_settings_first_blocks_new_motion_on_every_phase(self):
        attempts = []
        async def hook(name, phase):
            if self.backend.reservation:
                attempts.append(self.backend.admit_motion())
        self.backend.hook = hook
        result = await self.service.run(await self.mutation())
        self.assertEqual(result['outcome'], 'confirmed')
        self.assertTrue(attempts)
        self.assertFalse(any(attempts))

    async def test_uncertain_write_never_rolls_back_or_releases(self):
        for timing in ('before', 'after'):
            with self.subTest(timing=timing):
                await self.asyncSetUp()
                raw = await self.mutation()
                self.backend.failure = ('f710', 'commit', timing)
                result = await self.service.run(raw)
                self.assertEqual(result['outcome'], 'partial')
                self.assertTrue(self.backend.reservation)
                self.assertFalse(self.backend.admit_motion())
                self.assertEqual(self.backend.states['web']['linear_m_s'], .1)
                self.assertFalse(any(e[2] > .1 for e in self.backend.events))
                self.assertEqual((await self.service.run(raw))['outcome'], 'rejected')

    async def test_uncertain_prepare_and_release_keep_safe_ownership(self):
        for name, phase in [('web', 'prepare'), ('f710', 'prepare'), ('web', 'release'), ('f710', 'release')]:
            with self.subTest(name=name, phase=phase):
                await self.asyncSetUp()
                raw = await self.mutation()
                self.backend.failure = (name, phase, 'after')
                result = await self.service.run(raw)
                self.assertIn(result['outcome'], ('partial', 'unknown'))
                self.assertTrue(self.backend.reservation)
                self.assertFalse(self.backend.admit_motion())

    async def test_held_stale_input_and_external_generation_reject(self):
        state = await self.service.inspect()
        self.backend.idle['f710'] = False
        result = await self.service.run(await self.mutation(state))
        self.assertEqual(result['outcome'], 'rejected')
        self.assertEqual(self.backend.events, [])
        self.assertFalse(self.backend.reservation)
        self.backend.idle['f710'] = True
        self.backend.states['f710']['effective_linear_m_s'] = .2
        result = await self.service.run(await self.mutation(state))
        self.assertEqual(result['outcome'], 'rejected')
        self.assertEqual(self.backend.events, [])

    async def test_consumer_restart_mid_commit_is_partial(self):
        async def restart(name, phase):
            if name == 'f710' and phase == 'commit':
                self.backend.consumers['f710'].generation = str(uuid4())
        self.backend.hook = restart
        result = await self.service.run(await self.mutation())
        self.assertEqual(result['outcome'], 'partial')
        self.assertTrue(self.backend.reservation)

    async def test_hardware_and_policy_change_invalidate_before_effect(self):
        for field in ('hardware', 'policy'):
            await self.asyncSetUp()
            raw = await self.mutation()
            async def change(name, phase):
                if phase == 'prepare':
                    setattr(self.backend, field, 3 if field == 'hardware' else 'policy-2')
            self.backend.hook = change
            result = await self.service.run(raw)
            self.assertEqual(result['outcome'], 'unknown')
            self.assertEqual(self.backend.events, [])
            self.assertTrue(self.backend.reservation)

    async def test_replay_wrong_subject_role_expiry_and_tampering(self):
        raw = await self.mutation()
        body = json.loads(raw)
        for bad in (dict(body, actor=dict(ACTOR, role='observer')),
                    dict(body, authorization=dict(body['authorization'], robot_id='other')),
                    dict(body, authorization=dict(body['authorization'], expires_at_ms=1))):
            with self.assertRaises(ValueError): await self.service.run(json.dumps(bad))
        with self.assertRaises(ValueError): await self.service.run(json.dumps(body['request']))
        with self.assertRaises(ValueError): verify(body, KEY, 'other')
        self.assertEqual(self.backend.events, [])
        first = await self.service.run(raw)
        self.assertEqual(first['outcome'], 'confirmed')
        count = len(self.backend.events)
        self.assertEqual((await self.service.run(raw))['outcome'], 'rejected')
        self.assertEqual(len(self.backend.events), count)

    async def test_cross_purpose_tuning_signature_never_authorizes_runtime(self):
        from patrolbot_bridge.tuning_authorization import sign as tuning_sign
        body = json.loads(await self.mutation())
        # Same crypto/key but a different domain cannot pass runtime verification.
        import hashlib, hmac
        signed = copy.deepcopy(body)
        signature = signed['authorization'].pop('signature')
        signed['authorization']['signature'] = hmac.new(KEY, b'patrolbot-tuning-authorization-v1\n' +
            json.dumps(signed, sort_keys=True, separators=(',', ':')).encode(), hashlib.sha256).hexdigest()
        with self.assertRaises(ValueError): verify(signed, KEY, 'fixture')
        self.assertEqual(self.backend.events, [])


    async def test_physical_input_wins_prepare_race_without_stop_or_release_into_motion(self):
        async def motion_first(name, phase):
            if name == 'f710' and phase == 'prepare':
                # A physical edge arriving before F710's local reservation wins.
                self.assertFalse(self.backend.block_drive)
                self.backend.idle['f710'] = False
        self.backend.hook = motion_first
        result = await self.service.run(await self.mutation())
        self.assertEqual(result['outcome'], 'rejected')
        self.assertFalse(self.backend.reservation)
        self.assertFalse(self.backend.block_drive)
        self.assertFalse(self.backend.events)


if __name__ == '__main__': unittest.main()

class ManualFixRegressions(unittest.IsolatedAsyncioTestCase):
    asyncSetUp = Transactions.asyncSetUp
    mutation = Transactions.mutation
    async def test_rejected_prepare_preserves_healthy_policy_on_repeated_reads(self):
        raw = await self.mutation()
        self.backend.idle['f710'] = False
        result = await self.service.run(raw)
        self.assertEqual(result['outcome'], 'rejected')
        self.backend.idle['f710'] = True
        for _ in range(3):
            state = await self.service.inspect()
            self.assertEqual(state['status'], 'default')
            self.assertIsNone(state['pending'])
        self.assertTrue(self.backend.admit_motion())

    async def test_explicit_reconcile_lost_release_response_without_reapplying(self):
        self.backend.failure = ('f710', 'release', 'after')
        result = await self.service.run(await self.mutation())
        self.assertEqual(result['outcome'], 'partial')
        self.backend.failure = None
        state = await self.service.inspect()
        payload = dict(contract=c.CONTRACT, version=1, operation='reconcile', request_id=str(uuid4()),
            state_generation=state['state_generation'], target='manual', values={})
        writes = list(self.backend.events)
        result = await self.service.run(json.dumps(sign(dict(request=payload, actor=ACTOR), KEY, 'fixture')))
        self.assertEqual(result['outcome'], 'ok')
        self.assertFalse(self.backend.reservation)
        self.assertEqual(self.backend.events, writes)
        self.assertEqual(result['snapshot']['manual']['confirmed']['web']['linear_m_s'], .1)

    async def test_reconcile_all_lost_phases_and_partial_unequal_values_without_writes(self):
        for phase in ('prepare', 'commit', 'release'):
            for timing in ('before', 'after'):
                with self.subTest(phase=phase, timing=timing):
                    await self.asyncSetUp()
                    self.backend.failure = ('f710', phase, timing)
                    await self.service.run(await self.mutation())
                    self.backend.failure = None
                    state = await self.service.inspect()
                    before = list(self.backend.events)
                    payload = dict(contract=c.CONTRACT, version=1, operation='reconcile', request_id=str(uuid4()),
                        state_generation=state['state_generation'], target='manual', values={})
                    result = await self.service.run(json.dumps(sign(dict(request=payload, actor=ACTOR), KEY, 'fixture')))
                    self.assertEqual(result['outcome'], 'ok')
                    self.assertEqual(result['snapshot']['status'], 'default')
                    self.assertIsNone(result['snapshot']['manual']['requested'])
                    self.assertFalse(self.backend.reservation)
                    self.assertEqual(self.backend.events, before)
                    self.assertTrue(all(x.require_release for x in self.backend.consumers.values()))
                    self.assertEqual(c.response(result), result)
                    if phase == 'commit' and timing == 'before':
                        self.assertEqual(result['snapshot']['manual']['confirmed']['web']['linear_m_s'], .1)
                        self.assertEqual(result['snapshot']['manual']['confirmed']['f710']['linear_m_s'], .3)

    async def test_reconcile_busy_stale_generation_and_unsigned_cannot_unlock(self):
        self.backend.failure = ('f710', 'commit', 'after')
        await self.service.run(await self.mutation())
        self.backend.failure = None
        state = await self.service.inspect()
        payload = dict(contract=c.CONTRACT, version=1, operation='reconcile', request_id=str(uuid4()),
            state_generation=state['state_generation'], target='manual', values={})
        with self.assertRaises(ValueError): await self.service.run(json.dumps(payload))
        self.backend.idle['f710'] = False
        result = await self.service.run(json.dumps(sign(dict(request=payload, actor=ACTOR), KEY, 'fixture')))
        self.assertIn(result['outcome'], ('unknown', 'partial'))
        self.assertTrue(self.backend.reservation)
        self.assertFalse(self.backend.admit_motion())
        self.backend.idle['f710'] = True
        payload.update(request_id=str(uuid4()))
        result = await self.service.run(json.dumps(sign(dict(request=payload, actor=ACTOR), KEY, 'fixture')))
        self.assertEqual(result['outcome'], 'rejected')
        self.assertTrue(self.backend.reservation)

    async def test_consumer_restart_retires_requested_confirmation_without_replay(self):
        await self.service.run(await self.mutation())
        before = list(self.backend.events)
        self.backend.states['f710'].update(linear_m_s=.3, angular_rad_s=.7)
        self.backend.consumers['f710'].generation = str(uuid4())
        state = await self.service.inspect()
        self.assertEqual(state['status'], 'default')
        self.assertIsNone(state['manual']['requested'])
        self.assertEqual(state['manual']['confirmed']['f710']['linear_m_s'], .3)
        self.assertEqual(state['manual']['confirmed']['web']['linear_m_s'], .1)
        self.assertEqual(self.backend.events, before)

    async def test_missing_mutation_keys_do_not_remove_readable_limits(self):
        await self.service.run(await self.mutation())
        self.service.key = None
        for consumer in self.backend.consumers.values(): consumer.key = None
        state = await self.service.inspect()
        self.assertEqual(state['status'], 'confirmed')
        self.assertEqual(state['manual']['confirmed']['web']['linear_m_s'], .1)
        self.assertTrue(state['manual']['reason'])

    async def test_failure_before_ownership_does_not_poison_current_state(self):
        raw = await self.mutation()
        self.backend.failure = ('web', 'read', 'before')
        result = await self.service.run(raw)
        self.assertEqual(result['outcome'], 'rejected')
        self.assertFalse(self.backend.reservation)
        self.assertFalse(self.backend.block_drive)
        self.backend.failure = None
        self.assertEqual((await self.service.inspect())['status'], 'default')

    async def test_read_failure_after_verified_release_is_unknown_outcome_without_hold(self):
        raw = await self.mutation()
        async def fail_last_read(name, phase):
            if phase == 'read' and self.backend.events and not self.backend.reservation:
                raise TimeoutError('Post-release inspection lost')
        self.backend.hook = fail_last_read
        result = await self.service.run(raw)
        self.assertEqual(result['outcome'], 'unknown')
        self.assertFalse(self.backend.reservation)
        self.assertFalse(self.backend.block_drive)
        self.backend.hook = None
        self.assertEqual((await self.service.inspect())['status'], 'confirmed')

    async def test_reconcile_fences_a_delayed_old_prepare(self):
        from patrolbot_bridge.runtime_consumer_authorization import sign as consumer_sign
        old = self.backend.consumers['web'].state()
        state = await self.service.inspect()
        payload = dict(contract=c.CONTRACT, version=1, operation='reconcile', request_id=str(uuid4()),
            state_generation=state['state_generation'], target='manual', values={})
        result = await self.service.run(json.dumps(sign(dict(request=payload, actor=ACTOR), KEY, 'fixture')))
        self.assertEqual(result['outcome'], 'ok')
        late = NS(phase='prepare', request_id=str(uuid4()), expected_generation=old['generation'],
                  hardware_generation=2, linear_m_s=.1, angular_rad_s=.3)
        late.authorization_json = consumer_sign(claims(late), ACTOR, KEY, 'fixture', 'web')
        self.assertFalse(self.backend.consumers['web'].handle(late, NS()).accepted)
        self.assertFalse(self.backend.consumers['web'].held)

    async def test_rejection_cannot_resurrect_confirmation_retired_by_consumer_restart(self):
        await self.service.run(await self.mutation())
        raw = await self.mutation()
        self.backend.consumers['f710'].generation = str(uuid4())
        result = await self.service.run(raw)
        self.assertEqual(result['outcome'], 'rejected')
        self.assertEqual(result['snapshot']['status'], 'default')
        self.assertIsNone(result['snapshot']['manual']['requested'])

    async def test_core_hold_changes_retire_pre_hold_recovery_generation(self):
        state = await self.service.inspect()
        self.backend.uncertain()
        payload = dict(contract=c.CONTRACT, version=1, operation='reconcile', request_id=str(uuid4()),
            state_generation=state['state_generation'], target='manual', values={})
        result = await self.service.run(json.dumps(sign(dict(request=payload, actor=ACTOR), KEY, 'fixture')))
        self.assertEqual(result['outcome'], 'rejected')
        self.assertTrue(self.backend.block_drive)
        self.assertEqual(result['snapshot']['status'], 'unknown')
