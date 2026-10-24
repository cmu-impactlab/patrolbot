"""Exercise real scoped executor with isolated service I/O, not native claims."""

import copy
import json
from pathlib import Path
import threading

import pytest

from patrolbot_bridge.tuning_activation import ActivationCoordinator, AdmissionRefused, Evidence
from patrolbot_bridge.tuning_backend import NODES, parameter_plan
from patrolbot_bridge.tuning_scoped import PRODUCERS, ScopedLifecycleBackend
from patrolbot_bridge.tuning_store import RevisionStore

ROOT = Path(__file__).resolve().parents[4]
REGISTRY = json.loads((ROOT / 'contracts/robot-tuning.registry.json').read_text())
ACTOR = {'id': 1, 'username': 'isolated-service-fixture', 'role': 'administrator'}
SESSION = '11111111-1111-4111-8111-111111111111'
REQUEST = '33333333-3333-4333-8333-333333333333'


class Owner:
    def __init__(self):
        self.lock = threading.RLock()
        self.tuning_hold = False
        self.job = None

    def set_tuning_hold(self, held, reason=''):
        self.tuning_hold = held

    def reserve_tuning(self, job):
        if self.job is not None:
            return False
        self.job = job
        return True

    def release_tuning_reservation(self, job):
        if self.job != job:
            return False
        self.job = None
        return True


class Transport:
    def __init__(self, store):
        self.store = store
        self.values = copy.deepcopy(parameter_plan(store.load_revision(1), store.index))
        self.states = dict.fromkeys(NODES, 2)
        self.calls = []
        self.leases = {}
        self.fail = None
        self.bad_cache = False
        self.bad_unchanged = False
        self.capabilities = {'ready': True, 'implementation': 'ISOLATED-FAKE-NOT-NATIVE-PROOF',
                             'support': {node: 'parameters' if node == 'patrolbot_joy_teleop' else 'lifecycle' for node in NODES},
                             'policy_support': True}

    def scoped_capabilities(self):
        return copy.deepcopy(self.capabilities)

    def maintenance(self, peer, operation, context):
        self.calls.append(('lease', peer, operation))
        if self.fail == (peer, operation):
            raise TimeoutError('may have executed')
        if self.fail == ('busy', peer) and operation == 'acquire':
            raise AdmissionRefused('existing owner wins')
        if operation == 'acquire':
            assert self.store.latest_job()['reservation_attempts'][-1] == peer
            assert peer not in self.leases
            self.leases[peer] = dict(context)
        elif operation == 'release':
            assert self.leases.pop(peer) == context
        else:
            assert self.leases[peer] == context
        return {'idle': True, 'owner': dict(context)}

    def state(self, node):
        return self.states[node]

    def scoped_transition(self, node, verb, context):
        self.calls.append(('transition', node, verb))
        self.states[node] = {'DEACTIVATE': 2, 'CLEANUP': 1, 'CONFIGURE': 2, 'ACTIVATE': 3}[verb]
        if self.fail == (node, verb):
            raise TimeoutError('transition result lost')

    def set(self, node, values):
        self.calls.append(('set', node, dict(values)))
        self.values[node].update(values)
        if self.fail == (node, 'set'):
            raise TimeoutError('parameter result lost')

    def get(self, node, names):
        values = {key: self.values[node][key] for key in names}
        if self.bad_unchanged and node == 'amcl':
            values['alpha1'] = .99
        return values

    def stage_policies(self, plan, context):
        self.calls.append(('stage_policies', tuple(plan['cache_consumers'])))

    def verify_consumers(self, expected, plan, context):
        if self.bad_cache:
            raise ValueError('native cache still old despite matching GetParameters')
        return {'verified_nodes': sorted(NODES), 'verified_policies': ['core_clamp', 'f710_cache', 'web_sessions']}

    def verify_invariants(self, context):
        return {'motion_paths_held': True, 'protected_invariants_verified': True,
                'publisher_graph': {'/cmd_vel_out': ['cmd_vel_mux'], '/cmd_vel': ['teleop_velocity_smoother']}}

    def require_operator_seed(self, plan, context):
        self.calls.append(('require_seed', plan['revision']))

    def commit_policies(self, plan, context):
        assert self.store.read_startup()['revision'] == plan['revision']
        self.calls.append(('commit_policies', plan['revision']))
        return True


def setup(tmp_path, changes=None, generation=None):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize(actor=ACTOR, reason='isolated scoped algorithm fixture')
    store.save(request_id='22222222-2222-4222-8222-222222222222', base_revision=1,
               changes=changes or [{'id': 'nav2.controller.approach_velocity_scaling_dist', 'value': .7}], actor=ACTOR)
    owner = Owner()
    transport = Transport(store)
    backend = ScopedLifecycleBackend(store, transport,
        lambda: Evidence(raw_odom_fresh=None, stationary=None, hardware_ok=True, dock_ok=True,
            faults_clear=True, command_buffers_zero=True, physical_deadman_fresh=True,
            physical_deadman_held=False, web_deadman_fresh=True, web_deadman_held=False,
            autonomy_idle=True, maps_clear=True, undock_clear=True, lifecycle_clear=True),
        lambda _: pytest.fail('legacy seed callback must not execute'), enabled=True,
        owner=owner, generation=generation or (lambda: (1, 1)),
        selected_map=lambda: pytest.fail('unrelated plans never require selected map'))
    coordinator = ActivationCoordinator(store, owner, backend)
    coordinator.session_id = SESSION
    return store, owner, transport, backend, coordinator


def run(coordinator):
    plan = coordinator.preview()
    assert plan['executable'], plan['reasons']
    return coordinator.request(operation='apply', request_id=REQUEST, expected_revision=2, actor=ACTOR,
                               review={'digest': plan['digest'], 'plan_hash': plan['plan_hash']})


@pytest.mark.parametrize('state', [1, 2, 3])
def test_only_changed_consumer_reloads_and_prior_navigation_state_is_not_auto_activated(tmp_path, state):
    store, owner, transport, _, coordinator = setup(tmp_path)
    transport.states['controller_server'] = state
    result = run(coordinator)
    assert result['outcome'] == 'ok'
    mutations = [call for call in transport.calls if call[0] in ('set', 'transition', 'require_seed')]
    assert mutations and all(call[1] == 'controller_server' for call in mutations)
    transitions = [call[2] for call in mutations if call[0] == 'transition']
    assert transitions == ({1: ['CONFIGURE'], 2: ['CLEANUP', 'CONFIGURE'],
                            3: ['DEACTIVATE', 'CLEANUP', 'CONFIGURE', 'ACTIVATE']}[state])
    assert not owner.tuning_hold and not transport.leases and owner.job is None
    assert store.read_startup()['revision'] == 2


@pytest.mark.parametrize('pid,value,expected', [
    ('max_linear_m_s', .2, {'teleop_velocity_smoother', 'controller_server', 'velocity_smoother', 'patrolbot_joy_teleop'}),
    ('max_angular_rad_s', .5, {'teleop_velocity_smoother', 'controller_server', 'velocity_smoother', 'patrolbot_joy_teleop', 'behavior_server'}),
])
def test_speed_closure_refreshes_actual_consumers_not_amcl_or_map(tmp_path, pid, value, expected):
    _, _, transport, _, coordinator = setup(tmp_path, [{'id': pid, 'value': value}])
    assert run(coordinator)['outcome'] == 'ok'
    assert {call[1] for call in transport.calls if call[0] == 'set'} == expected
    assert not any(call[0] == 'require_seed' for call in transport.calls)
    assert not any(call[:2] == ('lease', 'lifecycle_manager_localization') for call in transport.calls)


def test_amcl_reset_has_explicit_seed_barrier_but_is_not_an_admission_prerequisite(tmp_path):
    _, _, transport, _, coordinator = setup(tmp_path, [{'id': 'amcl.alpha1', 'value': .06}])
    assert run(coordinator)['outcome'] == 'ok'
    assert ('require_seed', 2) in transport.calls
    assert {call[1] for call in transport.calls if call[0] == 'set'} == {'amcl'}


@pytest.mark.parametrize('failure', [('controller_server', 'CLEANUP'), ('controller_server', 'set'),
                                  ('controller_server', 'CONFIGURE')])
def test_partial_mutation_never_promotes_releases_or_retries(tmp_path, failure):
    store, owner, transport, _, coordinator = setup(tmp_path)
    transport.fail = failure
    assert run(coordinator)['outcome'] == 'unknown'
    assert owner.tuning_hold and owner.job is not None and transport.leases
    assert store.read_startup()['revision'] == 1
    before = list(transport.calls)
    assert run(coordinator)['outcome'] == 'unknown'
    assert transport.calls == before


@pytest.mark.parametrize('bad', ['bad_cache', 'bad_unchanged'])
def test_matching_changed_parameters_do_not_prove_unchanged_scope_or_consumed_cache(tmp_path, bad):
    store, owner, transport, _, coordinator = setup(tmp_path)
    setattr(transport, bad, True)
    assert run(coordinator)['outcome'] == 'unknown'
    assert store.read_startup()['revision'] == 1 and owner.tuning_hold


def test_definitive_admission_refusal_unwinds_only_our_acknowledged_peers(tmp_path):
    store, owner, transport, _, coordinator = setup(tmp_path)
    transport.fail = ('busy', 'web')
    assert run(coordinator)['outcome'] == 'refused'
    assert not transport.leases and owner.job is None and not owner.tuning_hold
    assert store.latest_job()['mutation_started'] is False


def test_lost_reservation_ack_retains_durable_attempt_and_local_exclusion(tmp_path):
    store, owner, transport, _, coordinator = setup(tmp_path)
    transport.fail = ('web', 'acquire')
    assert run(coordinator)['outcome'] == 'unknown'
    assert owner.tuning_hold and owner.job is not None
    assert store.latest_job()['reservation_attempts'] == ['navigation', 'physical', 'web']
    assert not any(call[2] == 'release' for call in transport.calls if call[0] == 'lease')


def test_missing_transport_protocol_never_becomes_activation_support(tmp_path):
    _, _, transport, backend, coordinator = setup(tmp_path)
    transport.capabilities['ready'] = False
    assert not backend.approved and not coordinator.preview()['executable']
