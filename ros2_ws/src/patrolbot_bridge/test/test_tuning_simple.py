"""Reduced executor with offline I/O; never a native/Docker commissioning claim."""

import copy
import json
from pathlib import Path

import pytest

from patrolbot_bridge.tuning_activation import ActivationCoordinator, AdmissionRefused, Evidence
from patrolbot_bridge.tuning_backend import NODES, parameter_plan
from patrolbot_bridge.tuning_mutability import NAVIGATION_NODES
from patrolbot_bridge.tuning_simple import SimpleApplyBackend
from patrolbot_bridge.tuning_store import RevisionStore

ROOT = Path(__file__).resolve().parents[4]
REGISTRY = json.loads((ROOT / 'contracts/robot-tuning.registry.json').read_text())
ACTOR = {'id': 1, 'username': 'offline-reduced-apply', 'role': 'administrator'}
REQUEST = '33333333-3333-4333-8333-333333333333'


class Owner:
    tuning_hold = False

    def set_tuning_hold(self, held, reason=''):
        self.tuning_hold = held


class OfflineTransport:
    def __init__(self, store):
        self.store = store
        self.calls = []
        self.closed = None
        self.fail = None
        self.press_during_install = False
        self.evidence = dict(raw_odom_fresh=None, stationary=None, hardware_ok=True, dock_ok=True,
            faults_clear=True, command_buffers_zero=True, physical_deadman_fresh=True,
            physical_deadman_held=False, web_deadman_fresh=True, web_deadman_held=False,
            autonomy_idle=True, maps_clear=True, undock_clear=True, lifecycle_clear=True)
        self.capabilities = {'ready': True, 'implementation': 'OFFLINE-FAKE-NOT-NATIVE-EVIDENCE',
            'states': dict.fromkeys(NODES, 3), 'hot_support': list(NODES), 'policy_support': True,
            'nav2_redeploy_supported': True}
        self.capabilities['states']['patrolbot_joy_teleop'] = 'running'
        self.values = parameter_plan(store.load_revision(1), store.index)

    def simple_capabilities(self):
        return copy.deepcopy(self.capabilities)

    def block_inputs(self, plan, value):
        assert self.store.latest_job()['input_gate_attempted']
        assert self.store.latest_job()['mutation_started'] is False
        if self.fail == 'admission':
            raise AdmissionRefused('existing operation owns admission')
        self.closed = value
        self.calls.append('block_input')
        if self.fail == 'block-ack':
            raise TimeoutError('input block may have executed')

    def input_gate_closed(self, value):
        return value == self.closed

    def release_inputs(self, value):
        assert value == self.closed
        self.calls.append('release_input')
        if self.fail == 'release':
            raise TimeoutError('release acknowledgment lost')
        self.closed = None
        return True

    def stage_policies(self, plan, value):
        assert self.closed == value
        self.calls.append('stage_changed_policy')

    def apply_hot(self, node, values, plan, value):
        assert self.closed == value
        assert self.store.latest_job()['mutation_started']
        self.calls.append(('hot', node))
        self.values[node].update(values)
        if self.press_during_install:
            self.evidence.update(physical_deadman_held=True, web_deadman_held=True,
                                 physical_deadman_fresh=False, web_deadman_fresh=False)
        if self.fail == 'hot':
            raise TimeoutError('write may have executed')

    def restart_navigation(self, plan, value):
        assert self.closed == value
        assert plan['services'] == ['navigation'] and plan['image_rebuild'] is False
        self.calls.append('restart_navigation')
        if self.fail == 'restart':
            raise TimeoutError('navigation replacement outcome unknown')
        new = parameter_plan(self.store.load_revision(plan['revision']), self.store.index)
        for node in NAVIGATION_NODES:
            if node in new:
                self.values[node] = new[node]

    def verify_application(self, expected, plan, value):
        assert self.closed == value
        self.calls.append('verify_actual_consumers')
        if self.fail == 'cache':
            raise ValueError('actual execution cache is old')
        assert self.values == expected
        return {'verified_nodes': sorted(NODES), 'parameters': copy.deepcopy(self.values),
            'verified_policies': ['core_clamp', 'f710_cache', 'web_sessions'], 'motion_paths_held': True,
            'protected_invariants_verified': True,
            'publisher_graph': {'/cmd_vel_out': ['cmd_vel_mux'], '/cmd_vel': ['teleop_velocity_smoother']}}

    def commit_policies(self, plan, value):
        assert self.closed == value
        assert self.store.read_startup()['revision'] == plan['revision']
        self.calls.append('bind_committed_active')
        return self.fail != 'commit'


def setup(tmp_path, changes=None):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize(actor=ACTOR, reason='isolated two-mode executor fixture')
    store.save(request_id='22222222-2222-4222-8222-222222222222', base_revision=1,
        changes=changes or [{'id': 'nav2.controller.lookahead_dist', 'value': .5}], actor=ACTOR)
    transport, owner = OfflineTransport(store), Owner()
    backend = SimpleApplyBackend(store, transport, lambda: Evidence(**transport.evidence),
                                  enabled=True, generation=lambda: (1, 1))
    coordinator = ActivationCoordinator(store, owner, backend)
    coordinator.session_id = '11111111-1111-4111-8111-111111111111'
    return store, transport, owner, backend, coordinator


def run(coordinator, *, plan=None, request_id=REQUEST):
    plan = plan or coordinator.preview()
    return coordinator.request(operation='apply', request_id=request_id, expected_revision=plan['desired_revision'],
        actor=ACTOR, review={'digest': plan['digest'], 'plan_hash': plan['plan_hash']})


def test_hot_path_has_one_input_block_no_lifecycle_or_container_choreography(tmp_path):
    store, transport, owner, _, coordinator = setup(tmp_path)
    assert coordinator.preview()['execution_kind'] == 'scoped_parameters'
    assert run(coordinator)['outcome'] == 'ok'
    assert transport.calls == ['block_input', ('hot', 'controller_server'), 'verify_actual_consumers',
                               'bind_committed_active', 'release_input']
    assert store.read_startup()['revision'] == 2 and transport.closed is None and not owner.tuning_hold


def test_read_only_active_inspection_survives_production_backend_switch(tmp_path):
    store, transport, _, backend, _ = setup(tmp_path)
    active = store.load_revision(1)
    expected = parameter_plan(active, store.index)
    transport.inspect_parameters = lambda values: (copy.deepcopy(values), {})
    transport.get = lambda node, names: {'tuning_policy_version': 1}
    state, actual = backend.inspect(1, active['meta']['generated_digest'])
    assert state == active and actual == expected
    assert transport.calls == [] and transport.closed is None
    assert store.read_startup()['revision'] == 1
    with pytest.raises(ValueError, match='digest mismatch'):
        backend.inspect(1, 'sha256:' + 'f' * 64)
    transport.get = lambda node, names: {'tuning_policy_version': True}
    with pytest.raises(ValueError, match='cached-policy implementation'):
        backend.inspect(1, active['meta']['generated_digest'])


def test_new_input_during_installation_is_ignored_not_an_implicit_apply_failure(tmp_path):
    _, transport, _, _, coordinator = setup(tmp_path)
    transport.press_during_install = True
    assert run(coordinator)['outcome'] == 'ok'
    assert transport.evidence['physical_deadman_held'] is True  # never relabelled released
    assert transport.calls.count('block_input') == transport.calls.count('release_input') == 1


def test_held_input_before_admission_still_refuses_without_stealing_it(tmp_path):
    _, transport, owner, _, coordinator = setup(tmp_path)
    transport.evidence['physical_deadman_held'] = True
    assert run(coordinator)['outcome'] == 'refused'
    assert not transport.calls and not owner.tuning_hold


def test_one_constructor_field_restarts_the_entire_nav2_container_once(tmp_path):
    store, transport, _, _, coordinator = setup(tmp_path, [
        {'id': 'nav2.controller.lookahead_dist', 'value': .5},
        {'id': 'nav2.behavior.max_rotational_vel', 'value': .5}])
    plan = coordinator.preview()
    assert plan['execution_kind'] == 'nav2_redeploy' and plan['services'] == ['navigation']
    assert plan['seed_required']
    assert run(coordinator, plan=plan)['outcome'] == 'ok'
    assert transport.calls.count('restart_navigation') == 1
    assert not any(isinstance(call, tuple) and call[0] == 'hot' for call in transport.calls)
    assert store.read_startup()['revision'] == 2


def test_speed_ceiling_updates_non_nav2_final_smoother_without_restarting_core_or_bringup(tmp_path):
    _, transport, _, _, coordinator = setup(tmp_path, [{'id': 'max_angular_rad_s', 'value': .5}])
    assert run(coordinator)['outcome'] == 'ok'
    assert transport.calls.count('restart_navigation') == 1
    assert ('hot', 'teleop_velocity_smoother') in transport.calls
    assert 'stage_changed_policy' in transport.calls
    assert not any(isinstance(call, tuple) and call[0] == 'hot' and call[1] != 'teleop_velocity_smoother'
                   for call in transport.calls)


@pytest.mark.parametrize('failure', ['hot', 'cache', 'commit', 'release', 'block-ack'])
def test_ambiguous_operation_never_retries_redeploys_wider_or_drops_input_hold(tmp_path, failure):
    store, transport, owner, _, coordinator = setup(tmp_path)
    transport.fail = failure
    reviewed = coordinator.preview()
    assert run(coordinator, plan=reviewed)['outcome'] == 'unknown'
    before = list(transport.calls)
    # Reusing a key for a newly previewed plan is a different intent and must
    # refuse; only the original exact request deduplicates to its unknown result.
    assert run(coordinator, plan=reviewed)['outcome'] == 'unknown'
    assert transport.calls == before and transport.closed is not None and owner.tuning_hold
    assert 'restart_navigation' not in transport.calls
    if failure not in ('commit', 'release'):
        assert store.read_startup()['revision'] == 1


def test_unknown_restart_never_launches_another_restart_or_a_whole_robot_fallback(tmp_path):
    store, transport, owner, _, coordinator = setup(tmp_path, [{'id': 'amcl.pf_z', 'value': 1.2}])
    transport.fail = 'restart'
    assert run(coordinator)['outcome'] == 'unknown'
    assert transport.calls == ['block_input', 'restart_navigation']
    assert owner.tuning_hold and store.read_startup()['revision'] == 1


def test_runtime_state_change_that_alters_hot_vs_restart_requires_new_review(tmp_path):
    _, transport, _, _, coordinator = setup(tmp_path, [{'id': 'max_linear_m_s', 'value': .2}])
    plan = coordinator.preview()
    assert plan['execution_kind'] == 'scoped_parameters'
    transport.capabilities['states']['velocity_smoother'] = 2
    assert run(coordinator, plan=plan)['outcome'] == 'conflict'
    assert not transport.calls


def test_stale_capability_cannot_leave_an_executable_plan_behind(tmp_path):
    _, transport, _, _, coordinator = setup(tmp_path)
    transport.capabilities['ready'] = False
    assert not coordinator.preview()['executable']
    assert run(coordinator)['outcome'] == 'refused'
    assert transport.calls == []


def test_support_withdrawn_after_input_closure_is_held_not_used_as_stale_authority(tmp_path):
    store, transport, owner, _, coordinator = setup(tmp_path)
    block = transport.block_inputs
    def withdraw(plan, context):
        block(plan, context)
        transport.capabilities['ready'] = False
    transport.block_inputs = withdraw
    assert run(coordinator)['outcome'] == 'unknown'
    assert transport.calls == ['block_input']
    assert owner.tuning_hold and store.read_startup()['revision'] == 1
