"""Unit tests exercise the production lifecycle algorithm with fake service I/O."""

import copy
import json
from pathlib import Path

from patrolbot_bridge.tuning_activation import ActivationCoordinator, AdmissionRefused, Evidence
from patrolbot_bridge.tuning_backend import (
    LifecycleBackend, MANAGER_COMMANDS, parameter_plan, RosParameterTransport,
)
from patrolbot_bridge.tuning_store import RevisionStore
import pytest


ROOT = Path(__file__).resolve().parents[4]
REGISTRY = json.loads((ROOT / 'contracts/robot-tuning.registry.json').read_text())
ADMIN = {'id': 1, 'username': 'fixture-admin', 'role': 'administrator'}


def idle(**changes):
    fields = {'raw_odom_fresh': True, 'stationary': True, 'hardware_ok': True, 'dock_ok': True,
              'faults_clear': True, 'command_buffers_zero': True, 'physical_deadman_fresh': True,
              'physical_deadman_held': False, 'web_deadman_fresh': True, 'web_deadman_held': False,
              'autonomy_idle': True, 'maps_clear': True, 'undock_clear': True,
              'lifecycle_clear': True}
    fields.update(changes)
    return Evidence(**fields)


class Transport:
    def __init__(self, plan):
        self.plan = copy.deepcopy(plan)
        self.calls = []
        self.failure = None
        self.bad_readback = False
        self.available = True

    def ready(self):
        return self.available

    def get(self, node, names):
        if names == ['tuning_policy_version']:
            return {'tuning_policy_version': 1}
        self.calls.append(('get', node))
        result = {key: self.plan[node][key] for key in names}
        if self.bad_readback and node == 'amcl':
            result['alpha1'] = .9
        return result

    def set(self, node, values):
        self.calls.append(('set', node))
        self.plan[node].update(values)
        if self.failure == node:
            raise TimeoutError('server may still execute')

    def manager(self, node, verb):
        assert verb in MANAGER_COMMANDS
        self.calls.append((verb, node))

    def transition(self, node, verb):
        self.calls.append((verb, node))

    def smoother_maintenance(self, held):
        self.calls.append(('smoother_hold', held))

    def navigation_maintenance(self, held):
        self.calls.append(('navigation_hold', held))

    def joystick_idle(self):
        self.calls.append(('joystick_idle', True))

    def web_idle(self):
        self.calls.append(('web_idle', True))

    def restore_map(self, selected):
        self.calls.append(('restore_map', selected['map_id']))
        return '1:2:100:200'


class Owner:
    tuning_hold = False

    def set_tuning_hold(self, held, reason=''):
        self.tuning_hold = held


def setup(tmp_path, probe=idle):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize(actor=ADMIN, reason='unit fixture')
    store.save(request_id='11111111-1111-4111-8111-111111111111', base_revision=1,
               changes=[{'id': 'amcl.alpha1', 'value': .06}], actor=ADMIN)
    state = store.load_revision(2)
    transport = Transport(parameter_plan(state, store.index))
    seeds = []
    backend = LifecycleBackend(
        store, transport, probe, lambda identity: seeds.append(identity), enabled=True,
        selected_map=lambda: {'path': '/fixed/catalog/map.yaml', 'map_id': 'floor-1'})
    return store, transport, backend, seeds, state['meta']['generated_digest']


def test_reload_keeps_exact_revision_selected_map_and_inactive_navigation(tmp_path):
    store, transport, backend, seeds, digest = setup(tmp_path)
    result = backend.reload(['bringup', 'navigation'], 2, digest)
    assert result.status == 'ok'
    assert seeds == ['1:2:100:200']
    assert ('restore_map', 'floor-1') in transport.calls
    assert ('RESUME', 'lifecycle_manager_navigation') not in transport.calls
    assert transport.calls[0] == ('navigation_hold', True)
    assert transport.calls[1] == ('joystick_idle', True)
    assert transport.calls[2] == ('web_idle', True)
    assert transport.calls[3] == ('smoother_hold', True)
    assert ('navigation_hold', False) not in transport.calls
    # Runtime reload alone must NOT promote desired to the startup artifact.
    assert store.read_startup()['revision'] == 1
    backend.committed(2, digest)
    assert transport.calls[-1] == ('smoother_hold', False)
    assert ('navigation_hold', False) not in transport.calls


@pytest.mark.parametrize('facts', [
    {'physical_deadman_held': True}, {'autonomy_idle': None},
    {'command_buffers_zero': False}, {'hardware_ok': False}, {'web_deadman_fresh': False},
])
def test_busy_or_unknown_ownership_mutates_no_node(tmp_path, facts):
    _, transport, backend, _, digest = setup(tmp_path, lambda: idle(**facts))
    assert backend.reload(['bringup', 'navigation'], 2, digest).status == 'failed'
    assert transport.calls == []


def test_wrong_digest_or_scope_never_touches_a_service(tmp_path):
    _, transport, backend, _, digest = setup(tmp_path)
    assert backend.reload(['navigation'], 2, digest).status == 'failed'
    assert backend.reload(['bringup', 'navigation'], 2, 'sha256:' + '0' * 64).status == 'failed'
    assert transport.calls == []


@pytest.mark.parametrize('failure', ['amcl', 'controller_server', 'teleop_velocity_smoother'])
def test_partial_failure_never_commits_or_releases_the_hold(tmp_path, failure):
    store, transport, backend, seeds, digest = setup(tmp_path)
    transport.failure = failure
    assert backend.reload(['bringup', 'navigation'], 2, digest).status == 'unknown'
    assert store.read_startup()['revision'] == 1
    assert seeds == []
    assert ('navigation_hold', False) not in transport.calls
    with pytest.raises(ValueError):
        backend.committed(2, digest)


def test_readback_mismatch_is_not_a_successful_parameter_response(tmp_path):
    store, transport, backend, seeds, digest = setup(tmp_path)
    transport.bad_readback = True
    assert backend.reload(['bringup', 'navigation'], 2, digest).status == 'unknown'
    assert store.read_startup()['revision'] == 1
    assert seeds == []


@pytest.mark.parametrize('operation', ['apply', 'redeploy', 'restore'])
def test_legacy_native_execution_cannot_bypass_review(tmp_path, operation):
    store, transport, backend, _, _ = setup(tmp_path)
    owner = Owner()
    coordinator = ActivationCoordinator(store, owner, backend)
    result = coordinator.request(
        operation=operation, request_id='22222222-2222-4222-8222-222222222222',
        expected_revision=2, actor=ADMIN, target_revision=1 if operation == 'restore' else None)
    assert result['outcome'] == 'refused'
    assert 'review_required' in result['detail']
    assert store.read_startup()['revision'] == 1
    assert not owner.tuning_hold
    assert transport.calls == []


def test_reviewed_but_unproven_scoped_plan_cannot_use_legacy_all_node_reload(tmp_path):
    store, transport, backend, _, _ = setup(tmp_path)
    transport.failure = 'amcl'
    coordinator = ActivationCoordinator(store, Owner(), backend)
    coordinator.session_id = '11111111-1111-4111-8111-111111111111'
    plan = coordinator.preview()
    result = coordinator.request(
        operation='apply', request_id='22222222-2222-4222-8222-222222222222',
        expected_revision=2, actor=ADMIN,
        review={'digest': plan['digest'], 'plan_hash': plan['plan_hash']})
    assert result['outcome'] == 'refused'
    assert 'implementation_unverified' in result['detail']
    assert store.read_startup()['revision'] == 1
    assert transport.calls == []
    assert not coordinator.hold


def test_request_id_cannot_be_repurposed_for_restore_or_another_actor(tmp_path):
    store, transport, backend, _, _ = setup(tmp_path)
    coordinator = ActivationCoordinator(store, Owner(), backend)
    request_id = '22222222-2222-4222-8222-222222222222'
    first = coordinator.request(operation='redeploy', request_id=request_id,
                                expected_revision=2, actor=ADMIN)
    assert first['outcome'] == 'refused'
    calls = list(transport.calls)
    changed = coordinator.request(operation='restore', request_id=request_id,
                                  expected_revision=2, actor=ADMIN, target_revision=1)
    assert changed['outcome'] == 'refused'
    assert transport.calls == calls
    changed = coordinator.request(operation='redeploy', request_id=request_id,
                                  expected_revision=2, actor=dict(ADMIN, id=2))
    assert changed['outcome'] == 'refused'
    assert store.get_request(request_id)['outcome'] == 'refused'


def test_hardware_generation_change_never_commits_a_runtime_success(tmp_path, monkeypatch):
    store, transport, backend, seeds, digest = setup(tmp_path)
    generation = [1]
    backend.generation = lambda: generation[0]
    restore = transport.restore_map

    def reconnect(selected):
        restore(selected)
        generation[0] += 1

    monkeypatch.setattr(transport, 'restore_map', reconnect)
    result = backend.reload(['bringup', 'navigation'], 2, digest)
    assert result.status == 'unknown'
    assert store.read_startup()['revision'] == 1
    assert not seeds


def test_manual_press_during_reload_keeps_hold_and_requires_explicit_recovery(tmp_path):
    state = {'held': False}
    store, transport, backend, _, digest = setup(
        tmp_path, lambda: idle(physical_deadman_held=state['held']))
    restore = transport.restore_map

    def press(selected):
        restore(selected)
        state['held'] = True

    transport.restore_map = press
    assert backend.reload(['bringup', 'navigation'], 2, digest).status == 'unknown'
    assert store.read_startup()['revision'] == 1
    assert ('navigation_hold', False) not in transport.calls


def test_manager_definition_has_pinned_jazzy_configure_and_cleanup_commands():
    from types import SimpleNamespace
    backend = object.__new__(RosParameterTransport)
    request = SimpleNamespace(**dict(zip(MANAGER_COMMANDS, range(7))))
    backend.types = {'manager': SimpleNamespace(Request=lambda: request)}
    backend.clients = {}
    assert backend.ready()
    del request.CONFIGURE
    assert not backend.ready()


def test_invalid_generated_domain_is_refused_before_acquiring_a_runtime_hold(tmp_path):
    store, transport, backend, _, _ = setup(tmp_path)
    store.save(request_id='33333333-3333-4333-8333-333333333333', base_revision=2,
               changes=[{'id': 'max_linear_m_s', 'value': 0.}], actor=ADMIN)
    owner = Owner()
    coordinator = ActivationCoordinator(store, owner, backend)
    plan = coordinator.preview()
    assert not plan['executable']
    assert any('0.01' in reason['detail'] for reason in plan['reasons'])
    result = coordinator.request(
        operation='apply', request_id='44444444-4444-4444-8444-444444444444',
        expected_revision=3, actor=ADMIN,
        review={'digest': plan['digest'], 'plan_hash': plan['plan_hash']})
    assert result['outcome'] == 'refused'
    assert 'implementation_unverified' in result['detail']
    assert transport.calls == []
    assert owner.tuning_hold is False
    assert store.read_startup()['revision'] == 1


def test_unreviewed_request_never_reserves_or_cancels_navigation(tmp_path):
    store, transport, backend, _, _ = setup(tmp_path)
    owner = Owner()
    owner.navigation_ready = True

    def busy(held):
        assert held
        raise AdmissionRefused('navigation owns motion')

    transport.navigation_maintenance = busy
    result = ActivationCoordinator(store, owner, backend).request(
        operation='redeploy', request_id='55555555-5555-4555-8555-555555555555',
        expected_revision=2, actor=ADMIN)
    assert result['outcome'] == 'refused'
    assert not owner.tuning_hold and owner.navigation_ready
    assert store.latest_job() is None
    assert transport.calls == []
    assert store.read_startup()['revision'] == 1


def test_unreviewed_request_never_calls_even_an_ambiguous_remote_reservation(tmp_path):
    store, transport, backend, _, _ = setup(tmp_path)
    owner = Owner()

    def timeout(held):
        assert held
        assert store.latest_job()['state'] == 'pending'
        raise TimeoutError('reservation may still execute')

    transport.navigation_maintenance = timeout
    result = ActivationCoordinator(store, owner, backend).request(
        operation='redeploy', request_id='55555555-5555-4555-8555-555555555555',
        expected_revision=2, actor=ADMIN)
    assert result['outcome'] == 'refused'
    assert not owner.tuning_hold
    assert store.latest_job() is None
    assert transport.calls == []


def test_successful_reload_keeps_durable_post_restart_admission_gate(tmp_path):
    store, _, backend, _, digest = setup(tmp_path)
    assert backend.reload(['bringup', 'navigation'], 2, digest).status == 'ok'
    reopened = RevisionStore(store.base, REGISTRY)
    admission = reopened.navigation_admission()
    assert admission['pending'] is True
    assert admission['revision'] == 2 and admission['digest'] == digest
    assert admission['map_identity'] == '1:2:100:200'


def test_changed_plan_refuses_before_remote_reservation(tmp_path):
    count = [0]

    def probe():
        count[0] += 1
        return idle(hardware_ok=count[0] < 3)

    store, transport, backend, _, _ = setup(tmp_path, probe)
    owner = Owner()
    result = ActivationCoordinator(store, owner, backend).request(
        operation='apply', request_id='66666666-6666-4666-8666-666666666666',
        expected_revision=2, actor=ADMIN,
        review={'digest': store.load_revision(2)['meta']['generated_digest'],
                'plan_hash': 'sha256:' + '0' * 64})
    assert result['outcome'] == 'conflict'
    assert transport.calls == []
    assert not owner.tuning_hold
    assert store.latest_job() is None


def test_unreviewed_restore_does_not_change_active_or_last_good(tmp_path):
    store, transport, backend, _, _ = setup(tmp_path)
    store.promote(revision=2, scopes=['bringup', 'navigation'], actor=ADMIN)
    store.save(request_id='33333333-3333-4333-8333-333333333333', base_revision=2,
               changes=[{'id': 'amcl.alpha2', 'value': .08}], actor=ADMIN)
    result = ActivationCoordinator(store, Owner(), backend).request(
        operation='restore', request_id='77777777-7777-4777-8777-777777777777',
        expected_revision=3, actor=ADMIN, target_revision=1)
    assert result['outcome'] == 'refused'
    assert store.require_ready()['last_known_good'] == 1
    assert store.snapshot_state()['active_revision'] == 2


@pytest.mark.parametrize('value', [None, False])
def test_legacy_reload_final_verification_no_longer_requires_odometry(tmp_path, value):
    _, _, backend, _, digest = setup(tmp_path, lambda: idle(raw_odom_fresh=value, stationary=value))
    assert backend.reload(['bringup', 'navigation'], 2, digest).status == 'ok'
