"""Offline protocol boundary tests, NOT native cache/lifecycle acceptance."""

import json
from pathlib import Path

import pytest

from patrolbot_bridge.tuning_activation import ActivationCoordinator, Evidence, FakeBackend, ReloadResult
from patrolbot_bridge.tuning_backend import NODES, parameter_plan
from patrolbot_bridge.tuning_store import RevisionStore

ROOT = Path(__file__).resolve().parents[4]
REGISTRY = json.loads((ROOT / 'contracts/robot-tuning.registry.json').read_text())
ACTOR = {'id': 1, 'username': 'offline-fixture', 'role': 'administrator'}
SESSION = '11111111-1111-4111-8111-111111111111'
REQUEST = '33333333-3333-4333-8333-333333333333'


class Owner:
    held = False

    def set_tuning_hold(self, held, reason=''):
        self.held = held


class ReviewedFake(FakeBackend):
    """Fabricated proof for testing certificate checks; never installed on ROS."""

    requires_review = True
    plan_support = {'controller_server': 'parameters'}
    plan_implementation = 'OFFLINE-FAKE-NOT-NATIVE-EVIDENCE'

    def __init__(self, store, owner):
        super().__init__(Evidence(raw_odom_fresh=None, stationary=None,
            hardware_ok=True, dock_ok=True, faults_clear=True, command_buffers_zero=True,
            physical_deadman_fresh=True, physical_deadman_held=False,
            web_deadman_fresh=True, web_deadman_held=False,
            autonomy_idle=True, maps_clear=True, undock_clear=True, lifecycle_clear=True))
        self.store, self.owner = store, owner
        self.calls = []
        self.generation_value = (1, 1)
        self.proof_patch = {}
        self.commit_ack = True

    def generation(self):
        return self.generation_value

    def reserve_navigation(self):
        assert self.store.latest_job()['state'] == 'pending'
        assert self.store.latest_job()['execution_plan']['nodes'] == ['controller_server']
        self.calls.append('reserve')

    def release_navigation_reservation(self):
        self.calls.append('release')

    def reload(self, *args):
        raise AssertionError('reviewed execution must never use all-node legacy reload')

    def reload_plan(self, plan, revision, digest, *, job_id):
        assert self.owner.held
        assert self.store.latest_job()['mutation_started'] is True
        assert self.store.latest_job()['plan_hash'] == plan['plan_hash']
        self.calls.append(('reload', plan['nodes']))
        return ReloadResult('ok')

    def verify_plan(self, plan, revision, digest, *, job_id):
        assert self.owner.held
        assert self.store.read_startup()['revision'] == 1
        self.calls.append('verify')
        return dict({'complete': True, 'job_id': job_id, 'revision': revision,
                     'digest': digest, 'plan_hash': plan['plan_hash'],
                     'hardware_generation': plan['hardware_generation'],
                     'scopes': ['bringup', 'navigation'], 'verified_nodes': sorted(NODES),
                     'verified_policies': ['core_clamp', 'f710_cache', 'web_sessions'],
                     'parameters': parameter_plan(self.store.load_revision(revision), self.store.index),
                     'motion_paths_held': True, 'protected_invariants_verified': True,
                     'publisher_graph': {'/cmd_vel_out': ['cmd_vel_mux'], '/cmd_vel': ['teleop_velocity_smoother']}}, **self.proof_patch)

    def committed_plan(self, plan, *, job_id):
        assert self.owner.held
        assert self.store.read_startup()['revision'] == plan['revision']
        self.calls.append('committed')
        return self.commit_ack


def setup(tmp_path):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize(actor=ACTOR, reason='offline protocol fixture')
    store.save(request_id='22222222-2222-4222-8222-222222222222', base_revision=1,
               changes=[{'id': 'nav2.controller.lookahead_dist', 'value': .5}], actor=ACTOR)
    owner = Owner()
    backend = ReviewedFake(store, owner)
    coordinator = ActivationCoordinator(store, owner, backend)
    coordinator.session_id = SESSION
    plan = coordinator.preview()
    assert plan['executable']
    return store, owner, backend, coordinator, plan


def request(coordinator, plan, request_id=REQUEST):
    return coordinator.request(operation='apply', request_id=request_id,
        expected_revision=plan['desired_revision'], actor=ACTOR,
        review={'digest': plan['digest'], 'plan_hash': plan['plan_hash']})


def test_scope_journal_and_full_certificate_are_bound_before_whole_revision_promotion(tmp_path):
    store, owner, backend, coordinator, plan = setup(tmp_path)
    result = request(coordinator, plan)
    assert result['outcome'] == 'ok'
    assert backend.calls == ['reserve', ('reload', ['controller_server']), 'verify', 'committed']
    assert store.read_startup()['revision'] == 2
    assert not owner.held
    job = store.latest_job()
    assert job['state'] == 'succeeded' and job['plan_hash'] == plan['plan_hash']
    assert job['verification']['scopes'] == ['bringup', 'navigation']
    assert job['verification']['verified_nodes'] == sorted(NODES)


@pytest.mark.parametrize('patch', [
    {'complete': False}, {'job_id': 'wrong-job'}, {'revision': 1},
    {'digest': 'sha256:' + '0' * 64}, {'plan_hash': 'sha256:' + '0' * 64},
    {'hardware_generation': 'sha256:' + '0' * 64},
    {'scopes': ['navigation']}, {'verified_nodes': ['controller_server']},
    {'verified_policies': ['core_clamp']},
    {'parameters': {}}, {'motion_paths_held': False}, {'protected_invariants_verified': False},
    {'publisher_graph': {'/cmd_vel_out': ['cmd_vel_mux', 'unplanned_publisher']}},
])
def test_partial_stale_or_foreign_proof_never_promotes_or_releases(tmp_path, patch):
    store, owner, backend, coordinator, plan = setup(tmp_path)
    backend.proof_patch = patch
    result = request(coordinator, plan)
    assert result['outcome'] == 'unknown'
    assert 'verification_mismatch' in result['detail']
    assert owner.held and coordinator.hold
    assert store.read_startup()['revision'] == 1
    assert store.latest_job()['state'] == 'unknown'
    assert 'committed' not in backend.calls


def test_changed_scope_requires_new_review_not_silent_escalation(tmp_path):
    store, owner, backend, coordinator, plan = setup(tmp_path)
    backend.plan_support = {'controller_server': 'lifecycle'}
    result = request(coordinator, plan)
    assert result['outcome'] == 'conflict'
    assert backend.calls == [] and not owner.held
    assert store.read_startup()['revision'] == 1


def test_commit_binding_uncertainty_survives_restart_without_retry(tmp_path):
    store, owner, backend, coordinator, plan = setup(tmp_path)
    backend.commit_ack = False
    result = request(coordinator, plan)
    assert result['outcome'] == 'unknown' and owner.held
    calls = list(backend.calls)
    restarted_owner = Owner()
    restarted = ActivationCoordinator(store, restarted_owner, backend)
    assert restarted.recover()['state'] == 'unknown'
    assert restarted_owner.held and backend.calls == calls
    assert store.read_startup()['revision'] == 2  # proven parameters, uncertain committed cache binding; still held


def test_unsettled_job_cannot_be_overwritten_even_without_explicit_startup_recover(tmp_path):
    store, owner, backend, coordinator, plan = setup(tmp_path)
    backend.commit_ack = False
    request(coordinator, plan)
    original_job = store.latest_job()['job_id']
    second_owner = Owner()
    restarted = ActivationCoordinator(store, second_owner, backend)
    result = request(restarted, plan, '44444444-4444-4444-8444-444444444444')
    assert result['outcome'] == 'refused' and 'recovery_required' in result['detail']
    assert store.latest_job()['job_id'] == original_job
    assert second_owner.held


def test_request_dedup_binds_review_and_never_repeats_the_executor(tmp_path):
    _, _, backend, coordinator, plan = setup(tmp_path)
    assert request(coordinator, plan)['outcome'] == 'ok'
    calls = list(backend.calls)
    assert request(coordinator, plan)['outcome'] == 'ok'
    altered = dict(plan, plan_hash='sha256:' + '0' * 64)
    assert request(coordinator, altered)['outcome'] == 'refused'
    assert backend.calls == calls
