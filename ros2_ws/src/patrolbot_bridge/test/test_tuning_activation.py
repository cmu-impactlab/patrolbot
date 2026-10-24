"""Activation / redeploy / restore state machine with an offline backend."""

import json
from pathlib import Path

from patrolbot_bridge.tuning_activation import (
    ActivationCoordinator,
    evaluate_evidence,
    Evidence,
    evidence_from_runtime,
    FakeBackend,
    FixedOperationPlan,
    ReloadResult,
    UnavailableBackend,
    AdmissionRefused,
)
from patrolbot_bridge.tuning_store import RevisionStore, StoreError
import pytest

REPO = Path(__file__).resolve().parents[4]
REGISTRY = json.loads((REPO / 'contracts' / 'robot-tuning.registry.json').read_text())
ADMIN = {'id': 1, 'username': 'admin', 'role': 'administrator'}
SAVE_UUID = '11111111-1111-4111-8111-111111111111'
UUID = 'aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa'


class Owner:
    """Stand-in for EpochRuntime's motion-owner hold surface."""

    def __init__(self):
        self.holds = []
        self.held = False

    def set_tuning_hold(self, held, reason=''):
        self.held = held
        self.holds.append((held, reason))


def good_evidence(**overrides):
    fields = {
        'raw_odom_fresh': True, 'stationary': True, 'hardware_ok': True, 'dock_ok': True,
        'faults_clear': True, 'command_buffers_zero': True, 'physical_deadman_fresh': True,
        'physical_deadman_held': False, 'web_deadman_fresh': True, 'web_deadman_held': False,
        'autonomy_idle': True, 'maps_clear': True, 'undock_clear': True, 'lifecycle_clear': True,
    }
    fields.update(overrides)
    return Evidence(**fields)


def make(tmp_path, evidence=None, approved=True, result=None, raises=None, record=None,
         backend=None):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize(actor=ADMIN, reason='test')
    store.save(request_id=SAVE_UUID, base_revision=1,
               changes=[{'id': 'amcl.alpha1', 'value': 0.06}], actor=ADMIN)
    owner = Owner()
    if backend is None:
        backend = FakeBackend(evidence if evidence is not None else good_evidence(),
                              approved=approved, result=result, raises=raises, record=record)
    coordinator = ActivationCoordinator(store, owner, backend)
    return store, owner, coordinator


def request(coordinator, operation='apply', request_id=UUID, expected=2, target=None):
    return coordinator.request(operation=operation, request_id=request_id,
                               expected_revision=expected, actor=ADMIN,
                               target_revision=target)


def test_default_backend_refuses_and_never_holds(tmp_path):
    store, owner, coordinator = make(tmp_path, backend=UnavailableBackend())
    result = request(coordinator)
    assert result['outcome'] == 'refused'
    assert 'activation_unavailable' in result['detail']
    assert owner.holds == []
    assert store.snapshot_state()['active_revision'] == 1


def test_missing_evidence_is_busy_not_permission():
    allowed, reason = evaluate_evidence(Evidence())
    assert not allowed
    assert 'busy' in reason


@pytest.mark.parametrize('field', [
    'hardware_ok', 'dock_ok', 'faults_clear',
    'command_buffers_zero', 'physical_deadman_fresh', 'web_deadman_fresh',
    'autonomy_idle', 'maps_clear', 'undock_clear', 'lifecycle_clear',
])
def test_each_required_fresh_field_blocks(field):
    assert evaluate_evidence(good_evidence(**{field: None}))[0] is False
    assert evaluate_evidence(good_evidence(**{field: False}))[0] is False


@pytest.mark.parametrize('field', ['physical_deadman_held', 'web_deadman_held'])
def test_held_deadman_blocks(field):
    allowed, reason = evaluate_evidence(good_evidence(**{field: True}))
    assert not allowed and 'busy' in reason


def test_full_evidence_allows():
    assert evaluate_evidence(good_evidence()) == (True, '')


@pytest.mark.parametrize('value', [None, False])
def test_command_idle_does_not_claim_or_require_measured_standstill(tmp_path, value):
    evidence = good_evidence(raw_odom_fresh=value, stationary=value)
    assert evaluate_evidence(evidence) == (True, '')
    store, owner, coordinator = make(tmp_path, evidence=evidence)
    result = request(coordinator)
    assert result['outcome'] == 'ok'
    assert store.read_startup()['revision'] == 2
    assert not owner.held
    assert 'stationary' not in coordinator.safe_apply_state()['reason']


def test_successful_activation_promotes_and_releases_hold(tmp_path):
    record = []
    store, owner, coordinator = make(tmp_path, record=record)
    result = request(coordinator)
    assert result['outcome'] == 'ok'
    assert record and record[0]['revision'] == 2
    assert store.snapshot_state()['active_revision'] == 2
    assert store.read_startup()['revision'] == 2
    # Hold set before reload, released after success.
    assert [h[0] for h in owner.holds] == [True, False]
    job = coordinator.status(result['job_id'])
    assert job['state'] == 'succeeded'


def test_busy_evidence_refuses_and_releases_hold(tmp_path):
    store, owner, coordinator = make(tmp_path, evidence=good_evidence(dock_ok=False))
    result = request(coordinator)
    assert result['outcome'] == 'refused'
    assert 'dock_ok' in result['detail']
    assert store.snapshot_state()['active_revision'] == 1
    assert coordinator.hold is False
    assert owner.holds == []


def test_ambiguous_reload_retains_hold_and_unknown(tmp_path):
    store, owner, coordinator = make(
        tmp_path, result=ReloadResult('unknown', 'no terminal result'))
    result = request(coordinator)
    assert result['outcome'] == 'unknown'
    assert coordinator.hold is True
    assert owner.held is True
    assert store.snapshot_state()['active_revision'] == 1
    assert coordinator.status(result['job_id'])['state'] == 'unknown'


def test_raised_reload_is_unknown_not_success(tmp_path):
    store, owner, coordinator = make(tmp_path, raises=RuntimeError('boom'))
    result = request(coordinator)
    assert result['outcome'] == 'unknown'
    assert coordinator.hold is True
    assert store.snapshot_state()['active_revision'] == 1


def test_failed_reload_keeps_last_good_and_unknown_hold_without_readback(tmp_path):
    store, owner, coordinator = make(
        tmp_path, result=ReloadResult('failed', 'node refused parameters'))
    result = request(coordinator)
    assert result['outcome'] == 'unknown'
    assert store.snapshot_state()['active_revision'] == 1
    assert coordinator.hold is True


def test_job_is_journalled_before_reload(tmp_path):
    seen = {}

    class Observing(FakeBackend):
        def reload(self, scopes, revision, digest):
            seen['job'] = store.latest_job()
            seen['held'] = owner.held
            return super().reload(scopes, revision, digest)

    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize(actor=ADMIN, reason='test')
    store.save(request_id=SAVE_UUID, base_revision=1,
               changes=[{'id': 'amcl.alpha1', 'value': 0.06}], actor=ADMIN)
    owner = Owner()
    backend = Observing(good_evidence())
    coordinator = ActivationCoordinator(store, owner, backend)
    request(coordinator)
    assert seen['job']['state'] in ('pending', 'running')
    assert seen['held'] is True


def test_stale_expected_revision_conflicts(tmp_path):
    store, owner, coordinator = make(tmp_path)
    result = request(coordinator, expected=1)
    assert result['outcome'] == 'conflict'
    assert owner.holds == []


def test_restore_cannot_target_a_future_revision(tmp_path):
    store, owner, coordinator = make(tmp_path)
    result = request(coordinator, operation='restore', expected=2, target=99)
    assert result['outcome'] == 'refused'


def test_restore_to_last_known_good_promotes_target(tmp_path):
    store, owner, coordinator = make(tmp_path)
    # First activate revision 2 so it becomes known-good history.
    request(coordinator)
    # Save a new desired revision 3, then restore revision 1 explicitly.
    store.save(request_id='22222222-2222-4222-8222-222222222222', base_revision=2,
               changes=[{'id': 'amcl.alpha1', 'value': 0.07}], actor=ADMIN)
    result = coordinator.request(operation='restore',
                                 request_id='33333333-3333-4333-8333-333333333333',
                                 expected_revision=3, actor=ADMIN, target_revision=1)
    assert result['outcome'] == 'ok'
    assert store.snapshot_state()['active_revision'] == 1


def test_recover_holds_on_unsettled_job(tmp_path):
    store, owner, coordinator = make(tmp_path, result=ReloadResult('unknown', 'lost ack'))
    request(coordinator)
    # Simulate a restart: a new coordinator over the same store must re-hold.
    restarted_owner = Owner()
    restarted = ActivationCoordinator(store, restarted_owner, FakeBackend(good_evidence()))
    job = restarted.recover()
    assert job is not None and job['state'] == 'unknown'
    assert restarted.hold is True
    assert restarted_owner.held is True


def test_recover_is_noop_when_last_job_settled(tmp_path):
    store, owner, coordinator = make(tmp_path)
    request(coordinator)
    restarted_owner = Owner()
    restarted = ActivationCoordinator(store, restarted_owner, FakeBackend(good_evidence()))
    assert restarted.recover() is None
    assert restarted.hold is False


def test_activation_is_unavailable_by_default(tmp_path):
    store, owner, coordinator = make(tmp_path, backend=UnavailableBackend())
    assert coordinator.activation_supported is False
    assert coordinator.safe_apply_state()['allowed'] is False


def test_fixed_operation_plan_is_allowlisted():
    plan = FixedOperationPlan()
    assert plan.services_for('navigation') == ('patrolbot-navigation',)
    with pytest.raises(StoreError):
        plan.services_for('../../bin/sh')
    built = plan.build(scope='bringup', revision=3, digest='sha256:' + 'a' * 64)
    assert built['services'] == ('patrolbot-bringup',)
    with pytest.raises(StoreError):
        plan.build(scope='bringup', revision=3, digest='rm -rf /')


def test_evidence_from_runtime_treats_non_booleans_as_unknown():
    evidence = evidence_from_runtime({'raw_odom_fresh': 1, 'stationary': True,
                                      'hardware_ok': 'yes'})
    assert evidence.raw_odom_fresh is None
    assert evidence.stationary is True
    assert evidence.hardware_ok is None


@pytest.mark.parametrize('failure_probe', [2, 3, 4])
def test_every_post_reservation_pre_mutation_refusal_unwinds_acknowledged_hold(tmp_path, failure_probe):
    """Offline owner race coverage, including the final check before reload."""
    calls = []

    class Reserved(FakeBackend):
        probes = 0

        def probe(self):
            self.probes += 1
            return good_evidence(hardware_ok=self.probes < failure_probe)

        def reserve_navigation(self):
            assert store.latest_job()['state'] == 'pending'
            calls.append('reserve')

        def release_navigation_reservation(self):
            calls.append('release')

    backend = Reserved(good_evidence(), record=calls)
    store, owner, coordinator = make(tmp_path, backend=backend)
    result = request(coordinator)
    assert result['outcome'] == 'refused'
    assert calls == ['reserve', 'release']
    assert not owner.held and not coordinator.hold
    assert store.latest_job()['state'] == 'failed'
    assert store.read_startup()['revision'] == 1


@pytest.mark.parametrize('ambiguous', [False, True])
def test_remote_reservation_race_never_cancels_goal_or_orphans_unknown_owner(tmp_path, ambiguous):
    class Reserved(FakeBackend):
        def reserve_navigation(self):
            assert store.latest_job()['state'] == 'pending'
            if ambiguous:
                raise TimeoutError('reservation may execute')
            raise AdmissionRefused('existing navigation goal won the mutex')

    store, owner, coordinator = make(tmp_path, backend=Reserved(good_evidence()))
    result = request(coordinator)
    assert result['outcome'] == ('unknown' if ambiguous else 'refused')
    assert coordinator.hold is ambiguous
    assert owner.held is ambiguous
    assert store.latest_job()['state'] == ('unknown' if ambiguous else 'failed')
    assert store.read_startup()['revision'] == 1
