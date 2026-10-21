"""Typed tuning service: actor provenance, outcomes, snapshot honesty."""

import json
from pathlib import Path

from patrolbot_bridge import tuning_service as svc
from patrolbot_bridge.tuning_activation import (
    ActivationCoordinator,
    Evidence,
    FakeBackend,
    UnavailableBackend,
)
from patrolbot_bridge.tuning_store import RevisionStore

REPO = Path(__file__).resolve().parents[4]
REGISTRY = json.loads((REPO / 'contracts' / 'robot-tuning.registry.json').read_text())
ADMIN = {'id': 1, 'username': 'admin', 'role': 'administrator'}
OPERATOR = {'id': 2, 'username': 'op', 'role': 'operator'}
SESSION = '123e4567-e89b-12d3-a456-426614174000'
REQ = '11111111-1111-4111-8111-111111111111'


def generation():
    return {'capability': svc.CAPABILITY, 'session_id': SESSION}


def save_request(changes, base_revision=1, request_id=REQ):
    return {
        'contract': svc.tuning_spec.CONTRACT, 'version': 1, 'operation': 'save',
        'request_id': request_id, 'generation': generation(),
        'base_revision': base_revision, 'changes': changes,
    }


def read_request(request_id=REQ):
    return {
        'contract': svc.tuning_spec.CONTRACT, 'version': 1, 'operation': 'read',
        'request_id': request_id, 'generation': generation(),
    }


def wrap(request, actor):
    return {'request': request, 'actor': actor}


def make(tmp_path, backend=None):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize(actor=ADMIN, reason='test')
    owner = type('Owner', (), {'set_tuning_hold': lambda self, h, r='': None})()
    coordinator = ActivationCoordinator(store, owner, backend or UnavailableBackend())
    return svc.TuningService(store, coordinator, session_id=SESSION, mutations_enabled=True,
                             write_admission=lambda: (True, ''))


def test_operator_can_read(tmp_path):
    service = make(tmp_path)
    result = service.handle(wrap(read_request(), OPERATOR))
    assert result['outcome'] == 'ok'
    assert result['snapshot']['advertised_capability'] == svc.READ_CAPABILITY


def test_operator_cannot_mutate(tmp_path):
    service = make(tmp_path)
    result = service.handle(wrap(save_request([{'id': 'amcl.alpha1', 'value': 0.06}]), OPERATOR))
    assert result['outcome'] == 'refused'
    assert result['detail'] == 'not_administrator'
    assert service.store.snapshot_state()['desired_revision'] == 1


def test_disconnected_sbc_refuses_save_without_mutating_revision(tmp_path):
    service = make(tmp_path)
    service.write_admission = lambda: (False, 'SBC link disconnected')
    result = service.handle(wrap(save_request([{'id': 'amcl.alpha1', 'value': .06}]), ADMIN))
    assert result['outcome'] == 'refused'
    assert result['detail'] == 'SBC link disconnected'
    assert result['snapshot']['writable'] is False
    assert service.store.snapshot_state()['desired_revision'] == 1


def test_missing_sbc_admission_provider_fails_closed_for_saves(tmp_path):
    service = make(tmp_path)
    service.write_admission = None
    result = service.handle(wrap(save_request([{'id': 'amcl.alpha1', 'value': .06}]), ADMIN))
    assert result['outcome'] == 'refused'
    assert service.store.snapshot_state()['desired_revision'] == 1


def test_save_rechecks_sbc_link_after_writer_admission(tmp_path):
    service = make(tmp_path)
    calls = [0]

    def admission():
        calls[0] += 1
        return calls[0] == 1, 'SBC link changed before persistence'

    service.write_admission = admission
    result = service.handle(wrap(save_request([{'id': 'amcl.alpha1', 'value': .06}]), ADMIN))
    assert result['outcome'] == 'refused'
    assert service.store.snapshot_state()['desired_revision'] == 1


def test_administrator_save_creates_desired_revision(tmp_path):
    service = make(tmp_path)
    result = service.handle(wrap(save_request([{'id': 'amcl.alpha1', 'value': 0.06}]), ADMIN))
    assert result['outcome'] == 'ok'
    snapshot = result['snapshot']
    assert snapshot['desired_revision'] == 2
    assert snapshot['active_revision'] is None
    assert snapshot['committed_active_revision'] == 1
    assert snapshot['pending_keys'] == ['amcl.alpha1']
    assert snapshot['desired']['amcl.alpha1'] == 0.06
    assert snapshot['active']['amcl.alpha1'] is None
    assert snapshot['committed_active']['amcl.alpha1'] == 0.05


def test_persistence_failure_after_save_reports_unknown_without_replaying(tmp_path, monkeypatch):
    service = make(tmp_path)

    def audit_unavailable(*args):
        raise OSError('simulated audit disk failure')

    monkeypatch.setattr(service.store, '_audit', audit_unavailable)
    request = wrap(save_request([{'id': 'amcl.alpha1', 'value': .06}]), ADMIN)
    result = service.handle(request)
    assert result['outcome'] == 'unknown'
    assert result['request_id'] == REQ
    assert service.store.snapshot_state()['desired_revision'] == 2
    result = service.handle(request)
    assert result['outcome'] == 'unknown'
    assert service.store.snapshot_state()['desired_revision'] == 2


def test_lost_snapshot_after_durable_save_is_not_reported_as_refused(tmp_path, monkeypatch):
    service = make(tmp_path)

    def unavailable():
        raise OSError('simulated readback disk failure')

    monkeypatch.setattr(service, 'runtime_snapshot', unavailable)
    result = service.handle(wrap(save_request([{'id': 'amcl.alpha1', 'value': .06}]), ADMIN))
    assert result['outcome'] == 'unknown'
    assert result['snapshot'] is None
    assert service.store.snapshot_state()['desired_revision'] == 2


def test_actor_with_extra_fields_is_rejected(tmp_path):
    service = make(tmp_path)
    actor = dict(ADMIN, administrator=True)
    result = service.handle(wrap(read_request(), actor))
    assert result['outcome'] == 'refused'
    assert result['detail'] == 'invalid_actor_fields'


def test_unknown_actor_role_is_rejected(tmp_path):
    service = make(tmp_path)
    result = service.handle(wrap(read_request(), {'id': 1, 'username': 'x', 'role': 'root'}))
    assert result['outcome'] == 'refused'


def test_envelope_with_unknown_field_is_rejected(tmp_path):
    service = make(tmp_path)
    payload = {'request': read_request(), 'actor': ADMIN, 'administrator': True}
    result = service.handle(json.dumps(payload))
    assert result['outcome'] == 'refused'
    assert result['detail'] == 'invalid_envelope'


def test_duplicate_json_keys_are_rejected(tmp_path):
    service = make(tmp_path)
    text = ('{"request": ' + json.dumps(read_request()) + ', "actor": '
            + json.dumps(ADMIN) + ', "actor": ' + json.dumps(ADMIN) + '}')
    result = service.handle(text)
    assert result['outcome'] == 'refused'


def test_oversized_payload_is_rejected(tmp_path):
    service = make(tmp_path)
    result = service.handle('{"request": "' + 'x' * (svc.REQUEST_MAX_BYTES + 10) + '"}')
    assert result['outcome'] == 'refused'
    assert result['detail'] == 'payload_too_large'


def test_nan_is_rejected(tmp_path):
    service = make(tmp_path)
    text = ('{"request": {"contract": "patrolbot-robot-tuning", "version": 1, '
            '"operation": "save", "request_id": "' + REQ + '", "generation": '
            '{"capability": "robot_tuning_v1", "session_id": "' + SESSION + '"}, '
            '"base_revision": 1, "changes": [{"id": "amcl.alpha1", "value": NaN}]}, '
            '"actor": ' + json.dumps(ADMIN) + '}')
    result = service.handle(text)
    assert result['outcome'] == 'refused'


def test_no_path_or_ros_input_is_accepted(tmp_path):
    service = make(tmp_path)
    request = save_request([{'id': 'amcl.alpha1', 'value': 0.06}])
    request['path'] = '/etc/passwd'
    result = service.handle(wrap(request, ADMIN))
    # Unknown field in a change/request is refused, never interpreted.
    assert result['outcome'] in ('refused', 'conflict')


def test_activation_unavailable_is_reported_honestly(tmp_path):
    service = make(tmp_path)
    result = service.handle(wrap(read_request(), ADMIN))
    snapshot = result['snapshot']
    assert snapshot['activation_supported'] is False
    assert snapshot['safe_apply']['allowed'] is False
    assert 'activation_unavailable' in snapshot['safe_apply']['reason']


def test_activation_with_approved_backend_requires_admin(tmp_path):
    backend = FakeBackend(Evidence())
    service = make(tmp_path, backend=backend)
    request = {
        'contract': svc.tuning_spec.CONTRACT, 'version': 1, 'operation': 'apply',
        'request_id': REQ, 'generation': generation(), 'expected_revision': 1,
    }
    result = service.handle(wrap(request, OPERATOR))
    assert result['outcome'] == 'refused'
    assert result['detail'] == 'not_administrator'


def test_snapshot_never_infers_active_from_desired(tmp_path):
    service = make(tmp_path)
    service.store.save(request_id=REQ, base_revision=1,
                       changes=[{'id': 'amcl.alpha1', 'value': 0.09}], actor=ADMIN)
    snapshot = service.runtime_snapshot()
    assert snapshot['desired']['amcl.alpha1'] == 0.09
    assert snapshot['active']['amcl.alpha1'] is None
    assert snapshot['committed_active']['amcl.alpha1'] == 0.05
    assert snapshot['effective_limits'] is None


def test_effective_limits_are_unknown_without_active(tmp_path):
    service = make(tmp_path)
    # Remove the active pointer to simulate an unknown runtime.
    manifest = service.store.require_ready()
    service.store._write_manifest(dict(manifest, active_revisions={}))
    result = service.handle(wrap(read_request(), ADMIN))
    assert result['outcome'] == 'refused'
    assert result['snapshot'] is None


def test_result_json_is_bounded_and_valid(tmp_path):
    service = make(tmp_path)
    text = service.result_json(wrap(read_request(), ADMIN))
    assert len(text.encode()) <= svc.RESPONSE_MAX_BYTES
    parsed = json.loads(text)
    assert parsed['outcome'] == 'ok'
    assert set(parsed) == {'request_id', 'outcome', 'detail', 'snapshot', 'job'}
