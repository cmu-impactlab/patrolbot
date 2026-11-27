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
import pytest

REPO = Path(__file__).resolve().parents[4]
REGISTRY = json.loads((REPO / 'contracts' / 'robot-tuning.registry.json').read_text())
ADMIN = {'id': 1, 'username': 'admin', 'role': 'administrator'}
OPERATOR = {'id': 2, 'username': 'op', 'role': 'operator'}
SESSION = '123e4567-e89b-12d3-a456-426614174000'
REQ = '11111111-1111-4111-8111-111111111111'


def test_strict_json_uses_shared_parser_without_dead_service_hooks():
    from patrolbot_bridge.tuning_json import loads

    assert svc.bounded_json_loads is loads
    assert not hasattr(svc, '_reject_constant')
    assert not hasattr(svc, '_no_duplicates')
    assert svc.strict_json_loads('{"value": 1}') == {'value': 1}


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
    return svc.TuningService(store, coordinator, session_id=SESSION)


def test_operator_can_read(tmp_path):
    service = make(tmp_path)
    result = service.handle(wrap(read_request(), OPERATOR))
    assert result['outcome'] == 'ok'
    assert result['snapshot']['advertised_capability'] == svc.READ_CAPABILITY


def test_snapshot_validates_each_revision_once_and_never_caches_between_requests(tmp_path, monkeypatch):
    service = make(tmp_path)
    load = service.store.load_revision
    calls = []

    def observed(revision):
        calls.append(revision)
        return load(revision)

    monkeypatch.setattr(service.store, 'load_revision', observed)
    service.runtime_snapshot()
    assert calls == [1]
    service.runtime_snapshot()
    assert calls == [1, 1]
    service.store.save(request_id=REQ, base_revision=1,
                       changes=[{'id': 'amcl.alpha1', 'value': .06}], actor=ADMIN)
    calls.clear()
    snapshot = service.runtime_snapshot()
    assert calls == [2, 1]
    assert snapshot['desired']['amcl.alpha1'] == .06
    assert snapshot['committed_active']['amcl.alpha1'] == .05


def test_read_does_not_hide_revision_corruption_after_a_successful_snapshot(tmp_path):
    service = make(tmp_path)
    assert service.handle(wrap(read_request(), OPERATOR))['outcome'] == 'ok'
    path = service.store.revision_dir(1) / 'overrides.yaml'
    path.write_text('corrupt: true\n')
    result = service.handle(wrap(read_request(), OPERATOR))
    assert result['outcome'] == 'refused'
    assert result['snapshot'] is None


@pytest.mark.parametrize('actor', [ADMIN, OPERATOR])
@pytest.mark.parametrize('operation,fields', [
    ('save', {'base_revision': 1, 'changes': [{'id': 'amcl.alpha1', 'value': .06}]}),
    ('apply', {'expected_revision': 1}),
    ('redeploy', {'expected_revision': 1}),
    ('restore', {'expected_revision': 1, 'restore_revision': 1, 'target': 'last_known_good'}),
])
def test_removed_operations_never_dispatch_or_change_existing_state(
        tmp_path, monkeypatch, actor, operation, fields):
    service = make(tmp_path, backend=FakeBackend(Evidence()))
    def must_not_run(*args, **kwargs):
        pytest.fail('read-only service dispatched a mutation')
    monkeypatch.setattr(service.store, 'save', must_not_run)
    monkeypatch.setattr(service.coordinator, 'request', must_not_run)
    result = service.handle(wrap(dict(read_request(), operation=operation, **fields), actor))
    assert result['outcome'] == 'refused'
    assert result['detail'] == 'unsupported_operation'
    assert service.store.snapshot_state()['desired_revision'] == 1
    assert service.store.latest_job() is None


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


def test_readback_never_advertises_writes_or_activation(tmp_path):
    service = make(tmp_path, backend=FakeBackend(Evidence()))
    result = service.handle(wrap(read_request(), ADMIN))
    snapshot = result['snapshot']
    assert snapshot['activation_supported'] is False
    assert snapshot['writable'] is False
    assert snapshot['save_supported'] is False
    assert all(not entry['activatable'] for entry in snapshot['catalog'])
    assert snapshot['safe_apply']['allowed'] is False
    assert 'read-only' in snapshot['safe_apply']['reason']


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
