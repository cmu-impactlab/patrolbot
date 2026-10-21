"""Exercise the shipped ROS callback without importing or starting ROS."""

import ast
import json
from pathlib import Path
from types import SimpleNamespace

from patrolbot_bridge.tuning_authorization import AuthorizationError, sign, verify
from patrolbot_bridge.tuning_service import ServiceError, strict_json_loads
import pytest


SOURCE = Path(__file__).resolve().parents[1] / 'patrolbot_bridge' / 'bridge_node.py'


def _callback():
    tree = ast.parse(SOURCE.read_text())
    node = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                and node.name == 'PatrolBotBridge')
    method = next(method for method in node.body if isinstance(method, ast.FunctionDef)
                  and method.name == '_tuning_callback')
    namespace = {'json': json, 'ServiceError': ServiceError,
                 'strict_json_loads': strict_json_loads,
                 'AuthorizationError': AuthorizationError,
                 'verify_tuning_authorization': verify}
    code = ast.Module(body=[method], type_ignores=[])
    exec(compile(ast.fix_missing_locations(code), str(SOURCE), 'exec'), namespace)
    return namespace['_tuning_callback']


def _invoke(payload):
    calls = []
    refreshed = []

    def backend(raw):
        calls.append(raw)
        return json.dumps({'outcome': 'ok'})

    bridge = SimpleNamespace(
        _tuning_service=SimpleNamespace(result_json=backend),
        _refresh_tuning_policy=lambda: refreshed.append(True),
    )
    response = SimpleNamespace(result_json='')
    _callback()(bridge, SimpleNamespace(payload_json=payload), response)
    # Policy refresh is owned by the retained observation worker, never a ROS
    # service callback that could starve the executor needed for completions.
    assert refreshed == []
    return json.loads(response.result_json), calls


@pytest.mark.parametrize('operation', ['save', 'apply', 'redeploy', 'restore'])
def test_forged_administrator_cannot_mutate_through_ros(operation):
    payload = json.dumps({
        'request': {'operation': operation},
        'actor': {'id': 1, 'username': 'forged-admin', 'role': 'administrator'},
    })
    result, calls = _invoke(payload)
    assert result['outcome'] == 'refused'
    assert 'read_only' in result['detail']
    assert calls == []


@pytest.mark.parametrize('operation', ['read', 'status'])
def test_read_operations_still_reach_strict_service_validation(operation):
    payload = json.dumps({'request': {'operation': operation}})
    result, calls = _invoke(payload)
    assert result['outcome'] == 'ok'
    assert calls == [payload]


@pytest.mark.parametrize('payload', [
    '{"request":{"operation":"read","operation":"save"}}',
    '{"request":{"operation":"save","operation":"read"}}',
    '{"request":{"operation":"read"},"extra":NaN}',
    '{not-json', '[]', '{}', '{"request":[]}',
])
def test_malformed_payload_cannot_bypass_read_only_boundary(payload):
    result, calls = _invoke(payload)
    assert result['outcome'] == 'refused'
    assert calls == []


def test_only_a_current_server_signature_reaches_the_mutation_dispatch():
    key = b'unit-test-only-private-key-material'
    calls = []
    body = {'request': {'operation': 'save'},
            'actor': {'id': 1, 'username': 'admin', 'role': 'administrator'}}
    bridge = SimpleNamespace(
        _tuning_service=SimpleNamespace(
            result_json=lambda raw: calls.append(json.loads(raw)) or '{"outcome":"ok"}'),
        _tuning_authorization_key=key, _tuning_robot_id='robot-1',
        _refresh_tuning_policy=lambda: None)
    response = SimpleNamespace(result_json='')
    signed = sign(body, key, 'robot-1')
    _callback()(bridge, SimpleNamespace(payload_json=json.dumps(signed)), response)
    assert json.loads(response.result_json)['outcome'] == 'ok'
    assert calls == [body]
    signed['actor']['id'] = 99
    _callback()(bridge, SimpleNamespace(payload_json=json.dumps(signed)), response)
    assert json.loads(response.result_json)['outcome'] == 'refused'
    assert calls == [body]
