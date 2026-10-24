"""Pinned native ABI adapter never upgrades manager-only evidence to all paths."""

import json
from types import SimpleNamespace as NS

import pytest

from patrolbot_bridge.tuning_activation import AdmissionRefused
from patrolbot_bridge.tuning_backend import RosParameterTransport

OWNER = {'job_id': '11111111-1111-4111-8111-111111111111',
         'digest': 'sha256:' + 'a' * 64, 'plan_hash': 'sha256:' + 'b' * 64}


class Immediate:
    def __init__(self, value):
        self.value = value

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self.value


def transport(result):
    sent = []
    client = NS(service_is_ready=lambda: True,
                call_async=lambda request: sent.append(request) or Immediate(result))
    instance = object.__new__(RosParameterTransport)
    instance.timeout = .1
    instance.types = {'maintenance': NS(Request=lambda: NS())}
    instance.maintenance_clients = {peer: client for peer in ('navigation', 'web', 'physical', 'lifecycle_manager_navigation')}
    return instance, sent


def native():
    return {'schema': 1, 'implementation': 'nav2-1.3.12-scoped-manager-v1',
            'held': True, 'faulted': False, 'job_id': OWNER['job_id'],
            'digest': 'a' * 64, 'plan_hash': 'b' * 64,
            'states': {'controller_server': 2}, 'prior_states': {'controller_server': 2},
            'all_action_paths_excluded': False}


def test_exact_native_digest_encoding_preserves_manager_only_disclaimer():
    instance, sent = transport(NS(accepted=True, detail='owned', evidence_json=json.dumps(native())))
    result = instance.maintenance('lifecycle_manager_navigation', 'inspect', OWNER)
    assert sent[0].digest == 'a' * 64 and sent[0].plan_hash == 'b' * 64
    assert result['owner'] == OWNER and result['version'] == 2
    assert result['all_action_paths_excluded'] is False


@pytest.mark.parametrize('field,value', [('implementation', 'unverified'), ('job_id', 'foreign'),
                                      ('digest', 'c' * 64), ('held', False), ('faulted', True)])
def test_foreign_or_uncertain_native_ack_is_not_success(field, value):
    evidence = dict(native(), **{field: value})
    instance, _ = transport(NS(accepted=True, detail='owned', evidence_json=json.dumps(evidence)))
    with pytest.raises(ValueError):
        instance.maintenance('lifecycle_manager_navigation', 'inspect', OWNER)


@pytest.mark.parametrize('detail,expected', [('foreign_owner', AdmissionRefused), ('lifecycle_busy', AdmissionRefused),
    ('state_outcome_unknown', RuntimeError), ('recovery_required', RuntimeError), ('new-unrecognized-error', RuntimeError)])
def test_definitive_busy_refusal_is_distinct_from_possibly_owned_remote_state(detail, expected):
    instance, _ = transport(NS(accepted=False, detail=detail, evidence_json='{}'))
    with pytest.raises(expected):
        instance.maintenance('lifecycle_manager_navigation', 'acquire', OWNER)


def test_fixed_payload_only_reaches_web_policy_handshake_not_arbitrary_peer():
    evidence = {'version': 2, 'owner': OWNER, 'idle': True}
    instance, sent = transport(NS(accepted=True, detail='owned', evidence_json=json.dumps(evidence)))
    payload = {'version': 1, 'effective_limits': {'max_linear_m_s': .2, 'max_angular_rad_s': .5}}
    instance.maintenance('web', 'stage', OWNER, payload=payload)
    assert json.loads(sent[0].payload_json) == payload
    with pytest.raises(ValueError):
        instance.maintenance('physical', 'stage', OWNER, payload=payload)
    with pytest.raises(ValueError):
        instance.maintenance('web', 'inspect', OWNER, payload=payload)


def test_endpoint_existence_is_not_native_commissioned_support():
    instance, _ = transport(NS(accepted=True, detail='owned', evidence_json='{}'))
    assert instance.scoped_capabilities()['ready'] is False
