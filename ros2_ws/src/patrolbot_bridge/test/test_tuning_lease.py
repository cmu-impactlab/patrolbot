"""Exact job maintenance leases: no silent timeout release or foreign unlock."""

from types import SimpleNamespace as NS
import json

import pytest

from patrolbot_bridge.tuning_lease import MaintenanceLease, context, serve

JOB = {'job_id': '11111111-1111-4111-8111-111111111111',
       'digest': 'sha256:' + 'a' * 64, 'plan_hash': 'sha256:' + 'b' * 64}


def test_unknown_or_busy_input_cannot_be_forced_into_maintenance():
    for idle in (None, False, 0, 'yes'):
        lease = MaintenanceLease()
        assert not lease.acquire(JOB, idle=idle)
        assert not lease.held


def test_exact_owner_is_idempotent_but_does_not_steal_or_clear_foreign_work():
    lease = MaintenanceLease()
    assert lease.acquire(JOB, idle=True)
    assert lease.acquire(JOB, idle=False)  # status of same reservation, not new permission
    for key, value in [('job_id', '22222222-2222-4222-8222-222222222222'),
                       ('digest', 'sha256:' + 'c' * 64), ('plan_hash', 'sha256:' + 'c' * 64)]:
        other = dict(JOB, **{key: value})
        assert not lease.acquire(other, idle=True)
        assert not lease.release(other)
        assert lease.matches(JOB)
    assert lease.release(JOB)
    assert lease.release(JOB)
    assert not lease.acquire(JOB, idle=True)  # late old acquire cannot re-create released hold


@pytest.mark.parametrize('bad', [None, {}, dict(JOB, service='arbitrary'), dict(JOB, digest='latest'),
                                dict(JOB, job_id='not-a-uuid')])
def test_context_never_accepts_arbitrary_paths_commands_or_implicit_latest(bad):
    with pytest.raises((ValueError, TypeError, AttributeError)):
        context(bad)


def test_fixed_service_reports_consumed_evidence_and_same_owner():
    lease = MaintenanceLease()
    result = serve(lease, NS(operation='acquire', **JOB), NS(), idle=lambda: True,
                   evidence=lambda: {'idle': True, 'policy': {'max_linear_cap': .2}})
    assert result.accepted and 'max_linear_cap' in result.evidence_json
    assert lease.matches(JOB)
    assert not serve(lease, NS(operation='shell', **JOB), NS(), idle=lambda: True,
                     evidence=lambda: {}).accepted
    assert lease.held


def test_observe_reports_actual_input_without_acquiring_or_releasing_an_owner():
    lease = MaintenanceLease()
    def observe():
        return serve(lease, NS(operation='observe', **JOB), NS(), idle=lambda: False,
                     evidence=lambda: {'input_blocked': True, 'input_released': False})
    result = observe()
    assert result.accepted and not lease.held
    assert json.loads(result.evidence_json)['owner'] is None
    assert lease.acquire(JOB, idle=True)
    result = observe()
    assert json.loads(result.evidence_json)['owner'] == JOB
    assert json.loads(result.evidence_json)['input_released'] is False
    assert lease.matches(JOB)


@pytest.mark.parametrize('bad', [{'value': float('nan')}, {'version': 3}, {'owner': JOB}, {'large': 'x' * (21 * 1024)}])
def test_evidence_encoding_failure_never_orphans_a_definitively_refused_reservation(bad):
    lease = MaintenanceLease()
    result = serve(lease, NS(operation='acquire', **JOB), NS(), idle=lambda: True, evidence=lambda: bad)
    assert not result.accepted and not lease.held
