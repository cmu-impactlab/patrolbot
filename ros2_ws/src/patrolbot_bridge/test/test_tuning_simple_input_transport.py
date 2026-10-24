"""Real reduced input adapter: acknowledged closure, safe unwind, unknown holds."""

import copy
from types import SimpleNamespace as NS

import pytest

from patrolbot_bridge.tuning_activation import AdmissionRefused
from patrolbot_bridge.tuning_backend import RosParameterTransport

OWNER = {'job_id': '11111111-1111-4111-8111-111111111111',
         'digest': 'sha256:' + 'a' * 64, 'plan_hash': 'sha256:' + 'b' * 64}


@pytest.fixture
def rig():
    calls, journal = [], [{'job_id': OWNER['job_id']}]
    epoch = NS(tuning_hold=False, _tuning_reservation_job=None)
    def reserve(job):
        calls.append('core_block')
        epoch.tuning_hold = True
        epoch._tuning_reservation_job = job
        return True
    def release(job):
        assert job == OWNER['job_id']
        calls.append('core_release')
        epoch.tuning_hold = False
        epoch._tuning_reservation_job = None
        return True
    epoch.reserve_tuning, epoch.release_tuning_reservation = reserve, release
    instance = object.__new__(RosParameterTransport)
    instance.node = NS(_epoch=epoch, _tuning_store=NS(
        get_job=lambda job: copy.deepcopy(journal[-1]), record_job=lambda job: journal.append(copy.deepcopy(job))))
    instance._simple_input_context = None
    instance._simple_input_established = False
    instance._simple_input_peers = []
    instance.timeout = .01
    instance.native_cache = lambda node: {'values': {}}
    instance._fixed_publisher_graph = lambda: {'/cmd_vel': {'node': 'teleop_velocity_smoother', 'gid': 'a'}}
    physical = {'input_blocked': True, 'input_released': False, 'release_required': True,
                'policy_version': 1, 'policy': {}, 'operator_linear_trim': False, 'operator_angular_trim': False,
                'buffers': dict.fromkeys(('linear_target', 'angular_target', 'linear_command', 'angular_command'), 0.)}
    web = {'input_blocked': True, 'session_active': False, 'buffers': [0., 0., False]}
    navigation = {'action_gate': {'implementation': 'nav2-1.3.12-composed-actions-v1',
                  'held': True, 'core_held': True, 'faulted': False, 'active_goals': 0, 'workers': 0}}
    state = NS(failure=None)
    def maintenance(peer, operation, owner):
        assert epoch.tuning_hold and owner == OWNER  # no remote RPC before core closure
        calls.append((peer, operation))
        if operation in ('acquire', 'block'):
            assert journal[-1]['input_block_attempts'][-1] == peer
        if state.failure == (peer, operation):
            raise TimeoutError('operation may have executed')
        if state.failure == (peer, 'refuse') and operation in ('acquire', 'block'):
            raise AdmissionRefused('definitive owner busy')
        return copy.deepcopy({'physical': physical, 'web': web, 'navigation': navigation}[peer])
    instance.maintenance = maintenance
    return NS(transport=instance, epoch=epoch, calls=calls, journal=journal,
              physical=physical, web=web, navigation=navigation, state=state)


def test_actual_input_adapter_closes_core_first_and_never_calls_held_input_released(rig):
    rig.transport.block_inputs({}, OWNER)
    assert rig.transport.input_gate_closed(OWNER)
    assert rig.calls[:3] == ['core_block', ('navigation', 'acquire'), ('physical', 'observe')]
    assert rig.physical['input_released'] is False
    assert rig.journal[-1]['input_block_attempts'] == ['navigation', 'physical', 'web']
    assert rig.transport.release_inputs(OWNER)
    assert rig.calls[-4:] == [('web', 'release'), ('physical', 'release'), ('navigation', 'release'), 'core_release']
    assert not rig.transport.input_gate_closed(OWNER)


@pytest.mark.parametrize('peer,operation', [('navigation', 'acquire'), ('physical', 'block'), ('web', 'block')])
def test_lost_block_ack_never_auto_releases_or_retries_the_core_boundary(rig, peer, operation):
    rig.state.failure = peer, operation
    with pytest.raises(TimeoutError):
        rig.transport.block_inputs({}, OWNER)
    assert rig.epoch.tuning_hold and rig.transport._simple_input_context == OWNER
    assert not any(isinstance(call, tuple) and call[1] == 'release' for call in rig.calls)
    assert rig.calls.count((peer, operation)) == 1


@pytest.mark.parametrize('peer', ['navigation', 'physical', 'web'])
def test_definitive_refusal_unwinds_acknowledged_peers_without_stealing_refused_owner(rig, peer):
    rig.state.failure = peer, 'refuse'
    with pytest.raises(AdmissionRefused):
        rig.transport.block_inputs({}, OWNER)
    assert not rig.epoch.tuning_hold and rig.transport._simple_input_context is None
    assert (peer, 'release') not in rig.calls
    assert rig.calls[-1] == 'core_release'


@pytest.mark.parametrize('peer,field,value', [('physical', 'buffers', {}),
    ('physical', 'release_required', False), ('web', 'session_active', True), ('web', 'buffers', [0., .1, False])])
def test_owner_ack_without_actual_input_drain_cannot_admit_parameter_mutation(rig, peer, field, value):
    getattr(rig, peer)[field] = value
    with pytest.raises(ValueError, match='blocked and zero'):
        rig.transport.block_inputs({}, OWNER)
    assert rig.epoch.tuning_hold and not rig.transport.input_gate_closed(OWNER)
    assert not any(isinstance(call, tuple) and call[1] == 'release' for call in rig.calls)


def test_foreign_release_cannot_open_core_or_any_producer_gate(rig):
    rig.transport.block_inputs({}, OWNER)
    before = list(rig.calls)
    assert not rig.transport.release_inputs(dict(OWNER, digest='sha256:' + 'c' * 64))
    assert rig.calls == before and rig.transport.input_gate_closed(OWNER)


def test_lost_release_ack_retains_core_hold_and_no_implicit_second_release(rig):
    rig.transport.block_inputs({}, OWNER)
    rig.state.failure = 'web', 'release'
    with pytest.raises(TimeoutError):
        rig.transport.release_inputs(OWNER)
    assert rig.epoch.tuning_hold and rig.calls.count(('web', 'release')) == 1
    assert ('physical', 'release') not in rig.calls and 'core_release' not in rig.calls


@pytest.mark.parametrize('field,value', [('held', False), ('faulted', True), ('active_goals', 1),
    ('workers', 1), ('workers', False), ('core_held', None), ('implementation', 'navigator-only')])
def test_navigator_only_or_running_direct_action_is_not_acknowledged_input_closure(rig, field, value):
    rig.navigation['action_gate'][field] = value
    with pytest.raises(ValueError, match='action input'):
        rig.transport.block_inputs({}, OWNER)
    assert rig.epoch.tuning_hold and not rig.transport.input_gate_closed(OWNER)


def test_topic_service_ordering_is_resolved_by_reads_not_a_second_acquire(rig):
    maintenance = rig.transport.maintenance
    first = [True]
    def delayed_core_hold(peer, operation, owner):
        result = maintenance(peer, operation, owner)
        if peer == 'navigation' and first[0]:
            first[0] = False
            result['action_gate']['core_held'] = False
        return result
    rig.transport.maintenance = delayed_core_hold
    rig.transport.timeout = .1
    rig.transport.block_inputs({}, OWNER)
    assert rig.transport.input_gate_closed(OWNER)
    assert rig.calls.count(('navigation', 'acquire')) == 1
    assert ('navigation', 'inspect') in rig.calls


def test_missing_core_hold_ack_never_releases_the_job_or_retries_acquire(rig):
    rig.navigation['action_gate']['core_held'] = False
    with pytest.raises(RuntimeError, match='closed core input gate'):
        rig.transport.block_inputs({}, OWNER)
    assert rig.epoch.tuning_hold and not rig.transport.input_gate_closed(OWNER)
    assert rig.calls.count(('navigation', 'acquire')) == 1
    assert ('navigation', 'release') not in rig.calls
