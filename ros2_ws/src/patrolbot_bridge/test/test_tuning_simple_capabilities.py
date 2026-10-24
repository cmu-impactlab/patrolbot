"""Read-only native support collection, freshness and fixed-composition refusals."""

import copy
from types import SimpleNamespace as NS

import pytest

from patrolbot_bridge import tuning_backend, tuning_nav2
from patrolbot_bridge.tuning_backend import CACHE_SOURCES, NODES, RosParameterTransport


@pytest.fixture
def rig(monkeypatch):
    now = [100.]
    monkeypatch.setattr(tuning_backend.time, 'monotonic', lambda: now[0])
    components = ['/bt_navigator', '/controller_server', '/behavior_server', '/planner_server',
                  '/smoother_server', '/route_server', '/waypoint_follower', '/amcl', '/velocity_smoother']
    names = [(name[1:], '/') for name in components]
    instance = object.__new__(RosParameterTransport)
    instance._simple_capability_sample = None
    instance.node = NS(_session_generation=1, _epoch=NS(guard=NS(generation=1)),
                       get_node_names_and_namespaces=lambda: names)
    instance.types = {'components': NS(Request=lambda: NS()), 'maintenance': NS(Request=lambda: NS(payload_json=''))}
    calls = []
    instance._call = lambda *args: calls.append(('components',)) or NS(full_node_names=components)
    instance.state = lambda node: calls.append(('state', node)) or 3
    instance.native_cache = lambda node: calls.append(('cache', node)) or {'schema': 1, 'values': {}}
    gate = {'implementation': 'nav2-1.3.12-composed-actions-v1', 'faulted': False}
    peers = {'navigation': {'action_gate': gate}, 'physical': {'policy_version': 1},
             'web': {'policy_handshake_version': 1}}
    def maintenance(peer, operation, context):
        assert operation == 'observe'
        calls.append((peer, operation))
        return copy.deepcopy(peers[peer])
    instance.maintenance = maintenance
    instance._fixed_publisher_graph = lambda: calls.append(('graph',)) or {}
    host = [True]
    def request(path, value, timeout):
        assert path == tuning_nav2.CORE_SOCKET and value == {'op': 'capability'}
        if not host[0]:
            raise FileNotFoundError('helper not installed')
        return {'ok': True, 'implementation': 'navigation-only-restart-v1'}
    monkeypatch.setattr(tuning_nav2, 'request', request)
    return NS(transport=instance, now=now, components=components, names=names,
              calls=calls, peers=peers, host=host)


def test_support_requires_actual_cached_implementation_and_composed_membership_not_just_services(rig):
    assert not rig.transport.simple_capabilities()['ready']
    rig.transport.poll_simple_capabilities()
    result = rig.transport.simple_capabilities()
    assert result['ready'] and result['nav2_redeploy_supported']
    assert result['hot_support'] == list(NODES)
    assert {call[1] for call in rig.calls if call[0] == 'cache'} == set(CACHE_SOURCES)
    assert {call for call in rig.calls if len(call) == 2 and call[1] == 'observe'} == {
        ('navigation', 'observe'), ('physical', 'observe'), ('web', 'observe')}
    result['states'].clear()
    assert rig.transport.simple_capabilities()['states']  # caller cannot alter the retained sample


def test_missing_host_helper_does_not_disable_a_verified_hot_only_plan(rig):
    rig.host[0] = False
    rig.transport.poll_simple_capabilities()
    result = rig.transport.simple_capabilities()
    assert result['ready'] and not result['nav2_redeploy_supported']


@pytest.mark.parametrize('fault', ['missing_component', 'duplicate', 'namespace', 'old_action_abi',
                                 'action_fault', 'physical_version', 'web_version', 'web_boolean', 'cache_failure', 'interface_abi'])
def test_unknown_native_deployment_or_implementation_never_advertises_execution(rig, fault):
    if fault == 'missing_component':
        rig.components.remove('/behavior_server')
    elif fault == 'duplicate':
        rig.names.append(('controller_server', '/'))
    elif fault == 'namespace':
        rig.names.remove(('amcl', '/'))
        rig.names.append(('amcl', '/foreign'))
    elif fault == 'old_action_abi':
        rig.peers['navigation']['action_gate']['implementation'] = 'navigator-only'
    elif fault == 'action_fault':
        rig.peers['navigation']['action_gate']['faulted'] = True
    elif fault == 'physical_version':
        rig.peers['physical']['policy_version'] = True
    elif fault == 'web_version':
        rig.peers['web']['policy_handshake_version'] = 0
    elif fault == 'web_boolean':
        rig.peers['web']['policy_handshake_version'] = True
    elif fault == 'interface_abi':
        rig.transport.types['maintenance'] = NS(Request=lambda: NS())
    else:
        rig.transport.native_cache = lambda node: (_ for _ in ()).throw(ValueError('cache mismatch'))
    rig.transport.poll_simple_capabilities()
    assert not rig.transport.simple_capabilities()['ready']


@pytest.mark.parametrize('fault', ['stale', 'clock_regression', 'hardware', 'origin'])
def test_support_observation_does_not_survive_time_or_session_boundaries(rig, fault):
    rig.transport.poll_simple_capabilities()
    if fault == 'stale': rig.now[0] += 3.01
    elif fault == 'clock_regression': rig.now[0] -= .01
    elif fault == 'hardware': rig.transport.node._session_generation += 1
    else: rig.transport.node._epoch.guard.generation += 1
    assert not rig.transport.simple_capabilities()['ready']


def test_the_full_read_interval_is_charged_to_capability_age(rig):
    def slow_read(node):
        rig.now[0] += 1.
        return {}
    rig.transport.native_cache = slow_read
    rig.transport.poll_simple_capabilities()
    assert not rig.transport.simple_capabilities()['ready']
