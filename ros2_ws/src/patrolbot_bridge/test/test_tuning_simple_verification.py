"""Exercise the real transport verifier, not an injected 'complete' proof.

I/O is deliberately offline; these tests do not commission ROS or redeployment.
"""

import copy
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from patrolbot_bridge.tuning_activation import Evidence
from patrolbot_bridge.tuning_backend import CACHE_SOURCES, NODES, RosParameterTransport, parameter_plan
from patrolbot_bridge.tuning_store import RevisionStore

ROOT = Path(__file__).resolve().parents[4]
REGISTRY = json.loads((ROOT / 'contracts/robot-tuning.registry.json').read_text())
OWNER = {'job_id': '11111111-1111-4111-8111-111111111111',
         'digest': 'sha256:' + 'a' * 64, 'plan_hash': 'sha256:' + 'b' * 64}
PRIORITIES = {'topics.safety_controller.priority': 10, 'topics.undock_turn.priority': 9,
              'topics.joy.priority': 8, 'topics.teleop.priority': 7,
              'topics.switch.priority': 6, 'topics.navigation.priority': 5}


@pytest.fixture
def rig(tmp_path):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize(actor={'id': 1, 'username': 'offline-cache-test', 'role': 'administrator'},
                     reason='isolated verification fixture')
    expected = parameter_plan(store.load_revision(1), store.index)
    actual = copy.deepcopy(expected)
    caches = {node: {'schema': 1, 'implementation': implementation,
                    'values': {key.removeprefix('FollowPath.'): value for key, value in expected[node].items()}}
              for node, (_, implementation) in CACHE_SOURCES.items()}
    caches['controller_server']['values']['base_desired_linear_vel'] = expected['controller_server']['FollowPath.desired_linear_vel']
    caches['amcl']['filter_generation'] = 1
    physical = {'input_blocked': True, 'release_required': True, 'policy_version': 1,
                'operator_linear_trim': False, 'operator_angular_trim': False,
                'policy': copy.deepcopy(expected['patrolbot_joy_teleop']),
                'buffers': dict.fromkeys(('linear_target', 'angular_target', 'linear_command', 'angular_command'), 0.)}
    web = {'input_blocked': True, 'session_active': False, 'buffers': [0., 0., False]}
    graph = {'/cmd_vel_out': {'node': 'cmd_vel_mux', 'gid': '01' * 24},
             '/cmd_vel': {'node': 'teleop_velocity_smoother', 'gid': '02' * 24}}
    evidence = dict(hardware_ok=True, dock_ok=True, faults_clear=True, command_buffers_zero=True,
                    autonomy_idle=True, maps_clear=True, undock_clear=True, lifecycle_clear=True)
    limits = {'max_linear_m_s': .3, 'max_angular_rad_s': .7}
    payload = {'effective_limits': limits}
    plan = {'execution_kind': 'scoped_parameters', 'cache_consumers': []}
    policy = NS(limits=lambda: limits,
                staged_evidence=lambda owner: {'limits': limits, 'drive_blocked': True})
    peers = {'physical': physical, 'web': web, 'navigation': {'action_gate': {
        'implementation': 'nav2-1.3.12-composed-actions-v1', 'held': True, 'core_held': True,
        'faulted': False, 'active_goals': 0, 'workers': 0}}}
    calls = []
    instance = object.__new__(RosParameterTransport)
    instance.node = NS(_tuning_evidence=lambda: Evidence(**evidence), _tuning_policy=policy)
    instance.input_gate_closed = lambda owner: owner == OWNER
    instance.get = lambda node, keys: copy.deepcopy(actual[node])
    instance.native_cache = lambda node: copy.deepcopy(caches[node])
    instance.maintenance = lambda peer, operation, owner: calls.append((peer, operation, owner)) or peers[peer]
    instance._fixed_publisher_graph = lambda: copy.deepcopy(graph)
    instance._simple_before_graph = copy.deepcopy(graph)
    instance._simple_before_caches = dict(copy.deepcopy(caches), physical=copy.deepcopy(physical))
    instance._policy_payload = lambda reviewed: payload
    instance.get_readonly = lambda node, keys: copy.deepcopy(PRIORITIES)
    return NS(transport=instance, expected=expected, actual=actual, caches=caches,
              physical=physical, web=web, evidence=evidence, graph=graph, plan=plan,
              payload=payload, calls=calls)


def verify(rig):
    return rig.transport.verify_application(rig.expected, rig.plan, OWNER)


def test_complete_revision_reads_actual_consumers_without_odom_seed_or_a_map(rig):
    proof = verify(rig)
    assert proof['verified_nodes'] == sorted(NODES)
    assert proof['parameters'] == rig.expected
    assert proof['motion_paths_held'] and proof['protected_invariants_verified']
    assert ('navigation', 'inspect', OWNER) in rig.calls


@pytest.mark.parametrize('node', NODES)
def test_successful_parameter_writes_do_not_excuse_missing_readback(rig, node):
    rig.actual[node].pop(next(iter(rig.actual[node])))
    with pytest.raises(ValueError, match='parameter readback'):
        verify(rig)


@pytest.mark.parametrize('node', CACHE_SOURCES)
def test_matching_ros_getter_never_masks_an_old_actual_execution_cache(rig, node):
    key = next(iter(rig.caches[node]['values']))
    rig.caches[node]['values'][key] = 'OLD EXECUTION VALUE'
    with pytest.raises(ValueError, match='consumed cache'):
        verify(rig)


def test_rpp_base_cruise_is_verified_separately_from_current_speed(rig):
    rig.caches['controller_server']['values']['base_desired_linear_vel'] = .01
    with pytest.raises(ValueError, match='base-cruise'):
        verify(rig)


@pytest.mark.parametrize('generation', [None, True, 0, 2, '1'])
def test_missing_or_reset_amcl_filter_cannot_claim_hot_localization_preservation(rig, generation):
    rig.caches['amcl']['filter_generation'] = generation
    with pytest.raises(ValueError, match='particle filter'):
        verify(rig)


def test_missing_generation_on_both_sides_is_not_evidence_of_preservation(rig):
    del rig.caches['amcl']['filter_generation']
    del rig.transport._simple_before_caches['amcl']['filter_generation']
    with pytest.raises(ValueError, match='particle filter'):
        verify(rig)


@pytest.mark.parametrize('field,value', [('input_blocked', False), ('release_required', False),
    ('policy_version', True), ('buffers', {}), ('buffers', {'linear_target': 0.}),
    ('operator_linear_trim', None), ('operator_angular_trim', 0)])
def test_incomplete_producer_evidence_never_counts_as_blocked_and_drained(rig, field, value):
    rig.physical[field] = value
    with pytest.raises(ValueError, match='blocked and zero'):
        verify(rig)


@pytest.mark.parametrize('value', [False, .1, float('nan'), None])
def test_physical_buffer_zeros_are_typed_complete_and_finite(rig, value):
    rig.physical['buffers']['linear_target'] = value
    with pytest.raises(ValueError, match='blocked and zero'):
        verify(rig)


@pytest.mark.parametrize('field,value', [('session_active', True), ('input_blocked', False),
    ('buffers', [False, 0., False]), ('buffers', [0., 0., 0]), ('buffers', [0., 0.]), ('buffers', [0., .1, False])])
def test_web_evidence_has_explicit_inactive_session_and_typed_zero_buffers(rig, field, value):
    rig.web[field] = value
    with pytest.raises(ValueError, match='blocked and zero'):
        verify(rig)


def test_lower_operator_trim_is_preserved_without_misreporting_ros_parameter(rig):
    rig.physical.update(operator_linear_trim=True)
    rig.physical['policy']['max_linear'] = .1
    rig.transport._simple_before_caches['physical']['policy']['max_linear'] = .1
    assert verify(rig)['physical_policy']['max_linear'] == .1
    rig.physical['policy']['max_linear'] = .15
    with pytest.raises(ValueError, match='raised a lower operator trim'):
        verify(rig)


def test_trim_flag_does_not_allow_exceeding_reviewed_ceiling(rig):
    rig.physical['operator_linear_trim'] = True
    rig.physical['policy']['max_linear'] = 1.
    with pytest.raises(ValueError, match='exceeds reviewed policy'):
        verify(rig)


def test_changed_core_and_web_policies_require_independent_exact_staging(rig):
    rig.plan['cache_consumers'] = ['core_clamp', 'web_sessions']
    with pytest.raises(ValueError, match='web producer'):
        verify(rig)
    rig.web.update(staged_policy=copy.deepcopy(rig.payload), committed=False)
    assert verify(rig)['verified_policies'] == ['core_clamp', 'f710_cache', 'web_sessions']
    rig.transport.node._tuning_policy.staged_evidence = lambda owner: {'limits': {}, 'drive_blocked': True}
    with pytest.raises(ValueError, match='core candidate clamp'):
        verify(rig)


def test_a_final_publisher_replacement_is_not_hidden_by_the_same_node_name(rig):
    rig.graph['/cmd_vel']['gid'] = '03' * 24
    with pytest.raises(ValueError, match='publisher/session changed'):
        verify(rig)


@pytest.mark.parametrize('field', ['hardware_ok', 'dock_ok', 'faults_clear', 'command_buffers_zero',
                                 'autonomy_idle', 'maps_clear', 'undock_clear', 'lifecycle_clear'])
def test_required_non_odometry_evidence_remains_required_during_input_block(rig, field):
    rig.evidence[field] = None
    with pytest.raises(ValueError, match='hardware evidence'):
        verify(rig)


def test_mux_priority_drift_is_not_an_accepted_configuration(rig):
    rig.transport.get_readonly = lambda node, keys: dict(PRIORITIES, **{'topics.joy.priority': 9})
    with pytest.raises(ValueError, match='mux priority'):
        verify(rig)


def test_lost_navigator_acknowledgment_is_not_masked_by_local_closure(rig):
    def maintenance(peer, operation, owner):
        if peer == 'navigation':
            raise TimeoutError('fresh exact-owner evidence unavailable')
        return {'physical': rig.physical, 'web': rig.web}[peer]
    rig.transport.maintenance = maintenance
    with pytest.raises(TimeoutError):
        verify(rig)


@pytest.mark.parametrize('fault', [None, 'boot', 'revision', 'seed', 'amcl_pose', 'map'])
def test_restart_verification_needs_exact_replacement_and_unseeded_admission(rig, fault):
    rig.plan.update(execution_kind='nav2_redeploy', revision=2, digest=OWNER['digest'])
    rig.transport._simple_restart_boot = dict(OWNER, state='boot_consumed', revision=2)
    admission = {'pending': True, 'revision': 2, 'digest': OWNER['digest'], 'map_identity': '1:2:3:4'}
    rig.transport.node._tuning_store = NS(navigation_admission=lambda: admission)
    rig.transport.node._epoch = NS(_tuning_nav_pending=True, _accepted_seed_stamp=None)
    rig.caches['amcl']['initial_pose_known'] = False
    rig.transport.types = {'map_ack': NS(Request=lambda: NS())}
    rig.transport._call = lambda *args: NS(success=True, message='wrong' if fault == 'map' else '1:2:3:4')
    if fault == 'boot': rig.transport._simple_restart_boot['state'] = 'dispatching'
    elif fault == 'revision': admission['revision'] = 1
    elif fault == 'seed': rig.transport.node._epoch._accepted_seed_stamp = 123
    elif fault == 'amcl_pose': rig.caches['amcl']['initial_pose_known'] = True
    if fault:
        with pytest.raises(ValueError, match='replacement'):
            verify(rig)
    else:
        assert verify(rig)['navigation_boot']['state'] == 'boot_consumed'


@pytest.mark.parametrize('schema,implementation,values', [(True, 'nav2-1.3.12-amcl-cache-v1', {}),
    (1, 'wrong-version', {}), (1, 'nav2-1.3.12-amcl-cache-v1', [])])
def test_native_cache_requires_the_exact_audited_abi(schema, implementation, values):
    instance = object.__new__(RosParameterTransport)
    instance.types = {'map_ack': NS(Request=lambda: NS())}
    instance._call = lambda *args: NS(success=True, message=json.dumps(
        {'schema': schema, 'implementation': implementation, 'values': values}))
    with pytest.raises(ValueError, match='not the audited version'):
        instance.native_cache('amcl')
