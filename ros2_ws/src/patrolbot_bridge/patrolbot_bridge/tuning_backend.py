"""Fixed-node lifecycle reload and exact parameter readback, owned by EpochRuntime."""

import math
import copy
import threading
import time

from . import tuning_spec
from .tuning_activation import AdmissionRefused, evaluate_evidence, ReloadResult
from .tuning_readback import READBACK_NODES
from .tuning_yaml import strict_yaml_load


NODES = ('teleop_velocity_smoother', 'amcl', 'controller_server', 'velocity_smoother',
         'behavior_server', 'patrolbot_joy_teleop')
MANAGERS = ('lifecycle_manager_navigation', 'lifecycle_manager_localization')
MANAGER_COMMANDS = ('STARTUP', 'PAUSE', 'RESUME', 'RESET', 'SHUTDOWN', 'CONFIGURE', 'CLEANUP')
MAINTENANCE_PEERS = {
    'navigation': ('bt_navigator', 'tuning_maintenance'),
    'physical': ('patrolbot', 'joy_tuning_maintenance'),
    'web': ('patrolbot', 'web_tuning_maintenance'),
    'switch': ('cmd_vel_mux', 'tuning_maintenance'),
    'undock': ('patrolbot', 'undock_tuning_maintenance'),
    'safety': ('patrolbot_safety_watchdog', 'tuning_maintenance'),
    'lifecycle_manager_script': ('lifecycle_manager_script', 'tuning_maintenance_v2'),
    'lifecycle_manager_navigation': ('lifecycle_manager_navigation', 'tuning_maintenance'),
    'lifecycle_manager_localization': ('lifecycle_manager_localization', 'tuning_maintenance'),
}
CACHE_SOURCES = {
    'amcl': ('tuning_cache', 'nav2-1.3.12-amcl-cache-v1'),
    'controller_server': ('FollowPath/tuning_cache', 'nav2-1.3.12-rpp-cache-v1'),
    'behavior_server': ('spin/tuning_cache', 'nav2-1.3.12-spin-cache-v1'),
    'velocity_smoother': ('tuning_cache', 'nav2-1.3.12-smoother-cache-v1'),
    'teleop_velocity_smoother': ('tuning_cache', 'nav2-1.3.12-smoother-cache-v1'),
}


def _lookup(document, dotted):
    current = document
    for part in dotted.split('.'):
        current = current[part]
    return current


def parameter_plan(revision, index):
    """Read fixed logical fields and required derived caps from immutable artifacts."""
    documents = {scope: strict_yaml_load(text) for scope, text in revision['generated'].items()}
    plan = {node: {} for node in NODES}
    for entry in index.values():
        if not entry.get('accepts_edits') or entry.get('protected') or entry.get('not_applicable'):
            continue
        source = entry['source']
        node = source['node']
        if node not in plan:
            raise ValueError('parameter source is not a fixed tuning node')
        scope = 'bringup.yaml' if node == 'teleop_velocity_smoother' else 'navigation.yaml'
        params = documents[scope][node]['ros__parameters']
        key = source['key']
        plan[node][key] = _lookup(params, key)
    for node, keys in {
        'teleop_velocity_smoother': ('max_velocity', 'min_velocity'),
        'velocity_smoother': ('max_velocity', 'min_velocity'),
        'controller_server': ('FollowPath.regulated_linear_scaling_min_speed',),
        'behavior_server': ('min_rotational_vel',),
        'amcl': ('set_initial_pose', 'always_reset_initial_pose'),
        'patrolbot_joy_teleop': tuple(tuning_spec.f710_policy(
            revision['overrides'], index)),
    }.items():
        scope = 'bringup.yaml' if node == 'teleop_velocity_smoother' else 'navigation.yaml'
        params = documents[scope][node]['ros__parameters']
        for key in keys:
            plan[node][key] = _lookup(params, key)
    # Enforce documented RPP domain AFTER all ceiling/producer generation.
    controller = plan['controller_server']
    if (plan['amcl']['set_initial_pose'] is not False
            or plan['amcl']['always_reset_initial_pose'] is not True):
        raise ValueError('configuration must require a fresh operator localization seed')
    if controller['FollowPath.min_approach_linear_velocity'] <= .01:
        raise ValueError('generated RPP approach minimum must be greater than 0.01')
    if controller['FollowPath.min_lookahead_dist'] > controller['FollowPath.max_lookahead_dist']:
        raise ValueError('generated lookahead minimum exceeds maximum')
    return plan


def matching(expected, actual):
    if isinstance(expected, list):
        return (isinstance(actual, (list, tuple)) and len(expected) == len(actual)
                and all(matching(a, b) for a, b in zip(expected, actual)))
    if isinstance(expected, bool) or isinstance(actual, bool):
        return type(expected) is type(actual) and expected == actual
    if type(expected) in (float, int) and type(actual) in (float, int):
        return math.isfinite(actual) and abs(expected - actual) <= 1e-9
    return type(expected) is type(actual) and expected == actual


class LifecycleBackend:
    """Reload only allowlisted nodes under the existing durable mutation hold."""

    # Legacy all-node reload remains an offline regression surface, not the
    # executor for the new reviewed scoped contract. No native plan support is
    # claimed until manager/cache/ownership tests commission that exact path.
    requires_review = True
    plan_support = {}
    plan_implementation = 'unverified'

    def __init__(self, store, transport, probe, require_seed, *, enabled=False,
                 selected_map=None, generation=None):
        self.store, self.transport = store, transport
        self._probe, self._require_seed = probe, require_seed
        self.enabled = enabled is True
        self.selected_map = selected_map
        self.generation = generation or (lambda: None)
        self._reload_generation = None
        self.detail = 'runtime tuning requires configured fixed-node services and safe idle'
        self._readback = None
        self._navigation_reserved = False

    @property
    def approved(self):
        return self.enabled and self.transport.ready()

    def probe(self):
        return self._probe()

    def preflight(self, revision, digest):
        state = self.store.load_revision(revision)
        if state['meta']['generated_digest'] != digest:
            raise ValueError('immutable revision digest mismatch')
        parameter_plan(state, self.store.index)
        selected = self.selected_map() if self.selected_map is not None else None
        if not isinstance(selected, dict) or not selected.get('path'):
            raise ValueError('selected map identity is unavailable')

    def reserve_navigation(self):
        self.transport.navigation_maintenance(True)
        self._navigation_reserved = True

    def release_navigation_reservation(self):
        self.transport.navigation_maintenance(False)
        self._navigation_reserved = False

    def inspect(self, revision, digest):
        state = self.store.load_revision(revision)
        if state['meta']['generated_digest'] != digest:
            raise ValueError('immutable revision digest mismatch')
        plan = parameter_plan(state, self.store.index)
        if self.transport.get('patrolbot_joy_teleop', ['tuning_policy_version']) != {
                'tuning_policy_version': 1}:
            raise ValueError('joystick lacks live cached-policy update support')
        actual = {node: self.transport.get(node, sorted(values)) for node, values in plan.items()}
        if not all(set(actual[node]) == set(values)
                   and all(matching(value, actual[node][key]) for key, value in values.items())
                   for node, values in plan.items()):
            raise ValueError('runtime parameter readback differs from reviewed revision')
        return state, actual

    def reload(self, scopes, revision, digest):
        if not self.approved or set(scopes) != {'bringup', 'navigation'}:
            return ReloadResult('failed', 'fixed lifecycle backend unavailable or scope mismatch')
        try:
            self.preflight(revision, digest)
        except (ValueError, OSError) as exc:
            return ReloadResult('failed', str(exc))
        allowed, reason = evaluate_evidence(self.probe())
        if not allowed:
            return ReloadResult('failed', reason)
        state = self.store.load_revision(revision)
        if state['meta']['generated_digest'] != digest:
            return ReloadResult('failed', 'immutable revision digest mismatch')
        plan = parameter_plan(state, self.store.index)
        if self.transport.get('patrolbot_joy_teleop', ['tuning_policy_version']) != {
                'tuning_policy_version': 1}:
            return ReloadResult('failed', 'joystick live-policy update support unavailable')
        selected = self.selected_map() if self.selected_map is not None else None
        if not isinstance(selected, dict) or not selected.get('path'):
            return ReloadResult('failed', 'selected map identity is unavailable')
        self._readback = None
        self._reload_generation = self.generation()
        try:
            if not self._navigation_reserved:
                self.reserve_navigation()
            self.transport.joystick_idle()
            self.transport.web_idle()
            # The persistent final-smoother manager must acknowledge that no
            # automatic configure/activate transition is still in flight.
            self.transport.smoother_maintenance(True)
            for manager in MANAGERS:
                self.transport.manager(manager, 'PAUSE')
            self.transport.transition('teleop_velocity_smoother', 'DEACTIVATE')
            for manager in MANAGERS:
                self.transport.manager(manager, 'CLEANUP')
            self.transport.transition('teleop_velocity_smoother', 'CLEANUP')
            # Never reuse AMCL's last stored pose after a lifecycle reload.
            self.transport.set('amcl', {
                'set_initial_pose': False, 'always_reset_initial_pose': True})
            for node, parameters in plan.items():
                self.transport.set(node, parameters)
            self.transport.manager('lifecycle_manager_localization', 'CONFIGURE')
            self.transport.manager('lifecycle_manager_localization', 'RESUME')
            map_identity = self.transport.restore_map(selected)
            self.transport.manager('lifecycle_manager_navigation', 'CONFIGURE')
            # Navigation remains INACTIVE until a new operator localization
            # seed is acknowledged by the existing EpochRuntime coordinator.
            self.transport.transition('teleop_velocity_smoother', 'CONFIGURE')
            self.transport.transition('teleop_velocity_smoother', 'ACTIVATE')
            self.inspect(revision, digest)
            if self.generation() != self._reload_generation:
                raise ValueError('hardware/origin generation changed during configuration')
            observed = self.probe()
            external = ('hardware_ok', 'dock_ok', 'faults_clear',
                        'physical_deadman_fresh', 'web_deadman_fresh')
            if (not all(getattr(observed, key) is True for key in external)
                    or observed.physical_deadman_held is not False
                    or observed.web_deadman_held is not False):
                raise ValueError('fresh external idle evidence changed during reload')
            self.store.record_navigation_admission({
                'pending': True, 'revision': revision, 'digest': digest,
                'map_identity': map_identity,
            })
            self._require_seed(map_identity)
            self._readback = (revision, digest)
            return ReloadResult('ok', 'parameters verified; fresh operator localization required')
        except Exception as exc:
            return ReloadResult('unknown', 'lifecycle/readback outcome unknown: '
                                + exc.__class__.__name__)

    def committed(self, revision, digest):
        if self._readback != (revision, digest):
            raise ValueError('runtime revision was not verified before startup commit')
        if self.generation() != self._reload_generation:
            raise ValueError('hardware/origin generation changed before commit')
        self.transport.smoother_maintenance(False)
        self._navigation_reserved = False
        # Navigator admission remains held until the existing EpochRuntime
        # observes a NEW seed and exact global-costmap map consumption.


class RosParameterTransport:
    """Bounded ROS service calls with separate reentrant completion callbacks."""

    def __init__(self, node, *, timeout=5.):
        from lifecycle_msgs.srv import ChangeState, GetState
        from nav2_msgs.srv import ManageLifecycleNodes
        from nav2_msgs.srv import LoadMap
        from rcl_interfaces.srv import GetParameters, SetParametersAtomically
        from std_srvs.srv import SetBool, Trigger
        from composition_interfaces.srv import ListNodes
        self.node, self.timeout = node, timeout
        self.types = {'get': GetParameters, 'set': SetParametersAtomically,
                      'state': GetState,
                      'change': ChangeState, 'manager': ManageLifecycleNodes, 'hold': SetBool,
                      'map': LoadMap, 'map_ack': Trigger, 'nav_hold': SetBool, 'components': ListNodes}
        self.clients = {}
        self.readonly_clients = {}
        self._scoped_protocol_evidence = None
        self._simple_input_context = None
        self._simple_input_peers = []
        self._simple_input_established = False
        self._simple_before_caches = {}
        self._simple_before_graph = None
        self._simple_capability_sample = None
        self._simple_restart_boot = None
        self._client('components', 'nav2_container', '_container/list_nodes')
        for name in NODES:
            for kind, suffix in (('get', 'get_parameters'), ('set', 'set_parameters_atomically')):
                self._client(kind, name, suffix)
        for name in MANAGERS:
            self._client('manager', name, 'manage_nodes')
        for name in NODES:
            if name != 'patrolbot_joy_teleop':
                self._client('change', name, 'change_state')
                self._client('state', name, 'get_state')
        for name, (suffix, _) in CACHE_SOURCES.items():
            self._client('cache', name, suffix)
        self._client('hold', 'lifecycle_manager_script', 'tuning_maintenance')
        self._client('map', 'map_server', 'load_map')
        self._client('map_ack', 'amcl', 'map_consumed')
        self._client('nav_hold', 'bt_navigator', 'tuning_hold')
        self._client('joy_idle', 'patrolbot', 'joy_tuning_idle')
        self._client('web_idle', 'patrolbot', 'web_tuning_idle')
        self.maintenance_clients = {}
        try:
            from patrolbot_interfaces.srv import TuningMaintenance
            self.types['maintenance'] = TuningMaintenance
            for peer, (node_name, suffix) in MAINTENANCE_PEERS.items():
                self.maintenance_clients[peer] = self.node.create_client(
                    TuningMaintenance, '/' + node_name + '/' + suffix, callback_group=self.node._callbacks)
            from patrolbot_interfaces.srv import TuningTransition
            for manager in MANAGERS:
                self.clients['subset_transition', manager] = self.node.create_client(
                    TuningTransition, '/' + manager + '/tuning_transition', callback_group=self.node._callbacks)
        except ImportError:
            # Legacy native images cannot advertise reviewed execution.
            pass

    def scoped_capabilities(self):
        """Read-only capability cache; no service waits inside snapshot locks.

        The native commissioning collector must bind exact image/cache/manager
        implementations and every motion path. Endpoint existence alone cannot
        populate this cache. Until that collector is integrated, native reviewed
        activation remains unavailable, regardless of the runtime enable flag.
        """
        return {'ready': False, 'implementation': 'unverified', 'support': {}, 'policy_support': False}

    def simple_capabilities(self):
        sample = getattr(self, '_simple_capability_sample', None)
        generation = (self.node._session_generation, self.node._epoch.guard.generation) if sample else None
        if sample and 0 <= time.monotonic() - sample[0] <= 3. and sample[1] == generation:
            return copy.deepcopy(sample[2])
        return {'ready': False, 'implementation': 'unverified', 'states': {},
                'hot_support': [], 'policy_support': False, 'nav2_redeploy_supported': False}

    def poll_simple_capabilities(self):
        """Read-only collection on the retained worker, never inside UI locks."""
        from .tuning_nav2 import CORE_SOCKET, request
        started = time.monotonic()
        generation = (self.node._session_generation, self.node._epoch.guard.generation)
        try:
            if not hasattr(self.types['maintenance'].Request(), 'payload_json'):
                raise ValueError('native maintenance interface lacks held policy payloads')
            components = self._call('components', 'nav2_container', self.types['components'].Request())
            required = {'/bt_navigator', '/controller_server', '/behavior_server', '/planner_server',
                        '/smoother_server', '/route_server', '/waypoint_follower', '/amcl', '/velocity_smoother'}
            if components is None or not required <= set(components.full_node_names):
                raise ValueError('fixed composed Nav2 consumers are unavailable')
            names = self.node.get_node_names_and_namespaces()
            if any(names.count((name[1:], '/')) != 1 for name in required):
                raise ValueError('duplicate or standalone Nav2 consumers are unsupported')
            states = {node: self.state(node) for node in NODES if node != 'patrolbot_joy_teleop'}
            states['patrolbot_joy_teleop'] = 'running'
            # These endpoints return audited execution caches, not parameter
            # setter existence or saved targets. Unknown ABI/version refuses.
            for node in CACHE_SOURCES:
                self.native_cache(node)
            probe = {'job_id': '00000000-0000-4000-8000-000000000000',
                     'digest': 'sha256:' + '0' * 64, 'plan_hash': 'sha256:' + '0' * 64}
            navigation = self.maintenance('navigation', 'observe', probe)
            gate = navigation.get('action_gate', {})
            if gate.get('implementation') != 'nav2-1.3.12-composed-actions-v1' or gate.get('faulted') is not False:
                raise ValueError('composed action input implementation is unavailable')
            physical = self.maintenance('physical', 'observe', probe)
            web = self.maintenance('web', 'observe', probe)
            if (type(physical.get('policy_version')) is not int or physical['policy_version'] != 1
                    or type(web.get('policy_handshake_version')) is not int or web['policy_handshake_version'] != 1):
                raise ValueError('manual cached-policy implementation is unavailable')
            self._fixed_publisher_graph()
            restart = False
            try:
                host = request(CORE_SOCKET, {'op': 'capability'}, timeout=.2)
                restart = host.get('implementation') == 'navigation-only-restart-v1'
            except (OSError, ValueError, RuntimeError):
                pass  # Missing host support does not disable verified hot-only plans.
            value = {'ready': True, 'implementation': 'nav2-1.3.12-composed-hot-or-restart-v1',
                     'states': states, 'hot_support': list(NODES), 'policy_support': True,
                     'nav2_redeploy_supported': restart}
            if generation != (self.node._session_generation, self.node._epoch.guard.generation):
                raise ValueError('hardware generation changed while collecting capabilities')
            self._simple_capability_sample = (started, generation, value)
        except Exception:
            self._simple_capability_sample = None

    def maintenance(self, peer, operation, value, *, payload=None):
        from .tuning_json import loads
        from .tuning_lease import context
        value = context(value)
        if peer not in MAINTENANCE_PEERS or operation not in ('acquire', 'block', 'observe', 'inspect', 'release', 'stage', 'committed', 'restore_trims'):
            raise ValueError('unallowlisted maintenance peer or operation')
        if operation in ('stage', 'committed') and (peer != 'web' or not isinstance(payload, dict)):
            raise ValueError('fixed web policy payload required')
        if operation == 'restore_trims' and (peer != 'physical' or not isinstance(payload, dict)):
            raise ValueError('fixed physical trim restoration payload required')
        if operation in ('acquire', 'block', 'observe', 'inspect', 'release') and payload is not None:
            raise ValueError('unexpected maintenance payload')
        if operation == 'block' and peer not in ('physical', 'web'):
            raise ValueError('block/observe is a fixed manual-input operation')
        if operation == 'observe' and peer not in ('physical', 'web', 'navigation'):
            raise ValueError('unallowlisted maintenance observation')
        client = self.maintenance_clients.get(peer)
        if client is None or not client.service_is_ready():
            raise RuntimeError('required native maintenance endpoint unavailable')
        request = self.types['maintenance'].Request()
        request.operation = operation
        native_peer = peer in ('navigation', 'lifecycle_manager_navigation', 'lifecycle_manager_localization')
        for key, item in value.items():
            setattr(request, key, item.removeprefix('sha256:') if native_peer and key != 'job_id' else item)
        if payload is not None:
            import json
            request.payload_json = json.dumps(payload, sort_keys=True, allow_nan=False)
        future = client.call_async(request)
        finished = threading.Event()
        future.add_done_callback(lambda _: finished.set())
        if not finished.wait(self.timeout):
            client.remove_pending_request(future)
            future.cancel()
            raise TimeoutError('maintenance acknowledgment outcome unknown')
        result = future.result()
        if result is None:
            raise RuntimeError('maintenance acknowledgment missing')
        if not result.accepted:
            definitive = result.detail in (
                'invalid_request', 'foreign_owner', 'lifecycle_busy', 'job_already_released',
                'lease_history_full', 'navigator_lease_refused', 'maintenance owner/input busy',
                'Web input or another maintenance job owns admission')
            if operation in ('acquire', 'block') and definitive:
                raise AdmissionRefused('native owner refused ' + peer)
            raise RuntimeError('native owner acknowledgment lost for ' + peer)
        evidence = loads(result.evidence_json, 20 * 1024)
        if native_peer:
            implementation = ('nav2-1.3.12-navigator-lease-v1' if peer == 'navigation'
                              else 'nav2-1.3.12-scoped-manager-v1')
            if (not isinstance(evidence, dict) or type(evidence.get('schema')) is not int or evidence['schema'] != 1
                    or evidence.get('implementation') != implementation
                    or evidence.get('faulted', False) is not False
                    or (operation not in ('release', 'observe') and (evidence.get('held') is not True
                        or evidence.get('job_id') != value['job_id']
                        or evidence.get('digest') != value['digest'].removeprefix('sha256:')
                        or evidence.get('plan_hash') != value['plan_hash'].removeprefix('sha256:')))):
                raise ValueError('foreign native manager/navigator acknowledgment')
            # Explicit adapter for the pinned native ABI; never infer all-path
            # exclusion from its manager-only/navigator-only lease evidence.
            owner = None
            if operation != 'release' and evidence.get('held') is True:
                owner = {'job_id': evidence['job_id'], 'digest': 'sha256:' + evidence['digest'],
                         'plan_hash': 'sha256:' + evidence['plan_hash']}
            evidence = dict(evidence, version=2, owner=owner)
        if (not isinstance(evidence, dict) or evidence.get('version') != 2
                or (operation not in ('release', 'observe') and evidence.get('owner') != value)):
            raise ValueError('foreign or malformed native maintenance evidence')
        return evidence

    def block_inputs(self, plan, context):
        # One core gate closes every DRIVE path. Manual producers stop accepting
        # input, and the existing navigator owner rejects queued/new goals. No
        # lease on each Nav2 node, mux or safety watchdog is introduced here.
        from .tuning_lease import context as validate_context
        context = validate_context(context)
        if self._simple_input_context is not None:
            raise AdmissionRefused('another Apply owns command admission')
        if not self.node._epoch.reserve_tuning(context['job_id']):
            raise AdmissionRefused('core hardware/lifecycle mutation owner busy')
        self._simple_input_context = context
        self._simple_input_peers = []
        self._simple_input_established = False
        self._simple_restart_boot = None
        try:
            for peer in ('navigation', 'physical', 'web'):
                job = self.node._tuning_store.get_job(context['job_id'])
                self.node._tuning_store.record_job(dict(job,
                    input_block_attempts=list(job.get('input_block_attempts', [])) + [peer]))
                if peer != 'navigation':
                    deadline = time.monotonic() + self.timeout
                    while True:
                        evidence = self.maintenance(peer, 'observe', context)
                        if evidence.get('input_blocked') is True:
                            break
                        if time.monotonic() >= deadline:
                            raise RuntimeError('manual input blocking acknowledgment is unavailable; retain the core gate')
                        threading.Event().wait(.025)
                blocked = self.maintenance(peer, 'acquire' if peer == 'navigation' else 'block', context)
                self._simple_input_peers.append(peer)
                if peer == 'navigation':
                    # Its job token closes action ingress immediately; wait for
                    # the retained core hold as well, so releasing the token
                    # later cannot briefly reopen actions before core release.
                    deadline = time.monotonic() + self.timeout
                    while (isinstance(blocked.get('action_gate'), dict)
                           and blocked['action_gate'].get('core_held') is False):
                        if time.monotonic() >= deadline:
                            raise RuntimeError('Nav2 has not observed the closed core input gate')
                        threading.Event().wait(.025)
                        blocked = self.maintenance('navigation', 'inspect', context)
                    self._verify_action_block(blocked)
                else:
                    self._verify_manual_block(peer, blocked)
            self._simple_input_established = True
            self._simple_before_caches = {node: self.native_cache(node) for node in CACHE_SOURCES}
            self._simple_before_caches['physical'] = self.maintenance('physical', 'inspect', context)
            self._simple_before_graph = self._fixed_publisher_graph()
        except AdmissionRefused:
            # Definitive refusals unwind acknowledged work only. RPC timeouts
            # or uncertain replies propagate with the core gate still closed.
            self.release_inputs(context)
            raise

    def input_gate_closed(self, context):
        return (self._simple_input_established and self._simple_input_context == context
                and self.node._epoch.tuning_hold
                and self.node._epoch._tuning_reservation_job == context['job_id'])

    def release_inputs(self, context):
        if self._simple_input_context != context:
            return False
        for peer in reversed(self._simple_input_peers):
            self.maintenance(peer, 'release', context)
        self._simple_input_peers.clear()
        if not self.node._epoch.release_tuning_reservation(context['job_id']):
            raise RuntimeError('core input gate reservation release unknown')
        self._simple_input_context = None
        self._simple_input_established = False
        return True

    def apply_hot(self, node, parameters, plan, context):
        from .tuning_mutability import classify
        if not self.input_gate_closed(context):
            raise ValueError('hot update requires the closed input gate')
        state = 'running' if node == 'patrolbot_joy_teleop' else self.state(node)
        if classify(node, parameters, state)[0] != 'hot':
            raise ValueError('runtime state no longer supports the reviewed hot update; review restart scope again')
        self.set(node, parameters)

    def restart_navigation(self, plan, context):
        from .tuning_nav2 import CORE_SOCKET, request, candidate_for_job
        if (not self.input_gate_closed(context) or plan.get('execution_kind') != 'nav2_redeploy'
                or plan.get('services') != ['navigation'] or plan.get('image_rebuild') is not False):
            raise ValueError('reviewed navigation-only restart under the input gate required')
        candidate_for_job(self.node._tuning_store, context)
        # Persist seed loss BEFORE any possible restart. No pose is stored or
        # replayed. The exact new map identity replaces this pending marker.
        pending_map = 'nav2-redeploy:' + context['job_id']
        self.node._tuning_store.record_navigation_admission({'pending': True,
            'revision': plan['revision'], 'digest': plan['digest'], 'map_identity': pending_map})
        self.node._tuning_require_seed(pending_map)
        self.navigation_maintenance(True)
        receipt = request(CORE_SOCKET, {'op': 'restart', 'context': context}, timeout=65.)
        deadline = time.monotonic() + 90.
        while True:
            if any(receipt.get(key) != value for key, value in context.items()) or receipt.get('revision') != plan['revision']:
                raise ValueError('foreign navigation restart receipt')
            if receipt.get('state') == 'boot_consumed':
                before, after = receipt['before'], receipt['after']
                if (before['container_id'] != after['container_id'] or before['image'] != after['image']
                        or before['started_at'] == after['started_at']):
                    raise ValueError('navigation replacement identity was not verified')
                break
            if receipt.get('state') != 'dispatching' or time.monotonic() >= deadline:
                raise TimeoutError('exact navigation replacement boot is unverified')
            threading.Event().wait(.1)
            receipt = request(CORE_SOCKET, {'op': 'status', 'context': context})
        # Only READS are repeated during node discovery. Each new owner is
        # acquired once, and no ambiguous mutation is retried.
        self._wait_replacement_readiness(deadline, context)
        self.maintenance('navigation', 'acquire', context)
        self.navigation_maintenance(True)  # replacement's independent seed hold
        blocked = self.maintenance('physical', 'block', context)
        self._verify_manual_block('physical', blocked)
        old = self._simple_before_caches['physical']
        trims = {key: old['policy'][key] for key, flag in (
            ('max_linear', 'operator_linear_trim'), ('max_angular', 'operator_angular_trim')) if old.get(flag) is True}
        if trims:
            self.maintenance('physical', 'restore_trims', context, payload=trims)
        map_ack = self._call('map_ack', 'amcl', self.types['map_ack'].Request())
        if map_ack is None or map_ack.success is not True or not map_ack.message:
            raise ValueError('replacement AMCL has not consumed its map')
        self.node._tuning_store.record_navigation_admission({'pending': True,
            'revision': plan['revision'], 'digest': plan['digest'], 'map_identity': map_ack.message})
        self.node._tuning_require_seed(map_ack.message)
        self._simple_restart_boot = receipt

    def _wait_replacement_readiness(self, deadline, context):
        while time.monotonic() < deadline:
            try:
                for node in CACHE_SOURCES:
                    self.native_cache(node)
                navigation = self.maintenance('navigation', 'observe', context)
                physical = self.maintenance('physical', 'observe', context)
                if (navigation.get('action_gate', {}).get('core_held') is True
                        and physical.get('input_blocked') is True):
                    return
            except (ValueError, RuntimeError, TimeoutError):
                pass
            threading.Event().wait(.1)
        raise TimeoutError('replacement native consumers are unavailable')

    def native_cache(self, node):
        from .tuning_json import loads
        if node not in CACHE_SOURCES:
            raise ValueError('unallowlisted native cache read')
        result = self._call('cache', node, self.types['map_ack'].Request())
        if result is None or result.success is not True:
            raise ValueError('native execution cache unavailable')
        value = loads(result.message, 20 * 1024)
        if (not isinstance(value, dict) or type(value.get('schema')) is not int or value['schema'] != 1
                or value.get('implementation') != CACHE_SOURCES[node][1]
                or not isinstance(value.get('values'), dict)):
            raise ValueError('native cache implementation is not the audited version')
        return value

    def _fixed_publisher_graph(self):
        result = {}
        for topic, owner in (('/cmd_vel_out', 'cmd_vel_mux'), ('/cmd_vel', 'teleop_velocity_smoother')):
            endpoints = self.node.get_publishers_info_by_topic(topic)
            if (len(endpoints) != 1 or endpoints[0].node_name != owner
                    or endpoints[0].node_namespace != '/' or endpoints[0].topic_type != 'geometry_msgs/msg/Twist'):
                raise ValueError('ordinary command publisher graph changed')
            result[topic] = {'node': owner, 'gid': bytes(endpoints[0].endpoint_gid).hex()}
        return result

    def inspect_parameters(self, expected):
        """Read-only parameter/native-cache observations, not Apply admission.

        Manual policy, command ownership and invariants are checked separately
        by verify_application; this helper never fabricates an activation proof.
        """
        actual = {node: self.get(node, sorted(values)) for node, values in expected.items()}
        for node, values in expected.items():
            if (set(actual[node]) != set(values)
                    or any(not matching(value, actual[node][key]) for key, value in values.items())):
                raise ValueError('parameter readback differs from reviewed revision: ' + node)
        caches = {node: self.native_cache(node) for node in CACHE_SOURCES}
        for node, observation in caches.items():
            consumed = observation['values']
            for key, value in expected[node].items():
                native_key = key.removeprefix('FollowPath.') if node == 'controller_server' else key
                if native_key not in consumed or not matching(value, consumed[native_key]):
                    raise ValueError('actual consumed cache differs: ' + node + ':' + key)
            if node == 'controller_server' and not matching(
                    expected[node]['FollowPath.desired_linear_vel'], consumed.get('base_desired_linear_vel')):
                raise ValueError('RPP base-cruise cache was not refreshed')
        return actual, caches

    @staticmethod
    def _verify_action_block(observation):
        # The older navigator token excludes only navigator goals. Direct
        # SimpleActionServer consumers share the new compiled gate exclusively
        # in the fixed single-process composed deployment, never by endpoint name.
        gate = observation.get('action_gate')
        if (not isinstance(gate, dict)
                or gate.get('implementation') != 'nav2-1.3.12-composed-actions-v1'
                or gate.get('held') is not True or gate.get('core_held') is not True
                or gate.get('faulted') is not False
                or type(gate.get('active_goals')) is not int or gate['active_goals'] != 0
                or type(gate.get('workers')) is not int or gate['workers'] != 0):
            raise ValueError('composed Nav2 action input is not blocked and idle')

    @staticmethod
    def _verify_manual_block(peer, observation):
        if peer == 'physical':
            buffers = observation.get('buffers')
            valid = (observation.get('input_blocked') is True
                and type(observation.get('policy_version')) is int and observation['policy_version'] == 1
                and observation.get('release_required') is True
                and type(observation.get('operator_linear_trim')) is bool
                and type(observation.get('operator_angular_trim')) is bool
                and isinstance(observation.get('policy'), dict) and isinstance(buffers, dict)
                and set(buffers) == {'linear_target', 'angular_target', 'linear_command', 'angular_command'}
                and all(type(value) in (int, float) and value == 0. for value in buffers.values()))
        elif peer == 'web':
            valid = (observation.get('input_blocked') is True and observation.get('session_active') is False
                     and matching([0., 0., False], observation.get('buffers')))
        else:
            valid = False
        if not valid:
            raise ValueError('manual input/buffers are not blocked and zero')

    def verify_application(self, expected, plan, context):
        if not self.input_gate_closed(context):
            raise ValueError('input admission is not closed')
        actual, caches = self.inspect_parameters(expected)
        if plan['execution_kind'] == 'scoped_parameters':
            before = self._simple_before_caches.get('amcl')
            if (not isinstance(before, dict)
                    or type(before.get('filter_generation')) is not int or before['filter_generation'] < 1
                    or type(caches['amcl'].get('filter_generation')) is not int
                    or caches['amcl'].get('filter_generation') != before.get('filter_generation')):
                raise ValueError('hot apply unexpectedly recreated the AMCL particle filter')
        elif plan['execution_kind'] == 'nav2_redeploy':
            receipt = getattr(self, '_simple_restart_boot', None)
            admission = self.node._tuning_store.navigation_admission()
            epoch = self.node._epoch
            if (not isinstance(receipt, dict) or receipt.get('state') != 'boot_consumed'
                    or any(receipt.get(key) != value for key, value in context.items())
                    or receipt.get('revision') != plan['revision']
                    or not isinstance(admission, dict) or admission.get('pending') is not True
                    or admission.get('revision') != plan['revision'] or admission.get('digest') != plan['digest']
                    or getattr(epoch, '_tuning_nav_pending', False) is not True
                    or getattr(epoch, '_accepted_seed_stamp', None) is not None
                    or caches['amcl'].get('initial_pose_known') is not False):
                raise ValueError('replacement boot and fresh-seed barrier are not verified')
            consumed = self._call('map_ack', 'amcl', self.types['map_ack'].Request())
            if consumed is None or consumed.success is not True or consumed.message != admission['map_identity']:
                raise ValueError('replacement map identity changed before verification')
        # A locally remembered closure is not a fresh acknowledgment that the
        # navigator still owns this exact job. In particular, replacement or
        # foreign-owner evidence must not be hidden by the final DRIVE gate.
        self._verify_action_block(self.maintenance('navigation', 'inspect', context))
        physical = self.maintenance('physical', 'inspect', context)
        web = self.maintenance('web', 'inspect', context)
        self._verify_manual_block('physical', physical)
        self._verify_manual_block('web', web)
        for key, value in expected['patrolbot_joy_teleop'].items():
            cached = physical['policy'].get(key)
            trim = {'max_linear': 'operator_linear_trim', 'max_angular': 'operator_angular_trim'}.get(key)
            if (trim and self._simple_before_caches.get('physical', {}).get(trim) is True
                    and physical.get(trim) is not True):
                raise ValueError('apply discarded a lower operator trim')
            if trim and physical.get(trim) is True:
                if type(cached) not in (int, float) or not 0 <= cached <= value:
                    raise ValueError('operator trim exceeds reviewed policy')
                old = self._simple_before_caches.get('physical', {}).get('policy', {}).get(key)
                if type(old) not in (int, float) or cached > old:
                    raise ValueError('apply raised a lower operator trim')
            elif not matching(value, cached):
                raise ValueError('F710 consumed policy was not refreshed')
        payload = self._policy_payload(plan)
        if plan['cache_consumers']:
            staged = self.node._tuning_policy.staged_evidence(context)
            if staged['limits'] != payload['effective_limits'] or staged['drive_blocked'] is not True:
                raise ValueError('core candidate clamp is not exact and command-blocked')
            if web.get('staged_policy') != payload or web.get('committed') is not False:
                raise ValueError('web producer did not independently observe the exact candidate policy')
        elif self.node._tuning_policy.limits() != payload['effective_limits']:
            raise ValueError('unchanged core policy evidence differs')
        graph = self._fixed_publisher_graph()
        if graph != self._simple_before_graph:
            raise ValueError('unreviewed final command publisher/session changed')
        evidence = self.node._tuning_evidence()
        if any(getattr(evidence, name) is not True for name in (
                'hardware_ok', 'dock_ok', 'faults_clear', 'command_buffers_zero',
                'autonomy_idle', 'maps_clear', 'undock_clear', 'lifecycle_clear')):
            raise ValueError('fresh non-odometry command/hardware evidence changed')
        priorities = {'topics.safety_controller.priority': 10, 'topics.undock_turn.priority': 9,
                      'topics.joy.priority': 8, 'topics.teleop.priority': 7,
                      'topics.switch.priority': 6, 'topics.navigation.priority': 5}
        if self.get_readonly('cmd_vel_mux', sorted(priorities)) != priorities:
            raise ValueError('protected mux priority semantics changed or unavailable')
        return {'parameters': actual, 'verified_nodes': sorted(NODES),
            'verified_policies': ['core_clamp', 'f710_cache', 'web_sessions'],
            'motion_paths_held': True, 'protected_invariants_verified': True,
            'publisher_graph': {topic: [value['node']] for topic, value in graph.items()},
            'cache_evidence': caches, 'physical_policy': physical['policy'],
            'input_policy': 'blocked; input is discarded, not queued/replayed',
            'navigation_boot': copy.deepcopy(getattr(self, '_simple_restart_boot', None))}

    def state(self, node):
        if node not in NODES or node == 'patrolbot_joy_teleop':
            raise ValueError('unallowlisted lifecycle observation node')
        result = self._call('state', node, self.types['state'].Request())
        if result is None or result.current_state.id not in (1, 2, 3):
            raise ValueError('native lifecycle state unknown or transitional')
        return result.current_state.id

    def scoped_transition(self, node, verb, context):
        from .tuning_scoped import MANAGER_FOR
        if node not in MANAGER_FOR or verb not in ('DEACTIVATE', 'CLEANUP', 'CONFIGURE', 'ACTIVATE'):
            raise ValueError('unallowlisted scoped transition')
        # Direct native transition only while its existing manager acknowledges
        # exact-job exclusion; no change to cached manager node_names/siblings.
        manager = MANAGER_FOR[node]
        self.maintenance(manager, 'inspect', context)
        if manager == 'lifecycle_manager_script':
            self.transition(node, verb)
            return
        from patrolbot_interfaces.srv import TuningTransition
        key = ('subset_transition', manager)
        if key not in self.clients:
            self.clients[key] = self.node.create_client(TuningTransition, '/' + manager + '/tuning_transition',
                                                       callback_group=self.node._callbacks)
        request = TuningTransition.Request()
        request.job_id = context['job_id']
        request.digest = context['digest'].removeprefix('sha256:')
        request.plan_hash = context['plan_hash'].removeprefix('sha256:')
        request.node_name = node
        request.transition_id, request.expected_state = {
            'CONFIGURE': (1, 1), 'CLEANUP': (2, 2), 'ACTIVATE': (3, 2), 'DEACTIVATE': (4, 3)}[verb]
        result = self._call('subset_transition', manager, request)
        if result is None or result.accepted is not True:
            raise RuntimeError('scoped manager transition outcome is unverified')
        self.maintenance(manager, 'inspect', context)

    def _policy_payload(self, plan):
        state = self.node._tuning_store.load_revision(plan['revision'])
        return {'version': 1, 'revision': plan['revision'], 'session_id': plan['session_id'],
                'hardware_session_generation': self.node._session_generation,
                'effective_limits': tuning_spec.effective_limits(state['overrides'], self.node._tuning_store.index)}

    def stage_policies(self, plan, context):
        payload = self._policy_payload(plan)
        owned = (self.node._epoch.tuning_hold and self.node._epoch._tuning_reservation_job == context['job_id'])
        self.node._tuning_policy.stage(context, payload['effective_limits'], maintenance_owned=owned)
        self.node._tuning_runtime_policy_binding = dict(payload, **context, phase='candidate')
        self.node._publish_tuning_runtime_policy()
        self._wait_web_policy(context, payload, phase='candidate')
        self.maintenance('web', 'stage', context, payload=payload)

    def _wait_web_policy(self, context, payload, *, phase):
        # Inspect-only polling resolves ROS topic/service ordering. Never retry
        # an ambiguous stage/commit operation or renew a stale policy timestamp.
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            evidence = self.maintenance('web', 'inspect', context)
            observed = evidence.get('observed_policy')
            if (isinstance(observed, dict) and observed.get('phase') == phase
                    and all(observed.get(key) == item for key, item in dict(payload, **context).items())):
                return
            threading.Event().wait(.025)
        raise TimeoutError('web producer has not observed exact held core policy')

    def commit_policies(self, plan, context):
        from .tuning_policy import limits_from_store
        payload = self._policy_payload(plan)
        generation, limits = limits_from_store(self.node._tuning_store)
        if not plan['cache_consumers']:
            # Unrelated edits do not rewrite/stage the same producer caps.
            self.node._tuning_policy.refresh()
            return (self.node._tuning_policy.generation == generation
                    and self.node._tuning_policy.limits() == limits == payload['effective_limits'])
        result = self.node._tuning_policy.commit_staged(context, generation=generation, committed_limits=limits,
            maintenance_owned=self.node._epoch.tuning_hold and self.node._epoch._tuning_reservation_job == context['job_id'])
        if result['limits'] != payload['effective_limits']:
            raise ValueError('core committed policy differs from reviewed candidate')
        self.node._tuning_runtime_policy_binding = dict(payload, **context, phase='committed', policy_generation=generation)
        self.node._publish_tuning_runtime_policy()
        self._wait_web_policy(context, payload, phase='committed')
        observed = self.maintenance('web', 'committed', context, payload=payload)
        return observed.get('committed') is True

    def _client(self, kind, node, suffix):
        srv = self.types.get(kind, self.types['map_ack'])
        self.clients[kind, node] = self.node.create_client(
            srv, '/' + node + '/' + suffix, callback_group=self.node._callbacks)

    def ready(self):
        request = self.types['manager'].Request()
        return (all(hasattr(request, verb) for verb in MANAGER_COMMANDS)
                and all(client.service_is_ready() for client in self.clients.values()))

    def _call(self, kind, node, request):
        client = self.clients[kind, node]
        if not client.service_is_ready():
            raise RuntimeError('fixed service unavailable')
        future = client.call_async(request)
        finished = threading.Event()
        future.add_done_callback(lambda _: finished.set())
        deadline = time.monotonic() + self.timeout
        while not finished.wait(.05):
            stop = getattr(self.node, '_stop_event', None)
            if time.monotonic() >= deadline or (stop is not None and stop.is_set()):
                client.remove_pending_request(future)
                future.cancel()
                raise TimeoutError('tuning service outcome unknown')
        return future.result()

    def get(self, node, names):
        from rclpy.parameter import parameter_value_to_python
        request = self.types['get'].Request()
        request.names = names
        result = self._call('get', node, request)
        if result is None or len(result.values) != len(names):
            raise ValueError('missing parameter readback')
        return dict(zip(names, (parameter_value_to_python(value) for value in result.values)))

    def get_readonly(self, node, names):
        """Read fixed observation sources without requiring mutation services."""
        from rclpy.parameter import parameter_value_to_python
        if node not in READBACK_NODES:
            raise ValueError('unallowlisted parameter observation node')
        if node not in self.readonly_clients:
            self.readonly_clients[node] = self.node.create_client(
                self.types['get'], '/' + READBACK_NODES[node] + '/get_parameters',
                callback_group=self.node._callbacks)
        client = self.readonly_clients[node]
        if not client.service_is_ready():
            return {}
        request = self.types['get'].Request()
        request.names = names
        future = client.call_async(request)
        finished = threading.Event()
        future.add_done_callback(lambda _: finished.set())
        if not finished.wait(.2):
            client.remove_pending_request(future)
            future.cancel()
            return {}
        result = future.result()
        if result is None or len(result.values) != len(names):
            return {}
        return dict(zip(names, (parameter_value_to_python(value) for value in result.values)))

    def set(self, node, values):
        from rclpy.parameter import Parameter
        request = self.types['set'].Request()
        request.parameters = [Parameter(name, value=value).to_parameter_msg()
                              for name, value in values.items()]
        result = self._call('set', node, request)
        if result is None or not result.result.successful:
            raise ValueError('parameter change refused')

    def manager(self, node, verb):
        if node not in MANAGERS or verb not in MANAGER_COMMANDS:
            raise ValueError('unallowlisted lifecycle manager operation')
        request = self.types['manager'].Request()
        request.command = getattr(request, verb)
        result = self._call('manager', node, request)
        if result is None or not result.success:
            raise ValueError('manager transition refused')

    def transition(self, node, verb):
        from lifecycle_msgs.msg import Transition
        if node not in NODES or node == 'patrolbot_joy_teleop' or verb not in (
                'DEACTIVATE', 'CLEANUP', 'CONFIGURE', 'ACTIVATE'):
            raise ValueError('unallowlisted lifecycle transition')
        request = self.types['change'].Request()
        request.transition.id = getattr(Transition, 'TRANSITION_' + verb)
        result = self._call('change', node, request)
        if result is None or not result.success:
            raise ValueError('lifecycle transition refused')

    def smoother_maintenance(self, enabled):
        request = self.types['hold'].Request()
        request.data = enabled
        result = self._call('hold', 'lifecycle_manager_script', request)
        if result is None or not result.success:
            raise ValueError('smoother manager has not acknowledged maintenance')

    def navigation_maintenance(self, enabled):
        request = self.types['nav_hold'].Request()
        request.data = enabled
        result = self._call('nav_hold', 'bt_navigator', request)
        if result is None:
            raise RuntimeError('navigation reservation result is unknown')
        if not result.success:
            raise AdmissionRefused('navigation busy; tuning cannot take ownership')

    def joystick_idle(self):
        self._idle_ack('joy_idle')

    def web_idle(self):
        self._idle_ack('web_idle')

    def _idle_ack(self, kind):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            response = self._call(kind, 'patrolbot', self.types['map_ack'].Request())
            if response is not None and response.success:
                return
            threading.Event().wait(.025)
        raise TimeoutError('manual producer did not acknowledge tuning idle')

    def restore_map(self, selected):
        request = self.types['map'].Request()
        request.map_url = selected['path']
        result = self._call('map', 'map_server', request)
        if result is None or result.result != 0:
            raise ValueError('selected map reload failed')
        grid = result.map
        if (grid.info.width != selected['width'] or grid.info.height != selected['height']
                or abs(grid.info.resolution - selected['resolution']) > 1e-6):
            raise ValueError('selected map readback geometry differs')
        origin = grid.info.origin
        q = origin.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        actual_origin = (origin.position.x, origin.position.y, yaw)
        if (not all(math.isfinite(value) for value in actual_origin)
                or any(abs(left - right) > 1e-6
                       for left, right in zip(actual_origin, selected['origin']))):
            raise ValueError('selected map readback origin differs')
        stamp = grid.info.map_load_time
        identity = f'{stamp.sec}:{stamp.nanosec}:{grid.info.width}:{grid.info.height}'
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            ack = self._call('map_ack', 'amcl', self.types['map_ack'].Request())
            if ack.success and ack.message == identity:
                return identity
            threading.Event().wait(.025)
        raise TimeoutError('AMCL has not consumed the selected map')
