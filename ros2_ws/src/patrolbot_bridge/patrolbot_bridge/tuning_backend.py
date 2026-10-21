"""Fixed-node lifecycle reload and exact parameter readback, owned by EpochRuntime."""

import math
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
            external = ('raw_odom_fresh', 'stationary', 'hardware_ok', 'dock_ok', 'faults_clear',
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
        from lifecycle_msgs.srv import ChangeState
        from nav2_msgs.srv import ManageLifecycleNodes
        from nav2_msgs.srv import LoadMap
        from rcl_interfaces.srv import GetParameters, SetParametersAtomically
        from std_srvs.srv import SetBool, Trigger
        self.node, self.timeout = node, timeout
        self.types = {'get': GetParameters, 'set': SetParametersAtomically,
                      'change': ChangeState, 'manager': ManageLifecycleNodes, 'hold': SetBool,
                      'map': LoadMap, 'map_ack': Trigger, 'nav_hold': SetBool}
        self.clients = {}
        self.readonly_clients = {}
        for name in NODES:
            for kind, suffix in (('get', 'get_parameters'), ('set', 'set_parameters_atomically')):
                self._client(kind, name, suffix)
        for name in MANAGERS:
            self._client('manager', name, 'manage_nodes')
        self._client('change', 'teleop_velocity_smoother', 'change_state')
        self._client('hold', 'lifecycle_manager_script', 'tuning_maintenance')
        self._client('map', 'map_server', 'load_map')
        self._client('map_ack', 'amcl', 'map_consumed')
        self._client('nav_hold', 'bt_navigator', 'tuning_hold')
        self._client('joy_idle', 'patrolbot', 'joy_tuning_idle')
        self._client('web_idle', 'patrolbot', 'web_tuning_idle')

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
