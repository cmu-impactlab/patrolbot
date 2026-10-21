"""Observed ROS values never require a saved revision or activation permission."""

import ast
import json
from pathlib import Path
import threading
from types import SimpleNamespace as NS

from patrolbot_bridge.tuning_activation import ActivationCoordinator, UnavailableBackend
from patrolbot_bridge.tuning_readback import ParameterObserver, READBACK_NODES
from patrolbot_bridge.tuning_service import TuningService
from patrolbot_bridge.tuning_store import RevisionStore

ROOT = Path(__file__).resolve().parents[4]
REGISTRY = json.loads((ROOT / 'contracts/robot-tuning.registry.json').read_text())
INDEX = {entry['id']: entry for entry in REGISTRY['parameters']}


def test_uninitialized_store_returns_real_observations_without_claiming_saved_activity(tmp_path):
    observer = ParameterObserver(INDEX, lambda node, names: {
        'max_velocity': [.21, 0., .6], 'alpha1': .123}, lambda: 7, clock=lambda: 10.)
    observer.poll()
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    service = TuningService(store, ActivationCoordinator(store, NS(), UnavailableBackend()),
                            parameter_readback=observer.current)
    snapshot = service.runtime_snapshot()
    assert snapshot['readback']['values']['amcl.alpha1'] == .123
    assert snapshot['readback']['limits'] == {'max_linear_m_s': .21, 'max_angular_rad_s': .6}
    assert snapshot['desired']['amcl.alpha1'] != .123
    assert snapshot['desired_revision'] is None and snapshot['active_revision'] is None
    assert snapshot['active_known'] is False and snapshot['writable'] is False
    assert snapshot['activation_supported'] is False
    assert snapshot['write_disabled_reason'] == 'Robot tuning has not been initialized.'
    assert not store.base.exists()


def test_missing_invalid_and_out_of_bounds_observations_are_not_replaced_by_yaml():
    def get(node, names):
        if node == 'controller_server':
            raise TimeoutError('unavailable')
        return {'max_velocity': [.4, 0., .7], 'alpha1': float('nan'), 'min_particles': True}

    observer = ParameterObserver(INDEX, get, lambda: 1, clock=lambda: 0.)
    observer.poll()
    result = observer.current()
    assert result['values']['max_linear_m_s'] == .4  # expose drift, never widen a policy cap
    assert result['limits'] is None
    assert result['values']['amcl.alpha1'] is None
    assert result['values']['amcl.min_particles'] is None
    assert result['values']['nav2.controller.desired_linear_vel'] is None


def test_observations_expire_and_do_not_cross_hardware_sessions():
    clock = [10.]
    generation = [1]
    observer = ParameterObserver(INDEX, lambda *_: {}, lambda: generation[0],
                                 clock=lambda: clock[0])
    observer.poll()
    clock[0] = 12.9
    assert 2.8 < observer.current()['age_s'] <= 3.
    clock[0] = 13.1
    assert observer.current() is None
    clock[0] = 9.
    assert observer.current() is None
    clock[0], generation[0] = 10., 2
    assert observer.current() is None


def test_observer_only_requests_allowlisted_sources():
    calls = []
    observer = ParameterObserver(INDEX, lambda node, names: calls.append((node, names)) or {},
                                 lambda: 1)
    observer.poll()
    assert {node for node, _ in calls} <= READBACK_NODES.keys()
    assert 'local_costmap.local_costmap' in {node for node, _ in calls}


def bridge_method(name, namespace):
    source = ROOT / 'ros2_ws/src/patrolbot_bridge/patrolbot_bridge/bridge_node.py'
    tree = ast.parse(source.read_text())
    method = next(method for klass in tree.body if isinstance(klass, ast.ClassDef)
                  for method in klass.body if isinstance(method, ast.FunctionDef)
                  and method.name == name)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])),
                 str(source), 'exec'), namespace)
    return namespace[name]


def test_save_admission_only_needs_fresh_sbc_not_motors_pose_or_idle():
    method = bridge_method('_tuning_save_admission', {'time': NS(monotonic=lambda: 10.)})
    # Deliberately has no motors, pose, odometry-valid or autonomy-idle attributes.
    owner = NS(_state_lock=threading.Lock(), _link_connected=True, _flags_at=9.8)
    assert method(owner) == (True, '')
    owner._link_connected = False
    assert method(owner)[0] is False
    owner._link_connected, owner._flags_at = True, 9.
    assert method(owner)[0] is False
    owner._flags_at = 11.
    assert method(owner)[0] is False


def test_verified_committed_readback_is_independent_of_activation_enablement():
    method = bridge_method('_poll_tuning_readback', {'time': NS(monotonic=lambda: 10.)})

    class Backend:
        @property
        def approved(self):
            raise AssertionError('readback must not ask for mutation permission')

        def inspect(self, revision, digest):
            return None, {'teleop_velocity_smoother': {'max_velocity': [.3, 0., .7]}}

    store = NS(index={pid: INDEX[pid] for pid in ('max_linear_m_s', 'max_angular_rad_s')},
               snapshot_state=lambda: {'active_revision': 1},
               load_revision=lambda _: {'meta': {'generated_digest': 'digest'}})
    owner = NS(_tuning_readback_lock=threading.Lock(), _epoch=NS(tuning_hold=False),
               _tuning_backend=Backend(), _tuning_store=store, _session_generation=1)
    method(owner)
    assert owner._tuning_readback['revision'] == 1
    assert owner._tuning_readback['values']['max_linear_m_s'] == .3


def test_observation_worker_serializes_waits_outside_ros_executor():
    method = bridge_method('_tuning_observation_worker', {'time': NS(monotonic=lambda: 10.)})
    calls = []
    stopped = [False]

    def wait(timeout):
        calls.append(('wait', timeout))
        stopped[0] = True

    owner = NS(_stop_event=NS(is_set=lambda: stopped[0], wait=wait),
               _refresh_tuning_policy=lambda: calls.append('policy'),
               _parameter_observer=NS(poll=lambda: calls.append('observations')),
               _poll_tuning_readback=lambda: calls.append('revision'))
    method(owner)
    assert calls == ['policy', 'observations', 'revision', ('wait', .5)]
    source = (ROOT / 'ros2_ws/src/patrolbot_bridge/patrolbot_bridge/bridge_node.py').read_text()
    for callback in ('self._poll_tuning_readback', 'self._parameter_observer.poll',
                     'self._refresh_tuning_policy'):
        assert not any('create_timer(' in line and callback in line
                       for line in source.splitlines())
    assert 'self.tuning_thread,' in source


def test_observation_worker_stops_before_any_more_ros_calls_on_shutdown():
    method = bridge_method('_tuning_observation_worker', {'time': NS(monotonic=lambda: 10.)})
    method(NS(_stop_event=NS(is_set=lambda: True)))
