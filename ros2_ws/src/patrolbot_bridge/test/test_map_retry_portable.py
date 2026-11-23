"""Production coordinator methods with portable message/service doubles.

This is NOT native ROS/DDS evidence. The native companion runs the same retry
scenarios in test_map_runtime.py with generated ROS types. No acceptance, tick,
lifecycle ordering, epoch, catalog, persistence or localization method is mocked.
"""

import ast
import math
from pathlib import Path
import socket
import sys
import threading
import time
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock

from patrolbot_bridge.epoch_recovery import EpochRecovery
from patrolbot_bridge.map_catalog import ActiveMapStore, MapCatalog
from patrolbot_bridge.map_transaction import MapTransaction
from patrolbot_bridge.odom_epoch import OdomEpochGuard
from patrolbot_bridge.pose_innovation import (
    innovation, OdomHistory, propagate, valid_planar_covariance,
)
import pytest


ROOT = Path(__file__).resolve().parents[1]


class Time:
    def __init__(self, sec=0, nanosec=0):
        self.sec, self.nanosec = sec, nanosec


def pose():
    return NS(position=NS(x=0.0, y=0.0, z=0.0),
              orientation=NS(x=0.0, y=0.0, z=0.0, w=0.0))


def covariance():
    return NS(pose=pose(), covariance=[0.0] * 36)


class PoseWithCovarianceStamped:
    def __init__(self):
        self.header = NS(frame_id='', stamp=Time())
        self.pose = covariance()


class OccupancyGrid:
    def __init__(self):
        self.header = NS(frame_id='')
        self.info = NS(map_load_time=Time(), width=0, height=0,
                       resolution=0.0, origin=pose())


class Twist:
    def __init__(self):
        self.linear = NS(x=0.0, y=0.0, z=0.0)
        self.angular = NS(x=0.0, y=0.0, z=0.0)


class Odometry:
    def __init__(self):
        self.header = NS()
        self.pose = covariance()
        self.twist = NS(twist=Twist())


class LaserScan:
    def __init__(self):
        self.header = NS()


class RosTime:
    def __init__(self, *, nanoseconds):
        self.nanoseconds = nanoseconds

    def to_msg(self):
        return Time(sec=self.nanoseconds // 10**9, nanosec=self.nanoseconds % 10**9)


class Goal:
    def __init__(self):
        self.pose = covariance()
        self.measured_pose = False


class Result:
    OK, REJECTED, FAILED, CANCELED, TIMEOUT, SEED_INNOVATION_REJECTED = range(6)


class Request:
    PAUSE, RESUME = 1, 2

    def __init__(self):
        self.transition = NS(id=0)
        self.goal_info = NS(stamp=Time())


class Parameter:
    def __init__(self, name, value):
        self.name, self.value = name, value

    def to_parameter_msg(self):
        return NS(name=self.name, value=NS(bool_value=self.value))


@pytest.fixture
def coordinator(monkeypatch):
    namespace = dict(
        math=math, os=__import__('os'), Path=Path, time=time, socket=socket,
        threading=threading, EpochRecovery=EpochRecovery, MapCatalog=MapCatalog,
        ActiveMapStore=ActiveMapStore, MapTransaction=MapTransaction,
        OdomEpochGuard=OdomEpochGuard, OdomHistory=OdomHistory,
        innovation=innovation, propagate=propagate,
        valid_planar_covariance=valid_planar_covariance,
        PoseWithCovarianceStamped=PoseWithCovarianceStamped, Twist=Twist,
        Odometry=Odometry, RosTime=RosTime, LaserScan=LaserScan,
        GoalStatusArray=NS, LocalizeOnMap=NS(Goal=Goal, Result=Result, Feedback=NS),
        GoalResponse=NS(ACCEPT=2, REJECT=1), CancelResponse=NS(ACCEPT=2, REJECT=1),
        QoSProfile=lambda **kw: NS(**kw),
        DurabilityPolicy=NS(TRANSIENT_LOCAL=1), ReliabilityPolicy=NS(RELIABLE=1),
        get_package_share_directory=lambda _: str(ROOT.parent / 'patrolbot_navigation'),
        ActionServer=Mock(),
    )
    for name in ('Trigger', 'SetBool', 'Empty', 'ChangeState', 'GetState',
                 'ClearEntireCostmap', 'LoadMap', 'ManageLifecycleNodes', 'GetMap',
                 'CancelGoal', 'SetParametersAtomically'):
        namespace[name] = NS(Request=Request)
    for name, values in (
        ('rcl_interfaces.srv', {'SetParametersAtomically': namespace['SetParametersAtomically']}),
        ('rclpy.parameter', {'Parameter': Parameter}),
    ):
        module = ModuleType(name)
        module.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, module)
    for filename in ('map_runtime', 'epoch_runtime'):
        path = ROOT / 'patrolbot_bridge' / (filename + '.py')
        tree = ast.parse(path.read_text())
        body = [item for item in tree.body if isinstance(item, (ast.ClassDef, ast.FunctionDef))]
        exec(compile(ast.Module(body=body, type_ignores=[]), str(path), 'exec'), namespace)
    module = ModuleType('patrolbot_bridge.map_runtime')
    module.__dict__.update(namespace)
    monkeypatch.setitem(sys.modules, module.__name__, module)
    # Load the native harness/scenarios, replacing only unavailable ROS imports.
    path = ROOT / 'test/test_map_runtime.py'
    tree = ast.parse(path.read_text())
    unavailable = ('builtin_interfaces', 'geometry_msgs', 'nav_msgs', 'sensor_msgs',
                   'patrolbot_interfaces', 'rclpy',
                   'patrolbot_bridge.epoch_runtime', 'patrolbot_bridge.map_runtime')
    tree.body = [item for item in tree.body if not (
        isinstance(item, ast.ImportFrom) and item.module.startswith(unavailable))]
    namespace.update(Time=Time, OccupancyGrid=OccupancyGrid, __file__=str(path))
    exec(compile(tree, str(path), 'exec'), namespace)
    return namespace


@pytest.fixture
def runtime(coordinator, tmp_path, monkeypatch):
    return coordinator['runtime'].__wrapped__(tmp_path, monkeypatch)


# Explicit inventory: the existing native suite stays excluded from portable CI.
SCENARIOS = (
    'sensor_loss_then_explicit_retry',
    'no_prior_localization',
    'unsafe_refusals',
    'pending_mutation_and_retired_completion',
    'unknown_mutation',
    'retired_sensor_callbacks',
    'replay_identity_and_busy',
    'no_implicit_resume',
    'invalid_pose_and_identity',
    'late_tf_probe',
    'diagnostic_cancel_is_noop',
    'refused_replay_never_becomes_mutation',
    'estimate_is_not_verification',
    'epoch_unknown_withdraws_ready_commands',
    'tuning_gate_cannot_cross_new_map',
    'definite_load_refusal_can_reconcile',
    'retired_epoch_negative_stays_unknown',
    'new_origin_through_real_parser',
    'guarded_reverse_read_only_probe_still_admissible',
    'definite_tuning_reserve_refusal_is_not_unknown',
    'completion_never_discards_a_pending_tuning_mutation',
)


@pytest.mark.parametrize('scenario', SCENARIOS)
def test_stationary_retry_production_methods(runtime, coordinator, scenario):
    coordinator['test_stationary_retry_' + scenario](runtime)


@pytest.mark.parametrize('phase', ['inspect', 'load', 'activate', 'resume',
                                  'refresh-localization', 'costmap-map'])
def test_settled_stage_retry(runtime, coordinator, phase):
    coordinator['test_stationary_retry_after_settled_stages'](runtime, phase)


@pytest.mark.parametrize('phase', ['pause', 'load', 'activate', 'resume',
                                  'refresh-localization'])
@pytest.mark.parametrize('unknown', [False, True])
def test_unresolved_stage_retry(runtime, coordinator, phase, unknown):
    coordinator['test_stationary_retry_unresolved_mutations'](runtime, phase, unknown)


@pytest.mark.parametrize('dispatch', [False, True])
def test_unknown_epoch_lifecycle(runtime, coordinator, dispatch):
    coordinator['test_stationary_retry_epoch_unknown_is_not_unlocked'](runtime, dispatch)


@pytest.mark.parametrize('retired', [False, True])
def test_negative_lifecycle_is_not_settlement(runtime, coordinator, retired):
    coordinator['test_stationary_retry_negative_lifecycle_is_not_settlement'](runtime, retired)


@pytest.mark.parametrize('dispatch', [False, True])
def test_unknown_map_blocks_manual_boundaries(runtime, coordinator, dispatch):
    coordinator['test_stationary_retry_unknown_map_blocks_manual_boundaries'](runtime, dispatch)


@pytest.mark.parametrize('phase', ['reserve', 'release'])
@pytest.mark.parametrize('failure', ['dispatch', 'result'])
def test_tuning_unknown_is_latched(runtime, coordinator, phase, failure):
    coordinator['test_stationary_retry_tuning_unknown_is_latched'](runtime, phase, failure)


@pytest.mark.parametrize('failure', ['dispatch', 'result', 'negative'])
def test_read_only_tuning_probe_can_retry(runtime, coordinator, failure):
    coordinator['test_stationary_retry_read_only_tuning_probe_can_retry'](runtime, failure)


@pytest.mark.parametrize('bad', ['stale', 'future', 'nonfinite', 'moving', 'discontinuity',
                                 'wrong-session', 'wrong-origin', 'malformed-origin'])
def test_wait_reset_rejects_untrusted_raw(runtime, coordinator, bad):
    coordinator['test_stationary_retry_wait_reset_rejects_untrusted_raw'](runtime, bad)


@pytest.mark.parametrize('block', ['map-hold', 'epoch-failed', 'pending-control',
                                   'hardware-lock', 'action', 'tuning'])
def test_guarded_reverse_keeps_global_locks(runtime, coordinator, block):
    coordinator['test_stationary_retry_guarded_reverse_keeps_global_locks'](runtime, block)


@pytest.mark.parametrize('latch', ['_map_outcome_unknown', '_lifecycle_outcome_unknown',
                                   '_tuning_gate_outcome_unknown'])
def test_unknown_owner_fences_otherwise_ready_robot(runtime, coordinator, latch):
    coordinator['test_stationary_retry_unknown_owner_fences_otherwise_ready_robot'](runtime, latch)


def test_full_replay_history_does_not_evict_or_admit(runtime, coordinator):
    runtime.MAP_REQUEST_LIMIT = 2
    ids = {'00000000-0000-4000-8000-000000000001', '00000000-0000-4000-8000-000000000002'}
    runtime._map_seen_requests.update(ids)
    request = coordinator['request'](
        runtime, request_id='00000000-0000-4000-8000-000000000003')
    assert runtime._map_accept(request) == coordinator['GoalResponse'].ACCEPT
    result = coordinator['finish_action'](runtime, request)
    assert result.code == result.REJECTED and 'request history full' in result.message
    assert runtime._map_seen_requests == ids and runtime.maps.token is None


@pytest.mark.parametrize('mutation', [False, True])
def test_retired_map_future_never_settles_a_different_owner(runtime, coordinator, mutation):
    runtime._map_step = 'activate' if mutation else 'readback'
    runtime._map_future_step = runtime._map_step
    runtime._map_future_owner = object()
    runtime._map_future_mutation = mutation
    runtime._map_future = coordinator['Future'](NS(success=True))
    with pytest.raises(RuntimeError, match='retired map service response'):
        runtime._map_call(None, None, 'next', lambda _: pytest.fail('retired response consumed'))
    assert runtime._map_outcome_unknown is mutation
    assert runtime._map_future is None


def test_malformed_lifecycle_result_does_not_settle(runtime):
    runtime._map_future_step = 'activate'
    assert not runtime._map_response_settled(None)
