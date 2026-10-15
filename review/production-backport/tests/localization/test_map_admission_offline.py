"""Run the actual admission/execute methods without ROS, DDS or robot access.

Only ROS boundary types are doubles; catalog transaction and covariance checks
are production code. MAP_ADMISSION_SOURCE selects a failing-before baseline.
"""
import ast
import math
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock
from uuid import UUID

import pytest

from patrolbot_bridge.map_transaction import MapTransaction
from patrolbot_bridge.pose_innovation import valid_planar_covariance

SOURCE = Path(os.environ.get('MAP_ADMISSION_SOURCE',
              str(Path(__file__).parents[2] / 'source/ros2_ws/src/patrolbot_bridge/patrolbot_bridge/map_runtime.py')))
tree = ast.parse(SOURCE.read_text())
class_node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MapRuntimeMixin')
class Result:
    REJECTED = 1
    CANCELED = 3
class Response:
    ACCEPT = 2
    REJECT = 1
namespace = dict(math=math, time=time, UUID=UUID, Twist=lambda: NS(),
                 valid_planar_covariance=valid_planar_covariance,
                 LocalizeOnMap=NS(Result=Result), GoalResponse=Response,
                 EpochRecovery=NS(READY='ready', WAIT_SEED='wait-operator-seed',
                                  WAIT_RESET='wait-amcl-reset', WAIT_POSE='wait-amcl-pose'))
exec(compile(ast.Module(body=[class_node], type_ignores=[]), str(SOURCE), 'exec'), namespace)


class Catalog:
    entry = dict(map_id='test-map', revision='revision', path='unused')
    def require(self, map_id, revision):
        if map_id != 'test-map' or revision != 'revision':
            raise ValueError('map revision unavailable')
        return self.entry
    def contains(self, entry, x, y): return 0 <= x <= 10 and 0 <= y <= 10


@pytest.fixture
def runtime():
    r = namespace['MapRuntimeMixin']()
    r.lock = threading.RLock()
    r.maps = MapTransaction(Catalog())
    r.guard = NS(ready=True, session=1)
    r.recovery = NS(token=object(), stage='wait-operator-seed')
    r._map_error = ''; r._map_goal = None; r._map_owner_request_id = None; r._map_future = None; r._map_step = None
    r._future = None; r._future_kind = None; r._phase = 'inspect'
    r.undock_hold = False; r._navigation_status = False
    r._navigation_idle_at = time.monotonic(); r._map_precheck_results = {}
    for name in ('_get_map', '_amcl_map', '_get', '_global_map', '_localization_active', '_navigation_idle'):
        setattr(r, name, NS(remove_pending_request=Mock(), service_is_ready=lambda: True))
    r.node = NS(_state_lock=threading.RLock(), _pose_lock=threading.RLock(),
                _control_lock=threading.RLock(), _undock_goal_lock=threading.RLock(),
                _pose_received=True, _link_connected=True, _fault_flags=0,
                _pose_at=time.monotonic(), _flags_at=time.monotonic(),
                _charge_state_at=time.monotonic(), _vx=0., _vth=0.,
                _undock_active=False, _undock_goal_reserved=False,
                _pending_controls={}, _lidar_ready=lambda: True,
                _undock_turn=NS(revoke=Mock()), cmd_vel_callback=Mock(), get_logger=Mock(return_value=Mock()))
    return r


def request():
    covariance = [0.] * 36
    for index in (0, 7, 35): covariance[index] = .1
    pose = NS(position=NS(x=1., y=1., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))
    return NS(request_id='aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee',
              map_id='test-map', map_revision='revision', pose=NS(pose=pose, covariance=covariance),
              operator_authorized=True, measured_pose=False)


def terminal_refusal(r, reason):
    goal = request()
    token, future, owner = r.maps.token, r._map_future, r._map_goal
    assert r._map_accept(goal) == Response.ACCEPT
    handle = NS(request=goal, is_cancel_requested=False, abort=Mock())
    result = r._map_execute(handle)
    assert result.code == result.REJECTED and not result.localized
    assert reason in result.message
    assert r.maps.token is token and r._map_future is future and r._map_goal is owner
    r.node.cmd_vel_callback.assert_not_called()


@pytest.mark.parametrize('step', ['startup-readback', 'startup-amcl', 'inspect', 'readback', 'amcl-map', 'costmap-map'])
def test_readonly_pending_failed_transaction_does_not_block_fresh_pose(runtime, step):
    runtime.maps.stage = 'failed'; runtime.maps.hold = True
    future = NS(cancel=Mock(), done=lambda: False)
    runtime._map_future, runtime._map_step = future, step
    assert runtime._map_accept(request()) == Response.ACCEPT
    assert runtime._map_future is None and runtime.maps.stage == 'pause-navigation'
    future.cancel.assert_called_once()
    assert runtime.maps.hold


@pytest.mark.parametrize('step', ['pause', 'resume', 'load', 'parameters', 'activate', 'unknown-step'])
def test_unknown_map_write_never_discarded(runtime, step):
    runtime.maps.stage = 'failed'; runtime.maps.hold = True
    runtime._map_future = NS(cancel=Mock(), done=lambda: True)
    runtime._map_step = step
    terminal_refusal(runtime, 'map mutation pending')
    runtime._map_future.cancel.assert_not_called()


@pytest.mark.parametrize('latch', ['_map_mutation_unknown', '_lifecycle_mutation_unknown', '_unknown_write_owners'])
def test_unknown_latches_never_cleared(runtime, latch):
    value = [{'kind': 'map', 'future': None, 'observation': 'unknown'}] if latch == '_unknown_write_owners' else True
    setattr(runtime, latch, value)
    terminal_refusal(runtime, 'write outcome unknown')
    assert getattr(runtime, latch) is value


@pytest.mark.parametrize('problem', ['lidar', 'fault', 'moving', 'origin', 'undock', 'navigation', 'raw-odom'])
def test_real_dangers_have_specific_result_without_mutation(runtime, problem):
    if problem == 'lidar': runtime.node._lidar_ready = lambda: False
    if problem == 'fault': runtime.node._fault_flags = 7
    if problem == 'moving': runtime.node._vx = .1
    if problem == 'origin': runtime.guard.ready = False
    if problem == 'undock': runtime.undock_hold = True
    if problem == 'navigation': runtime._navigation_status = True
    if problem == 'raw-odom': runtime.node._pose_received = False
    terminal_refusal(runtime, {'lidar': 'LiDAR', 'moving': 'stationary', 'origin': 'origin',
                              'raw-odom': 'odometry'}.get(problem, problem))


@pytest.mark.parametrize('invalid', ['unauthorized', 'nonfinite', 'revision', 'duplicate'])
def test_invalid_request_never_withdraws_readonly_future(runtime, invalid):
    goal = request()
    future = NS(cancel=Mock())
    runtime._map_future, runtime._map_step = future, 'amcl-map'
    if invalid == 'unauthorized': goal.operator_authorized = False
    if invalid == 'nonfinite': goal.pose.pose.position.x = float('nan')
    if invalid == 'revision': goal.map_revision = 'wrong'
    if invalid == 'duplicate': runtime.maps._seen.add(goal.request_id)
    assert runtime._map_accept(goal) == Response.REJECT
    assert runtime._map_future is future
    future.cancel.assert_not_called()


def test_absent_idle_cache_accepts_only_hold(runtime):
    runtime._navigation_idle_at = None
    assert runtime._map_accept(request()) == Response.ACCEPT
    assert runtime._map_step == 'wait-navigation-idle' and runtime.maps.hold
