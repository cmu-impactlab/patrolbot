"""
Exercise the real coordinator with ROS types and deterministic service peers.

No DDS graph, sockets, velocity publisher, or physical robot is used here.
"""

import copy
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
from patrolbot_bridge.epoch_recovery import EpochRecovery
from patrolbot_bridge.epoch_runtime import EpochRuntime
from patrolbot_bridge.map_runtime import map_identity
from patrolbot_interfaces.action import LocalizeOnMap
import pytest
from rclpy.action import GoalResponse

ORIGIN = '123e4567-e89b-12d3-a456-426614174000'
REQUEST = 'bd98f746-f930-49e9-8b8f-17395cd84a65'


class Future:
    def __init__(self, value):
        self.value = value
        self.complete = True

    def done(self):
        return self.complete

    def result(self):
        return self.value

    def cancel(self):
        self.complete = True


class Client:
    def __init__(self, node, name):
        self.node, self.name, self.calls = node, name, []

    def service_is_ready(self):
        return True

    def remove_pending_request(self, future):
        pass

    def call_async(self, request):
        self.calls.append(request)
        if self.name.endswith('/get_state'):
            result = NS(current_state=NS(id=3))
        elif self.name.endswith('/set_parameters_atomically'):
            result = NS(result=NS(successful=True))
        elif self.name.endswith('/load_map'):
            entry = self.node.runtime.maps.target
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            grid.info.width = entry['width']
            grid.info.height = entry['height']
            grid.info.resolution = entry['resolution']
            grid.info.origin.position.x, grid.info.origin.position.y = entry['origin'][:2]
            grid.info.origin.orientation.w = 1.0
            grid.info.map_load_time = self.node.stamp()
            self.node.grid = grid
            result = NS(result=0, map=grid)
        elif self.name.endswith('/request_nomotion_update'):
            pose = copy.deepcopy(self.node.seeds[-1])
            pose.header.stamp = self.node.stamp()
            self.node.runtime.pose(pose)
            result = NS()
        elif self.name.endswith('/map'):
            result = NS(map=self.node.grid)
        elif self.name.endswith('/map_consumed'):
            result = NS(success=True, message=map_identity(self.node.grid))
        else:
            result = NS(success=True)
        return Future(result)


class Node:
    def __init__(self):
        self.now = 10_000_000_000
        self._pose_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._control_lock = threading.Lock()
        self._undock_goal_lock = threading.Lock()
        self.sock_lock = threading.Lock()
        self.sock = None
        self._stop_event = threading.Event()
        self._callbacks = Mock()
        self.context = NS(ok=lambda: True)
        self._odom_validator = Mock()
        self._odom_covariance = Mock()
        self._odom_tf_timeout = 0.5
        self._link_connected = True
        self._fault_flags = 0
        self._lidar_ready = Mock(return_value=True)
        self._undock_active = False
        self._undock_goal_reserved = False
        self._pending_controls = {}
        self._vx = self._vth = 0.0
        self.clients = {}
        self.seeds = []
        self.cmd_vel_callback = Mock()
        self.refresh()

    def refresh(self):
        self.now += 10_000_000
        self._pose_received = True
        self._pose_at = self._flags_at = self._charge_state_at = time.monotonic()

    def stamp(self):
        return Time(sec=self.now // 1_000_000_000, nanosec=self.now % 1_000_000_000)

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=self.now, to_msg=self.stamp))

    def get_logger(self):
        return Mock()

    def create_subscription(self, *args):
        return args

    def create_timer(self, *args):
        return args

    def create_client(self, _, name):
        self.clients[name] = Client(self, name)
        return self.clients[name]

    def create_publisher(self, *args):
        return NS(publish=self.publish)

    def publish(self, msg):
        self.seeds.append(copy.deepcopy(msg))
        self.runtime.seed_accepted(msg)
        self.now += 1
        pose = copy.deepcopy(msg)
        pose.header.stamp = self.stamp()
        self.runtime._odom_history.add(self.now, (0.0, 0.0, 0.0), self.runtime.guard.epoch)
        self.runtime.pose(pose)


@pytest.fixture
def runtime(tmp_path, monkeypatch):
    monkeypatch.setenv('PATROLBOT_STATE_DIR', str(tmp_path))
    navigation = Path(__file__).resolve().parents[2] / 'patrolbot_navigation'
    with patch(
        'patrolbot_bridge.map_runtime.get_package_share_directory', return_value=str(navigation)
    ), patch('patrolbot_bridge.map_runtime.ActionServer'):
        node = Node()
        runtime = EpochRuntime(node)
        node.runtime = runtime
    runtime.maps.active = runtime._startup_map
    session = runtime.begin_session()
    runtime.announce(session, ORIGIN)
    runtime.recovery.reset_complete(runtime.recovery.token, node.now)
    runtime._token = runtime.recovery.token
    runtime._navigation_status = False
    runtime._navigation_idle_at = time.monotonic()
    return runtime


def request(runtime, map_id='cmuq-floor1', request_id=REQUEST):
    goal = LocalizeOnMap.Goal()
    goal.request_id = request_id
    goal.map_id = map_id
    goal.map_revision = runtime.maps.catalog.maps[map_id]['revision']
    goal.operator_authorized = True
    goal.pose.pose.position.x = goal.pose.pose.position.y = 1.0
    goal.pose.pose.orientation.w = 1.0
    for index in (0, 7, 35):
        goal.pose.covariance[index] = 0.1
    return goal


def step(runtime):
    runtime.node.refresh()
    # Each deterministic step represents a fresh service-poll interval.
    runtime._tf_last_query = float('-inf')
    if runtime._navigation_probe is None:
        runtime._navigation_probe_at = time.monotonic() - 0.11
    runtime.first_tf_ns = runtime.node.now - 100
    runtime._odom_history.add(runtime.node.now, (0.0, 0.0, 0.0), runtime.guard.epoch)
    runtime.tick()


def run_until(runtime, predicate):
    for _ in range(100):
        if predicate():
            return
        step(runtime)
    raise AssertionError((runtime.maps.stage, runtime._map_step, runtime.maps.error))


def test_round_trip_orders_seed_after_consumption_and_persistence(runtime):
    for i, map_id in enumerate(('cmuq-floor1', 'cmuq-floor2')):
        goal = request(runtime, map_id, f'bd98f746-f930-49e9-8b8f-{17395 + i:012d}')
        runtime.node.refresh()
        assert runtime._map_accept(goal) == GoalResponse.ACCEPT
        assert not runtime.commands_allowed
        run_until(runtime, lambda: runtime._map_step == 'seed')
        assert runtime.maps.active['map_id'] == map_id
        assert runtime._map_store.load()['map_id'] == map_id
        assert len(runtime.node.seeds) == i
        run_until(runtime, lambda: runtime.maps.stage == 'ready')
        assert len(runtime.node.seeds) == i + 1
        assert runtime.commands_allowed
    assert [r.transition.id for r in runtime._change.calls] == [4, 2, 1, 3] * 2
    assert len(runtime._clear_global.calls) == len(runtime._clear_local.calls) == 2
    assert runtime.node.cmd_vel_callback.call_count == 2


@pytest.mark.parametrize(
    'phase',
    [
        'pause',
        'inspect',
        'deactivate',
        'cleanup',
        'load',
        'parameters',
        'configure',
        'activate',
        'readback',
        'amcl-map',
        'tf',
        'seed',
        'localization',
        'resume',
        'clear-global',
        'clear-local',
        'costmap-map',
    ],
)
def test_cancel_at_every_stage_retains_hold_and_never_reseeds(runtime, phase):
    goal = request(runtime)
    runtime.node.refresh()
    assert runtime._map_accept(goal) == GoalResponse.ACCEPT
    run_until(runtime, lambda: runtime._map_step == phase)
    seeds = len(runtime.node.seeds)
    active = runtime.maps.active
    runtime._map_cancel(NS(request=goal))
    for _ in range(5):
        step(runtime)
    assert runtime.maps.stage == 'failed' and not runtime.commands_allowed
    assert len(runtime.node.seeds) == seeds
    assert runtime.maps.active == active


@pytest.mark.parametrize(
    'failure', ['disconnect', 'origin', 'timeout', 'moving', 'stale', 'nonfinite']
)
def test_conflicting_state_fails_closed(runtime, failure):
    runtime.node.refresh()
    assert runtime._map_accept(request(runtime)) == GoalResponse.ACCEPT
    run_until(runtime, lambda: runtime._map_step == 'load')
    if failure == 'disconnect':
        runtime.guard.disconnect(runtime.guard.session)
    if failure == 'origin':
        runtime.recovery._start_reset('aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee')
    if failure == 'timeout':
        runtime._map_deadline = 0
    if failure == 'moving':
        runtime.node._vx = 0.1
    if failure == 'stale':
        runtime.node._flags_at = 0
    if failure == 'nonfinite':
        runtime.node._vth = float('nan')
    runtime.tick()
    assert runtime.maps.stage == 'failed' and not runtime.commands_allowed
    assert runtime.node.seeds == []


def test_late_map_ack_does_not_release_failed_transaction(runtime):
    runtime.node.refresh()
    runtime._map_accept(request(runtime))
    run_until(runtime, lambda: runtime._map_step == 'amcl-map')
    step(runtime)
    runtime.maps.fail('timeout')
    for _ in range(4):
        step(runtime)
    assert runtime.maps.active['map_id'] == 'cmuq-floor2'
    assert not runtime.commands_allowed and runtime.node.seeds == []


def test_lidar_blocks_map_admission_and_failure_keeps_hold(runtime):
    runtime.node._lidar_ready.return_value = False
    assert runtime._map_accept(request(runtime)) == GoalResponse.REJECT
    runtime.node._lidar_ready.return_value = True
    runtime.node.refresh()
    assert runtime._map_accept(request(runtime)) == GoalResponse.ACCEPT
    runtime.node._lidar_ready.return_value = False
    runtime.tick()
    assert runtime.maps.stage == 'failed' and runtime.maps.hold


def test_slow_costmap_refresh_requests_one_scan_update_without_reseeding(runtime):
    runtime.node.refresh()
    assert runtime._map_accept(request(runtime)) == GoalResponse.ACCEPT
    run_until(runtime, lambda: runtime._map_step == 'costmap-map')
    runtime.node.now += 2_000_000_000
    run_until(runtime, lambda: runtime._map_step == 'refresh-localization')
    assert not runtime.commands_allowed
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    assert len(runtime.node.seeds) == 1
    assert len(runtime._refresh_localization.calls) == 1


def test_map_reset_withdraws_tf_evidence_without_erasing_fresh_base_telemetry(runtime):
    runtime.node.refresh()
    assert runtime._map_accept(request(runtime)) == GoalResponse.ACCEPT
    run_until(runtime, lambda: runtime._map_step == 'readback')
    # AMCL reset is not an odometry-origin change. The next tick can precede
    # the next hardware packet without inventing stale stationary telemetry.
    runtime.tick()
    assert runtime.maps.stage != 'failed'
    assert runtime.node._pose_received


@pytest.mark.parametrize('variance', [0.25718849416411727, 0.0, float('nan')])
def test_final_refresh_rejects_invalid_covariance_without_waiting_for_transaction_timeout(
    runtime, variance
):
    runtime.node.refresh()
    assert runtime._map_accept(request(runtime)) == GoalResponse.ACCEPT
    run_until(runtime, lambda: runtime._map_step == 'costmap-map')
    runtime.node.now += 2_000_000_000
    run_until(runtime, lambda: runtime._map_step == 'refresh-localization')
    original = runtime._refresh_localization.call_async

    def invalid_update(req):
        result = original(req)
        runtime._pending_pose.pose.covariance[7] = variance
        return result

    runtime._refresh_localization.call_async = invalid_update
    run_until(runtime, lambda: runtime.maps.stage == 'failed')
    assert 'LOCALIZATION_QUALITY_REJECTED' in runtime.maps.error
    assert 'y variance' in runtime.maps.error
    assert runtime.maps.hold and not runtime.commands_allowed
    assert len(runtime.node.seeds) == 1
    assert len(runtime._refresh_localization.calls) == 1
    assert runtime.maps.active['map_id'] == 'cmuq-floor1'


def test_refresh_missing_pose_has_bounded_specific_failure(runtime):
    runtime.node.refresh()
    runtime._map_accept(request(runtime))
    run_until(runtime, lambda: runtime._map_step == 'costmap-map')
    runtime.node.now += 2_000_000_000
    run_until(runtime, lambda: runtime._map_step == 'refresh-localization')
    runtime._refresh_localization.call_async = lambda req: Future(NS())
    run_until(runtime, lambda: runtime._map_step == 'costmap-map')
    runtime._map_refresh_deadline = time.monotonic() - 1
    run_until(runtime, lambda: runtime.maps.stage == 'failed')
    assert 'LOCALIZATION_UPDATE_TIMEOUT' in runtime.maps.error
    assert 'newer' in runtime.maps.error
    assert runtime.maps.hold and len(runtime.node.seeds) == 1


def test_refresh_rejects_pre_request_pose_even_if_fresh(runtime):
    runtime.node.refresh()
    runtime._map_accept(request(runtime))
    run_until(runtime, lambda: runtime._map_step == 'costmap-map')
    runtime.node.now += 2_000_000_000
    run_until(runtime, lambda: runtime._map_step == 'refresh-localization')
    # Fresh evidence received before dispatch cannot acknowledge that refresh.
    runtime._pending_pose.header.stamp = runtime.node.stamp()
    runtime._refresh_localization.call_async = lambda req: Future(NS())
    run_until(runtime, lambda: runtime._map_step == 'costmap-map')
    for _ in range(6):
        step(runtime)
    assert runtime.maps.hold and runtime.maps.stage != 'ready'
    runtime._map_refresh_deadline = time.monotonic() - 1
    run_until(runtime, lambda: runtime.maps.stage == 'failed')


def test_late_valid_pose_cannot_release_quality_failure(runtime):
    runtime.node.refresh()
    runtime._map_accept(request(runtime))
    run_until(runtime, lambda: runtime._map_step == 'costmap-map')
    runtime.maps.fail('LOCALIZATION_QUALITY_REJECTED: y variance exceeds limit')
    runtime.node.publish(runtime.node.seeds[-1])
    for _ in range(5):
        step(runtime)
    assert runtime.maps.stage == 'failed' and not runtime.commands_allowed


@pytest.mark.parametrize(
    'failure', ['stale', 'wrong-seed', 'service-pending', 'service-unavailable']
)
def test_refresh_evidence_and_service_failures_remain_held(runtime, failure):
    runtime.node.refresh()
    runtime._map_accept(request(runtime))
    run_until(runtime, lambda: runtime._map_step == 'costmap-map')
    runtime.node.now += 2_000_000_000
    run_until(runtime, lambda: runtime._map_step == 'refresh-localization')
    if failure == 'service-unavailable':
        runtime._refresh_localization.service_is_ready = lambda: False
        for _ in range(4):
            step(runtime)
        assert runtime._map_refresh_started is None
    elif failure == 'service-pending':
        future = Future(NS())
        future.complete = False
        runtime._refresh_localization.call_async = lambda req: future
        for _ in range(4):
            step(runtime)
    else:
        run_until(runtime, lambda: runtime._map_step == 'costmap-map')
        if failure == 'stale':
            runtime.node.now += 2_000_000_000
        else:
            runtime._accepted_seed_stamp = -1
        for _ in range(4):
            step(runtime)
    assert runtime.maps.hold and runtime.maps.stage != 'ready'
    runtime._map_refresh_deadline = time.monotonic() - 1
    run_until(runtime, lambda: runtime.maps.stage == 'failed')
    assert runtime.maps.error.startswith('LOCALIZATION_UPDATE_TIMEOUT:')
    expected = {
        'stale': 'pose age',
        'wrong-seed': 'seed acceptance',
        'service-pending': 'service did not complete',
        'service-unavailable': 'service did not complete',
    }
    assert expected[failure] in runtime.maps.error
    assert len(runtime.node.seeds) == 1 and not runtime.commands_allowed


@pytest.mark.parametrize('dx,yaw', [(0.11, 0.0), (0.0, 0.0541052068)])
@pytest.mark.parametrize('phase', ['localization', 'costmap-map'])
def test_measured_seed_rejects_confident_displacement(runtime, dx, yaw, phase):
    import math

    runtime.node.refresh()
    goal = request(runtime)
    assert runtime._map_accept(goal) == GoalResponse.ACCEPT
    runtime._map_request = NS(pose=goal.pose, measured_pose=True)
    run_until(runtime, lambda: runtime._map_step == phase)
    pose = runtime._pending_pose
    pose.pose.pose.position.x += dx
    pose.pose.pose.orientation.z = math.sin(yaw / 2)
    pose.pose.pose.orientation.w = math.cos(yaw / 2)
    for i in (0, 7, 35):
        pose.pose.covariance[i] = 0.0009
    assert not runtime._map_localization_fresh()
    assert runtime.maps.stage == 'failed'
    assert 'SEED_INNOVATION_REJECTED' in runtime.maps.error
    assert not runtime.commands_allowed
    runtime.node.publish(runtime.node.seeds[-1])
    for _ in range(3):
        step(runtime)
    assert runtime.maps.stage == 'failed' and not runtime.commands_allowed


def test_measured_seed_missing_history_retains_hold(runtime):
    goal = request(runtime)
    runtime.node.refresh()
    runtime._map_accept(goal)
    runtime._map_request = NS(pose=goal.pose, measured_pose=True)
    run_until(runtime, lambda: runtime._map_step == 'localization')
    runtime._odom_history.samples.clear()
    assert not runtime._map_localization_fresh()
    assert 'history unavailable' in runtime.maps.error
    assert runtime.maps.hold


def test_startup_idle_probe_unavailable_has_specific_reason(runtime):
    runtime._navigation_idle.service_is_ready = lambda: False
    assert runtime.navigation_idle_problem() == 'wait-navigation-idle-service-unavailable'


def test_stationary_accumulated_drift_withdraws_readiness(runtime):
    goal = request(runtime)
    runtime.node.refresh()
    runtime._map_accept(goal)
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    for dx in (0.04, 0.08, 0.12):
        runtime.node.refresh()
        pose = copy.deepcopy(runtime.node.seeds[-1])
        pose.header.stamp = runtime.node.stamp()
        pose.pose.pose.position.x += dx
        runtime._odom_history.add(runtime.node.now, (0.0, 0.0, 0.0), runtime.guard.epoch)
        runtime.pose(pose)
    assert not runtime.commands_allowed
    assert runtime.recovery.stage == EpochRecovery.WAIT_SEED
    assert runtime._localization_fault.startswith('LOCALIZATION_DEGRADED:')


@pytest.mark.parametrize('delta,accepted', [(0.19, True), (0.21, False)])
def test_moving_updates_use_odometry_and_bounded_corrections(runtime, delta, accepted):
    runtime.node.refresh()
    runtime._map_accept(request(runtime))
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    runtime.node.refresh()
    runtime.node._vx = 0.1
    pose = copy.deepcopy(runtime.node.seeds[-1])
    pose.header.stamp = runtime.node.stamp()
    pose.pose.pose.position.x += 0.2 + delta
    runtime._odom_history.add(runtime.node.now, (0.2, 0.0, 0.0), runtime.guard.epoch)
    runtime.pose(pose)
    assert runtime.commands_allowed == accepted
    if accepted:
        assert runtime._pose_anchor[0][0] == pose.pose.pose.position.x
    else:
        assert runtime.recovery.stage == EpochRecovery.WAIT_SEED


def test_stationary_anchor_survives_bounded_history_eviction(runtime):
    runtime.node.refresh()
    runtime._map_accept(request(runtime))
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    runtime._odom_history.samples.clear()
    runtime.node.refresh()
    pose = copy.deepcopy(runtime.node.seeds[-1])
    pose.header.stamp = runtime.node.stamp()
    runtime._odom_history.add(runtime.node.now, (0.0, 0.0, 0.0), runtime.guard.epoch)
    runtime.pose(pose)
    assert runtime.commands_allowed


@pytest.mark.parametrize('bad', [float('nan'), 0.0, 0.26])
def test_invalid_tracking_covariance_requires_new_localization(runtime, bad):
    runtime.node.refresh()
    runtime._map_accept(request(runtime))
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    runtime.node.refresh()
    pose = copy.deepcopy(runtime.node.seeds[-1])
    pose.header.stamp = runtime.node.stamp()
    pose.pose.covariance[0] = bad
    runtime._odom_history.add(runtime.node.now, (0.0, 0.0, 0.0), runtime.guard.epoch)
    runtime.pose(pose)
    assert not runtime.commands_allowed
    assert runtime.recovery.stage == EpochRecovery.WAIT_SEED


@pytest.mark.parametrize('components', [(0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 2.0)])
def test_initial_amcl_invalid_quaternion_never_mutates_pending_pose(runtime, components):
    from geometry_msgs.msg import PoseWithCovarianceStamped

    runtime.node.refresh()
    runtime._map_accept(request(runtime))
    run_until(runtime, lambda: runtime._map_step == 'seed')
    assert runtime._pose_anchor is None
    previous = runtime._pending_pose
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = runtime.node.stamp()
    q = pose.pose.pose.orientation
    q.x, q.y, q.z, q.w = components
    for index in (0, 7, 35):
        pose.pose.covariance[index] = 0.001
    runtime.pose(pose)
    assert runtime._pending_pose is previous
    assert not runtime.commands_allowed


def test_tracking_negative_eigenvalue_covariance_withdraws_readiness(runtime):
    runtime.node.refresh()
    runtime._map_accept(request(runtime))
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    runtime.node.refresh()
    pose = copy.deepcopy(runtime.node.seeds[-1])
    pose.header.stamp = runtime.node.stamp()
    for index in (0, 7, 35):
        pose.pose.covariance[index] = 0.001
    pose.pose.covariance[1] = pose.pose.covariance[6] = 0.01
    runtime._odom_history.add(runtime.node.now, (0.0, 0.0, 0.0), runtime.guard.epoch)
    runtime.node.cmd_vel_callback.reset_mock()
    runtime.pose(pose)
    assert not runtime.commands_allowed
    assert runtime.recovery.stage == EpochRecovery.WAIT_SEED
    assert 'invalid localization covariance' in runtime._localization_fault
    runtime.node.cmd_vel_callback.assert_called_once()
