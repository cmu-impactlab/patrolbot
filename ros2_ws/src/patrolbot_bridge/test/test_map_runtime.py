"""
Exercise the real coordinator with ROS types and deterministic service peers.

No DDS graph, sockets, velocity publisher, or physical robot is used here.
"""

import ast
import copy
import math
from pathlib import Path
import threading
import time
from types import MethodType, SimpleNamespace as NS
from unittest.mock import Mock, patch

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from patrolbot_bridge.epoch_recovery import EpochRecovery
from patrolbot_bridge.epoch_runtime import EpochRuntime
from patrolbot_bridge.manual_active import manual_drive_bypass
from patrolbot_bridge.manual_telemetry import ManualTelemetry
from patrolbot_bridge.map_runtime import map_identity
from patrolbot_bridge.odom_covariance import OdomCovariance
from patrolbot_bridge.odom_validation import OdomSample, OdomValidator
from patrolbot_bridge.scan_timing import compute_scan_stamp_ns, parse_laser_age
from patrolbot_bridge.tuning_policy import clamp_drive
from patrolbot_interfaces.action import LocalizeOnMap
import pytest
from rclpy.action import GoalResponse
from rclpy.time import Time as RosTime
from sensor_msgs.msg import LaserScan

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
    node.refresh()
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
        assert finish_action(runtime, goal).localized
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
    refused(runtime, request(runtime), 'fresh ready LiDAR')
    runtime.node._lidar_ready.return_value = True
    runtime.node.refresh()
    assert runtime._map_accept(request(runtime, request_id=RETRY)) == GoalResponse.ACCEPT
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


RETRY = '986e4515-5de8-4332-bb9a-7de5773ee93d'


def new_request(runtime):
    serial = getattr(runtime, '_test_request_serial', 0) + 1
    runtime._test_request_serial = serial
    return request(runtime, request_id=f'00000000-0000-4000-8000-{serial:012x}')


def finish_action(runtime, req):
    goal = Mock(request=req, is_cancel_requested=False)
    return runtime._map_execute(goal)


def fail_sensor(runtime, req=None):
    req = req or request(runtime)
    runtime.node.refresh()
    assert runtime._map_accept(req) == GoalResponse.ACCEPT
    runtime.node._lidar_ready.return_value = False
    runtime.tick()
    assert runtime.maps.stage == 'failed'
    assert 'fresh ready LiDAR lost' in runtime.maps.error
    result = finish_action(runtime, req)
    assert not result.localized and result.code == result.FAILED
    runtime.node._lidar_ready.return_value = True
    runtime.node.refresh()
    runtime._navigation_idle_at = time.monotonic()
    return req


def refused(runtime, req, reason):
    before = (runtime.maps.token, runtime.maps.stage, runtime.maps.hold,
              len(runtime.node.seeds), runtime.node.cmd_vel_callback.call_count)
    calls = {name: len(client.calls) for name, client in runtime.node.clients.items()}
    assert runtime._map_accept(req) == GoalResponse.ACCEPT  # diagnostic result only
    result = finish_action(runtime, req)
    assert result.code == result.REJECTED and not result.localized
    assert reason in result.message
    assert before == (runtime.maps.token, runtime.maps.stage, runtime.maps.hold,
                      len(runtime.node.seeds), runtime.node.cmd_vel_callback.call_count)
    assert calls == {name: len(client.calls) for name, client in runtime.node.clients.items()}
    return result


def test_stationary_retry_sensor_loss_then_explicit_retry(runtime):
    fail_sensor(runtime)
    old = runtime.maps.token
    req = request(runtime, request_id=RETRY)
    assert runtime._map_accept(req) == GoalResponse.ACCEPT
    assert runtime.maps.token != old and runtime.maps.hold
    assert not runtime.commands_allowed and not runtime._navigation_ready
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    result = finish_action(runtime, req)
    assert result.code == result.OK and result.localized
    assert runtime.commands_allowed and len(runtime.node.seeds) == 1


def test_stationary_retry_no_prior_localization(runtime):
    runtime.maps.active = None
    runtime.recovery.stage = EpochRecovery.WAIT_POSE
    runtime._accepted_seed_stamp = None
    runtime._pending_pose = None
    runtime._navigation_ready = False
    runtime.node._motors_enabled = True
    req = request(runtime)
    assert runtime._map_accept(req) == GoalResponse.ACCEPT
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    assert finish_action(runtime, req).localized


def test_stationary_retry_unsafe_refusals(runtime):
    fail_sensor(runtime)
    for name, value, reason in (
        ('_vx', 0.1, 'stationary'),
        ('_vth', float('nan'), 'telemetry'),
        ('_flags_at', 0.0, 'telemetry'),
        ('_flags_at', time.monotonic() + 100.0, 'telemetry'),
        ('_pose_at', float('nan'), 'telemetry'),
        ('_pose_received', False, 'odometry'),
        ('_link_connected', False, 'transport'),
        ('_fault_flags', 1, 'hardware'),
        ('_undock_goal_reserved', True, 'owns'),
        ('_pending_controls', {'pending': object()}, 'owns'),
    ):
        previous = getattr(runtime.node, name)
        setattr(runtime.node, name, value)
        refused(runtime, new_request(runtime), reason)
        setattr(runtime.node, name, previous)
    runtime.guard.ready = False
    refused(runtime, new_request(runtime), 'origin/transport')


def test_stationary_retry_pending_mutation_and_retired_completion(runtime):
    req = request(runtime)
    assert runtime._map_accept(req) == GoalResponse.ACCEPT
    # The PAUSE request has been dispatched but not acknowledged.
    pending = Future(NS(success=True))
    pending.complete = False
    runtime._navigation.call_async = lambda _: pending
    step(runtime)
    assert runtime._map_future is pending
    runtime.node._lidar_ready.return_value = False
    runtime.tick()
    finish_action(runtime, req)
    runtime.node._lidar_ready.return_value = True
    runtime.node.refresh()
    refused(runtime, request(runtime, request_id=RETRY), 'pending')
    assert runtime._map_future is pending and not pending.complete
    pending.complete = True
    retry = new_request(runtime)
    assert runtime._map_accept(retry) == GoalResponse.ACCEPT
    token, stage = runtime.maps.token, runtime.maps.stage
    # Neither the retired result nor its stage check may advance this request.
    assert runtime._map_step == 'pause' and runtime.maps.stage == 'pause-navigation'
    assert not runtime.maps.advance(runtime.maps.token.__class__(
        req.request_id, token.generation - 1, token.origin, token.session), stage)
    assert runtime.maps.token == token and runtime.maps.stage == stage
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    assert finish_action(runtime, retry).localized


def test_stationary_retry_unknown_mutation(runtime):
    req = request(runtime)
    assert runtime._map_accept(req) == GoalResponse.ACCEPT

    class Unknown(Future):
        def result(self):
            raise RuntimeError('transport lost response')

    runtime._navigation.call_async = lambda _: Unknown(None)
    step(runtime)
    step(runtime)
    assert runtime.maps.stage == 'failed'
    finish_action(runtime, req)
    refused(runtime, request(runtime, request_id=RETRY), 'unknown')
    assert runtime.maps.hold and not runtime.commands_allowed


def test_stationary_retry_retired_sensor_callbacks(runtime):
    req = request(runtime)
    assert runtime._map_accept(req) == GoalResponse.ACCEPT
    run_until(runtime, lambda: runtime._map_step == 'costmap-map')
    old_seed = copy.deepcopy(runtime.node.seeds[-1])
    old_pose = copy.deepcopy(runtime._pending_pose)
    runtime.node._lidar_ready.return_value = False
    runtime.tick()
    finish_action(runtime, req)
    runtime.node._lidar_ready.return_value = True
    runtime.node.refresh()
    retry = request(runtime, request_id=RETRY)
    assert runtime._map_accept(retry) == GoalResponse.ACCEPT
    runtime.seed(old_seed)
    runtime.seed_accepted(old_seed)
    runtime.pose(old_pose)
    assert runtime._pending_pose is None and runtime._accepted_seed_stamp is None
    assert not runtime.commands_allowed
    run_until(runtime, lambda: runtime._map_step == 'localization')
    accepted = runtime._accepted_seed_stamp
    runtime.seed_accepted(old_seed)
    runtime.pose(old_pose)
    assert runtime._accepted_seed_stamp == accepted
    assert runtime.stamp_ns(runtime._pending_pose) > runtime.maps.seed_stamp
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    assert finish_action(runtime, retry).localized


def test_stationary_retry_replay_identity_and_busy(runtime):
    first = fail_sensor(runtime)
    refused(runtime, first, 'consumed')
    wrong = new_request(runtime)
    wrong.map_revision = 'wrong-revision'
    refused(runtime, wrong, 'revision')
    wrong = new_request(runtime)
    wrong.operator_authorized = False
    refused(runtime, wrong, 'authorization')
    retry = request(runtime, request_id=RETRY)
    assert runtime._map_accept(retry) == GoalResponse.ACCEPT
    # Reserve during goal-callback/execute gap, not merely once execute starts.
    assert runtime._map_accept(request(runtime)) == GoalResponse.REJECT
    token = runtime.maps.token
    retired = finish_action(runtime, first)
    assert retired.code == retired.REJECTED
    assert runtime._map_action_pending and runtime.maps.token == token
    runtime._future = Future(NS(success=True))
    runtime.maps.fail('definite failure')
    finish_action(runtime, retry)
    refused(runtime, request(runtime, request_id='b757dbfb-9adb-4661-97ab-e9a603190f3d'),
            'lifecycle')


def test_stationary_retry_no_implicit_resume(runtime):
    fail_sensor(runtime)
    calls = {name: len(client.calls) for name, client in runtime.node.clients.items()
             if not name.endswith(('navigation_idle', 'initial_pose_tf_ready'))}
    for _ in range(5):
        step(runtime)
    session = runtime.begin_session()
    runtime.announce(session, ORIGIN)
    runtime.node.refresh()
    for _ in range(5):
        step(runtime)
    assert runtime.maps.stage == 'failed' and runtime.maps.hold
    assert not runtime.node.seeds and not runtime._navigation_ready
    assert calls == {name: len(client.calls) for name, client in runtime.node.clients.items()
                     if not name.endswith(('navigation_idle', 'initial_pose_tf_ready'))}
    retry = request(runtime, request_id=RETRY)
    assert runtime._map_accept(retry) == GoalResponse.ACCEPT
    assert runtime.maps.token.session == session
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    assert finish_action(runtime, retry).localized


def test_stationary_retry_invalid_pose_and_identity(runtime):
    fail_sensor(runtime)
    for part in ('nan', 'roll', 'z', 'quaternion', 'covariance', 'uuid', 'map'):
        req = new_request(runtime)
        if part == 'nan':
            req.pose.pose.position.x = float('nan')
        elif part == 'roll':
            import math

            req.pose.pose.orientation.x = math.sin(0.1)
            req.pose.pose.orientation.w = math.cos(0.1)
        elif part == 'z':
            req.pose.pose.position.z = 0.1
        elif part == 'quaternion':
            req.pose.pose.orientation.w = 0.0
        elif part == 'covariance':
            req.pose.covariance[0] = -0.1
        elif part == 'uuid':
            req.request_id = 'not-a-uuid'
        elif part == 'map':
            req.map_id = 'unknown-map'
        refused(runtime, req, '')
    runtime.guard.epoch = 'aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee'
    refused(runtime, new_request(runtime), 'origin/transport')


def test_stationary_retry_late_tf_probe(runtime):
    fail_sensor(runtime)
    # A read-only TF probe from the failed generation is still in flight.
    runtime.first_tf_ns = runtime.node.now
    runtime._tf_last_query = float('-inf')
    runtime._poll_tf_readiness()
    old_probe = runtime._tf_probe
    old_probe.complete = False
    retry = request(runtime, request_id=RETRY)
    assert runtime._map_accept(retry) == GoalResponse.ACCEPT
    runtime.first_tf_ns = runtime.node.now
    old_probe.complete = True
    runtime._poll_tf_readiness()
    assert not runtime.seed_tf_ready
    assert runtime.maps.hold and not runtime.commands_allowed
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    assert finish_action(runtime, retry).localized


def test_stationary_retry_diagnostic_cancel_is_noop(runtime):
    first = fail_sensor(runtime)
    assert runtime._map_accept(first) == GoalResponse.ACCEPT  # replay refusal
    before = runtime.maps.error
    runtime._map_cancel(NS(request=first))
    assert runtime.maps.error == before
    assert runtime._map_accept(request(runtime, request_id=RETRY)) == GoalResponse.REJECT
    result = finish_action(runtime, first)
    assert result.code == result.REJECTED and 'consumed' in result.message


@pytest.mark.parametrize('phase', ['inspect', 'load', 'activate', 'resume',
                                  'refresh-localization', 'costmap-map'])
def test_stationary_retry_after_settled_stages(runtime, phase):
    first = request(runtime)
    assert runtime._map_accept(first) == GoalResponse.ACCEPT
    if phase == 'refresh-localization':
        run_until(runtime, lambda: runtime._map_step == 'costmap-map')
        runtime.node.now += 2_000_000_000
    run_until(runtime, lambda: runtime._map_step == phase)
    # Dispatch a response but fail before consuming it. Some peers mutate their
    # simulated server state on dispatch (LoadMap), so retry must inspect/reset,
    # not apply this old response or claim the previous map is verified.
    step(runtime)
    assert runtime._map_future is not None
    old_future = runtime._map_future
    runtime.node._lidar_ready.return_value = False
    runtime.tick()
    finish_action(runtime, first)
    runtime.node._lidar_ready.return_value = True
    runtime.node.refresh()
    retry = request(runtime, request_id=RETRY)
    assert runtime._map_accept(retry) == GoalResponse.ACCEPT
    assert runtime._map_future is None and runtime._map_step == 'pause'
    assert runtime.maps.seed_stamp is None and not runtime._navigation_ready
    old_future.complete = True
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    assert finish_action(runtime, retry).localized


@pytest.mark.parametrize('phase', ['pause', 'load', 'activate', 'resume',
                                  'refresh-localization'])
@pytest.mark.parametrize('unknown', [False, True])
def test_stationary_retry_unresolved_mutations(runtime, phase, unknown):
    first = request(runtime)
    assert runtime._map_accept(first) == GoalResponse.ACCEPT
    if phase == 'refresh-localization':
        run_until(runtime, lambda: runtime._map_step == 'costmap-map')
        runtime.node.now += 2_000_000_000
    run_until(runtime, lambda: runtime._map_step == phase)
    step(runtime)
    pending = runtime._map_future
    pending.complete = False
    runtime.maps.fail('map localization timed out')
    finish_action(runtime, first)
    if unknown:
        pending.complete = True

        def missing_result():
            raise RuntimeError('lost response, execution unknown')

        pending.result = missing_result
    refused(runtime, request(runtime, request_id=RETRY), 'unknown' if unknown else 'pending')
    assert runtime.maps.hold and not runtime.commands_allowed
    assert runtime._map_future is pending


@pytest.mark.parametrize('dispatch', [False, True])
def test_stationary_retry_epoch_unknown_is_not_unlocked(runtime, dispatch):
    fail_sensor(runtime)
    # Exercise the actual epoch lifecycle owner as well as map admission.
    runtime._future_kind = 'mutation'
    if dispatch:
        def lost_dispatch(_):
            raise RuntimeError('dispatch completion unknown')

        runtime._change.call_async = lost_dispatch
        runtime._request('deactivate')
    else:
        class Unknown(Future):
            def result(self):
                raise RuntimeError('response lost')

        runtime._future = Unknown(None)
        # To exercise epoch response handling, temporarily remove map ownership.
        runtime.maps.stage = 'idle'
        runtime.tick()
        runtime.maps.stage = 'failed'
    assert runtime._lifecycle_outcome_unknown
    refused(runtime, request(runtime, request_id=RETRY), 'unknown')


def test_stationary_retry_refused_replay_never_becomes_mutation(runtime):
    req = request(runtime)
    runtime.node._lidar_ready.return_value = False
    refused(runtime, req, 'LiDAR')
    runtime.node._lidar_ready.return_value = True
    refused(runtime, req, 'consumed')
    retry = request(runtime, request_id=RETRY)
    assert runtime._map_accept(retry) == GoalResponse.ACCEPT
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    assert finish_action(runtime, retry).localized


def test_stationary_retry_estimate_is_not_verification(runtime):
    req = request(runtime)
    runtime._map_seed_pub.publish = lambda msg: runtime.node.seeds.append(copy.deepcopy(msg))
    assert runtime._map_accept(req) == GoalResponse.ACCEPT
    run_until(runtime, lambda: runtime._map_step == 'localization')
    assert runtime.maps.seed_stamp is not None
    for _ in range(5):
        step(runtime)
    assert runtime.maps.stage == 'wait-localization'
    assert not runtime.commands_allowed and not runtime._navigation_ready
    assert runtime.recovery.stage == EpochRecovery.WAIT_POSE
    runtime.seed_accepted(runtime.node.seeds[-1])
    step(runtime)
    assert not runtime.commands_allowed and runtime._pending_pose is None
    fresh = copy.deepcopy(runtime.node.seeds[-1])
    fresh.header.stamp = runtime.node.stamp()
    runtime.pose(fresh)
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    assert finish_action(runtime, req).localized


def test_stationary_retry_epoch_unknown_withdraws_ready_commands(runtime):
    req = request(runtime)
    assert runtime._map_accept(req) == GoalResponse.ACCEPT
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    assert finish_action(runtime, req).localized
    assert runtime.commands_allowed

    class Unknown(Future):
        def result(self):
            raise RuntimeError('lifecycle result unknown')

    runtime._future_kind = 'mutation'
    runtime._future = Unknown(None)
    runtime.tick()
    assert runtime.maps.stage == 'ready' and not runtime.maps.hold
    assert runtime._lifecycle_outcome_unknown and not runtime.commands_allowed
    refused(runtime, request(runtime, request_id=RETRY), 'unknown')


def test_stationary_retry_tuning_gate_cannot_cross_new_map(runtime):
    fail_sensor(runtime)
    runtime._tuning_nav_pending = True
    runtime._tuning_nav_reserved = True
    runtime._tuning_map_phase = 'release'
    runtime._tuning_map_probe = Future(NS(success=True))
    runtime._tuning_map_probe.complete = False
    refused(runtime, new_request(runtime), 'mutation unresolved')
    # Settled/retired read-only map evidence cannot release the new map gate.
    runtime._tuning_map_phase = 'global-map'
    old_probe = Future(NS(success=True, message='retired-map'))
    runtime._tuning_map_probe = old_probe
    retry = new_request(runtime)
    assert runtime._map_accept(retry) == GoalResponse.ACCEPT
    assert runtime._tuning_map_probe is None
    assert runtime._tuning_map_identity is None
    old_probe.value = NS(success=True, message='late-retired-map')
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    assert not runtime._tuning_nav_release.calls
    # Completion remains responsible for associating the new verified identity.
    runtime.node._tuning_map_identity_changed = Mock()
    assert finish_action(runtime, retry).localized
    assert runtime._tuning_map_identity == runtime._map_identity


@pytest.mark.parametrize('retired', [False, True])
def test_stationary_retry_negative_lifecycle_is_not_settlement(runtime, retired):
    req = request(runtime)
    assert runtime._map_accept(req) == GoalResponse.ACCEPT
    runtime._navigation.call_async = lambda _: Future(NS(success=False))
    step(runtime)
    if retired:
        runtime.maps.fail('sensor lost with lifecycle response outstanding')
    else:
        step(runtime)
        assert runtime.maps.stage == 'failed'
    finish_action(runtime, req)
    refused(runtime, request(runtime, request_id=RETRY), 'unknown')
    assert runtime._map_outcome_unknown
    assert runtime.maps.hold and not runtime.commands_allowed


def test_stationary_retry_definite_load_refusal_can_reconcile(runtime):
    req = request(runtime)
    assert runtime._map_accept(req) == GoalResponse.ACCEPT
    run_until(runtime, lambda: runtime._map_step == 'load')
    original = runtime._load_map.call_async
    runtime._load_map.call_async = lambda _: Future(NS(result=1))
    step(runtime)
    step(runtime)
    assert runtime.maps.stage == 'failed' and not runtime._map_outcome_unknown
    finish_action(runtime, req)
    runtime._load_map.call_async = original
    retry = request(runtime, request_id=RETRY)
    assert runtime._map_accept(retry) == GoalResponse.ACCEPT
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    assert finish_action(runtime, retry).localized


def test_stationary_retry_retired_epoch_negative_stays_unknown(runtime):
    fail_sensor(runtime)
    runtime._phase = 'pause-navigation'
    runtime._future_kind = 'mutation'
    runtime._future = Future(NS(success=False))
    runtime.recovery._start_reset(ORIGIN)
    assert runtime.recovery.token != runtime._token
    runtime.maps.stage = 'idle'  # let the real epoch owner drain its response
    runtime.tick()
    runtime.maps.stage = 'failed'
    assert runtime._lifecycle_outcome_unknown
    refused(runtime, request(runtime, request_id=RETRY), 'unknown')


def source_class(filename, name, methods):
    """Load hard-boundary methods without constructing a ROS node or graph."""
    path = Path(__file__).resolve().parents[1] / 'patrolbot_bridge' / (filename + '.py')
    tree = ast.parse(path.read_text())
    klass = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
    klass.bases = []
    klass.body = [n for n in klass.body if isinstance(n, ast.FunctionDef) and n.name in methods]
    assert {n.name for n in klass.body} == set(methods)
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef)]
    namespace = dict(globals(), Time=RosTime, rclpy=NS(time=NS(Time=RosTime)))
    exec(compile(ast.Module(body=functions + [klass], type_ignores=[]), str(path), 'exec'),
         namespace)
    return namespace[name]


def attach_boundaries(runtime):
    n = runtime.node
    n._epoch = runtime
    n._session_generation = runtime.guard.session
    n._hardware_operation_lock = threading.Lock()
    n._motor_disable_generation = 0
    n._server_hello = NS(features={'MANUAL_UNDOCK_V1'})
    n._dock_state, n._charge_state, n._estop_pressed = 'DOCKED_CONFIRMED', 3, False
    n._dock_state_at = time.monotonic()
    n._undock_profile_commissioned = True
    n._manual_active_at = n._web_teleop_active_at = time.monotonic()
    n.sock = Mock()
    manual = source_class('manual_undock', 'ManualUndockRuntime',
                          {'observe', 'sensors_ready', 'admission_problem', 'start'})()
    manual.node, manual.raw = n, ManualTelemetry()
    manual.busy = manual.web_session_active = False
    n._manual_undock = manual
    turn = source_class('undock_turn', 'UndockTurnRuntime', {'reserve_reverse'})()
    turn.node, turn.epoch, turn.busy, turn.phase = n, runtime, False, 'idle'
    turn.actions, turn.idle_at = {}, time.monotonic()
    turn.revoke = Mock()
    turn.hardware_problem = Mock(return_value='')
    n._undock_turn = turn
    names = {'_parse_telemetry', '_parse_telemetry_guarded', '_laser_scan_message',
             '_manual_drive_allowed', 'cmd_vel_callback'}
    bridge = source_class('bridge_node', 'PatrolBotBridge', names)
    for name in names:
        method = vars(bridge)[name]
        setattr(n, name, method.__func__ if isinstance(method, staticmethod)
                else MethodType(method, n))
    n.cmd_vel_callback = Mock(wraps=n.cmd_vel_callback)
    n._odom_validator = OdomValidator(max_linear_speed=.3, max_angular_speed=1.2,
                                     expected_period=.05)
    n._odom_covariance = OdomCovariance(
        sigma_xy_per_m=.1, sigma_yaw_per_rad=.1, sigma_yaw_per_m=.1,
        sigma_vx=.1, sigma_vyaw=.1, min_variance=.0001, max_variance=1.)
    n._odom_tf_stale = False
    n._record_odom_rejection = Mock()
    n.SCAN_RANGE_MIN = .05
    n.odom_pub, n.scan_pub, n.unfiltered_scan_pub = Mock(), Mock(), None
    n._publish_with_timing = Mock()

    def broadcast(*_):
        runtime.first_tf_ns = n.now
    n._broadcast_odom_tf = Mock(side_effect=broadcast)
    return n


def wire(runtime, odom='0,0,0,0,0', *, session=None, epoch=None, received=None):
    runtime.node._parse_telemetry(
        f'ODOM:{odom}|LASER:1,1,|LASER_AGE_MS:0|ODOM_EPOCH:{epoch or runtime.guard.epoch}',
        time.monotonic() if received is None else received,
        runtime.guard.session if session is None else session)


def assert_motion_fenced(runtime):
    n = runtime.node
    # Refresh real evidence: a stale deadman/raw stream must not mask the fence.
    wire(runtime)
    n._manual_active_at = time.monotonic()
    n._web_teleop_active_at = 0.
    assert n._manual_undock.sensors_ready()
    n.sock.sendall.reset_mock()
    drive = Twist()
    drive.linear.x = .1
    n.cmd_vel_callback(drive)
    n.sock.sendall.assert_not_called()
    n.cmd_vel_callback(Twist())
    n.sock.sendall.assert_called_once_with(b'DRIVE:0.0:0.0\n')
    n._manual_active_at = 0.
    assert 'mutation' in n._manual_undock.admission_problem()
    n._manual_undock.start()
    assert not n._manual_undock.busy and not n._undock_goal_reserved
    assert not n._hardware_operation_lock.locked()
    assert not n._undock_turn.reserve_reverse()
    assert not n._hardware_operation_lock.locked()


@pytest.mark.parametrize('dispatch', [False, True])
def test_stationary_retry_unknown_map_blocks_manual_boundaries(runtime, dispatch):
    n = attach_boundaries(runtime)
    req = request(runtime)
    assert runtime._map_accept(req) == GoalResponse.ACCEPT

    def lost(*_):
        raise RuntimeError('map lifecycle response unknown')
    if dispatch:
        runtime._navigation.call_async = lost
    else:
        pending = Future(None)
        pending.result = lost
        runtime._navigation.call_async = lambda _: pending
    run_until(runtime, lambda: runtime.maps.stage == 'failed')
    finish_action(runtime, req)
    assert runtime._map_outcome_unknown and not runtime._failed
    assert runtime._map_goal is None and runtime._map_future is None
    refused(runtime, new_request(runtime), 'unknown')
    assert_motion_fenced(runtime)
    # Control: a definitely settled failed map retains the existing manual escape.
    runtime._map_outcome_unknown = False
    n._manual_active_at = time.monotonic()
    assert n._manual_drive_allowed()
    n._manual_active_at = 0.
    assert n._manual_undock.admission_problem() == ''
    assert not n._undock_turn.reserve_reverse()  # guarded reverse still honors maps.hold


@pytest.mark.parametrize('phase', ['reserve', 'release'])
@pytest.mark.parametrize('failure', ['dispatch', 'result'])
def test_stationary_retry_tuning_unknown_is_latched(runtime, phase, failure):
    fail_sensor(runtime)
    attach_boundaries(runtime)
    runtime._tuning_nav_pending = True
    runtime._tuning_nav_reserved = phase == 'release'
    runtime._tuning_map_phase = phase
    runtime.recovery.stage = EpochRecovery.READY
    runtime._navigation_ready = True
    calls = []

    def lost(*_):
        raise RuntimeError('tuning mutation outcome unknown')

    def dispatch(request):
        calls.append(request.data)
        if failure == 'dispatch':
            return lost()
        pending = Future(None)
        pending.result = lost
        return pending
    runtime._tuning_nav_release.call_async = dispatch
    runtime.tick_tuning_map_gate()
    runtime.tick_tuning_map_gate()
    assert runtime._tuning_gate_outcome_unknown and runtime._tuning_map_probe is None
    refused(runtime, new_request(runtime), 'mutation unresolved')
    assert_motion_fenced(runtime)
    # A transport/origin change cannot settle a remotely executing mutation.
    session = runtime.begin_session()
    runtime.announce(session, 'aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee')
    runtime.maps.stage = 'idle'  # remove the failed-map fence to test epoch dispatch too
    mutation_calls = {name: len(c.calls) for name, c in runtime.node.clients.items()}
    for _ in range(5):
        runtime.tick()
    assert calls == [phase == 'reserve']
    assert mutation_calls == {name: len(c.calls) for name, c in runtime.node.clients.items()}
    refused(runtime, new_request(runtime), 'mutation unresolved')
    assert runtime._tuning_gate_outcome_unknown and not runtime.commands_allowed


@pytest.mark.parametrize('failure', ['dispatch', 'result', 'negative'])
def test_stationary_retry_read_only_tuning_probe_can_retry(runtime, failure):
    fail_sensor(runtime)
    runtime._tuning_nav_pending = runtime._tuning_nav_reserved = True
    runtime._tuning_map_phase = 'global-map'
    runtime.recovery.stage = EpochRecovery.READY
    runtime._navigation_ready = True
    original = runtime._global_map.call_async

    def lost(*_):
        raise RuntimeError('read-only probe failed')
    if failure == 'dispatch':
        runtime._global_map.call_async = lost
    else:
        pending = Future(NS(success=False))
        if failure == 'result':
            pending.result = lost
        runtime._tuning_map_probe = pending
    runtime.tick_tuning_map_gate()
    assert not runtime.tuning_gate_unresolved
    runtime._global_map.call_async = original
    retry = new_request(runtime)
    assert runtime._map_accept(retry) == GoalResponse.ACCEPT
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    runtime.node._tuning_map_identity_changed = Mock()
    assert finish_action(runtime, retry).localized


def test_stationary_retry_new_origin_through_real_parser(runtime):
    fail_sensor(runtime)
    n = attach_boundaries(runtime)
    wire(runtime, odom='5,0,0,0,0')
    old_session, old_origin, old_token = runtime.guard.session, runtime.guard.epoch, runtime.maps.token
    old_stamp = n.now
    session = runtime.begin_session()
    n._session_generation = session
    runtime.announce(session, 'aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee')
    assert runtime.recovery.stage == EpochRecovery.WAIT_RESET
    assert not runtime.streams_allowed and not n._pose_received
    n._publish_with_timing.reset_mock()
    n._broadcast_odom_tf.reset_mock()
    # Old-session frames, even with the newly announced origin, cannot refresh raw evidence.
    wire(runtime, session=old_session)
    assert not n._manual_undock.sensors_ready()
    refused(runtime, new_request(runtime), 'current-origin odometry')
    # A fresh request is consumed at refusal, then a fresh explicit ID can proceed.
    premature = new_request(runtime)
    refused(runtime, premature, 'current-origin odometry')
    wire(runtime)
    assert n._manual_undock.sensors_ready()
    assert not n._pose_received and runtime.first_tf_ns is None
    assert runtime.recovery._reset_ns is None and not runtime.commands_allowed
    n._publish_with_timing.assert_not_called()
    n._broadcast_odom_tf.assert_not_called()
    refused(runtime, premature, 'consumed')
    req = new_request(runtime)
    assert runtime._map_accept(req) == GoalResponse.ACCEPT
    assert runtime.maps.token.origin == runtime.recovery.token != old_token.origin
    assert runtime.maps.token.session == session
    assert runtime.recovery.stage == EpochRecovery.WAIT_RESET

    # Use wire receipt, never Node.refresh()/step()'s synthetic pose or TF receipt.
    for _ in range(100):
        n.now += 10_000_000
        n._flags_at = n._charge_state_at = time.monotonic()
        wire(runtime)
        if runtime.recovery.stage == EpochRecovery.WAIT_RESET:
            assert not n._pose_received and runtime.first_tf_ns is None
            assert not runtime.commands_allowed
        runtime._tf_last_query = float('-inf')
        runtime.tick()
        if runtime.maps.stage == 'ready':
            break
    assert runtime.maps.stage == 'ready', runtime.maps.error
    assert [r.transition.id for r in runtime._change.calls] == [4, 2, 1, 3]
    assert finish_action(runtime, req).localized and runtime.commands_allowed
    assert len(n.seeds) == 1 and runtime.maps.seed_stamp > runtime.recovery._reset_ns > old_stamp
    assert runtime._odom_history.origin == runtime.guard.epoch != old_origin
    with pytest.raises(ValueError, match='origin mismatch'):
        runtime._odom_history.at(old_stamp, old_origin)
    assert n._publish_with_timing.call_count > 0
    n._record_odom_rejection.assert_not_called()


@pytest.mark.parametrize('bad', ['stale', 'future', 'nonfinite', 'moving', 'discontinuity',
                                 'wrong-session', 'wrong-origin', 'malformed-origin'])
def test_stationary_retry_wait_reset_rejects_untrusted_raw(runtime, bad):
    fail_sensor(runtime)
    n = attach_boundaries(runtime)
    session = runtime.begin_session()
    n._session_generation = session
    runtime.announce(session, 'aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee')
    kwargs = {}
    odom = '0,0,0,0,0'
    if bad in ('stale', 'future'):
        kwargs['received'] = time.monotonic() + (-1 if bad == 'stale' else 1)
    elif bad == 'nonfinite':
        odom = 'nan,0,0,0,0'
    elif bad == 'moving':
        odom = '0,0,0,.01,0'
    elif bad == 'discontinuity':
        wire(runtime)
        odom = '10,0,0,0,0'
    elif bad == 'wrong-session':
        kwargs['session'] = session - 1
    elif bad in ('wrong-origin', 'malformed-origin'):
        kwargs['epoch'] = ORIGIN if bad == 'wrong-origin' else 'unknown'
    wire(runtime, odom, **kwargs)
    refused(runtime, new_request(runtime), 'stationary' if bad == 'moving' else '')
    assert runtime.maps.stage == 'failed' and runtime.maps.hold
    assert not n._pose_received and runtime.recovery._reset_ns is None
    assert not runtime.streams_allowed and not runtime.commands_allowed
    n._publish_with_timing.assert_not_called()


@pytest.mark.parametrize('block', ['map-hold', 'epoch-failed', 'pending-control',
                                   'hardware-lock', 'action', 'tuning'])
def test_stationary_retry_guarded_reverse_keeps_global_locks(runtime, block):
    n = attach_boundaries(runtime)
    runtime.maps.hold = False
    if block == 'map-hold':
        runtime.maps.hold = True
    elif block == 'epoch-failed':
        runtime._failed = True
    elif block == 'pending-control':
        n._pending_controls['pending'] = object()
    elif block == 'hardware-lock':
        n._hardware_operation_lock.acquire()
    elif block == 'action':
        n._undock_turn.actions['navigation'] = True
    elif block == 'tuning':
        runtime.tuning_hold = True
    assert not n._undock_turn.reserve_reverse()
    assert not runtime.undock_hold
    assert n._hardware_operation_lock.locked() == (block == 'hardware-lock')


def test_stationary_retry_guarded_reverse_read_only_probe_still_admissible(runtime):
    n = attach_boundaries(runtime)
    runtime._future, runtime._future_kind = Future(NS(success=True)), 'read-only'
    assert n._undock_turn.reserve_reverse()
    assert n._hardware_operation_lock.locked() and runtime.undock_hold
    assert runtime._future is None  # retired only after exclusive ownership


@pytest.mark.parametrize('latch', ['_map_outcome_unknown', '_lifecycle_outcome_unknown',
                                   '_tuning_gate_outcome_unknown'])
def test_stationary_retry_unknown_owner_fences_otherwise_ready_robot(runtime, latch):
    req = request(runtime)
    assert runtime._map_accept(req) == GoalResponse.ACCEPT
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    assert finish_action(runtime, req).localized and runtime.commands_allowed
    attach_boundaries(runtime)
    setattr(runtime, latch, True)
    assert not runtime._failed and not runtime.maps.hold
    assert runtime._navigation_ready and not runtime.commands_allowed
    assert_motion_fenced(runtime)


def test_stationary_retry_definite_tuning_reserve_refusal_is_not_unknown(runtime):
    fail_sensor(runtime)
    runtime._tuning_nav_pending, runtime._tuning_nav_reserved = True, False
    runtime._tuning_map_phase = 'reserve'
    runtime._tuning_map_probe = Future(NS(success=False))
    runtime.tick_tuning_map_gate()
    assert not runtime._tuning_nav_reserved and not runtime.tuning_gate_unresolved
    pending = Future(NS(success=True))
    pending.complete = False
    reserve = Mock(return_value=pending)
    runtime._tuning_nav_release.call_async = reserve
    retry = new_request(runtime)
    assert runtime._map_accept(retry) == GoalResponse.ACCEPT
    assert runtime.maps.token.request_id == retry.request_id
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    assert not runtime.commands_allowed
    reserve.assert_not_called()
    runtime.node._tuning_map_identity_changed = Mock()
    assert finish_action(runtime, retry).localized
    reserve.assert_not_called()
    # The new gate may start only after the map action is terminal. Once it
    # starts, its pending future must remain the sole owner.
    runtime.tick_tuning_map_gate()
    reserve.assert_called_once()
    assert reserve.call_args.args[0].data is True
    runtime.tick_tuning_map_gate()
    assert runtime._tuning_map_probe is pending and runtime.tuning_gate_unresolved
    assert not runtime.commands_allowed
    reserve.assert_called_once()


def test_stationary_retry_completion_never_discards_a_pending_tuning_mutation(runtime):
    fail_sensor(runtime)
    retry = new_request(runtime)
    assert runtime._map_accept(retry) == GoalResponse.ACCEPT
    run_until(runtime, lambda: runtime.maps.stage == 'ready')
    # Defense in depth: deliver the unexpected owner only at completion.
    runtime._tuning_nav_pending, runtime._tuning_nav_reserved = True, False
    runtime._tuning_map_phase = 'reserve'
    pending = Future(NS(success=True))
    pending.complete = False
    runtime._tuning_map_probe = pending
    runtime.node._tuning_map_identity_changed = Mock()
    result = finish_action(runtime, retry)
    assert not result.localized and result.code == result.FAILED
    assert runtime.maps.stage == 'failed' and runtime.maps.hold
    assert runtime._tuning_gate_outcome_unknown
    pending.complete = True
    runtime.tick_tuning_map_gate()
    assert runtime._tuning_map_probe is pending and runtime.tuning_gate_unresolved
    runtime.node._tuning_map_identity_changed.assert_not_called()
    assert not runtime.commands_allowed
