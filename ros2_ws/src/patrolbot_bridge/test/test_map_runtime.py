"""Exercise the real coordinator with ROS types and deterministic service peers.

No DDS graph, sockets, velocity publisher, or physical robot is used here.
"""
import copy
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
from patrolbot_interfaces.action import LocalizeOnMap
from rclpy.action import GoalResponse

from patrolbot_bridge.epoch_runtime import EpochRuntime
from patrolbot_bridge.epoch_recovery import EpochRecovery
from patrolbot_bridge.map_runtime import map_identity

ORIGIN = '123e4567-e89b-12d3-a456-426614174000'
REQUEST = 'bd98f746-f930-49e9-8b8f-17395cd84a65'


class Future:
    def __init__(self, value):
        self.value = value
        self.complete = True
    def done(self): return self.complete
    def result(self): return self.value
    def cancel(self): self.complete = True


class Client:
    def __init__(self, node, name):
        self.node, self.name, self.calls = node, name, []
    def service_is_ready(self): return True
    def remove_pending_request(self, future): pass
    def call_async(self, request):
        self.calls.append(request)
        if self.name.endswith('/get_state'):
            result = NS(current_state=NS(id=3))
        elif self.name.endswith('/set_parameters_atomically'):
            result = NS(result=NS(successful=True))
        elif self.name.endswith('/load_map'):
            entry = self.node.runtime.maps.target
            grid = OccupancyGrid(); grid.header.frame_id = 'map'
            grid.info.width = entry['width']; grid.info.height = entry['height']
            grid.info.resolution = entry['resolution']
            grid.info.origin.position.x, grid.info.origin.position.y = entry['origin'][:2]
            grid.info.origin.orientation.w = 1.
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
        self._pose_lock = threading.Lock(); self._state_lock = threading.Lock()
        self._control_lock = threading.Lock(); self._undock_goal_lock = threading.Lock()
        self.sock_lock = threading.Lock(); self.sock = None
        self._stop_event = threading.Event(); self._callbacks = Mock()
        self.context = NS(ok=lambda: True)
        self._odom_validator = Mock(); self._odom_covariance = Mock()
        self._odom_tf_timeout = .5
        self._link_connected = True; self._fault_flags = 0
        self._undock_active = False; self._undock_goal_reserved = False
        self._pending_controls = {}; self._vx = self._vth = 0.
        self.clients = {}; self.seeds = []; self.cmd_vel_callback = Mock()
        self.refresh()
    def refresh(self):
        self.now += 10_000_000
        self._pose_received = True
        self._pose_at = self._flags_at = self._charge_state_at = time.monotonic()
    def stamp(self): return Time(sec=self.now//1_000_000_000, nanosec=self.now%1_000_000_000)
    def get_clock(self): return NS(now=lambda: NS(nanoseconds=self.now, to_msg=self.stamp))
    def get_logger(self): return Mock()
    def create_subscription(self, *args): return args
    def create_timer(self, *args): return args
    def create_client(self, _, name):
        self.clients[name] = Client(self, name); return self.clients[name]
    def create_publisher(self, *args): return NS(publish=self.publish)
    def publish(self, msg):
        self.seeds.append(copy.deepcopy(msg))
        self.runtime.seed_accepted(msg)
        self.now += 1
        pose = copy.deepcopy(msg); pose.header.stamp = self.stamp()
        self.runtime.pose(pose)


@pytest.fixture
def runtime(tmp_path, monkeypatch):
    monkeypatch.setenv('PATROLBOT_STATE_DIR', str(tmp_path))
    navigation = Path(__file__).resolve().parents[2] / 'patrolbot_navigation'
    with patch('patrolbot_bridge.map_runtime.get_package_share_directory', return_value=str(navigation)), \
            patch('patrolbot_bridge.map_runtime.ActionServer'):
        node = Node(); runtime = EpochRuntime(node); node.runtime = runtime
    runtime.maps.active = runtime._startup_map
    session = runtime.begin_session(); runtime.announce(session, ORIGIN)
    runtime.recovery.reset_complete(runtime.recovery.token, node.now)
    runtime._token = runtime.recovery.token
    runtime._navigation_status = False
    runtime._navigation_idle_at = time.monotonic()
    return runtime


def request(runtime, map_id='cmuq-floor1', request_id=REQUEST):
    goal = LocalizeOnMap.Goal(); goal.request_id = request_id
    goal.map_id = map_id; goal.map_revision = runtime.maps.catalog.maps[map_id]['revision']
    goal.operator_authorized = True
    goal.pose.pose.position.x = goal.pose.pose.position.y = 1.
    goal.pose.pose.orientation.w = 1.
    for index in (0, 7, 35): goal.pose.covariance[index] = .1
    return goal


def step(runtime):
    runtime.node.refresh()
    # Each deterministic step represents a fresh service-poll interval.
    runtime._tf_last_query = float("-inf")
    if runtime._navigation_probe is None:
        runtime._navigation_probe_at = time.monotonic() - .11
    runtime.first_tf_ns = runtime.node.now - 100
    runtime.tick()


def run_until(runtime, predicate):
    for _ in range(100):
        if predicate(): return
        step(runtime)
    raise AssertionError((runtime.maps.stage, runtime._map_step, runtime.maps.error))


def test_round_trip_orders_seed_after_consumption_and_persistence(runtime):
    for i, map_id in enumerate(('cmuq-floor1', 'cmuq-floor2')):
        goal = request(runtime, map_id, f'bd98f746-f930-49e9-8b8f-{17395+i:012d}')
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
    assert [r.transition.id for r in runtime._change.calls] == [4,2,1,3]*2
    assert len(runtime._clear_global.calls) == len(runtime._clear_local.calls) == 2
    assert runtime.node.cmd_vel_callback.call_count == 2


@pytest.mark.parametrize('phase', ['pause','inspect','deactivate','cleanup','load','parameters',
                                  'configure','activate','readback','amcl-map','tf','seed',
                                  'localization','resume','clear-global','clear-local','costmap-map'])
def test_cancel_at_every_stage_retains_hold_and_never_reseeds(runtime, phase):
    goal = request(runtime); runtime.node.refresh()
    assert runtime._map_accept(goal) == GoalResponse.ACCEPT
    run_until(runtime, lambda: runtime._map_step == phase)
    seeds = len(runtime.node.seeds); active = runtime.maps.active
    runtime._map_cancel(NS(request=goal))
    for _ in range(5): step(runtime)
    assert runtime.maps.stage == 'failed' and not runtime.commands_allowed
    assert len(runtime.node.seeds) == seeds
    assert runtime.maps.active == active


@pytest.mark.parametrize('failure', ['disconnect','origin','timeout','moving','stale','nonfinite'])
def test_conflicting_state_fails_closed(runtime, failure):
    runtime.node.refresh(); assert runtime._map_accept(request(runtime)) == GoalResponse.ACCEPT
    run_until(runtime, lambda: runtime._map_step == 'load')
    if failure == 'disconnect': runtime.guard.disconnect(runtime.guard.session)
    if failure == 'origin': runtime.recovery._start_reset('aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee')
    if failure == 'timeout': runtime._map_deadline = 0
    if failure == 'moving': runtime.node._vx = .1
    if failure == 'stale': runtime.node._flags_at = 0
    if failure == 'nonfinite': runtime.node._vth = float('nan')
    runtime.tick()
    assert runtime.maps.stage == 'failed' and not runtime.commands_allowed
    assert runtime.node.seeds == []


def test_late_map_ack_does_not_release_failed_transaction(runtime):
    runtime.node.refresh(); runtime._map_accept(request(runtime))
    run_until(runtime, lambda: runtime._map_step == 'amcl-map')
    step(runtime)
    runtime.maps.fail('timeout')
    for _ in range(4): step(runtime)
    assert runtime.maps.active['map_id'] == 'cmuq-floor2'
    assert not runtime.commands_allowed and runtime.node.seeds == []



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
