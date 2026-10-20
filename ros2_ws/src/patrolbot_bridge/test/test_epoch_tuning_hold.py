"""
Regression tests for the EpochRuntime tuning activation hold.

These drive the real EpochRuntime admission boundary rather than a stand-in
owner. Engaging the hold must refuse commands even when autonomy is otherwise
ready, must stop a held seed/tick from injecting a pose or issuing lifecycle
transitions, must clear pending seed/navigation readiness, and clearing it must
not silently rearm that cleared state. The fixture mirrors
test_epoch_runtime.py: a plain Node carrying the fields the callbacks read, a
Client whose call_async resolves synchronously, and real ROS messages.
"""

import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from patrolbot_bridge.epoch_recovery import EpochRecovery
from patrolbot_bridge.epoch_runtime import EpochRuntime

A = '123e4567-e89b-12d3-a456-426614174000'


class Future:
    complete = True

    def __init__(self, result):
        self.value = result

    def done(self):
        return self.complete

    def result(self):
        return self.value

    def cancel(self):
        self.cancelled = True


class Client:
    ready = True

    def __init__(self, name):
        self.name = name
        self.calls = []
        self.future = None
        self.success = True

    def remove_pending_request(self, future):
        self.removed = future

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.calls.append(request)
        if self.name.endswith('get_state'):
            response = NS(current_state=NS(id=3))
        elif self.name.endswith('set_parameters_atomically'):
            response = NS(result=NS(successful=self.success))
        else:
            response = NS(success=self.success)
        self.future = Future(response)
        return self.future


class Node:
    def __init__(self):
        self.now = 100
        self._pose_lock = threading.Lock()
        self.sock_lock = threading.Lock()
        self.sock = Mock()
        self._pose_received = True
        self._pose_at = time.monotonic()
        self._odom_tf_timeout = 0.5
        self._vx = self._vth = 0.0
        self.cmd_vel_callback = Mock()
        self._odom_validator = Mock()
        self._odom_covariance = Mock()
        self.logger = Mock()
        self._lidar_ready = Mock(return_value=True)
        self.clients = {}

    def create_client(self, _, name):
        self.clients[name] = Client(name)
        return self.clients[name]

    def create_subscription(self, *args):
        return args

    def create_timer(self, *args):
        return args

    def get_clock(self):
        if hasattr(self, 'runtime') and self.runtime.guard.epoch:
            self.runtime._odom_history.add(
                self.now, (0.0, 0.0, 0.0), self.runtime.guard.epoch
            )
        return NS(
            now=lambda: NS(
                nanoseconds=self.now,
                to_msg=lambda: Time(
                    sec=self.now // 1_000_000_000, nanosec=self.now % 1_000_000_000
                ),
            )
        )

    def get_logger(self):
        return self.logger


def setup_runtime():
    node = Node()
    # These tests isolate the epoch admission boundary; map transactions have
    # their own real-ROS-type coordinator tests in test_map_runtime.py.
    with patch.object(EpochRuntime, 'init_maps'), patch.object(
        EpochRuntime, 'poll_navigation_idle'
    ):
        runtime = EpochRuntime(node)
    node.runtime = runtime
    runtime._map_error = ''
    runtime.maps = NS(active={'map_id': 'cmuq-floor2'}, stage='idle', hold=False)
    runtime.poll_navigation_idle = Mock()
    runtime._navigation_status = False
    runtime._navigation_idle_at = time.monotonic()
    session = runtime.begin_session()
    runtime.announce(session, A)
    return node, runtime, session


def reset(runtime):
    for _ in range(10):
        runtime.tick()
        if runtime.recovery.stage == EpochRecovery.WAIT_SEED:
            runtime.first_tf_ns = runtime.node.now
            runtime.node._pose_received = True
            runtime.node._pose_at = time.monotonic()
            runtime.tick()
            runtime.tick()
            assert runtime.seed_tf_ready
            return
    raise AssertionError('reset did not reach the seed barrier')


def pose(stamp):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp.nanosec = stamp
    msg.pose.pose.orientation.w = 1.0
    for i in (0, 7, 35):
        msg.pose.covariance[i] = 0.1
    return msg


def ready_runtime():
    """Drive the real runtime to the fully ready, command-allowed state."""
    node, runtime, _ = setup_runtime()
    reset(runtime)
    node.now = 110
    runtime.seed(pose(110))
    runtime.seed_accepted(pose(110))
    node.now = 120
    runtime.pose(pose(120))
    for _ in range(3):
        runtime.tick()
    # Keep the odometry receipt inside its freshness window so a refusable
    # command below can only be refused by the tuning hold, nothing else.
    with node._pose_lock:
        node._pose_received = True
        node._pose_at = time.monotonic()
    assert runtime.commands_allowed
    return node, runtime


def test_hold_blocks_commands_even_when_fully_ready():
    node, runtime = ready_runtime()

    runtime.set_tuning_hold(True, 'pending activation job')
    assert runtime.tuning_hold is True
    assert runtime._tuning_hold_reason == 'pending activation job'
    node.logger.warning.assert_called_with(
        'Tuning activation hold engaged: pending activation job')

    # set_tuning_hold also clears navigation readiness; put every *other*
    # admission gate back to ready so the hold flag is the only thing left that
    # can refuse the command.
    runtime._navigation_ready = True
    with runtime.node._pose_lock:
        runtime.node._pose_received = True
        runtime.node._pose_at = time.monotonic()
    assert not runtime.commands_allowed


def test_held_seed_and_tick_issue_no_pose_or_lifecycle_action():
    # Control: with no hold, a ready runtime whose navigation readiness was
    # cleared re-acquires it with a fresh lifecycle request on the next tick.
    _, control = ready_runtime()
    before = len(control._navigation.calls)
    control._navigation_ready = False
    control.tick()
    assert len(control._navigation.calls) == before + 1

    node, runtime = ready_runtime()
    runtime.set_tuning_hold(True, 'pending activation job')
    seeded = Mock(wraps=runtime.recovery.operator_seed)
    runtime.recovery.operator_seed = seeded
    nav_before = len(runtime._navigation.calls)
    change_before = len(runtime._change.calls)
    stage_before = runtime.recovery.stage
    seed_ns_before = runtime.recovery._seed_ns

    node.now = 130
    runtime.seed(pose(130))
    for _ in range(5):
        runtime.tick()

    # The held seed never reached the recovery barrier, and no held tick
    # injected a pose or issued a lifecycle transition.
    seeded.assert_not_called()
    assert runtime.recovery.stage == stage_before
    assert runtime.recovery._seed_ns == seed_ns_before
    assert runtime._pending_pose is None
    assert len(runtime._navigation.calls) == nav_before
    assert len(runtime._change.calls) == change_before
    assert runtime._navigation_ready is False
    assert not runtime.commands_allowed


def test_engaging_hold_clears_pending_seed_and_navigation_readiness():
    _, runtime = ready_runtime()
    assert runtime._navigation_ready is True
    assert runtime._pending_pose is not None
    assert runtime._accepted_seed_stamp == 110

    runtime.set_tuning_hold(True, 'pending activation job')

    assert runtime._pending_pose is None
    assert runtime._accepted_seed_stamp is None
    assert runtime._navigation_ready is False


def test_clearing_hold_does_not_rearm_stale_seed_or_navigation():
    _, runtime = ready_runtime()
    stale_pose = runtime._pending_pose
    assert stale_pose is not None

    runtime.set_tuning_hold(True, 'pending activation job')
    runtime.set_tuning_hold(False, '')

    # Dropping the flag is not enough to restore what the hold cleared; a fresh
    # seed and lifecycle round-trip must be earned again.
    assert runtime.tuning_hold is False
    assert runtime._navigation_ready is False
    assert runtime._pending_pose is None
    assert runtime._accepted_seed_stamp is None
    assert not runtime.commands_allowed
    # The stale pose is gone, not merely hidden behind the flag.
    assert runtime._pending_pose is not stale_pose
