"""Exercise asynchronous recovery with actual ROS message/service classes."""
from types import SimpleNamespace as NS
import threading
import time
from unittest.mock import Mock, patch

from geometry_msgs.msg import PoseWithCovarianceStamped

from patrolbot_bridge.epoch_recovery import EpochRecovery
from patrolbot_bridge.epoch_runtime import EpochRuntime

A = '123e4567-e89b-12d3-a456-426614174000'
B = 'aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee'


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
        self._odom_tf_timeout = .5
        self._odom_validator = Mock()
        self._odom_covariance = Mock()
        self.logger = Mock()
        self.clients = {}

    def create_client(self, _, name):
        self.clients[name] = Client(name)
        return self.clients[name]

    def create_subscription(self, *args):
        return args

    def create_timer(self, *args):
        return args

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=self.now))

    def get_logger(self):
        return self.logger


def setup_runtime():
    node = Node()
    runtime = EpochRuntime(node)
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
        msg.pose.covariance[i] = .1
    return msg


def test_runtime_resets_before_streams_and_requires_accepted_seed():
    node, runtime, _ = setup_runtime()
    assert not runtime.streams_allowed
    assert not runtime.commands_allowed
    reset(runtime)
    transitions = [r.transition.id for r in runtime._change.calls]
    assert transitions == [4, 2, 1, 3]
    params = {p.name: p.value.bool_value for p in runtime._parameters.calls[0].parameters}
    assert params == {'set_initial_pose': False, 'always_reset_initial_pose': True}
    assert runtime.streams_allowed and not runtime.commands_allowed
    node.now = 110
    runtime.seed(pose(110))
    node.now = 120
    runtime.pose(pose(120))
    assert not runtime.commands_allowed  # a rejected seed must not release
    runtime.seed_accepted(pose(110))
    assert not runtime.commands_allowed
    for _ in range(3):
        runtime.tick()
    assert [r.command for r in runtime._navigation.calls] == [1, 2]
    assert runtime.commands_allowed


def test_ack_before_seed_callback_is_correlated_without_republishing_seed():
    node, runtime, _ = setup_runtime()
    reset(runtime)
    node.now = 110
    runtime.seed_accepted(pose(110))
    runtime.seed(pose(110))
    node.now = 120
    runtime.pose(pose(120))
    assert not runtime.commands_allowed
    for _ in range(3):
        runtime.tick()
    assert runtime.commands_allowed


def test_reset_refusal_keeps_streams_and_commands_held():
    _, runtime, _ = setup_runtime()
    runtime._change.success = False
    for _ in range(5):
        runtime.tick()
    assert not runtime.streams_allowed
    assert not runtime.commands_allowed
    assert len(runtime._change.calls) == 1


def test_unavailable_services_do_not_manufacture_reset_completion():
    _, runtime, _ = setup_runtime()
    runtime._localization_active.ready = False
    for _ in range(5):
        runtime.tick()
    assert not runtime.streams_allowed
    assert not runtime._get.calls


def test_timeout_does_not_start_overlapping_lifecycle_requests():
    _, runtime, _ = setup_runtime()
    runtime.tick()
    runtime.tick()
    runtime._future.complete = False
    with patch('patrolbot_bridge.epoch_runtime.time.monotonic', return_value=10**12):
        for _ in range(5):
            runtime.tick()
    assert len(runtime._get.calls) == 1
    assert not runtime.streams_allowed


def test_origin_change_resets_estimators_once_and_rejects_old_frames():
    node, runtime, old = setup_runtime()
    reset(runtime)
    current = runtime.begin_session()
    runtime.announce(current, B)
    runtime.announce(current, B)
    node._odom_validator.reset_epoch.assert_called_once()
    node._odom_covariance.reset.assert_called_once()
    assert not runtime.frame_allowed(old, '|ODOM_EPOCH:' + A)
    assert not runtime.streams_allowed


def test_same_origin_reconnect_does_not_repeat_lifecycle_or_covariance_reset():
    node, runtime, _ = setup_runtime()
    reset(runtime)
    calls = len(runtime._change.calls)
    current = runtime.begin_session()
    runtime.announce(current, A)
    runtime.tick()
    assert runtime.streams_allowed
    assert len(runtime._change.calls) == calls
    node._odom_covariance.reset.assert_not_called()


def test_mismatched_frame_closes_transport_and_withdraws_pose():
    node, runtime, session = setup_runtime()
    reset(runtime)
    assert not runtime.frame_allowed(session, '|ODOM_EPOCH:' + B)
    node.sock.shutdown.assert_called_once()
    assert not node._pose_received
    assert not runtime.commands_allowed


def test_navigation_pause_failure_never_releases_commands():
    node, runtime, _ = setup_runtime()
    reset(runtime)
    node.now = 110
    runtime.seed(pose(110))
    runtime.seed_accepted(pose(110))
    node.now = 120
    runtime.pose(pose(120))
    runtime._navigation.success = False
    for _ in range(5):
        runtime.tick()
    assert not runtime.commands_allowed
    assert len(runtime._navigation.calls) == 1


def test_new_seed_revokes_navigation_permission_until_fresh_acceptance():
    node, runtime, _ = setup_runtime()
    reset(runtime)
    node.now = 110
    runtime.seed(pose(110))
    runtime.seed_accepted(pose(110))
    node.now = 120
    runtime.pose(pose(120))
    for _ in range(3):
        runtime.tick()
    assert runtime.commands_allowed
    node.now = 130
    runtime.seed(pose(130))
    assert not runtime.commands_allowed
    assert not runtime._navigation_ready


def test_localization_autostart_must_finish_before_reset_requests():
    _, runtime, _ = setup_runtime()
    runtime._localization_active.success = False
    for _ in range(5):
        runtime.tick()
    assert not runtime._get.calls
    assert not runtime._change.calls
    assert not runtime.streams_allowed


def test_seed_before_first_transform_cannot_start_recovery_acceptance():
    node, runtime, _ = setup_runtime()
    reset(runtime)
    runtime.first_tf_ns = None
    node.now = 110
    runtime.seed(pose(110))
    assert runtime.recovery.stage == EpochRecovery.WAIT_SEED
    assert runtime.status_stage == 'wait-first-odometry'
    runtime.first_tf_ns = 111
    node.now = 112
    runtime.seed(pose(110))
    assert runtime.recovery.stage == EpochRecovery.WAIT_SEED
    runtime.seed(pose(112))
    assert runtime.recovery.stage == EpochRecovery.WAIT_POSE


def test_command_gate_withdraws_on_stale_missing_or_invalid_odometry():
    node, runtime, _ = setup_runtime()
    reset(runtime)
    node.now = 110
    runtime.seed(pose(110))
    runtime.seed_accepted(pose(110))
    node.now = 120
    runtime.pose(pose(120))
    for _ in range(3):
        runtime.tick()
    with patch('patrolbot_bridge.epoch_runtime.time.monotonic', return_value=10.0):
        node._pose_at = 9.75
        assert runtime.commands_allowed
        for receipt in (9.0, 11.0, float('nan'), float('inf')):
            node._pose_at = receipt
            assert runtime.guard.ready
            assert not runtime.commands_allowed
        node._pose_at = 9.75
        node._pose_received = False
        assert not runtime.commands_allowed
        node._pose_received = True
        assert runtime.commands_allowed


def test_tf_readiness_missing_service_keeps_streams_but_rejects_seed():
    node, runtime, _ = setup_runtime()
    reset(runtime)
    runtime._tf_ready_at = None
    runtime._tf_readiness.ready = False
    node.now = 110
    runtime.seed(pose(110))
    assert runtime.streams_allowed
    assert runtime.status_stage == 'wait-amcl-tf'
    assert runtime.recovery.stage == EpochRecovery.WAIT_SEED
    assert not runtime.commands_allowed


def test_tf_readiness_expired_response_is_not_a_fresh_receipt():
    _, runtime, _ = setup_runtime()
    reset(runtime)
    runtime._tf_last_query = float('-inf')
    with patch('patrolbot_bridge.epoch_runtime.time.monotonic', return_value=10.0):
        runtime.tick()
    with patch('patrolbot_bridge.epoch_runtime.time.monotonic', return_value=11.0):
        runtime.tick()
        assert not runtime.seed_tf_ready
        assert runtime.status_stage == 'wait-amcl-tf'


def test_tf_readiness_timeout_releases_pending_request_without_acceptance():
    _, runtime, _ = setup_runtime()
    reset(runtime)
    runtime._tf_last_query = float('-inf')
    with patch('patrolbot_bridge.epoch_runtime.time.monotonic', return_value=10.0):
        runtime.tick()
        pending = runtime._tf_probe
        pending.complete = False
    with patch('patrolbot_bridge.epoch_runtime.time.monotonic', return_value=11.0):
        runtime.tick()
        assert not runtime.seed_tf_ready
        assert pending.cancelled
        assert runtime._tf_readiness.removed is pending


def test_tf_readiness_response_from_previous_connection_is_ignored():
    _, runtime, session = setup_runtime()
    reset(runtime)
    runtime._tf_last_query = float('-inf')
    runtime.tick()
    pending = runtime._tf_probe
    runtime.disconnect(session)
    new_session = runtime.begin_session()
    runtime.announce(new_session, A)
    runtime.first_tf_ns = runtime.node.now
    runtime.tick()
    assert pending.done()
    assert not runtime.seed_tf_ready


def test_tf_readiness_response_from_previous_epoch_is_ignored():
    _, runtime, session = setup_runtime()
    reset(runtime)
    runtime._tf_last_query = float('-inf')
    runtime.tick()
    runtime.announce(session, B)
    runtime.tick()
    assert not runtime.seed_tf_ready
    assert not runtime.commands_allowed


def test_seed_before_consumer_readiness_cannot_be_replayed_later():
    node, runtime, _ = setup_runtime()
    reset(runtime)
    runtime._tf_ready_since_ns = 115
    node.now = 120
    runtime.seed(pose(110))
    assert runtime.recovery.stage == EpochRecovery.WAIT_SEED
    runtime.seed(pose(120))
    assert runtime.recovery.stage == EpochRecovery.WAIT_POSE
