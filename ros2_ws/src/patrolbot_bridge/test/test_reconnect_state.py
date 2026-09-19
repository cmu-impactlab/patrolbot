"""Exercise the real connection loop offline, replacing only I/O and ROS setup."""

import ast
from pathlib import Path
import socket
import threading
from types import SimpleNamespace
from unittest.mock import Mock

from patrolbot_bridge.odom_covariance import OdomCovariance
from patrolbot_bridge.odom_validation import OdomSample, OdomValidator


def connection_loop():
    """Load the production method without importing unavailable ROS messages."""
    source = Path(__file__).parents[1] / 'patrolbot_bridge' / 'bridge_node.py'
    tree = ast.parse(source.read_text())
    bridge = next(node for node in tree.body
                  if isinstance(node, ast.ClassDef) and node.name == 'PatrolBotBridge')
    method = next(node for node in bridge.body
                  if isinstance(node, ast.FunctionDef) and node.name == '_connect_loop')
    namespace = {'socket': SimpleNamespace(
        socket=Mock(side_effect=lambda *args: Mock()),
        AF_INET=socket.AF_INET, SOCK_STREAM=socket.SOCK_STREAM,
        SOL_SOCKET=socket.SOL_SOCKET, SO_KEEPALIVE=socket.SO_KEEPALIVE)}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'),
         namespace)
    return namespace['_connect_loop']


def test_repeated_reconnect_preserves_estimator_but_invalidates_controls():
    covariance = OdomCovariance(
        sigma_xy_per_m=0.01, sigma_yaw_per_rad=0.02,
        sigma_yaw_per_m=0.005, sigma_vx=0.03, sigma_vyaw=0.04,
        min_variance=1e-6, max_variance=1.0)
    validator = OdomValidator(
        max_linear_speed=0.3, max_angular_speed=1.2, expected_period=0.05)
    for i in range(11):
        pose = OdomSample(10.0 + i * 0.01, 0.0, 0.0, 0.0, 0.0)
        assert validator.check_and_accept(pose, 1.0 + i * 0.05).accepted
        covariance.update(pose.x, pose.y, pose.yaw)
    driven = covariance.pose_covariance()
    assert driven[0] > 1e-6
    bridge = SimpleNamespace(
        running=True, CONNECT_TIMEOUT=0.1, RECV_TIMEOUT=0.1,
        RECONNECT_DELAY=0.0, server_ip='unused', server_port=0,
        sock_lock=threading.Lock(), _state_lock=threading.Lock(),
        _pose_lock=threading.Lock(), _session_generation=0,
        _odom_covariance=covariance, _odom_validator=validator,
        _stop_event=Mock(), get_logger=Mock(return_value=Mock()),
        _fail_pending_controls=Mock(), _fail_pending_undock=Mock())

    def receive(_sock):
        # Bound even a regression: the production loop catches callback errors.
        if bridge._session_generation == 3:
            bridge.running = False
        assert bridge._link_connected
        assert bridge._charge_state_at == bridge._flags_at == 0.0
        assert bridge._dock_state == ''
        assert bridge._server_hello is None
        assert not bridge._undock_active
        assert covariance.pose_covariance() == driven
        # A socket reconnect cannot clear the continuity reference either.
        result = validator.check_and_accept(
            OdomSample(0.0, 0.0, 0.0, 0.0, 0.0),
            1.5 + bridge._session_generation * 0.05)
        assert result.reason == 'position-discontinuity'

    bridge._receive_loop = receive
    connection_loop()(bridge)
    assert bridge._session_generation == 3
    bridge.get_logger.return_value.warn.assert_not_called()
    assert not bridge._link_connected
    assert bridge.sock is None
    assert bridge._fail_pending_controls.call_count == 3
    assert bridge._fail_pending_undock.call_count == 3
    assert [call.args[0] for call in bridge._fail_pending_controls.call_args_list] == [1, 2, 3]
    assert covariance.pose_covariance() == driven
    sample = OdomSample(10.11, 0.0, 0.0, 0.0, 0.0)
    assert validator.check_and_accept(sample, 1.7).accepted
    covariance.update(sample.x, sample.y, sample.yaw)
    assert covariance.pose_covariance()[0] > driven[0]
