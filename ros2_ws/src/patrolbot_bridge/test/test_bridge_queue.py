from types import SimpleNamespace as NS
import threading
from unittest.mock import Mock, patch
import pytest

from patrolbot_bridge.bridge_node import PatrolBotBridge


def node():
    return NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000)),
              _odom_tf_timeout=.5, _record_odom_rejection=Mock(),
              _odom_validator=Mock(check_and_accept=Mock(return_value=NS(accepted=False, reason='sentinel'))),
              get_logger=lambda: Mock())


def test_queued_odometry_is_not_restamped_into_freshness():
    n=node()
    with patch('patrolbot_bridge.bridge_node.time.monotonic', return_value=10.):
        PatrolBotBridge._parse_telemetry_guarded(n, 'ODOM:0,0,0,0,0|LASER:1,2,3', received_at=9.)
    n._odom_validator.check_and_accept.assert_not_called()
    n._record_odom_rejection.assert_called_once_with('stale-or-future-queued-odometry')


def test_valid_queue_uses_network_receipt_for_continuity():
    n=node()
    with patch('patrolbot_bridge.bridge_node.time.monotonic', return_value=10.):
        PatrolBotBridge._parse_telemetry_guarded(n, 'ODOM:0,0,0,0,0|LASER:1,2,3', received_at=9.9)
    assert n._odom_validator.check_and_accept.call_args.args[1] == 9.9


@pytest.mark.parametrize('phase', ['turn', 'stopping', 'restoring'])
def test_owned_turn_observes_validated_raw_pose_before_any_dds_publication(phase):
    n = node()
    n._odom_validator.check_and_accept.return_value = NS(accepted=True)
    n._pose_lock = threading.Lock()
    n._pose_received, n._pose_at = True, 9.9
    n._odom_covariance = Mock(
        pose_covariance=Mock(return_value=[0.] * 36),
        twist_covariance=Mock(return_value=[0.] * 36))
    n._epoch = NS(_odom_history=Mock(), guard=NS(epoch='epoch'))
    n.odom_pub = Mock()
    events = []
    def observe():
        assert n._px == .02 and n._pth == 3.15
        assert not n._pose_lock.locked()
        events.append('observe')
    n._undock_turn = NS(phase=phase, observe_turn_odom=Mock(side_effect=observe))
    n._publish_with_timing = lambda *_args: events.append('publish')
    with patch('patrolbot_bridge.bridge_node.time.monotonic', return_value=10.):
        PatrolBotBridge._parse_telemetry_guarded(
            n, 'ODOM:.02,0,3.15,0,.7|LASER:', received_at=9.95)
    n._record_odom_rejection.assert_not_called()
    assert events == ['observe', 'publish']
