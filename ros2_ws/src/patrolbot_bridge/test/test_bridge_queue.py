from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

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
