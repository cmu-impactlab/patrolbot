from functools import partial
from threading import Lock
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from patrolbot_bridge.bridge_node import PatrolBotBridge
from patrolbot_bridge.odom_validation import OdomValidator
import pytest


def node():
    return NS(
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000)),
        _odom_tf_timeout=0.5,
        _record_odom_rejection=Mock(),
        _odom_validator=Mock(
            check_and_accept=Mock(return_value=NS(accepted=False, reason='sentinel', detail=''))
        ),
        get_logger=lambda: Mock(),
    )


def test_queued_odometry_is_not_restamped_into_freshness():
    n = node()
    with patch('patrolbot_bridge.bridge_node.time.monotonic', return_value=10.0):
        PatrolBotBridge._parse_telemetry_guarded(n, 'ODOM:0,0,0,0,0|LASER:1,2,3', received_at=9.0)
    n._odom_validator.check_and_accept.assert_not_called()
    n._record_odom_rejection.assert_called_once_with('stale-or-future-queued-odometry')


def test_valid_queue_uses_network_receipt_for_continuity():
    n = node()
    with patch('patrolbot_bridge.bridge_node.time.monotonic', return_value=10.0):
        PatrolBotBridge._parse_telemetry_guarded(n, 'ODOM:0,0,0,0,0|LASER:1,2,3', received_at=9.9)
    assert n._odom_validator.check_and_accept.call_args.args[1] == 9.9


def test_distinct_sip_updates_pose_but_fresh_duplicate_only_publishes_scan():
    n = node()
    n._odom_validator = OdomValidator(
        max_linear_speed=.30, max_angular_speed=1.20, expected_period=.05,
    )
    n._epoch = NS(_odom_history=Mock(), guard=NS(epoch='origin'))
    n._pose_lock = Lock()
    n._pose_received = False
    n._odom_tf_stale = False
    n._undock_turn = Mock(phase='turn')
    n._odom_covariance = Mock(
        pose_covariance=Mock(return_value=[0.] * 36),
        twist_covariance=Mock(return_value=[0.] * 36),
    )
    n._broadcast_odom_tf = Mock()
    n._publish_validated_odom = partial(PatrolBotBridge._publish_validated_odom, n)
    n._publish_with_timing = Mock()
    n.odom_pub, n.scan_pub, n.unfiltered_scan_pub = object(), object(), None
    n.SCAN_RANGE_MIN = .05
    n._laser_scan_message = PatrolBotBridge._laser_scan_message

    def observe(received, stamp, age, x=0.):
        line = (f'ODOM:{x},0,0,0,0|LASER:1,2,3|LASER_AGE_MS:5'
                f'|ODOM_SOURCE_MS:{stamp},{age}')
        n._publish_with_timing.reset_mock()
        with patch('patrolbot_bridge.bridge_node.time.monotonic', return_value=received + .01):
            PatrolBotBridge._parse_telemetry_guarded(
                n, line, received_at=received, source_timing=True,
            )
        return [call.args[2] for call in n._publish_with_timing.call_args_list]

    assert observe(10.00, 1000, 10) == ['/odom', '/scan']
    assert n._pose_at == pytest.approx(9.99)
    assert n._undock_scan_at == pytest.approx(9.995)
    assert observe(10.05, 1000, 60) == ['/scan']
    assert n._pose_at == pytest.approx(9.99)
    assert n._undock_scan_at == pytest.approx(10.045)
    assert n._odom_covariance.update.call_count == 1
    assert n._undock_turn.observe_turn_odom.call_count == 1
    assert n._epoch._odom_history.add.call_count == 1
    assert n._broadcast_odom_tf.call_count == 1
    assert observe(10.53, 1000, 499) == []
    assert n._pose_at == pytest.approx(9.99)

    # Same-origin transport loss withdraws the pose but not the validator's
    # reference; a duplicate cannot restore navigation even within timeout.
    assert observe(10.60, 1600, 10) == ['/odom', '/scan']
    n._pose_received = False
    assert observe(10.65, 1600, 60) == []
    assert not n._pose_received
    assert observe(10.70, 1700, 10) == ['/odom', '/scan']
    assert n._pose_received and n._pose_at == pytest.approx(10.69)
