"""Malformed-wire acceptance against the unchanged deployed ROS image.

Run with PATROLBOT_RUN_INTEGRATION=1 in a network-isolated container. This
external acceptance suite imports the image's own fake-SBC fixture and bridge;
it does not replace runtime source or require a live robot connection.
"""

import os
from pathlib import Path
import sys
from unittest.mock import patch
import time

import pytest

pytest.importorskip('rclpy')
IMAGE_TEST_ROOT = Path('/opt/patrolbot_ws/src/patrolbot_bridge/test')
if not IMAGE_TEST_ROOT.is_dir():
    pytest.skip('requires the built PatrolBot image', allow_module_level=True)
sys.path.insert(0, str(IMAGE_TEST_ROOT))
from test_fake_sbc_integration import harness, wait_until  # noqa: E402,F401

pytestmark = pytest.mark.skipif(
    os.environ.get('PATROLBOT_RUN_INTEGRATION') != '1',
    reason='requires network-isolated fake hardware')


@pytest.mark.parametrize('wire', [
    'ODOM:bad,0,0,0,0|LASER:1,1,1',
    'ODOM:0,0|LASER:1,1,1',
    'ODOM:|LASER:1,1,1',
])
def test_malformed_odom_preserves_last_valid_state_and_recovers(harness, wire):
    fake, bridge, _client, _executor = harness
    assert wait_until(lambda: bridge._pose_at > 0)
    fake.telemetry_paused = True
    time.sleep(0.15)
    before = bridge._pose_at
    covariance = bridge._odom_covariance.pose_covariance()
    rejected = bridge._odom_rejections.get('malformed-telemetry', 0)
    with patch.object(bridge, '_publish_with_timing',
                      wraps=bridge._publish_with_timing) as publish:
        bridge._parse_telemetry(wire)
        assert not any(call.args[2] in ('/odom', '/scan')
                       for call in publish.call_args_list)
    assert bridge._pose_at == before
    assert bridge._odom_covariance.pose_covariance() == covariance
    assert bridge._odom_rejections['malformed-telemetry'] == rejected + 1
    fake.telemetry_paused = False
    assert wait_until(lambda: bridge._pose_at > before)
