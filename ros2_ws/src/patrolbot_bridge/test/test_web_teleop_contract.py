#!/usr/bin/env python3
"""
Pin the cross-repo dashboard-teleop interface (contracts/web-teleop.json).

The canonical contract lives in this repo; patrolbot-dashboard vendors an
identical copy and its CI compares the two byte for byte. This test keeps the
robot side honest: the mux table, the teleop input topic, the bridge's two
evidence subscriptions and the shared evidence freshness must all match what
the contract promises the web bridge, so a drift on either side fails here
rather than on the robot.

No ROS imports, no launch files — the mux is read as YAML and the bridge as
source text, which is exactly the deployed configuration's source of truth.
"""

import json
from pathlib import Path

from patrolbot_bridge.manual_active import MANUAL_ACTIVE_TIMEOUT
import yaml

REPO = Path(__file__).resolve().parents[4]
CONTRACT_PATH = REPO / 'contracts' / 'web-teleop.json'
MUX_PATH = REPO / 'ros2_ws' / 'src' / 'patrolbot-launch' / 'param' / 'defaults' / 'mux.yaml'
SMOOTHER_PATH = (REPO / 'ros2_ws' / 'src' / 'patrolbot-launch' / 'param' / 'defaults'
                 / 'smoother.yaml')
BRIDGE_PATH = (REPO / 'ros2_ws' / 'src' / 'patrolbot_bridge' / 'patrolbot_bridge'
               / 'bridge_node.py')

CONTRACT = json.loads(CONTRACT_PATH.read_text())


def _mux_topics():
    mux = yaml.safe_load(MUX_PATH.read_text())
    return mux['cmd_vel_mux']['ros__parameters']['topics']


def test_mux_priorities_and_timeouts_match_the_contract():
    topics = _mux_topics()
    assert set(topics) == set(CONTRACT['mux'])
    for key, entry in CONTRACT['mux'].items():
        assert topics[key]['priority'] == entry['priority'], key
        assert topics[key]['timeout'] == entry['timeout_s'], key


def test_dashboard_teleop_stays_below_the_human_channels_above_autonomy():
    mux = CONTRACT['mux']
    assert mux['teleop']['priority'] < mux['joy']['priority']
    assert mux['joy']['priority'] < mux['undock_turn']['priority']
    assert mux['undock_turn']['priority'] < mux['safety_controller']['priority']
    assert mux['teleop']['priority'] > mux['navigation']['priority']


def test_mux_teleop_input_is_the_contract_velocity_topic():
    # mux.yaml declares the topic relative to the node (global namespace), so
    # compare with the leading slash normalised away.
    mux_name = _mux_topics()['teleop']['topic']
    contract_name = CONTRACT['topics']['velocity']['name']
    assert mux_name.lstrip('/') == contract_name.lstrip('/')


def test_bridge_subscribes_to_both_evidence_topics():
    source = BRIDGE_PATH.read_text()
    for key in ('evidence', 'joystick_evidence'):
        assert f"'{CONTRACT['topics'][key]['name']}'" in source, key


def test_evidence_freshness_matches_the_shared_timeout():
    assert CONTRACT['timing']['evidence_max_age_s'] == MANUAL_ACTIVE_TIMEOUT


def test_web_bridge_watchdog_is_inside_the_evidence_window():
    watchdog_s = CONTRACT['timing']['web_bridge_zero_watchdog_ms'] / 1000.0
    assert 0.0 < watchdog_s <= CONTRACT['timing']['evidence_max_age_s']


def test_velocity_limits_match_the_teleop_smoother_ceiling():
    """
    The dashboard's velocity bounds are the robot's own speed ceiling.

    The pair is only honest if the web bridge can never ask for more than the
    final smoother is configured to pass, so the contract -- which both repos
    check against -- must carry exactly that ceiling.
    """
    params = yaml.safe_load(SMOOTHER_PATH.read_text())
    smoother = params['teleop_velocity_smoother']['ros__parameters']
    limits = CONTRACT['velocity_limits']
    assert limits['linear_m_s'] == {'min': smoother['min_velocity'][0],
                                    'max': smoother['max_velocity'][0]}
    assert limits['angular_rad_s'] == {'min': smoother['min_velocity'][2],
                                       'max': smoother['max_velocity'][2]}
