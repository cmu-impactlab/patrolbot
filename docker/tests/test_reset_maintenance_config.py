"""Offline standard Compose commissioning contract; no Docker invocation."""
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_reset_is_a_separate_explicit_default_off_switch():
    compose = yaml.safe_load((ROOT / 'docker-compose.yml').read_text())
    bridge = compose['services']['web-bridge']
    env = bridge['environment']
    assert env['WEB_BRIDGE_ENABLE_COMMANDS'] == '${WEB_BRIDGE_ENABLE_COMMANDS:-0}'
    assert env['WEB_BRIDGE_ENABLE_RESET'] == '${WEB_BRIDGE_ENABLE_RESET:-0}'
    assert env['PATROLBOT_RESET_SOCKET'] == '/run/patrolbot-system-reset/supervisor.sock'
    mount = next(item for item in bridge['volumes'] if isinstance(item, dict))
    assert mount['source'] == mount['target'] == '/run/patrolbot-system-reset'
    assert mount['read_only'] is True and mount['bind']['create_host_path'] is False
    assert bridge['group_add'] == ['${PATROLBOT_RESET_GID:?set PATROLBOT_RESET_GID to the host patrolbot-reset group ID}']
    assert 'docker.sock' not in str(bridge) and 'ssh' not in str(bridge).lower()
    example = (ROOT / '.env.example').read_text().splitlines()
    assert 'WEB_BRIDGE_ENABLE_COMMANDS=0' in example
    assert 'WEB_BRIDGE_ENABLE_RESET=0' in example
