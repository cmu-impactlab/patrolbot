"""Standard Compose must retain commissioned features without optional overlays."""

import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
COMPOSE = ROOT / 'docker/docker-compose.yml'
RESET = ROOT / 'docker/system-reset/compose.override.yaml'


def deployment():
    return yaml.safe_load(COMPOSE.read_text())


def test_reset_integration_is_in_standard_compose_not_an_optional_overlay():
    web = deployment()['services']['web-bridge']
    assert web['environment']['PATROLBOT_RESET_SOCKET'] == (
        '/run/patrolbot-system-reset/supervisor.sock')
    mounts = [v for v in web['volumes'] if isinstance(v, dict)]
    reset = next(v for v in mounts if v['target'] == '/run/patrolbot-system-reset')
    assert reset == {'type': 'bind', 'source': '/run/patrolbot-system-reset',
                     'target': '/run/patrolbot-system-reset', 'read_only': True,
                     'bind': {'create_host_path': False}}
    assert len(web['group_add']) == 1
    assert web['group_add'][0].startswith('${PATROLBOT_RESET_GID:?')
    assert yaml.safe_load(RESET.read_text()) == {'services': {}}


def test_reset_client_does_not_gain_docker_or_ssh_credentials_or_implicit_commands():
    web = deployment()['services']['web-bridge']
    assert web['environment']['WEB_BRIDGE_ENABLE_COMMANDS'] == '${WEB_BRIDGE_ENABLE_COMMANDS:-0}'
    assert web.get('privileged', False) is False
    assert web['security_opt'] == ['no-new-privileges:true']
    text = json.dumps(web)
    for forbidden in ('/var/run/docker.sock', '/run/docker.sock', '/.ssh',
                      'ssh_config', 'id_ed25519', 'restart_sbc', 'software_reset"'):
        assert forbidden not in text
    # Wiring does not run a reset. The host supervisor retains request admission.
    assert 'supervisor.py' not in web['command'][0]


def test_reset_mount_directory_is_created_before_docker_restores_containers():
    policy = ROOT / 'docker/system-reset/patrolbot-system-reset.tmpfiles.conf'
    entries = [line.split() for line in policy.read_text().splitlines()
               if line.strip() and not line.lstrip().startswith('#')]
    assert entries == [['d', '/run/patrolbot-system-reset', '0750',
                        'ubuntu', 'patrolbot-reset', '-']]
    unit = (ROOT / 'docker/system-reset/patrolbot-system-reset.service').read_text()
    assert 'RuntimeDirectory=patrolbot-system-reset' in unit
    assert 'RuntimeDirectoryMode=0750' in unit
    assert 'RuntimeDirectoryPreserve=restart' in unit
    # The tmpfiles policy creates no socket, issues no command and does not
    # weaken the existing fail-closed Compose bind-mount requirement.
    mount = next(v for v in deployment()['services']['web-bridge']['volumes']
                 if isinstance(v, dict) and v['target'] == '/run/patrolbot-system-reset')
    assert mount['bind']['create_host_path'] is False


def test_env_template_requires_operator_resolved_group_without_a_root_fallback():
    template = (ROOT / 'docker/.env.example').read_text()
    assert '\nPATROLBOT_RESET_GID=\n' in template
    assert 'getent group patrolbot-reset' in template
    assert 'PATROLBOT_RESET_GID:-' not in COMPOSE.read_text()


def compose_config(*extra, missing_gid=False):
    if shutil.which('docker') is None:
        pytest.skip('Docker Compose client not installed; static contract checks still run')
    version = subprocess.run(['docker', 'compose', 'version'], capture_output=True)
    if version.returncode:
        pytest.skip('Docker Compose plugin not installed')
    env = {key: value for key, value in os.environ.items()
           if not key.startswith(('PATROLBOT_', 'WEB_BRIDGE_', 'COMPOSE_'))}
    env.update(PATROLBOT_IMAGE='patrolbot:ci-only', PATROLBOT_REVISION='ci-only')
    if not missing_gid:
        env['PATROLBOT_RESET_GID'] = '65532'
    # Disable implicit local .env loading; tests must not touch operator secrets.
    return subprocess.run(['docker', 'compose', '--env-file', '/dev/null', '-f', str(COMPOSE),
                           *extra, '--profile', 'web-bridge', 'config', '--format', 'json'],
                          env=env, capture_output=True, text=True, timeout=15)


def test_actual_compose_retains_reset_mount_and_group_on_normal_recreation():
    result = compose_config()
    assert result.returncode == 0, result.stderr
    web = json.loads(result.stdout)['services']['web-bridge']
    assert web['environment']['PATROLBOT_RESET_SOCKET'].endswith('/supervisor.sock')
    assert web['group_add'] == ['65532']
    reset = next(v for v in web['volumes'] if v['target'] == '/run/patrolbot-system-reset')
    # Some Compose versions omit false/default booleans from normalized JSON.
    # The source-level check above separately requires explicit false.
    assert reset.get('bind', {}).get('create_host_path', False) is False


def test_legacy_override_is_idempotent_and_no_longer_required():
    base, combined = compose_config(), compose_config('-f', str(RESET))
    assert base.returncode == combined.returncode == 0
    assert json.loads(base.stdout) == json.loads(combined.stdout)


def test_missing_reset_prerequisite_fails_config_instead_of_silently_losing_feature():
    result = compose_config(missing_gid=True)
    assert result.returncode != 0
    assert 'PATROLBOT_RESET_GID' in result.stderr
