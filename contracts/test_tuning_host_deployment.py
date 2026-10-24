"""Static standard-deployment boundaries; NOT native Docker runtime acceptance."""

import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_host_fallback_remains_disabled_and_uninstalled():
    policy = json.loads((ROOT / 'docker/tuning-host/policy.example.json').read_text())
    assert policy['enabled'] is False
    assert policy['candidate_runtime_integrated'] is False
    assert policy['services'] == ['bringup', 'bridge', 'navigation']
    compose = yaml.safe_load((ROOT / 'docker/docker-compose.yml').read_text())
    assert set(compose['services']) == {'bringup', 'bridge', 'navigation', 'web-bridge', 'gimbal'}
    for name, service in compose['services'].items():
        serialized = json.dumps(service)
        assert 'docker.sock' not in serialized
        assert 'tuning-host/signing' not in serialized
        assert 'tuning_host' not in serialized
        assert '--candidate-boot' not in serialized
        assert 'PATROLBOT_TUNING_CANDIDATE_BOOT' not in service.get('environment', {})


def test_web_bridge_has_neither_candidate_boot_material_nor_tuning_signing_keys():
    compose = yaml.safe_load((ROOT / 'docker/docker-compose.yml').read_text())
    web = json.dumps(compose['services']['web-bridge'])
    for forbidden in ('patrolbot-secrets', 'patrolbot-state', 'tuning.key',
                      'tuning-host', 'candidate_boot', 'docker.sock'):
        assert forbidden not in web
    assert '/run/patrolbot-system-reset' in web
    assert compose['services']['web-bridge']['group_add'][0].startswith('${PATROLBOT_RESET_GID:?')


def test_crypto_verification_available_without_baking_private_keys_or_host_helper_into_images():
    for name in ('Dockerfile', 'Dockerfile.ros-tests'):
        text = (ROOT / 'docker' / name).read_text()
        assert 'python3-cryptography' in text
        assert 'COPY docker/tuning-host' not in text
        assert 'signing.pem' not in text
    package = (ROOT / 'ros2_ws/src/patrolbot_bridge/package.xml').read_text()
    assert '<exec_depend>python3-cryptography</exec_depend>' in package


def test_standard_consumers_keep_active_only_launcher_and_persistent_state_modes():
    services = yaml.safe_load((ROOT / 'docker/docker-compose.yml').read_text())['services']
    for role in ('bringup', 'navigation'):
        assert services[role]['command'] == [
            'ros2', 'run', 'patrolbot_bridge', 'tuning_run', '--role', role]
        assert 'patrolbot-state:/var/lib/patrolbot:ro' in services[role]['volumes']
    assert 'patrolbot-state:/var/lib/patrolbot' in services['bridge']['volumes']
    assert 'patrolbot-secrets:/run/patrolbot-secrets:ro' in services['bridge']['volumes']
    # No fallback helper runs implicitly as an entrypoint or on a container restart.
    source = (ROOT / 'ros2_ws/src/patrolbot_bridge/patrolbot_bridge/tuning_run.py').read_text()
    assert 'resolve_candidate' not in source
    assert 'HostDispatcher' not in source


def test_navigation_only_helper_sockets_do_not_expose_host_control_to_other_consumers():
    services = yaml.safe_load((ROOT / 'docker/docker-compose.yml').read_text())['services']
    core = '/run/patrolbot-tuning/core:/run/patrolbot-tuning/core:ro'
    boot = '/run/patrolbot-tuning/boot:/run/patrolbot-tuning/boot:ro'
    assert core in services['bridge']['volumes']
    assert boot in services['navigation']['volumes']
    for name, service in services.items():
        encoded = json.dumps(service)
        if name != 'bridge': assert '/run/patrolbot-tuning/core' not in encoded
        if name != 'navigation': assert '/run/patrolbot-tuning/boot' not in encoded
        assert 'docker.sock' not in encoded
    policy = json.loads((ROOT / 'docker/tuning-host/navigation.example.json').read_text())
    assert policy['enabled'] is False
    service = (ROOT / 'docker/tuning-host/patrolbot-navigation-apply.service').read_text()
    assert 'RuntimeDirectoryPreserve=yes' in service
    assert 'navigation_supervisor.py' in service
