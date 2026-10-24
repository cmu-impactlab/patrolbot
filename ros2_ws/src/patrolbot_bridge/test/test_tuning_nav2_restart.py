"""Offline exact-job host/startup slice; no Docker daemon or live restart."""

import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from patrolbot_bridge.tuning_nav2 import candidate_for_job
from patrolbot_bridge.tuning_startup import navigation_boot
from patrolbot_bridge.tuning_store import StoreUnavailable
from patrolbot_bridge.tuning_yaml import strict_yaml_load
from patrolbot_bridge import tuning_nav2
from patrolbot_bridge.tuning_backend import RosParameterTransport
from test_tuning_simple import setup, ACTOR, ROOT

SPEC = importlib.util.spec_from_file_location('offline_navigation_supervisor', ROOT / 'docker/tuning-host/navigation_supervisor.py')
HOST = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HOST)
JOB = '11111111-1111-4111-8111-111111111111'


@pytest.fixture
def rig(tmp_path):
    store, _, _, _, coordinator = setup(tmp_path, [{'id': 'amcl.pf_z', 'value': 1.2}])
    plan = coordinator.preview()
    job = coordinator._journal_job(JOB, 'apply', 2, 2, ACTOR, plan['digest'], 'running',
                                   'offline restart slice', execution_plan=plan)
    store.record_job(dict(job, mutation_started=True, input_gate_attempted=True))
    value = {key: job[key] for key in ('job_id', 'digest', 'plan_hash')}
    identity = {'container_id': 'c' * 64, 'started_at': 'before', 'image': 'sha256:' + 'd' * 64,
                'state_directory': str(store.base), 'core': {'container_id': 'e' * 64, 'started_at': 'core-before'}}
    calls = []
    def restart():
        calls.append('restart-navigation-only')
        identity['started_at'] = 'after'
    driver = NS(identity=lambda: copy.deepcopy(identity), restart=restart)
    control = tmp_path / 'host-control'
    control.mkdir()
    supervisor = HOST.Supervisor(store, driver, control)
    registry = tmp_path / 'registry.json'
    registry.write_text(json.dumps(store.registry))
    default = ROOT / 'ros2_ws/src/patrolbot_navigation/config/nav2_params.yaml'
    return NS(store=store, plan=plan, value=value, supervisor=supervisor, driver=driver,
              identity=identity, calls=calls, control=control, registry=registry, default=default)


def test_navigation_only_dispatch_boots_exact_candidate_once_without_promoting_or_restarting_core(rig):
    assert rig.supervisor.boot() == {'ok': True, 'state': 'active'}
    dispatched = rig.supervisor.restart(rig.value)
    assert dispatched['state'] == 'dispatching'
    reply = rig.supervisor.boot()
    assert reply['state'] == 'candidate' and reply['revision'] == 2
    path = navigation_boot(rig.default, state_dir=rig.store.base,
                           registry_file=rig.registry, boot_reply=reply)
    try:
        assert strict_yaml_load(Path(path).read_text())['amcl']['ros__parameters']['pf_z'] == 1.2
    finally:
        Path(path).unlink()
    assert rig.supervisor.boot() == {'ok': True, 'state': 'active'}
    assert rig.supervisor.restart(rig.value)['state'] == 'boot_consumed'
    assert rig.calls == ['restart-navigation-only']
    assert rig.store.read_startup()['revision'] == 1


def test_ordinary_boot_after_consumed_candidate_still_uses_committed_active(rig):
    rig.supervisor.restart(rig.value)
    rig.supervisor.boot()
    reply = rig.supervisor.boot()
    path = navigation_boot(rig.default, state_dir=rig.store.base, registry_file=rig.registry, boot_reply=reply)
    try:
        assert strict_yaml_load(Path(path).read_text())['amcl']['ros__parameters']['pf_z'] != 1.2
    finally:
        Path(path).unlink()


def test_candidate_read_does_not_wait_on_the_live_coordinators_writer(rig):
    # This is the same file API the separate helper/boot process uses; it must
    # not acquire a reader lock while Apply waits under its writer lock.
    rig.store.reader = lambda: pytest.fail('candidate read must not acquire the Apply writer')
    with rig.store.writer():
        assert candidate_for_job(rig.store, rig.value)['revision'] == 2


@pytest.mark.parametrize('field,value', [('state', 'unknown'), ('mutation_started', False),
    ('input_gate_attempted', False), ('digest', 'sha256:' + 'f' * 64), ('plan_hash', 'sha256:' + 'f' * 64)])
def test_no_current_exact_running_job_means_no_dispatch(rig, field, value):
    rig.store.record_job(dict(rig.store.latest_job(), **{field: value}))
    with pytest.raises((ValueError, StoreUnavailable)):
        rig.supervisor.restart(rig.value)
    assert rig.calls == []


def test_a_lost_docker_reply_is_consumed_not_retried_after_helper_restart(rig):
    def timeout():
        rig.calls.append('one-ambiguous-restart')
        raise TimeoutError('Docker may have restarted navigation')
    rig.driver.restart = timeout
    with pytest.raises(TimeoutError):
        rig.supervisor.restart(rig.value)
    replacement = HOST.Supervisor(rig.store, rig.driver, rig.control)
    assert replacement.restart(rig.value)['state'] == 'dispatching'
    assert rig.calls == ['one-ambiguous-restart']


@pytest.mark.parametrize('field,value', [('container_id', 'foreign'), ('image', 'sha256:' + 'e' * 64),
                                       ('started_at', 'before')])
def test_old_or_unrelated_container_cannot_consume_candidate_boot(rig, field, value):
    rig.supervisor.restart(rig.value)
    rig.identity[field] = value
    with pytest.raises(ValueError):
        rig.supervisor.boot()


@pytest.mark.parametrize('field,value', [('revision', 1), ('artifact', 'amcl: {}'),
                                       ('digest', 'sha256:' + 'e' * 64), ('boot_id', 'latest')])
def test_boot_replies_cannot_substitute_desired_latest_or_another_candidate(rig, field, value):
    rig.supervisor.restart(rig.value)
    reply = dict(rig.supervisor.boot(), **{field: value})
    with pytest.raises((ValueError, StoreUnavailable)):
        navigation_boot(rig.default, state_dir=rig.store.base, registry_file=rig.registry, boot_reply=reply)


def test_fixed_host_driver_has_no_shell_build_pull_reset_or_arbitrary_container_argument():
    driver = HOST.FixedNavigationDocker('sha256:' + 'a' * 64)
    calls = []
    driver.run = lambda arguments, timeout: calls.append((arguments, timeout))
    driver.restart()
    assert calls == [(['restart', '--time', '10', 'patrolbot-navigation'], 60)]


def test_unknown_job_ordinary_boot_does_not_reuse_a_staged_candidate(rig):
    rig.supervisor.restart(rig.value)
    rig.store.record_job(dict(rig.store.latest_job(), state='unknown'))
    assert rig.supervisor.boot() == {'ok': True, 'state': 'active'}


def test_a_host_reboot_never_reuses_an_interrupted_candidate_permission(rig):
    rig.supervisor.restart(rig.value)
    replacement = HOST.Supervisor(rig.store, rig.driver, rig.control)
    replacement.host_boot_id = '22222222-2222-4222-8222-222222222222'
    assert replacement.boot() == {'ok': True, 'state': 'active'}
    assert rig.store.read_startup()['revision'] == 1
    assert rig.calls == ['restart-navigation-only']


def test_a_core_restart_cannot_turn_an_ordinary_boot_into_an_old_apply(rig):
    rig.supervisor.restart(rig.value)
    rig.identity['core']['started_at'] = 'core-restarted'
    assert rig.supervisor.boot() == {'ok': True, 'state': 'active'}
    assert rig.store.read_startup()['revision'] == 1


def test_running_legacy_job_without_a_plan_does_not_authorize_candidate_startup(rig):
    rig.store.record_job(dict(rig.store.latest_job(), execution_plan=None, plan_hash=None))
    assert rig.supervisor.boot() == {'ok': True, 'state': 'active'}
    assert rig.calls == []


def transport(rig, monkeypatch):
    instance = object.__new__(RosParameterTransport)
    calls = []
    instance.input_gate_closed = lambda owner: owner == rig.value
    instance.node = NS(_tuning_store=rig.store, _tuning_require_seed=lambda identity: calls.append(('require_seed', identity)))
    instance.navigation_maintenance = lambda held: calls.append(('seed_goal_hold', held))
    instance._wait_replacement_readiness = lambda deadline, owner: calls.append(('read_only_readiness',))
    physical = {'input_blocked': True, 'release_required': True, 'policy_version': 1,
                'operator_linear_trim': False, 'operator_angular_trim': False, 'policy': {},
                'buffers': dict.fromkeys(('linear_target', 'angular_target', 'linear_command', 'angular_command'), 0.)}
    instance.maintenance = lambda peer, operation, owner, **kwargs: calls.append((peer, operation, kwargs)) or physical
    instance._simple_before_caches = {'physical': {'policy': {'max_linear': .1}, 'operator_linear_trim': True}}
    instance._call = lambda *args: NS(success=True, message='1:2:100:200')
    instance.types = {'map_ack': NS(Request=lambda: NS())}
    def request(path, value, **kwargs):
        assert path == tuning_nav2.CORE_SOCKET
        calls.append(('host', value['op']))
        if value['op'] == 'restart':
            assert rig.store.navigation_admission()['pending']
            rig.supervisor.restart(value['context'])
            rig.supervisor.boot()
        return rig.supervisor.status(value['context'])
    monkeypatch.setattr(tuning_nav2, 'request', request)
    return instance, calls


def test_real_restart_adapter_keeps_core_up_reacquires_inputs_and_persists_exact_seed_barrier(rig, monkeypatch):
    instance, calls = transport(rig, monkeypatch)
    instance.restart_navigation(rig.plan, rig.value)
    assert rig.calls == ['restart-navigation-only']
    assert calls.index(('seed_goal_hold', True)) < calls.index(('host', 'restart'))
    assert ('navigation', 'acquire', {}) in calls and ('physical', 'block', {}) in calls
    assert ('physical', 'restore_trims', {'payload': {'max_linear': .1}}) in calls
    assert rig.store.navigation_admission() == {'pending': True, 'revision': 2,
        'digest': rig.value['digest'], 'map_identity': '1:2:100:200'}
    assert instance._simple_restart_boot['state'] == 'boot_consumed'
    assert rig.store.read_startup()['revision'] == 1  # verification/commit is a separate boundary


def test_restart_timeout_never_launches_another_host_operation_or_releases_input(rig, monkeypatch):
    instance, calls = transport(rig, monkeypatch)
    def lost_reply(*args, **kwargs):
        calls.append(('host', 'restart'))
        raise TimeoutError('restart may have occurred')
    monkeypatch.setattr(tuning_nav2, 'request', lost_reply)
    with pytest.raises(TimeoutError):
        instance.restart_navigation(rig.plan, rig.value)
    assert calls.count(('host', 'restart')) == 1
    assert not any('release' in call for call in calls)
    assert rig.store.navigation_admission()['pending']


def test_boot_only_socket_cannot_request_a_restart_or_arbitrary_operation(rig, tmp_path):
    import threading
    path = tmp_path / 'boot.sock'
    server = HOST.Server(path, rig.supervisor, boot_only=True)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    try:
        with pytest.raises(RuntimeError):
            tuning_nav2.request(path, {'op': 'restart', 'context': rig.value})
        assert tuning_nav2.request(path, {'op': 'boot'}) == {'ok': True, 'state': 'active'}
        assert rig.calls == []
    finally:
        server.shutdown()
        thread.join(timeout=3.)
        server.server_close()


@pytest.mark.parametrize('fault', [None, 'image', 'service', 'project', 'command', 'entrypoint',
                                 'writable', 'volume', 'stopped', 'boot_socket'])
def test_production_driver_requires_the_pinned_image_launcher_and_read_only_fixed_volume(fault):
    image = 'sha256:' + 'a' * 64
    document = {'Id': 'c' * 64, 'Name': '/patrolbot-navigation', 'Image': image,
        'Config': {'Entrypoint': ['/entrypoint.sh'],
                   'Labels': {'com.docker.compose.service': 'navigation', 'com.docker.compose.project': 'patrolbot'},
                   'Cmd': ['ros2', 'run', 'patrolbot_bridge', 'tuning_run', '--role', 'navigation']},
        'State': {'Running': True, 'StartedAt': 'new-start'},
        'Mounts': [{'Destination': '/var/lib/patrolbot', 'Type': 'volume', 'RW': False,
                    'Name': 'patrolbot_patrolbot-state', 'Source': '/var/lib/docker/volumes/patrolbot_patrolbot-state/_data'},
                   {'Destination': '/run/patrolbot-tuning/boot', 'Type': 'bind', 'RW': False,
                    'Source': '/run/patrolbot-tuning/boot'}]}
    if fault == 'image': document['Image'] = 'mutable:tag'
    elif fault == 'service': document['Config']['Labels']['com.docker.compose.service'] = 'bridge'
    elif fault == 'project': document['Config']['Labels']['com.docker.compose.project'] = 'foreign'
    elif fault == 'command': document['Config']['Cmd'] = ['sh', '-c', 'arbitrary']
    elif fault == 'entrypoint': document['Config']['Entrypoint'] = ['sh', '-c']
    elif fault == 'writable': document['Mounts'][0]['RW'] = True
    elif fault == 'volume': document['Mounts'][0]['Name'] = 'other-state'
    elif fault == 'stopped': document['State']['Running'] = False
    elif fault == 'boot_socket': document['Mounts'].pop()
    driver = HOST.FixedNavigationDocker(image)
    def inspect(arguments):
        assert arguments == ['inspect', 'patrolbot-navigation', 'patrolbot-bridge']
        core = {'Id': 'e' * 64, 'Name': '/patrolbot-bridge', 'Image': image,
                'State': {'Running': True, 'StartedAt': 'core-before'},
                'Config': {'Labels': {'com.docker.compose.service': 'bridge'}}}
        return json.dumps([document, core])
    driver.run = inspect
    if fault:
        with pytest.raises(ValueError): driver.identity()
    else:
        assert driver.identity()['state_directory'].endswith('patrolbot_patrolbot-state/_data/config')
