"""Offline protocol/crash fixtures, NOT native/container acceptance evidence."""

from dataclasses import replace
import copy
import json
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace
import uuid

from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
import pytest

from patrolbot_bridge import tuning_handoff as handoff
from patrolbot_bridge import tuning_host as host
from patrolbot_bridge import tuning_host_auth as auth
from patrolbot_bridge.tuning_candidate_boot import authorize
from patrolbot_bridge.tuning_recover import recover
from patrolbot_bridge.tuning_startup import resolve, resolve_candidate
from patrolbot_bridge.tuning_store import RevisionStore, StoreUnavailable
from patrolbot_bridge.tuning_yaml import strict_yaml_load


ROOT = Path(__file__).resolve().parents[4]
REGISTRY_PATH = ROOT / 'contracts/robot-tuning.registry.json'
REGISTRY = json.loads(REGISTRY_PATH.read_text())
ADMIN = {'id': 1, 'username': 'offline-fixture', 'role': 'administrator'}
JOB = '22222222-2222-4222-8222-222222222222'


def new_id():
    return str(uuid.uuid4())


class OfflineDockerDriver:
    """Only count calls/fault injection. Never executes Docker or ROS."""
    def __init__(self):
        self.calls = 0
        self.error = None
        self.during_dispatch = None

    def preflight(self, store, policy):
        pass

    def dispatch(self):
        self.calls += 1
        if self.during_dispatch:
            self.during_dispatch()
        if self.error:
            raise self.error


@pytest.fixture
def fixture(tmp_path):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize(actor=ADMIN, reason='offline handoff fixture')
    store.save(request_id=new_id(), base_revision=1, actor=ADMIN,
               changes=[{'id': 'max_linear_m_s', 'value': .1}])
    active, candidate = store.load_revision(1), store.load_revision(2)
    # Fabricated review solely for protocol binding tests; no real support claim.
    plan = {
        'execution_kind': 'robot_docker', 'services': list(handoff.SERVICES),
        'verification_scopes': ['bringup', 'navigation'], 'seed_required': True,
        'revision': 2, 'digest': candidate['meta']['generated_digest'],
        'base_revision': 1, 'base_digest': active['meta']['generated_digest'],
        'desired_revision': 2, 'executable': True, 'reasons': [],
    }
    plan['plan_hash'] = handoff.digest(plan)
    store.record_job({'job_id': JOB, 'state': 'running', 'revision': 2,
                      'digest': plan['digest'], 'plan_hash': plan['plan_hash'],
                      'execution_plan': plan, 'mutation_started': False})
    store._write_manifest(dict(store.require_ready(), latest_job=JOB))
    staged = handoff.stage(store, job_id=JOB)
    key = Ed25519PrivateKey.generate()
    policy = host.LocalPolicy(config_digest=handoff.digest('offline config'),
                              volume_id=new_id(), host_boot_id=new_id(), enabled=True)
    common = {
        'version': 1, 'authorization_id': new_id(), 'kind': 'execute_robot_docker',
        **{k: staged[k] for k in ('job_id', 'revision', 'digest', 'plan_hash',
                                 'handoff_digest', 'store_generation', 'services')},
        'config_digest': policy.config_digest, 'volume_id': policy.volume_id,
        'host_boot_id': policy.host_boot_id, 'issued_at': 1000, 'expires_at': 1100,
    }
    boot = dict(common, kind='candidate_boot', authorization_id=new_id(), boot_id=new_id(),
                execution_authorization_id=common['authorization_id'])
    control = tmp_path / 'host-control'
    control.mkdir(mode=0o700)
    driver = OfflineDockerDriver()

    def offline_admission(staged):
        return {'job_id': staged['job_id'], 'handoff_digest': staged['handoff_digest'],
                'motion_paths_held': True, 'command_idle': True,
                'mutation_owners_reserved': True}

    dispatcher = host.HostDispatcher(store, public_key=key.public_key(), policy=policy,
                                     driver=driver, admission=offline_admission,
                                     control_dir=control, clock=lambda: 1001,
                                     _owner=os.geteuid())
    return {'store': store, 'staged': staged, 'key': key, 'execute': common, 'boot': boot,
            'driver': driver, 'dispatcher': dispatcher, 'control': control, 'policy': policy}


def dispatch(f):
    return f['dispatcher'].dispatch(
        execution_authorization=auth.sign(f['execute'], f['key']),
        boot_authorization=auth.sign(f['boot'], f['key']))


def candidate(f, scope='bringup', **overrides):
    values = {'scope': scope, 'authorization': auth.sign(f['boot'], f['key']),
              'public_key': f['key'].public_key(),
              'launch_receipt': json.loads((f['control'] / (JOB + '.json')).read_text()),
              'now': 1002, 'host_boot_id': f['policy'].host_boot_id}
    return authorize(f['store'], **dict(values, **overrides))


def test_staging_is_immutable_and_never_startup_permission(fixture, tmp_path):
    f = fixture
    store = f['store']
    before = {str(p): p.read_bytes() for p in store.revisions_dir.rglob('*') if p.is_file()}
    assert handoff.stage(store, job_id=JOB) == f['staged']
    assert store.read_startup()['revision'] == 1
    assert store.require_ready()['desired_revision'] == 2
    assert store.require_ready()['last_known_good'] == 1
    assert before == {str(p): p.read_bytes() for p in store.revisions_dir.rglob('*') if p.is_file()}
    # Even a real signed boot grant does not authorize pre-dispatch startup.
    with pytest.raises(StoreUnavailable, match='receipt'):
        authorize(store, scope='bringup', authorization=auth.sign(f['boot'], f['key']),
                  public_key=f['key'].public_key(), launch_receipt={}, now=1001,
                  host_boot_id=f['policy'].host_boot_id)


@pytest.mark.parametrize('field,value', [
    ('services', ['bringup', 'bridge', 'navigation', 'web-bridge']),
    ('execution_kind', 'scoped_parameters'), ('seed_required', False),
    ('verification_scopes', ['bringup']), ('executable', False),
])
def test_unreviewed_or_expanded_scope_refuses(fixture, field, value):
    f = fixture
    job = f['store'].get_job(JOB)
    job['execution_plan'][field] = value
    job['execution_plan']['plan_hash'] = handoff.digest({
        k: v for k, v in job['execution_plan'].items() if k != 'plan_hash'})
    job['plan_hash'] = job['execution_plan']['plan_hash']
    f['store'].record_job(job)
    with pytest.raises(StoreUnavailable):
        dispatch(f)
    assert f['driver'].calls == 0


@pytest.mark.parametrize('corruption', ['generated', 'staging', 'job', 'missing_volume', 'symlink'])
def test_lost_or_corrupt_state_refuses_without_default_fallback(fixture, corruption):
    f = fixture
    store = f['store']
    if corruption == 'generated':
        (store.revision_dir(2) / 'generated/bringup.yaml').write_text('corrupt')
    elif corruption == 'staging':
        handoff._path(store, JOB).write_text('{}')
    elif corruption == 'job':
        store.record_job(dict(store.get_job(JOB), docker_handoff={}))
    elif corruption == 'missing_volume':
        store.initialized_path.unlink()
    else:
        original = handoff._path(store, JOB)
        target = original.with_suffix('.backup')
        original.rename(target)
        original.symlink_to(target)
    with pytest.raises(StoreUnavailable):
        dispatch(f)
    assert f['driver'].calls == 0


@pytest.mark.parametrize('change', [
    {'digest': 'sha256:' + '0' * 64}, {'plan_hash': 'sha256:' + '0' * 64},
    {'handoff_digest': 'sha256:' + '0' * 64}, {'store_generation': str(uuid.uuid4())},
    {'job_id': str(uuid.uuid4())}, {'revision': 1}, {'revision': True},
    {'config_digest': 'sha256:' + '0' * 64}, {'volume_id': str(uuid.uuid4())},
    {'host_boot_id': str(uuid.uuid4())}, {'services': ['bringup']},
    {'shell': 'reboot'}, {'expires_at': 1001}, {'issued_at': 1002},
    {'expires_at': 1400}, {'expires_at': float('inf')}, {'version': True},
])
def test_signed_but_wrong_exact_authorization_never_dispatches(fixture, change):
    f = fixture
    f['execute'].update(change)
    with pytest.raises((StoreUnavailable, ValueError)):
        dispatch(f)
    assert f['driver'].calls == 0
    assert f['store'].read_startup()['revision'] == 1


def test_wrong_key_tamper_and_capability_kind_are_rejected(fixture):
    f = fixture
    envelope = auth.sign(f['execute'], Ed25519PrivateKey.generate())
    with pytest.raises(StoreUnavailable, match='signature'):
        f['dispatcher'].dispatch(execution_authorization=envelope,
                                 boot_authorization=auth.sign(f['boot'], f['key']))
    envelope = auth.sign(copy.deepcopy(f['execute']), f['key'])
    envelope['claims']['revision'] = 1
    with pytest.raises(StoreUnavailable, match='signature'):
        auth.verify(envelope, f['key'].public_key(), kind='execute_robot_docker', now=1001,
                    host_boot_id=f['policy'].host_boot_id)
    with pytest.raises(StoreUnavailable, match='authorization'):
        auth.verify(auth.sign(f['boot'], f['key']), f['key'].public_key(),
                    kind='execute_robot_docker', now=1001, host_boot_id=f['policy'].host_boot_id)


def test_distinct_exact_boot_permission_required(fixture):
    f = fixture
    f['boot']['execution_authorization_id'] = new_id()
    with pytest.raises(StoreUnavailable, match='distinct_exact'):
        dispatch(f)
    assert f['driver'].calls == 0


def test_successful_fake_dispatch_is_not_verified_activation_or_retry_permission(fixture):
    f = fixture
    result = dispatch(f)
    assert result == {'job_id': JOB, 'outcome': 'awaiting_runtime_verification',
                      'hold_required': True, 'runtime_verified': False}
    assert f['store'].get_job(JOB)['state'] == 'running'
    assert f['store'].get_job(JOB)['mutation_started'] is True
    assert f['store'].read_startup()['revision'] == 1
    assert f['store'].require_ready()['last_known_good'] == 1
    assert f['dispatcher'].status(job_id=JOB)['next_action'] == 'status_or_explicit_recovery'
    # Even new signed grants for the same job cannot bypass the durable receipt.
    f['execute']['authorization_id'] = new_id()
    f['boot']['execution_authorization_id'] = f['execute']['authorization_id']
    with pytest.raises(StoreUnavailable, match='already_consumed'):
        dispatch(f)
    assert f['driver'].calls == 1


@pytest.mark.parametrize('error', [TimeoutError('offline timeout'), OSError('offline failure'),
                                  KeyboardInterrupt()])
def test_possible_partial_mutation_stays_unknown_held_and_never_retries(fixture, error):
    f = fixture
    f['driver'].error = error
    with pytest.raises(type(error)):
        dispatch(f)
    assert f['store'].get_job(JOB)['state'] == 'unknown'
    assert f['store'].get_job(JOB)['docker_handoff']['hold_required'] is True
    assert f['store'].read_startup()['revision'] == 1
    with pytest.raises(StoreUnavailable, match='already_consumed'):
        dispatch(f)
    assert f['driver'].calls == 1
    with pytest.raises(StoreUnavailable, match='not_dispatched'):
        candidate(f)


def test_crash_after_host_receipt_before_job_write_is_consumed_not_retried(fixture, monkeypatch):
    f = fixture
    original = f['store'].record_job

    def crash(job):
        raise OSError('offline simulated process loss after receipt fsync')

    monkeypatch.setattr(f['store'], 'record_job', crash)
    with pytest.raises(OSError):
        dispatch(f)
    monkeypatch.setattr(f['store'], 'record_job', original)
    assert (f['control'] / (JOB + '.json')).exists()
    assert f['driver'].calls == 0
    assert f['store'].get_job(JOB)['docker_handoff']['hold_required'] is True
    with pytest.raises(StoreUnavailable, match='already_consumed'):
        dispatch(f)
    with pytest.raises(StoreUnavailable, match='not_dispatched'):
        candidate(f)


@pytest.mark.parametrize('scope', ['bringup', 'navigation'])
def test_explicit_candidate_reads_exact_revision_under_retained_hold(fixture, scope, monkeypatch):
    f = fixture
    dispatch(f)
    # Candidate readers must work without opening writer files on RO mounts.
    monkeypatch.setattr(f['store'], 'writer', lambda: pytest.fail('candidate used writable lock'))
    result = candidate(f, scope)
    assert result['artifact'] == f['store'].load_revision(2)['generated'][scope + '.yaml']
    assert result['hold_required'] is True
    assert result['fresh_operator_seed_required'] is True
    assert f['store'].read_startup()['revision'] == 1


def test_ordinary_restart_is_active_only_even_with_valid_leftover_boot_grant(fixture, tmp_path,
                                                                         monkeypatch):
    f = fixture
    dispatch(f)
    default = ROOT / 'ros2_ws/src/patrolbot-launch/param/defaults/smoother.yaml'
    monkeypatch.setenv('PATROLBOT_TUNING_CANDIDATE_BOOT', '1')
    ordinary = resolve(default, 'bringup', enabled=True, state_dir=f['store'].base,
                       registry_file=REGISTRY_PATH)
    try:
        assert strict_yaml_load(Path(ordinary).read_text()) == strict_yaml_load(
            f['store'].load_revision(1)['generated']['bringup.yaml'])
        receipt = json.loads((f['control'] / (JOB + '.json')).read_text())
        path, binding = resolve_candidate(
            default, 'bringup', store=f['store'], authorization=auth.sign(f['boot'], f['key']),
            public_key=f['key'].public_key(), launch_receipt=receipt, now=1002,
            host_boot_id=f['policy'].host_boot_id)
        try:
            assert strict_yaml_load(Path(path).read_text()) == strict_yaml_load(
                f['store'].load_revision(2)['generated']['bringup.yaml'])
            assert binding['hold_required'] is True
        finally:
            Path(path).unlink()
    finally:
        Path(ordinary).unlink()


@pytest.mark.parametrize('field', ['boot_id', 'authorization_id', 'boot_authorization_id',
                                 'handoff_digest', 'volume_id', 'config_digest', 'host_boot_id'])
def test_foreign_replacement_runtime_receipt_cannot_boot(fixture, field):
    f = fixture
    dispatch(f)
    receipt = json.loads((f['control'] / (JOB + '.json')).read_text())
    receipt[field] = 'foreign'
    with pytest.raises(StoreUnavailable, match='receipt'):
        candidate(f, launch_receipt=receipt)


def test_boot_grant_expiry_is_not_fallback_to_active_or_defaults(fixture):
    f = fixture
    dispatch(f)
    with pytest.raises(StoreUnavailable, match='expired'):
        candidate(f, now=1100)


def test_recovery_settles_job_invalidates_leftover_boot_grants_preserves_history(fixture):
    f = fixture
    dispatch(f)
    staged_before = handoff._path(f['store'], JOB).read_bytes()
    job = recover(f['store'].base, REGISTRY, job_id=JOB, revision=1,
                  digest=f['store'].load_revision(1)['meta']['generated_digest'],
                  reason='offline operator rollback', confirm_stopped=True, _effective_uid=0)
    assert job['recovery']['runtime_verified'] is False
    assert f['store'].read_startup()['revision'] == 1
    assert f['store'].require_ready()['desired_revision'] == 2
    assert f['store'].require_ready()['last_known_good'] == 1
    assert handoff._path(f['store'], JOB).read_bytes() == staged_before
    with pytest.raises(StoreUnavailable, match='unsettled'):
        candidate(f)


def test_offline_recovery_cannot_race_host_docker_dispatch(fixture):
    f = fixture

    def attempt_recovery():
        with pytest.raises(BlockingIOError):
            recover(f['store'].base, REGISTRY, job_id=JOB, revision=1,
                    digest=f['store'].load_revision(1)['meta']['generated_digest'],
                    reason='offline racing rollback', confirm_stopped=True, _effective_uid=0)
    f['driver'].during_dispatch = attempt_recovery
    dispatch(f)
    assert f['store'].get_job(JOB)['state'] == 'running'


@pytest.mark.parametrize('gate', ['disabled', 'no_driver', 'no_admission', 'unknown_hold',
                                 'native_integration_missing'])
def test_default_and_unknown_gates_fail_before_host_receipt_or_mutation(fixture, gate):
    f = fixture
    d = f['dispatcher']
    if gate == 'disabled':
        d.policy = replace(d.policy, enabled=False)
    elif gate == 'no_driver':
        d.driver = None
    elif gate == 'no_admission':
        d.admission = None
    elif gate == 'unknown_hold':
        d.admission = lambda _: None
    else:
        d.driver = host.FixedDockerDriver(pinned_image='sha256:' + '1' * 64)
    with pytest.raises(StoreUnavailable):
        dispatch(f)
    assert not (f['control'] / (JOB + '.json')).exists()
    assert f['driver'].calls == 0
    assert f['store'].get_job(JOB)['mutation_started'] is False


def test_host_lock_excludes_second_process_and_reader_uses_shared_readonly_lock(fixture):
    f = fixture
    path = f['store'].lock_path
    with f['store'].reader():
        script = ('import fcntl,os,sys; fd=os.open(sys.argv[1],os.O_RDWR); '
                  'fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)')
        result = subprocess.run([os.sys.executable, '-c', script, str(path)], capture_output=True)
        assert result.returncode != 0
        assert b'BlockingIOError' in result.stderr
    with f['dispatcher']._lock():
        with pytest.raises(BlockingIOError):
            dispatch(f)
    assert f['driver'].calls == 0


def test_unprotected_control_directory_or_symlink_refuses(fixture, tmp_path):
    f = fixture
    f['control'].chmod(0o777)
    with pytest.raises(StoreUnavailable, match='unprotected'):
        dispatch(f)
    f['control'].chmod(0o700)
    link = tmp_path / 'control-link'
    link.symlink_to(f['control'], target_is_directory=True)
    f['dispatcher'].control = link
    with pytest.raises(StoreUnavailable, match='symlinked'):
        dispatch(f)


def test_actual_docker_argv_has_no_remote_shell_build_pull_or_scope_inputs():
    command = host.FixedDockerDriver.command()
    assert command[:3] == ['/usr/bin/docker', '--host', 'unix:///var/run/docker.sock']
    assert command[-3:] == list(handoff.SERVICES)
    assert '--no-build' in command and command[command.index('--pull') + 1] == 'never'
    assert '--no-deps' in command
    assert not {'build', 'pull', 'down', 'restart', 'exec', 'run', 'system', 'prune'} & set(command)
    assert host.FixedDockerDriver.candidate_runtime_integrated is False


@pytest.mark.parametrize('fault', [None, 'config', 'tag', 'volume_driver', 'volume_options',
                                 'volume_path', 'container_name', 'container_image',
                                 'container_volume'])
def test_fixed_docker_inventory_preflight_with_offline_json_only(fixture, tmp_path,
                                                               monkeypatch, fault):
    """Fake Docker JSON probes test the allowlist, NOT actual Docker acceptance."""
    f = fixture
    image = 'sha256:' + '1' * 64
    compose, env = tmp_path / 'compose.yaml', tmp_path / 'compose.env'
    compose.write_text('offline approved config')
    env.write_text('offline no secrets')
    compose.chmod(0o600)
    env.chmod(0o600)
    monkeypatch.setattr(host, 'COMPOSE', compose)
    monkeypatch.setattr(host, 'ENV_FILE', env)
    original = host.protected_path
    monkeypatch.setattr(host, 'protected_path', lambda path: original(path, _owner=os.geteuid()))
    driver = host.FixedDockerDriver(pinned_image=image)
    monkeypatch.setattr(driver, 'candidate_runtime_integrated', True)
    monkeypatch.setattr(host, 'current_host_boot_id', lambda: f['policy'].host_boot_id)
    policy = replace(f['policy'], config_digest=handoff.digest({
        'compose': compose.read_text(), 'environment': env.read_text()}))
    rendered = {'services': {service: {'image': image} for service in handoff.SERVICES}}
    volume = [{'Name': host.STATE_VOLUME, 'Driver': 'local', 'Options': None,
               'Mountpoint': str(f['store'].base.parent)}]
    containers = [{
        'Name': '/' + name, 'Image': image,
        'Config': {'Labels': {'com.docker.compose.project': 'patrolbot'}},
        'Mounts': [{'Type': 'volume', 'Name': host.STATE_VOLUME,
                    'Source': str(f['store'].base.parent), 'Destination': '/var/lib/patrolbot'}],
    } for name in host.CONTAINERS]
    if fault == 'config':
        env.write_text('changed')
    elif fault == 'tag':
        rendered['services']['bridge']['image'] = 'patrolbot:mutable'
    elif fault == 'volume_driver':
        volume[0]['Driver'] = 'remote-driver'
    elif fault == 'volume_options':
        volume[0]['Options'] = {'type': 'nfs'}
    elif fault == 'volume_path':
        volume[0]['Mountpoint'] = '/unexpected/path'
    elif fault == 'container_name':
        containers[0]['Name'] = '/foreign-container'
    elif fault == 'container_image':
        containers[0]['Image'] = 'sha256:' + '2' * 64
    elif fault == 'container_volume':
        containers[0]['Mounts'][0]['Name'] = 'foreign-volume'
    calls = []

    def fake_run(arguments):
        calls.append(arguments)
        payload = (rendered if 'config' in arguments else
                   volume if 'volume' in arguments else containers)
        return SimpleNamespace(stdout=json.dumps(payload))

    monkeypatch.setattr(driver, '_run', fake_run)
    if fault:
        with pytest.raises(StoreUnavailable):
            driver.preflight(f['store'], policy)
    else:
        driver.preflight(f['store'], policy)
        assert len(calls) == 3
    assert all(command[:3] == ['/usr/bin/docker', '--host', 'unix:///var/run/docker.sock']
               for command in calls)
    assert not any('up' in command for command in calls)
