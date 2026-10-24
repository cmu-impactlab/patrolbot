"""Host-local, single-operation Docker dispatch. No listening socket or RPC API.

The shipped Docker driver refuses dispatch until candidate-runtime integration
exists. Offline driver injection tests persistence only, not Docker acceptance.
All Docker commands are fixed argv, use a local Unix daemon and approved config;
no request supplies a path, executable, service, image, shell or Docker option.
"""

from contextlib import contextmanager
from dataclasses import dataclass
import fcntl
import json
import os
from pathlib import Path
import stat
import subprocess
import time

from . import tuning_handoff as handoff
from .tuning_host_auth import bind_handoff, verify
from .tuning_initialize import RuntimeGuard
from .tuning_store import StoreUnavailable, _atomic_write, _read_bytes, _read_json


CONTROL = Path('/var/lib/patrolbot-tuning-host')
COMPOSE = Path('/etc/patrolbot/tuning-host/compose.yaml')
ENV_FILE = Path('/etc/patrolbot/tuning-host/compose.env')
CONTAINERS = ('patrolbot-bringup', 'patrolbot-bridge', 'patrolbot-navigation')
STATE_VOLUME = 'patrolbot_patrolbot-state'


def current_host_boot_id():
    from .tuning_store import _canonical_uuid
    value = _read_bytes(Path('/proc/sys/kernel/random/boot_id'), 128).decode().strip()
    if not _canonical_uuid(value):
        raise StoreUnavailable('host_boot_identity_unavailable')
    return value


def protected_path(path, *, directory=False, _owner=0):
    """Host config/control must be local-root owned, non-symlink, non-writable."""
    path = Path(path)
    if not path.is_absolute():
        raise StoreUnavailable('host_path_not_absolute')
    for parent in (path, *path.parents):
        if parent.is_symlink():
            raise StoreUnavailable('symlinked_host_path')
    info = path.stat()
    expected = stat.S_ISDIR if directory else stat.S_ISREG
    if (not expected(info.st_mode) or info.st_uid != _owner
            or info.st_mode & (stat.S_IWGRP | stat.S_IWOTH)):
        raise StoreUnavailable('unprotected_host_path')
    return path


@dataclass(frozen=True)
class LocalPolicy:
    """Loaded/approved locally, never deserialized from a dashboard request."""
    config_digest: str
    volume_id: str
    host_boot_id: str
    enabled: bool = False


class FixedDockerDriver:
    """Fixed root-owned Compose snapshot, fixed scope, no builds/pulls/resets.

    Not yet runnable: core/F710 candidate policy consumption and launch identity
    authentication are missing in the shipped runtime. Do not change this gate
    based on unit mocks, Docker exit status, or operator enablement alone.
    """
    candidate_runtime_integrated = False

    def __init__(self, *, pinned_image):
        if not handoff.is_digest(pinned_image):
            raise StoreUnavailable('pinned_local_image_required')
        self.pinned_image = pinned_image

    @staticmethod
    def _compose_prefix():
        return ['/usr/bin/docker', '--host', 'unix:///var/run/docker.sock',
                'compose', '--project-name', 'patrolbot', '--env-file', str(ENV_FILE),
                '-f', str(COMPOSE)]

    @staticmethod
    def command():
        return [*FixedDockerDriver._compose_prefix(),
                'up', '-d', '--no-build', '--pull', 'never',
                '--no-deps', '--force-recreate', *handoff.SERVICES]

    @staticmethod
    def _run(arguments, *, timeout=30):
        # No ambient DOCKER_HOST/CONTEXT, Compose, PATH or user credentials.
        return subprocess.run(arguments, check=True, capture_output=True, text=True,
                              timeout=timeout, env={'PATH': '/usr/bin:/bin', 'HOME': '/root'},
                              cwd='/', shell=False)

    def preflight(self, store, policy):
        if self.candidate_runtime_integrated is not True:
            raise StoreUnavailable('candidate_runtime_integration_unavailable')
        if current_host_boot_id() != policy.host_boot_id:
            raise StoreUnavailable('host_boot_identity_changed')
        protected_path(COMPOSE)
        protected_path(ENV_FILE)
        actual_digest = handoff.digest({
            'compose': _read_bytes(COMPOSE, 512 * 1024).decode(),
            'environment': _read_bytes(ENV_FILE, 128 * 1024).decode(),
        })
        if actual_digest != policy.config_digest:
            raise StoreUnavailable('approved_host_config_changed')
        rendered = json.loads(self._run([
            *self._compose_prefix(), 'config', '--format', 'json']).stdout)
        # Never let a mutable local tag resolve to a different image at recreation.
        # A reviewed config snapshot must select the exact existing local image ID.
        for service in handoff.SERVICES:
            if rendered.get('services', {}).get(service, {}).get('image') != self.pinned_image:
                raise StoreUnavailable('approved_compose_image_not_pinned')
        prefix = ['/usr/bin/docker', '--host', 'unix:///var/run/docker.sock']
        volume = json.loads(self._run([*prefix, 'volume', 'inspect', STATE_VOLUME]).stdout)
        if (len(volume) != 1 or volume[0].get('Name') != STATE_VOLUME
                or volume[0].get('Driver') != 'local' or volume[0].get('Options')
                or Path(volume[0].get('Mountpoint', '')) / 'config' != store.base):
            raise StoreUnavailable('unapproved_tuning_volume')
        containers = json.loads(self._run([*prefix, 'inspect', *CONTAINERS]).stdout)
        if len(containers) != len(CONTAINERS):
            raise StoreUnavailable('fixed_container_inventory_changed')
        for container, name in zip(containers, CONTAINERS):
            if (container.get('Name') != '/' + name
                    or container.get('Image') != self.pinned_image
                    or container.get('Config', {}).get('Labels', {}).get(
                        'com.docker.compose.project') != 'patrolbot'):
                raise StoreUnavailable('fixed_container_identity_changed')
            mounts = [m for m in container.get('Mounts', [])
                      if m.get('Destination') == '/var/lib/patrolbot']
            if (len(mounts) != 1 or mounts[0].get('Name') != STATE_VOLUME
                    or mounts[0].get('Type') != 'volume'
                    or mounts[0].get('Source') != volume[0]['Mountpoint']):
                raise StoreUnavailable('fixed_container_volume_changed')

    def dispatch(self):
        self._run(self.command(), timeout=120)


class HostDispatcher:
    """Durably consume one exact authorization BEFORE any possible mutation.

    admission is a trusted adapter to existing command/lifecycle/hardware owners,
    not a Boolean from a request. None refuses. Tests inject named offline fakes.
    Caller provisions control directory/public keys externally; no mkdir/install.
    """

    def __init__(self, store, *, public_key, policy, driver=None, admission=None,
                 control_dir=CONTROL, clock=time.time, _owner=0):
        self.store = store
        self.public_key = public_key
        self.policy = policy
        self.driver = driver
        self.admission = admission
        self.control = Path(control_dir)
        self.clock = clock
        self._owner = _owner

    @contextmanager
    def _lock(self):
        protected_path(self.control, directory=True, _owner=self._owner)
        path = self.control / 'lock'
        fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NONBLOCK
                     | getattr(os, 'O_NOFOLLOW', 0), 0o600)
        try:
            info = os.fstat(fd)
            if (not stat.S_ISREG(info.st_mode) or info.st_uid != self._owner
                    or info.st_mode & 0o077):
                raise StoreUnavailable('invalid_host_dispatch_lock')
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            yield
        finally:
            os.close(fd)

    def dispatch(self, *, execution_authorization, boot_authorization):
        if self.policy.enabled is not True:
            raise StoreUnavailable('host_docker_disabled')
        if self.driver is None or self.admission is None:
            raise StoreUnavailable('host_runtime_admission_unavailable')
        # Shared consumer exclusion survives the store-lock release during Docker.
        # An offline rollback cannot race a dispatch/startup in another process.
        with self._lock(), RuntimeGuard(self.store.base):
            return self._dispatch_locked(execution_authorization, boot_authorization)

    def _dispatch_locked(self, execution_authorization, boot_authorization):
        with self.store.writer():
            now = self.clock()
            claims = verify(execution_authorization, self.public_key,
                            kind='execute_robot_docker', now=now,
                            host_boot_id=self.policy.host_boot_id)
            boot = verify(boot_authorization, self.public_key, kind='candidate_boot',
                          now=now, host_boot_id=self.policy.host_boot_id)
            staged = handoff.read(self.store, job_id=claims['job_id'])
            for grant in (claims, boot):
                bind_handoff(grant, staged)
                if (grant['config_digest'] != self.policy.config_digest
                        or grant['volume_id'] != self.policy.volume_id):
                    raise StoreUnavailable('host_local_policy_binding_changed')
            if (boot['execution_authorization_id'] != claims['authorization_id']
                    or boot['authorization_id'] == claims['authorization_id']):
                raise StoreUnavailable('distinct_exact_boot_authorization_required')
            receipt_path = self.control / (claims['job_id'] + '.json')
            if receipt_path.exists() or receipt_path.is_symlink():
                raise StoreUnavailable('host_dispatch_already_consumed_status_or_recovery_only')
            job = self.store.get_job(claims['job_id'])
            if (job.get('state') != 'running' or job.get('mutation_started') is not False
                    or job['docker_handoff'].get('state') != 'staged'):
                raise StoreUnavailable('host_dispatch_not_fresh')
            self.driver.preflight(self.store, self.policy)
            evidence = self.admission(staged)
            if evidence != {
                    'job_id': claims['job_id'], 'handoff_digest': claims['handoff_digest'],
                    'motion_paths_held': True, 'command_idle': True,
                    'mutation_owners_reserved': True}:
                raise StoreUnavailable('host_command_hold_not_acknowledged')
            # Recheck expiry after potentially slow admission, before journalling.
            verify(execution_authorization, self.public_key, kind='execute_robot_docker',
                   now=self.clock(), host_boot_id=self.policy.host_boot_id)
            verify(boot_authorization, self.public_key, kind='candidate_boot',
                   now=self.clock(), host_boot_id=self.policy.host_boot_id)
            receipt = {
                'version': 1, 'state': 'dispatching', 'job_id': claims['job_id'],
                'boot_id': boot['boot_id'], 'authorization_id': claims['authorization_id'],
                'boot_authorization_id': boot['authorization_id'],
                'handoff_digest': claims['handoff_digest'],
                'config_digest': claims['config_digest'], 'volume_id': claims['volume_id'],
                'host_boot_id': claims['host_boot_id'],
            }
            _atomic_write(self.control / (claims['job_id'] + '.boot.json'),
                          handoff.canonical_bytes(boot_authorization))
            # Consumed even if the process crashes immediately afterward. No retry.
            _atomic_write(receipt_path, handoff.canonical_bytes(receipt))
            self.store.record_job(dict(job, mutation_started=True, docker_handoff=dict(
                job['docker_handoff'], state='dispatching', boot_id=boot['boot_id'],
                authorization_id=claims['authorization_id'])))
            self.store._audit({'event': 'docker_dispatch_boundary', 'job_id': claims['job_id'],
                               'authorization_id': claims['authorization_id'],
                               'at': self.clock()})
        # Never hold the store writer while Docker waits for replacement runtimes.
        # The durable receipt excludes any second dispatch, even in another process.
        try:
            self.driver.dispatch()
        except BaseException:
            handoff.mark_unknown(self.store, job_id=claims['job_id'],
                                 detail='Host Docker outcome unknown; explicit recovery required.')
            raise
        return {'job_id': claims['job_id'], 'outcome': 'awaiting_runtime_verification',
                'hold_required': True, 'runtime_verified': False}

    def status(self, *, job_id):
        # Reuses canonical UUID/path validation; never returns a retry instruction.
        handoff._path(self.store, job_id)
        path = self.control / (job_id + '.json')
        with self._lock():
            if not path.exists() and not path.is_symlink():
                return {'job_id': job_id, 'outcome': 'not_dispatched', 'runtime_verified': False}
            receipt = _read_json(path)
            return {'job_id': job_id, 'outcome': 'dispatched_or_interrupted',
                    'receipt': receipt, 'hold_required': True,
                    'runtime_verified': False, 'next_action': 'status_or_explicit_recovery'}
