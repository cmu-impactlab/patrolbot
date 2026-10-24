#!/usr/bin/env python3
"""Host-only navigation restart and single-use replacement boot.

Two filesystem-scoped sockets: core may request restart/status; navigation may
only ask for boot data. Neither is mounted into the dashboard/web bridge. All
Docker argv and container names are fixed. No secrets, shell, builds or SBC I/O.
"""

import json
import os
import re
from pathlib import Path
import socketserver
import subprocess
import threading
from uuid import uuid4

from patrolbot_bridge.tuning_nav2 import candidate_for_job, MAX_REPLY
from patrolbot_bridge.tuning_lease import context
from patrolbot_bridge.tuning_store import _atomic_write, _read_json
from patrolbot_bridge.tuning_host import current_host_boot_id

DOCKER = ['/usr/bin/docker', '--host', 'unix:///var/run/docker.sock']
CONTROL = Path('/var/lib/patrolbot-tuning-host/navigation')
CONTAINER = 'patrolbot-navigation'


class FixedNavigationDocker:
    def __init__(self, image):
        if not isinstance(image, str) or not re.fullmatch(r'sha256:[0-9a-f]{64}', image):
            raise ValueError('root-approved immutable image ID required')
        self.image = image

    @staticmethod
    def run(argv, timeout=15):
        return subprocess.run([*DOCKER, *argv], check=True, capture_output=True, text=True,
            timeout=timeout, env={'PATH': '/usr/bin:/bin', 'HOME': '/root'}, cwd='/', shell=False).stdout

    def identity(self):
        containers = json.loads(self.run(['inspect', CONTAINER, 'patrolbot-bridge']))
        if not isinstance(containers, list) or len(containers) != 2:
            raise ValueError('fixed navigation container unavailable')
        container, core = containers
        config, state = container.get('Config', {}), container.get('State', {})
        mounts = [mount for mount in container.get('Mounts', []) if mount.get('Destination') == '/var/lib/patrolbot']
        boot_mounts = [mount for mount in container.get('Mounts', [])
                       if mount.get('Destination') == '/run/patrolbot-tuning/boot']
        if (container.get('Name') != '/' + CONTAINER or container.get('Image') != self.image
                or config.get('Labels', {}).get('com.docker.compose.service') != 'navigation'
                or config.get('Labels', {}).get('com.docker.compose.project') != 'patrolbot'
                or config.get('Cmd') != ['ros2', 'run', 'patrolbot_bridge', 'tuning_run', '--role', 'navigation']
                or config.get('Entrypoint') != ['/entrypoint.sh']
                or state.get('Running') is not True or not state.get('StartedAt')
                or len(mounts) != 1 or mounts[0].get('Type') != 'volume'
                or mounts[0].get('Name') != 'patrolbot_patrolbot-state' or mounts[0].get('RW') is not False
                or len(boot_mounts) != 1 or boot_mounts[0].get('Type') != 'bind'
                or boot_mounts[0].get('Source') != '/run/patrolbot-tuning/boot'
                or boot_mounts[0].get('RW') is not False):
            raise ValueError('navigation image/launcher/state identity changed')
        if (core.get('Name') != '/patrolbot-bridge' or core.get('State', {}).get('Running') is not True
                or not core.get('State', {}).get('StartedAt')
                or core.get('Config', {}).get('Labels', {}).get('com.docker.compose.service') != 'bridge'):
            raise ValueError('the existing core bridge is unavailable')
        return {'container_id': container['Id'], 'started_at': state['StartedAt'], 'image': self.image,
                'state_directory': str(Path(mounts[0]['Source']) / 'config'),
                'core': {'container_id': core['Id'], 'started_at': core['State']['StartedAt'], 'image': core['Image']}}

    def restart(self):
        self.run(['restart', '--time', '10', CONTAINER], timeout=60)


class Supervisor:
    def __init__(self, store, driver, directory=CONTROL):
        self.store, self.driver, self.directory = store, driver, Path(directory)
        self.lock = threading.RLock()
        self.host_boot_id = current_host_boot_id()

    def _path(self, value):
        return self.directory / (context(value)['job_id'] + '.json')

    def status(self, value):
        with self.lock:
            path = self._path(value)
            if not path.exists():
                return {'ok': True, 'state': 'not_dispatched', **context(value)}
            record = _read_json(path)
            if any(record.get(key) != item for key, item in context(value).items()):
                raise ValueError('foreign restart status')
            return dict(record, ok=True)

    def restart(self, value):
        value = context(value)
        with self.lock:
            path = self._path(value)
            if path.exists():
                # Durable deduplication is status only, NEVER a second restart.
                return self.status(value)
            identity = self.driver.identity()
            if identity['state_directory'] != str(self.store.base):
                raise ValueError('host state volume does not match reviewed store')
            candidate = candidate_for_job(self.store, value)
            record = dict(value, revision=candidate['revision'], boot_id=str(uuid4()),
                state='dispatching', before=identity, host_boot_id=self.host_boot_id)
            _atomic_write(path, json.dumps(record, sort_keys=True).encode())
        # Do not hold the helper mutex or a robot store writer while Docker
        # starts the replacement: tuning_run needs the boot socket meanwhile.
        self.driver.restart()
        return self.status(value)

    def boot(self):
        with self.lock:
            job = self.store.latest_job()
            if (not job or job.get('state') != 'running'
                    or (job.get('execution_plan') or {}).get('execution_kind') != 'nav2_redeploy'):
                return {'ok': True, 'state': 'active'}
            value = context({key: job[key] for key in ('job_id', 'digest', 'plan_hash')})
            path = self._path(value)
            if not path.exists():
                return {'ok': True, 'state': 'active'}
            record = _read_json(path)
            if any(record.get(key) != item for key, item in value.items()):
                raise ValueError('candidate boot receipt belongs to a different job')
            if record['state'] != 'dispatching' or record.get('host_boot_id') != self.host_boot_id:
                return {'ok': True, 'state': 'active'}
            candidate = candidate_for_job(self.store, value)
            identity = self.driver.identity()
            if identity.get('core') != record['before'].get('core'):
                # This is no longer the still-running coordinator's Apply.
                # Ordinary recovery after a core restart must use ACTIVE.
                return {'ok': True, 'state': 'active'}
            if (identity['container_id'] != record['before']['container_id']
                    or identity['image'] != record['before']['image']
                    or identity['state_directory'] != record['before']['state_directory']
                    or identity['started_at'] == record['before']['started_at']):
                raise ValueError('candidate boot is not the expected replacement')
            record.update(state='boot_consumed', after=identity)
            # Consume BEFORE replying. Lost reply/crash cannot authorize another
            # candidate boot. A subsequent ordinary restart uses only ACTIVE.
            _atomic_write(path, json.dumps(record, sort_keys=True).encode())
            return dict(candidate, ok=True, state='candidate', boot_id=record['boot_id'])


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        from patrolbot_bridge.tuning_json import loads
        self.connection.settimeout(5.)
        try:
            line = self.rfile.readline(4097)
            if len(line) > 4096 or not line.endswith(b'\n'):
                raise ValueError('invalid local request')
            value = loads(line.decode(), 4096)
            if self.server.boot_only:
                if value != {'op': 'boot'}:
                    raise ValueError('navigation socket permits only boot')
                reply = self.server.supervisor.boot()
            elif isinstance(value, dict) and set(value) == {'op', 'context'} and value['op'] in ('restart', 'status'):
                reply = getattr(self.server.supervisor, value['op'])(value['context'])
            elif value == {'op': 'capability'}:
                identity = self.server.supervisor.driver.identity()
                if identity['state_directory'] != str(self.server.supervisor.store.base):
                    raise ValueError('host state volume differs from the configured store')
                self.server.supervisor.store.require_ready()
                reply = {'ok': True, 'implementation': 'navigation-only-restart-v1'}
            else:
                raise ValueError('unallowlisted navigation operation')
            encoded = json.dumps(reply, allow_nan=False).encode() + b'\n'
            if len(encoded) > MAX_REPLY:
                raise ValueError('oversized local reply')
        except Exception as exc:
            encoded = json.dumps({'ok': False, 'detail': type(exc).__name__}).encode() + b'\n'
        self.wfile.write(encoded)


class Server(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True

    def __init__(self, path, supervisor, *, boot_only=False):
        self.supervisor, self.boot_only = supervisor, boot_only
        super().__init__(str(path), Handler)


def main():
    import fcntl
    from patrolbot_bridge.tuning_host import protected_path
    from patrolbot_bridge.tuning_store import RevisionStore
    policy_path = protected_path('/etc/patrolbot/tuning-host/navigation.json')
    policy = _read_json(policy_path)
    if policy.get('enabled') is not True:
        raise SystemExit('navigation restart is not enabled by the host operator')
    protected_path(CONTROL, directory=True)
    lock = (CONTROL / 'supervisor.lock').open('a')
    fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    registry = _read_json(protected_path('/etc/patrolbot/tuning-host/robot-tuning.registry.json'))
    store = RevisionStore(policy['state_directory'], registry)
    supervisor = Supervisor(store, FixedNavigationDocker(policy['image']))
    servers = []
    for role in ('core', 'boot'):
        directory = Path('/run/patrolbot-tuning') / role
        protected_path(directory, directory=True)
        path = directory / 'supervisor.sock'
        if path.exists():
            path.unlink()
        server = Server(path, supervisor, boot_only=role == 'boot')
        os.chmod(path, 0o660)
        servers.append(server)
    thread = threading.Thread(target=servers[1].serve_forever, daemon=True)
    thread.start()
    servers[0].serve_forever()


if __name__ == '__main__':
    main()
