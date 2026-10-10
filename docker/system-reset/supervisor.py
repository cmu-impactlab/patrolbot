#!/usr/bin/env python3
"""Fixed-operation, fail-closed host supervisor for a PatrolBot software reset."""
from __future__ import annotations

import json
import fcntl
import os
import re
import socket
import socketserver
import sqlite3
import subprocess
import threading
import time
import uuid
from pathlib import Path

ROOT = Path('/home/ubuntu/patrolbot-repo')
PREFLIGHT = ROOT / 'docker/system-reset/preflight.py'
OVERRIDE = ROOT / 'docker/system-reset/compose.override.yaml'
SSH_CONFIG = Path('/home/ubuntu/patrolbot-support/system-reset/ssh_config')
DB = Path('/var/lib/patrolbot-system-reset/status.sqlite3')
SOCKET = Path('/run/patrolbot-system-reset/supervisor.sock')
MAX_LINE = 4096
REQUEST_RE = re.compile(r'^[0-9a-f]{8}-[0-9a-f]{4}-[1-8][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$')
CORE = ('bringup', 'bridge', 'navigation')
TERMINAL = ('succeeded', 'failed', 'uncertain')


class ResetError(RuntimeError):
    pass


class ProductionAdapter:
    """Only fixed argv commands; no client data reaches a shell or subprocess."""
    def run(self, argv, *, timeout, input_text=None):
        try:
            result = subprocess.run(argv, input=input_text, text=True,
                                    capture_output=True, timeout=timeout, check=False,
                                    env={**os.environ, 'PATH': '/usr/sbin:/usr/bin:/sbin:/bin'})
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise ResetError(f'command outcome unavailable: {type(exc).__name__}') from exc
        if result.returncode:
            raise ResetError(f'fixed command failed ({result.returncode})')
        return result.stdout

    def capture(self, post=False):
        code = PREFLIGHT.read_text(encoding='utf-8')
        shell = ('source /opt/ros/jazzy/setup.bash && '
                 'source /opt/patrolbot_ws/install/setup.bash && python3 -' +
                 (' --post-reset' if post else ''))
        raw = self.run(['/usr/bin/docker', 'exec', '-i', 'patrolbot-navigation',
                        'bash', '-lc', shell], timeout=18, input_text=code)
        try:
            value = json.loads(raw.strip().splitlines()[-1])
        except (ValueError, IndexError) as exc:
            raise ResetError('read-only ROS capture was malformed') from exc
        if not isinstance(value, dict):
            raise ResetError('read-only ROS capture was malformed')
        return value

    def restart_sbc(self):
        # Target and remote command are fixed deployment capabilities. SSH keys
        # must be provisioned by the operator; this service never creates them.
        self.run(['/usr/bin/ssh', '-F', str(SSH_CONFIG), 'patrolbot-reset-sbc', 'restart'],
                 timeout=25)

    def capability(self):
        if not all(path.exists() for path in (PREFLIGHT, ROOT / 'docker/.env',
                                               ROOT / 'docker/docker-compose.yml', OVERRIDE, SSH_CONFIG)):
            return False
        try:
            load_state = self.run(['/usr/bin/ssh', '-F', str(SSH_CONFIG), 'patrolbot-reset-sbc', 'status'], timeout=8).strip()
            if load_state != 'LoadState=loaded':
                return False
            self.run(['/usr/bin/docker', 'compose', '--env-file', str(ROOT / 'docker/.env'),
                      '-f', str(ROOT / 'docker/docker-compose.yml'), '-f', str(OVERRIDE), 'config', '--quiet'], timeout=20)
        except ResetError:
            return False
        return True

    def restart_pi(self):
        # Restart the existing commissioned containers, preserving their exact
        # images/configuration/mounts. A reset must never pull or deploy a tag.
        self.run(['/usr/bin/docker', 'restart', '--time', '30',
                  *('patrolbot-' + name for name in CORE)], timeout=180)
        self.run(['/usr/bin/docker', 'restart', '--time', '30',
                  'patrolbot-web-bridge'], timeout=90)

    def wait_healthy(self):
        deadline = time.monotonic() + 180
        while time.monotonic() < deadline:
            names = ['patrolbot-' + name for name in (*CORE, 'web-bridge')]
            containers = json.loads(self.run(['/usr/bin/docker', 'inspect', *names], timeout=10))
            if len(containers) == len(names) and all(
                c['State']['Running'] and not c['State'].get('Restarting', False) and
                (c['State'].get('Health', {}).get('Status') == 'healthy'
                 if c['Name'].lstrip('/') != 'patrolbot-web-bridge' else True)
                for c in containers):
                return
            time.sleep(3)
        raise ResetError('Pi services did not become healthy before deadline')


class Store:
    def __init__(self, path=DB):
        self.path = path
        self.lock = threading.RLock()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.connect() as db:
            db.execute('CREATE TABLE IF NOT EXISTS requests (request_id TEXT PRIMARY KEY, '
                       'state TEXT NOT NULL, message TEXT NOT NULL, updated REAL NOT NULL)')

    def connect(self):
        db = sqlite3.connect(self.path, timeout=5)
        db.execute('PRAGMA synchronous=FULL')
        return db

    def latest(self):
        with self.connect() as db:
            row = db.execute('SELECT request_id,state,message,updated FROM requests '
                             'ORDER BY updated DESC LIMIT 1').fetchone()
        return row

    def get(self, request_id):
        with self.connect() as db:
            return db.execute('SELECT request_id,state,message,updated FROM requests '
                              'WHERE request_id=?', (request_id,)).fetchone()

    def write(self, request_id, state, message):
        with self.connect() as db:
            db.execute('INSERT INTO requests VALUES(?,?,?,?) ON CONFLICT(request_id) '
                       'DO UPDATE SET state=excluded.state,message=excluded.message,'
                       'updated=excluded.updated', (request_id, state, message[:240], time.time()))


class Supervisor:
    def __init__(self, adapter=None, store=None):
        self.adapter = adapter or ProductionAdapter()
        self.store = store or Store()
        self.lock = threading.RLock()
        # A process restart cannot prove whether an interrupted command ran.
        latest = self.store.latest()
        if latest and latest[1] in ('accepted', 'running'):
            self.store.write(latest[0], 'uncertain', 'supervisor restarted during reset; operator-local reconciliation required')
        self.available = (self.adapter.capability() if hasattr(self.adapter, 'capability') else True)

    @staticmethod
    def response(row):
        if not row:
            return {'ok': True, 'state': 'idle'}
        return {'ok': True, 'request_id': row[0], 'state': row[1],
                'message': row[2], 'updated_unix': row[3]}

    def submit(self, request_id):
        if not self.available:
            return {'ok': False, 'state': 'unavailable',
                    'message': 'reset prerequisites are not verified on this host'}
        if not isinstance(request_id, str) or not REQUEST_RE.fullmatch(request_id):
            return {'ok': False, 'state': 'rejected', 'message': 'request_id must be a UUID'}
        with self.lock:
            existing = self.store.get(request_id)
            if existing:
                return self.response(existing)
            latest = self.store.latest()
            if latest and latest[1] in ('accepted', 'running'):
                return {'ok': False, 'state': 'busy', 'message': 'another reset is in progress'}
            if latest and latest[1] == 'uncertain':
                return {'ok': False, 'state': 'uncertain', 'message':
                        'operator-local reconciliation is required before another reset'}
            self.store.write(request_id, 'accepted', 'accepted; waiting for safety capture')
            threading.Thread(target=self.execute, args=(request_id,), daemon=True).start()
            return self.response(self.store.get(request_id))

    def execute(self, request_id):
        self.store.write(request_id, 'running', 'capturing fresh docked and stationary state')
        reset_started = False
        try:
            before = self.adapter.capture(False)
            time.sleep(1)
            repeated = self.adapter.capture(False)
            if before.get('safe') is not True or repeated.get('safe') is not True:
                raise ResetError('fresh docked, stationary, motors-disabled interlock failed')
            if before.get('odom_epoch') != repeated.get('odom_epoch'):
                raise ResetError('origin changed during preflight; reset refused')
            old_epoch = repeated.get('odom_epoch')
            if not isinstance(old_epoch, str) or not old_epoch:
                raise ResetError('origin identity unavailable before reset')
            reset_started = True
            self.adapter.restart_sbc()
            self.adapter.restart_pi()
            self.adapter.wait_healthy()
            deadline = time.monotonic() + 180
            after = None
            while time.monotonic() < deadline:
                try:
                    after = self.adapter.capture(True)
                    if (after.get('safe') is True and after.get('odom_epoch') and
                            after['odom_epoch'] != old_epoch):
                        self.store.write(request_id, 'succeeded',
                                         'services healthy; new origin recovery hold confirmed; '
                                         'a fresh operator pose seed is required before navigation')
                        return
                except ResetError:
                    pass
                time.sleep(3)
            raise ResetError('reset outcome or new-origin recovery hold could not be confirmed')
        except ResetError as exc:
            self.store.write(request_id, 'uncertain' if reset_started else 'failed', str(exc))
        except Exception as exc:  # Keep unexpected partial outcomes non-repeatable.
            self.store.write(request_id, 'uncertain' if reset_started else 'failed',
                             f'unexpected failure: {type(exc).__name__}')


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.connection.settimeout(2)
        line = self.rfile.readline(MAX_LINE + 1)
        if len(line) > MAX_LINE or not line.endswith(b'\n'):
            return self.reply({'ok': False, 'state': 'rejected', 'message': 'invalid request'})
        try:
            value = json.loads(line)
        except (UnicodeDecodeError, ValueError):
            return self.reply({'ok': False, 'state': 'rejected', 'message': 'invalid JSON'})
        if not isinstance(value, dict):
            return self.reply({'ok': False, 'state': 'rejected', 'message': 'invalid request'})
        op = value.get('op')
        if op == 'current' and set(value) == {'op'}:
            if not self.server.supervisor.available:
                return self.reply({'ok': False, 'state': 'unavailable',
                                   'message': 'reset prerequisites are not verified on this host'})
            return self.reply(self.server.supervisor.response(self.server.supervisor.store.latest()))
        if op == 'status' and set(value) in ({'op'}, {'op', 'request_id'}):
            rid = value.get('request_id')
            if rid is not None and (not isinstance(rid, str) or not REQUEST_RE.fullmatch(rid)):
                return self.reply({'ok': False, 'state': 'rejected', 'message': 'invalid request_id'})
            row = self.server.supervisor.store.get(rid) if rid else self.server.supervisor.store.latest()
            return self.reply(self.server.supervisor.response(row))
        if op == 'request' and set(value) == {'op', 'request_id', 'operation'} and value.get('operation') == 'software_reset':
            return self.reply(self.server.supervisor.submit(value.get('request_id')))
        return self.reply({'ok': False, 'state': 'rejected', 'message': 'unsupported request'})

    def reply(self, value):
        self.wfile.write(json.dumps(value, separators=(',', ':')).encode() + b'\n')


class Server(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, path, supervisor=None):
        self.supervisor = supervisor or Supervisor()
        super().__init__(str(path), Handler)


def main():
    DB.parent.mkdir(parents=True, exist_ok=True)
    process_lock = (DB.parent / 'supervisor.lock').open('a')
    fcntl.flock(process_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    SOCKET.parent.mkdir(parents=True, exist_ok=True)
    try:
        SOCKET.unlink()
    except FileNotFoundError:
        pass
    server = Server(SOCKET)
    os.chmod(SOCKET, 0o660)
    server.serve_forever(poll_interval=0.5)


if __name__ == '__main__':
    main()
