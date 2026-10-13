#!/usr/bin/env python3
"""Fixed-operation host supervisor for an explicit emergency software reset."""
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

    def stop_pi(self):
        # Closing the old bridge session invokes the SBC's stop/motor-disable path.
        self.run(['/usr/bin/docker', 'stop', '--time', '10',
                  'patrolbot-bridge', 'patrolbot-navigation', 'patrolbot-bringup'], timeout=60)

    def restart_sbc(self):
        # Target and remote command are fixed deployment capabilities. SSH keys
        # must be provisioned by the operator; this service never creates them.
        self.run(['/usr/bin/ssh', '-F', str(SSH_CONFIG), 'patrolbot-reset-sbc', 'restart'],
                 timeout=25)

    def capability(self):
        # Availability describes installed management access, not robot health.
        return Path('/usr/bin/docker').is_file() and SSH_CONFIG.is_file()

    def restart_pi(self):
        # Start the existing containers; never pull, recreate or change configuration.
        self.run(['/usr/bin/docker', 'restart', '--time', '10',
                  *('patrolbot-' + name for name in CORE)], timeout=90)

    def restart_web_bridge(self):
        self.run(['/usr/bin/docker', 'restart', '--time', '10',
                  'patrolbot-web-bridge'], timeout=60)


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
            self.store.write(latest[0], 'uncertain', 'supervisor restarted during reset; outcome unknown; no automatic retry')
        self.available = (self.adapter.capability() if hasattr(self.adapter, 'capability') else True)
        if not self.available:
            threading.Thread(target=self._recover_capability, daemon=True).start()

    def _recover_capability(self):
        # Retry local installation checks without blocking status replies.
        while not self.available:
            time.sleep(5)
            self.available = self.adapter.capability()

    @staticmethod
    def response(row):
        if not row:
            return {'ok': True, 'state': 'idle'}
        return {'ok': True, 'request_id': row[0], 'state': row[1],
                'message': row[2], 'updated_unix': row[3]}

    def submit(self, request_id):
        if not self.available:
            return {'ok': False, 'state': 'unavailable',
                    'message': 'reset management tools or access configuration are unavailable'}
        if not isinstance(request_id, str) or not REQUEST_RE.fullmatch(request_id):
            return {'ok': False, 'state': 'rejected', 'message': 'request_id must be a UUID'}
        with self.lock:
            existing = self.store.get(request_id)
            if existing:
                return self.response(existing)
            latest = self.store.latest()
            if latest and latest[1] in ('accepted', 'running'):
                return {'ok': False, 'state': 'busy', 'message': 'another reset is in progress'}
            self.store.write(request_id, 'accepted', 'accepted; restarting robot software')
            threading.Thread(target=self.execute, args=(request_id,), daemon=True).start()
            return self.response(self.store.get(request_id))

    def execute(self, request_id):
        self.store.write(request_id, 'running', "stopping old software and restarting services on both robot computers")
        errors = []
        for name in ('stop_pi', 'restart_sbc', 'restart_pi', 'restart_web_bridge'):
            try:
                getattr(self.adapter, name)()
            except Exception as exc:
                detail = str(exc) if isinstance(exc, ResetError) else type(exc).__name__
                errors.append(f'{name}: {detail}')
        if errors:
            self.store.write(request_id, 'uncertain', '; '.join(errors))
        else:
            self.store.write(request_id, 'succeeded',
                             'Software restart operations completed. Robot readiness is checked separately.')


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
                                   'message': 'reset management tools or access configuration are unavailable'})
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
