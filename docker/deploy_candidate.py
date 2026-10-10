"""Workstation entry point: check by default; --apply performs an authorized rollout.

Requires the prescribed Pi SSHFS mount. Backups are verified off-host before
retirement; no robot pose/motion command is sent. Healthy deployment does not
establish physical localization acceptance.
"""
import argparse
import base64
from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
import re
import shlex
import subprocess
import tarfile
from uuid import uuid4

from rollout_acceptance import archive_verified, records, require_soak, retire_verified

REPO = Path(__file__).resolve().parents[1]
HELPERS = ('rollout_files.py', 'rollout_runtime.py', 'rollout_docker.py', 'rollout_preflight.py')


def remote(args, *, input=None, binary=False, timeout=60):
    command = ['/usr/bin/ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
               'robot-pi2', shlex.join(args)]
    return subprocess.run(command, input=input, text=not binary, capture_output=True,
                          check=True, timeout=timeout).stdout


def container_file(container, path):
    archive = remote(['docker', 'cp', container + ':' + path, '-'], binary=True)
    with tarfile.open(fileobj=io.BytesIO(archive)) as tar:
        files = [item for item in tar.getmembers() if item.isfile()]
        if len(files) != 1:
            raise RuntimeError('unexpected container file archive')
        return tar.extractfile(files[0]).read()


def validate_payload(payload, manifest):
    expected = {'manifest.json', *HELPERS, *(c['path'] for c in manifest['source_changes'])}
    with tarfile.open(fileobj=io.BytesIO(payload)) as tar:
        members = tar.getmembers()
        if (len(members) != len(expected) or {m.name for m in members} != expected
                or not all(m.isfile() for m in members)):
            raise RuntimeError('payload contains unexpected or non-regular entries')
        if json.loads(tar.extractfile('manifest.json').read()) != manifest:
            raise RuntimeError('payload manifest mismatch')
        for change in manifest['source_changes']:
            if hashlib.sha256(tar.extractfile(change['path']).read()).hexdigest() != change['after']:
                raise RuntimeError('payload source mismatch: ' + change['path'])
        for name in HELPERS:
            committed = subprocess.check_output(['git', '-C', str(REPO), 'show', 'HEAD:docker/' + name])
            if tar.extractfile(name).read() != committed:
                raise RuntimeError('payload helper differs from committed source: ' + name)


ROLLBACK = '''from pathlib import Path
import json
from rollout_files import restore
from rollout_docker import DockerRollout
backup=Path(__file__).resolve().parent
manifest=json.loads((backup/'manifest.json').read_text())
root=Path('/home/ubuntu/patrolbot-repo')
adapter=DockerRollout(root,manifest)
capture=adapter.preflight(backup/'rollout_preflight.py')
if capture.get('safe') is not True:raise RuntimeError('rollback preflight refused')
old=manifest['expected_previous_image']
adapter.ids={old:manifest['expected_previous_image_id']}
restore(root,backup)
adapter.restart(old)
if adapter.postflight(old) is not True:raise RuntimeError('rollback postflight failed')
print('Previous deployment restored and verified')
'''


def apply_remote(payload, backup_name):
    script = '''import base64,io,json,shutil,sys,tarfile,tempfile
from pathlib import Path
with tempfile.TemporaryDirectory(prefix='patrolbot-rollout-') as staging:
    payload=Path(staging)
    with tarfile.open(fileobj=io.BytesIO(base64.b64decode(PAYLOAD))) as tar:
        tar.extractall(payload,filter='data')
    sys.path.insert(0,staging)
    from rollout_runtime import rollout
    from rollout_docker import DockerRollout
    manifest=json.loads((payload/'manifest.json').read_text())
    root=Path('/home/ubuntu/patrolbot-repo')
    backup=Path('/home/ubuntu/backups')/BACKUP
    adapter=DockerRollout(root,manifest)
    def restart(image):
        if image==manifest['candidate_image']:
            for name in HELPERS:
                shutil.copy2(payload/name,backup/name)
            shutil.copy2(payload/'manifest.json',backup/'manifest.json')
            (backup/'rollback.py').write_text(ROLLBACK)
        adapter.restart(image)
    result=rollout(root,payload,backup,manifest,
        verify_candidate=adapter.verify_candidate,
        preflight=lambda:adapter.preflight(payload/'rollout_preflight.py'),
        restart=restart,postflight=adapter.postflight)
    print(json.dumps(result),flush=True)
'''
    transaction = script
    script = '''import json,os
from pathlib import Path
state_path=Path('/home/ubuntu/backups')/('.deployment-state-'+BACKUP+'.json')
def publish_state(status):
    temporary=state_path.with_suffix('.tmp')
    temporary.write_text(json.dumps({'snapshot':BACKUP,'status':status,'pid':os.getpid()}))
    os.replace(temporary,state_path)
publish_state('running')
try:
    exec(TRANSACTION)
except BaseException:
    publish_state('failed')
    raise
else:
    publish_state('succeeded')
'''
    variables = '\n'.join(name + '=' + repr(value) for name, value in {
        'PAYLOAD': base64.b64encode(payload).decode(), 'BACKUP': backup_name,
        'HELPERS': HELPERS, 'ROLLBACK': ROLLBACK, 'TRANSACTION': transaction}.items())
    # Await the remote transaction's terminal result; an observation timeout
    # must not launch a competing deployment or rollback.
    return remote(['python3', '-'], input=variables + '\n' + script, timeout=None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--payload', type=Path, required=True)
    parser.add_argument('--soak-container', required=True)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    payload = args.payload.read_bytes()
    validate_payload(payload, manifest)
    info = json.loads(remote(['docker', 'inspect', args.soak_container]))[0]
    if info['State'].get('Running'):
        raise RuntimeError('soak is still running; deployment refused')
    expected = subprocess.check_output(['git', '-C', str(REPO), 'show',
        manifest['candidate_revision'] + ':docker/tests/integration_scan_outage_soak.py'])
    require_soak(info, remote(['docker', 'logs', args.soak_container]),
                 manifest['candidate_image_id'],
                 container_file(args.soak_container, '/validation/test.py'), expected)
    fake_path = 'ros2_ws/src/patrolbot_bridge/test/test_fake_sbc_integration.py'
    expected_fake = subprocess.check_output(['git', '-C', str(REPO), 'show',
                                             manifest['candidate_revision'] + ':' + fake_path])
    if container_file(args.soak_container, '/validation/' + fake_path) != expected_fake:
        raise RuntimeError('soak simulator differs from the committed candidate')
    if not args.apply:
        print('Payload and completed soak verified; no deployment performed')
        return
    pi = REPO / 'pi'
    subprocess.run(['mountpoint', '-q', str(pi)], check=True)
    backups = pi / 'backups'
    snapshot_name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-issue60-' + uuid4().hex[:8]
    archive_root = REPO / 'backups' / (snapshot_name + '-offhost')
    archive_root.mkdir(mode=0o700)
    lock = backups / '.deployment.lock'
    # Exclusive creation also serializes other copies of this entry point.
    with lock.open('x') as handle:
        handle.write(snapshot_name + '\n')
    release_lock = True
    try:
        previous = sorted(p for p in backups.iterdir() if p.is_dir())
        for p in previous:
            if p.is_symlink() or not re.fullmatch(r'\d{8}T\d{6}Z-[A-Za-z0-9-]+', p.name):
                raise RuntimeError('unrecognized snapshot; refusing retention changes')
        archived = {p.name: archive_verified(p, archive_root / p.name) for p in previous}
        release_lock = False
        try:
            print(apply_remote(payload, snapshot_name), end='')
        finally:
            state_path = backups / ('.deployment-state-' + snapshot_name + '.json')
            state = json.loads(state_path.read_text()) if state_path.exists() else {}
            if state.get('snapshot') != snapshot_name or state.get('status') not in ('succeeded', 'failed'):
                raise RuntimeError('remote transaction outcome unknown; deployment lock retained')
            (archive_root / 'transaction-state.json').write_text(json.dumps(state))
            current = backups / snapshot_name
            if current.exists():
                archive_verified(current, archive_root / current.name)
                saved = json.loads((current / 'files.json').read_text())
                expected_names = {'docker/.env', *(c['path'] for c in manifest['source_changes'])}
                if set(saved) != expected_names or not (current / 'rollback.py').is_file():
                    raise RuntimeError('incomplete rollback snapshot; retention refused')
                for name, expected_hash in saved.items():
                    if hashlib.sha256((current / name).read_bytes()).hexdigest() != expected_hash:
                        raise RuntimeError('invalid rollback snapshot; retention refused')
                # Verify every old snapshot still matches its off-host copy
                # before retiring any, then preserve the two newest snapshots.
                for p in previous:
                    if records(p) != archived[p.name] or records(archive_root / p.name) != archived[p.name]:
                        raise RuntimeError('snapshot changed; retention refused')
                for p in previous[:-1]:
                    retire_verified(p, archive_root / p.name, archived[p.name])
                print(json.dumps({'backup': '/home/ubuntu/backups/' + snapshot_name,
                                  'offhost_backup': str(archive_root),
                                  'rollback': '/home/ubuntu/backups/' + snapshot_name + '/rollback.py'}))
            state_path.unlink()
            release_lock = True
    finally:
        if release_lock:
            lock.unlink()


if __name__ == '__main__':
    main()
