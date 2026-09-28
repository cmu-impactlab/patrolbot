"""Exercise entry-point retention and uncertain remote outcomes without a Pi."""
import contextlib
import ast
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import tarfile
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import deploy_candidate as deploy


def test_payload_and_bootstrap():
    for case in ('valid', 'changed_source', 'changed_helper', 'extra_file', 'symlink', 'duplicate'):
        manifest = {'source_changes': [{'path': 'src/file.py',
                     'after': hashlib.sha256(b'source').hexdigest()}]}
        files = {'manifest.json': json.dumps(manifest).encode(),
                 'src/file.py': b'source', **{name: b'helper' for name in deploy.HELPERS}}
        if case == 'changed_source': files['src/file.py'] = b'wrong'
        if case == 'changed_helper': files[deploy.HELPERS[0]] = b'wrong'
        if case == 'extra_file': files['unexpected'] = b'file'
        stream = io.BytesIO()
        with tarfile.open(fileobj=stream, mode='w:gz') as tar:
            for name, data in files.items():
                item = tarfile.TarInfo(name)
                item.size = len(data)
                if case == 'symlink' and name == 'src/file.py':
                    item.type = tarfile.SYMTYPE
                    item.linkname = '/outside'
                    tar.addfile(item)
                else:
                    tar.addfile(item, io.BytesIO(data))
            if case == 'duplicate':
                item = tarfile.TarInfo('manifest.json')
                item.size = len(files['manifest.json'])
                tar.addfile(item, io.BytesIO(files['manifest.json']))
        refused = False
        with patch.object(subprocess, 'check_output', return_value=b'helper'):
            try:
                deploy.validate_payload(stream.getvalue(), manifest)
            except RuntimeError:
                refused = True
        assert refused == (case != 'valid'), case
        print('PASS payload', case)

    def compile_remote(args, *, input, timeout):
        assert args == ['python3', '-'] and timeout is None
        compile(input, '<remote-bootstrap>', 'exec')
        for node in ast.parse(input).body:
            if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
                if node.targets[0].id in ('TRANSACTION', 'ROLLBACK'):
                    compile(ast.literal_eval(node.value), '<remote-phase>', 'exec')
        return ''
    with patch.object(deploy, 'remote', compile_remote):
        deploy.apply_remote(b'archive', 'test-snapshot')
    print('PASS remote bootstrap syntax')


def test_transaction_outcomes():
    for case in ('success', 'failed_known', 'unknown', 'incomplete_backup'):
        with tempfile.TemporaryDirectory() as directory:
            repo = Path(directory)
            backups = repo / 'pi/backups'
            backups.mkdir(parents=True)
            (repo / 'backups').mkdir()
            previous = ['20260924T081318Z-health-tools', '20260924T082137Z-image-labels']
            for name in previous:
                (backups / name).mkdir()
                (backups / name / 'saved').write_text('previous deployment')
            manifest = {'candidate_revision': 'revision', 'candidate_image_id': 'candidate',
                        'source_changes': [{'path': 'src/file.py'}]}
            manifest_path, payload = repo / 'manifest.json', repo / 'payload.tar.gz'
            manifest_path.write_text(json.dumps(manifest))
            payload.write_bytes(b'payload')
            info = {'Image': 'candidate', 'State': {'Status': 'exited', 'Running': False,
                    'ExitCode': 0, 'OOMKilled': False}, 'Mounts': [],
                    'HostConfig': {'NetworkMode': 'none', 'IpcMode': 'private', 'Devices': [], 'CapDrop': ['ALL']}}
            output = '\n'.join(json.dumps(e) for e in [
                {'event': 'outage_started', 'wall_seconds': 28800},
                {'event': 'outage_acceptance', 'wall_seconds': 28801,
                 'recovery_without_restart': True, 'nonzero_hold': False, 'hold_messages': 280000},
                {'event': 'managed_shutdown_complete'}])

            def remote(args, **_):
                return json.dumps([info]) if args[1] == 'inspect' else output

            def apply(_, snapshot):
                current = backups / snapshot
                (current / 'src').mkdir(parents=True)
                (current / 'docker').mkdir()
                (current / 'src/file.py').write_text('old source')
                (current / 'docker/.env').write_text('private environment')
                saved = {name: hashlib.sha256((current / name).read_bytes()).hexdigest()
                         for name in ('src/file.py', 'docker/.env')}
                (current / 'files.json').write_text(json.dumps(saved))
                if case != 'incomplete_backup':
                    (current / 'rollback.py').write_text('# reviewed rollback tool')
                (backups / ('.deployment-state-' + snapshot + '.json')).write_text(json.dumps({
                    'snapshot': snapshot, 'status': 'running' if case == 'unknown' else
                    ('succeeded' if case == 'success' else 'failed')}))
                if case != 'success':
                    raise RuntimeError('simulated connection loss or deployment failure')
                return '{}\n'

            arguments = ['deploy', '--manifest', str(manifest_path), '--payload', str(payload),
                         '--soak-container', 'test', '--apply']
            failure = None
            with patch.object(deploy, 'REPO', repo), patch.object(sys, 'argv', arguments), \
                    patch.object(deploy, 'validate_payload'), patch.object(deploy, 'remote', remote), \
                    patch.object(deploy, 'container_file', return_value=b'fixture'), \
                    patch.object(deploy, 'apply_remote', apply), \
                    patch.object(subprocess, 'check_output', return_value=b'fixture'), \
                    patch.object(subprocess, 'run'), contextlib.redirect_stdout(io.StringIO()):
                try:
                    deploy.main()
                except RuntimeError as error:
                    failure = error
            assert (failure is None) == (case == 'success')
            locked = (backups / '.deployment.lock').exists()
            assert locked == (case in ('unknown', 'incomplete_backup'))
            snapshots = [p for p in backups.iterdir() if p.is_dir()]
            assert len(snapshots) == (3 if locked else 2)
            assert (backups / previous[0]).exists() == locked
            offhost = next((repo / 'backups').iterdir())
            assert (offhost / previous[0] / 'saved').read_text() == 'previous deployment'
            assert (offhost.stat().st_mode & 0o777) == 0o700
            print('PASS entry point', case)


if __name__ == '__main__':
    test_payload_and_bootstrap()
    test_transaction_outcomes()
