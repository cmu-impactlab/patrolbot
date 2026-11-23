"""Offline CLI, snapshot isolation, and failure/evidence plumbing."""

import importlib.util
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
import tarfile

import pytest


SPEC = importlib.util.spec_from_file_location(
    'local_pre_push', Path(__file__).resolve().parents[1] / 'local-pre-push.py')
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)


@pytest.mark.parametrize('args', [[], ['--gate', 'other'], ['--gate', 'native']])
def test_invalid_arguments(args, monkeypatch, tmp_path):
    monkeypatch.delenv('PATROLBOT_NATIVE_IMAGE', raising=False)
    with pytest.raises(SystemExit) as error:
        runner.arguments(['--results', str(tmp_path), *args])
    assert error.value.code == 2


def test_explicit_selection_and_defaults(monkeypatch, tmp_path):
    monkeypatch.setenv('PATROLBOT_NATIVE_IMAGE', 'cached:env')
    assert runner.arguments(['--results', str(tmp_path)]).gate == 'both'
    args = runner.arguments(['--results', str(tmp_path), '--gate', 'portable',
                             '--python', '/local/python', '--image', 'cached:explicit'])
    assert (args.gate, args.python, args.image) == ('portable', '/local/python', 'cached:explicit')


def test_snapshot_current_dirty_bytes_and_ignores(tmp_path, monkeypatch):
    root = tmp_path / 'repo'
    root.mkdir()
    subprocess.run(['git', 'init', '-q', str(root)], check=True)
    monkeypatch.setattr(runner, 'ROOT', root)
    (root / '.gitignore').write_text('.env\nignored\n.aws/\n')
    (root / 'tracked').write_text('old')
    runner.git('add', '.')
    runner.git('-c', 'user.name=Test', '-c', 'user.email=test@example.invalid',
               'commit', '-qm', 'fixture')
    clean = runner.snapshot(tmp_path / 'clean.tar')
    assert clean == runner.snapshot(tmp_path / 'clean-again.tar')
    (root / 'tracked').write_text('dirty')
    (root / 'new').write_text('new source')
    (root / '.env').write_text('credential')
    (root / 'ignored').write_text('ignored')
    # Even force-added ignored credentials must not enter the archive.
    runner.git('add', '-f', '.env')
    first = runner.snapshot(tmp_path / 'first.tar')
    assert first == runner.snapshot(tmp_path / 'second.tar')
    assert first != clean
    assert stat.S_IMODE((tmp_path / 'first.tar').stat().st_mode) == 0o600
    with tarfile.open(tmp_path / 'first.tar') as archive:
        assert archive.getnames() == ['.gitignore', 'new', 'tracked']
        assert archive.extractfile('tracked').read() == b'dirty'
    (root / '.env.local').write_text('unignored credential')
    with pytest.raises(ValueError, match='unsafe snapshot'):
        runner.snapshot(tmp_path / 'credential.tar')
    (root / '.env.local').unlink()
    (root / 'new').unlink()
    (root / 'new').symlink_to('/etc/passwd')
    with pytest.raises(ValueError, match='unsafe snapshot'):
        runner.snapshot(tmp_path / 'bad.tar')


@pytest.mark.parametrize('name', ['.env', 'nested/.env', 'nested/.config/config.json',
                                  'nested/.ssh/key', 'nested/signing.PEM'])
def test_negated_ignore_cannot_include_sensitive_file(tmp_path, monkeypatch, name):
    root = tmp_path / 'repo'
    subprocess.run(['git', 'init', '-q', str(root)], check=True)
    monkeypatch.setattr(runner, 'ROOT', root)
    (root / '.gitignore').write_text(f'{name}\n!{name}\n')
    target = root / name
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text('fake secret value')
    archive = tmp_path / 'snapshot.tar'
    with pytest.raises(ValueError, match='unsafe snapshot'):
        runner.snapshot(archive)
    assert b'fake secret value' not in archive.read_bytes()


@pytest.mark.parametrize('code', [0, 7])
def test_run_keeps_real_exit_and_log(tmp_path, code):
    result = runner.run([sys.executable, '-c', f'print("evidence"); exit({code})'], tmp_path)
    assert result['returncode'] == code
    assert (tmp_path / 'output.log').read_text().strip() == 'evidence'
    assert json.loads((tmp_path / 'result.json').read_text()) == result


@pytest.mark.parametrize('command', [['/missing/python'],
                                     [sys.executable, '-c', 'import time; time.sleep(60)']])
def test_launch_and_timeout_fail_with_evidence(tmp_path, command):
    result = runner.run(command, tmp_path, timeout=0.1)
    assert result['returncode'] != 0
    assert result['error']
    assert (tmp_path / 'output.log').is_file()
    assert json.loads((tmp_path / 'result.json').read_text()) == result


def test_interrupted_process_records_failure_and_retires_group(tmp_path, monkeypatch):
    calls = []

    class Process:
        pid = 123

        def wait(self, timeout):
            calls.append(timeout)
            if len(calls) == 1:
                runner.interrupted(signal.SIGTERM, None)

    signals = []
    monkeypatch.setattr(runner.subprocess, 'Popen', lambda *a, **k: Process())
    monkeypatch.setattr(runner.os, 'killpg', lambda pid, sig: signals.append((pid, sig)))
    with pytest.raises(KeyboardInterrupt, match='SIGTERM'):
        runner.run(['fake'], tmp_path)
    assert signals == [(123, signal.SIGTERM), (123, signal.SIGKILL)]
    assert calls[1] == 5
    assert json.loads((tmp_path / 'result.json').read_text())['returncode'] == 1


def test_native_isolation_and_full_candidate_build(tmp_path):
    command = runner.native_command('sha256:cached', tmp_path / 'source', tmp_path, 'unique')
    for flag in ('--pull=never', '--network=none', '--ipc=private', '--read-only',
                 '--cpus=2', '--memory=4g', '--memory-swap=4g', '--pids-limit=512'):
        assert flag in command
    assert not any('device' in flag or 'docker.sock' in flag for flag in command)
    assert f'type=bind,src={tmp_path / "source"},dst=/source,readonly' in command
    assert '/tmp:rw,noexec,nosuid,nodev,size=512m,mode=1777' in command
    assert '/work:rw,exec,nosuid,nodev,size=2g,mode=1777' in command
    assert command[-1].endswith('python3 -B /work/source/docker/native_ci_gate.py\n')
    for package in ('interfaces', 'bridge', 'dock_manager', 'navigation'):
        assert '/work/source/ros2_ws/src/patrolbot_' + package in command[-1]
    assert command[-1].index('colcon --log-base /evidence/build-colcon-log build') < command[-1].index('native_ci_gate.py')


def test_failure_does_not_hide_next_gate_and_summary(tmp_path, monkeypatch):
    monkeypatch.setenv('PATROLBOT_NATIVE_IMAGE', 'cached')
    monkeypatch.setenv('DOCKER_CONTEXT', 'remote-fixture')
    monkeypatch.setenv('DOCKER_HOST', 'ssh://fixture.invalid')
    monkeypatch.setenv('DOCKER_CONFIG', '/not-used')
    # Supply a real empty archive instead of exercising source enumeration again.
    def snapshot(path):
        with tarfile.open(path, 'w'):
            pass
        return {'sha256': 'fixture'}
    monkeypatch.setattr(runner, 'snapshot', snapshot)
    calls = []

    def run(command, directory, **kwargs):
        calls.append(command)
        directory.mkdir(parents=True, exist_ok=True)
        if command[0] == 'docker':
            assert kwargs['env']['DOCKER_HOST'] == 'unix:///var/run/docker.sock'
            assert 'DOCKER_CONTEXT' not in kwargs['env'] and 'DOCKER_CONFIG' not in kwargs['env']
        if command[:3] == ['docker', 'image', 'inspect']:
            (directory / 'output.log').write_text('sha256:' + 'a' * 64 + '\n')
        return {'returncode': 7 if command[0] != 'docker' else 0, 'command': command}

    monkeypatch.setattr(runner, 'run', run)
    results = tmp_path / 'evidence'
    assert runner.main(['--results', str(results)]) == 1
    assert stat.S_IMODE(results.stat().st_mode) == 0o700
    assert stat.S_IMODE((results / 'summary.json').stat().st_mode) == 0o600
    summary = json.loads((results / 'summary.json').read_text())
    assert summary['gates']['portable']['returncode'] == 7
    assert summary['gates']['native']['returncode'] == 0
    assert any(command[:2] == ['docker', 'run'] for command in calls)
    assert calls[-1][:3] == ['docker', 'rm', '-f']
    with pytest.raises(FileExistsError):
        runner.main(['--results', str(results)])


def test_snapshot_error_keeps_failed_root_summary(tmp_path, monkeypatch):
    def fail(path):
        raise ValueError('unsafe source')
    monkeypatch.setattr(runner, 'snapshot', fail)
    evidence = tmp_path / 'failed'
    assert runner.main(['--gate', 'portable', '--results', str(evidence)]) == 1
    assert json.loads((evidence / 'summary.json').read_text()) == {
        'gates': {}, 'returncode': 1, 'error': 'unsafe source'}


def test_missing_image_fails_without_launching_container(tmp_path, monkeypatch):
    def snapshot(path):
        with tarfile.open(path, 'w'):
            pass
        return {}
    monkeypatch.setattr(runner, 'snapshot', snapshot)
    calls = []

    def run(command, directory, **kwargs):
        calls.append(command)
        return {'returncode': 1}
    monkeypatch.setattr(runner, 'run', run)
    evidence = tmp_path / 'failed'
    assert runner.main(['--gate', 'native', '--image', 'missing',
                        '--results', str(evidence)]) == 1
    assert calls == [['docker', 'image', 'inspect', '--format', '{{.Id}}', 'missing']]
    assert json.loads((evidence / 'summary.json').read_text())['gates'] == {
        'native': {'returncode': 1}}


def test_native_interruption_cleans_up_exact_container(tmp_path, monkeypatch):
    def snapshot(path):
        with tarfile.open(path, 'w'):
            pass
        return {}

    calls = []

    def run(command, directory, **kwargs):
        calls.append(command)
        if command[:3] == ['docker', 'image', 'inspect']:
            directory.mkdir(parents=True)
            (directory / 'output.log').write_text('sha256:' + 'a' * 64 + '\n')
        elif command[:2] == ['docker', 'run']:
            raise KeyboardInterrupt('interrupted by SIGTERM')
        return {'returncode': 0}

    monkeypatch.setattr(runner, 'snapshot', snapshot)
    monkeypatch.setattr(runner, 'run', run)
    old_mask = os.umask(0o077)
    try:
        result = tmp_path / 'interrupted'
        assert runner.main(['--gate', 'native', '--image', 'cached', '--results', str(result)]) == 1
        assert os.umask(old_mask) == 0o077
    finally:
        os.umask(old_mask)
    launch = calls[1]
    assert calls[2] == ['docker', 'rm', '-f', launch[launch.index('--name') + 1]]
    assert json.loads((result / 'summary.json').read_text())['error'] == 'interrupted by SIGTERM'
