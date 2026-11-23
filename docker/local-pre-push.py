#!/usr/bin/env python3
"""Run the complete local portable/native gates against one current-source snapshot."""

import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import tarfile
import tempfile
import time
import uuid


ROOT = Path(__file__).resolve().parents[1]
NATIVE = r'''
set -eo pipefail
cp -a /source/. /work/source
cd /work
source /opt/ros/jazzy/setup.bash
MAKEFLAGS=-j2 colcon --log-base /evidence/build-colcon-log build --parallel-workers 1 \
  --base-paths /work/source/ros2_ws/src/patrolbot_interfaces \
    /work/source/ros2_ws/src/patrolbot_bridge \
    /work/source/ros2_ws/src/patrolbot_dock_manager \
    /work/source/ros2_ws/src/patrolbot_navigation \
  --build-base /work/build --install-base /work/install \
  --cmake-args -DBUILD_TESTING=OFF 2>&1 | tee /evidence/build.log
source /work/install/setup.bash
export PATROLBOT_TEST_WORKSPACE=/work/source
export PATROLBOT_TEST_EVIDENCE=/evidence/tests
python3 -B /work/source/docker/native_ci_gate.py
'''


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gate', choices=('both', 'portable', 'native'), default='both')
    parser.add_argument('--results', type=Path, required=True,
                        help='new results directory (never overwritten)')
    parser.add_argument('--python', default=sys.executable, help='local portable Python executable')
    parser.add_argument('--image', default=os.environ.get('PATROLBOT_NATIVE_IMAGE'),
                        help='cached native dependency image; defaults to PATROLBOT_NATIVE_IMAGE')
    args = parser.parse_args(argv)
    if args.gate != 'portable' and not args.image:
        parser.error('--image or PATROLBOT_NATIVE_IMAGE is required for the native gate')
    return args


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args])


def snapshot(destination):
    """Copy current tracked + nonignored untracked bytes, never Git/auth metadata."""
    paths = sorted(set(git('ls-files', '--cached', '--others', '--exclude-standard', '-z')
                       .decode().strip('\0').split('\0')) - {''})
    # --no-index also excludes force-added ignored credentials/generated files.
    ignored = subprocess.run(['git', '-C', str(ROOT), 'check-ignore', '--no-index', '-z', '--stdin'],
                             input='\0'.join(paths).encode() + b'\0', capture_output=True)
    if ignored.returncode not in (0, 1):
        raise RuntimeError('git check-ignore failed: ' + ignored.stderr.decode())
    excluded = set(ignored.stdout.decode().strip('\0').split('\0'))
    manifest = []
    # Source archives stay private even when snapshot() is used independently.
    descriptor = os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, 'wb') as stream, tarfile.open(fileobj=stream, mode='w') as archive:
        for name in paths:
            path = ROOT / name
            if name in excluded or (not path.exists() and not path.is_symlink()):
                continue
            if (any(parent.is_symlink() for parent in (path, *path.parents) if parent != ROOT)
                    or not path.resolve().is_relative_to(ROOT) or not path.is_file()
                    or any(part in ('.git', '.ssh', '.aws', '.agents', '.codex', '.claude',
                                    '.docker', '.kube', '.gnupg', '.azure', '.config', '.env')
                           for part in Path(name).parts)
                     or path.name in ('.netrc', '.npmrc', '.pypirc', '.git-credentials',
                                      '.envrc', 'id_rsa', 'id_ed25519', 'id_ecdsa', 'id_dsa')
                     or path.suffix.lower() in ('.key', '.pem', '.p12', '.pfx')
                    or (path.name.startswith('.env.') and path.name != '.env.example')):
                raise ValueError('unsafe snapshot path: ' + name)
            data = path.read_bytes()
            info = tarfile.TarInfo(name)
            info.size = len(data)
            info.mode = 0o755 if path.stat().st_mode & 0o111 else 0o644
            archive.addfile(info, io.BytesIO(data))
            manifest.append({'path': name, 'sha256': hashlib.sha256(data).hexdigest(),
                             'mode': info.mode})
    return {'head': git('rev-parse', 'HEAD').decode().strip(), 'files': manifest,
            'sha256': hashlib.sha256(destination.read_bytes()).hexdigest()}


def run(command, directory, *, cwd=None, env=None, timeout=1800):
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    print('+ ' + shlex.join(command), flush=True)
    result = {'command': command, 'cwd': str(cwd) if cwd else None, 'returncode': None}
    (directory / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    started = time.monotonic()
    interrupted = None
    with (directory / 'output.log').open('w') as log:
        try:
            process = subprocess.Popen(command, cwd=cwd, env=env, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            try:
                result['returncode'] = process.wait(timeout=timeout)
                try:
                    os.killpg(process.pid, 0)
                except ProcessLookupError:
                    pass
                else:
                    result.update(returncode=1, error='gate left a live process group')
            finally:
                # Also retire descendants of a finished or interrupted local gate.
                for sig in (signal.SIGTERM, signal.SIGKILL):
                    try:
                        os.killpg(process.pid, sig)
                    except ProcessLookupError:
                        break
                process.wait(timeout=5)
        except (OSError, subprocess.TimeoutExpired, KeyboardInterrupt) as error:
            result['error'] = str(error) or type(error).__name__
            result['returncode'] = 1
            if isinstance(error, KeyboardInterrupt):
                interrupted = error
    result['elapsed_seconds'] = time.monotonic() - started
    (directory / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(f"{directory.name}: exit {result['returncode']}; {directory / 'output.log'}", flush=True)
    if interrupted is not None:
        raise interrupted
    return result


def native_command(image, source, evidence, name):
    return ['docker', 'run', '--pull=never', '--name', name,
            '--network=none', '--ipc=private', '--cpus=2', '--memory=4g',
            '--memory-swap=4g', '--pids-limit=512', '--cap-drop=ALL',
            '--security-opt=no-new-privileges', '--read-only',
            '--user', f'{os.getuid()}:{os.getgid()}',
            '--tmpfs', '/tmp:rw,noexec,nosuid,nodev,size=512m,mode=1777',
            '--tmpfs', '/work:rw,exec,nosuid,nodev,size=2g,mode=1777',
            '--mount', f'type=bind,src={source},dst=/source,readonly',
            '--mount', f'type=bind,src={evidence},dst=/evidence',
            '--env', 'HOME=/tmp', '--env', 'PYTHONDONTWRITEBYTECODE=1',
            '--env', 'ROS_DOMAIN_ID=101', '--env', 'ROS_LOCALHOST_ONLY=1',
            '--env', 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST',
            '--env', 'PATROLBOT_RUN_INTEGRATION=1',
            '--env', 'PATROLBOT_RUN_MANUAL_UNDOCK_NATIVE=1',
            '--env', 'PATROLBOT_TEST_TIMEOUT=300',
            '--entrypoint', '/bin/bash', image, '-c', NATIVE]


def interrupted(signum, frame):
    raise KeyboardInterrupt(f'interrupted by {signal.Signals(signum).name}')


def process_environment(home):
    return {**{key: value for key, value in os.environ.items()
               if key in ('PATH', 'LANG', 'LC_ALL', 'SYSTEMROOT')}, 'HOME': str(home)}


def main(argv=None):
    args = arguments(argv)
    previous_umask = os.umask(0o077)
    previous_signals = {sig: signal.signal(sig, interrupted)
                        for sig in (signal.SIGTERM, signal.SIGHUP)}
    try:
        return verify(args)
    finally:
        for sig, handler in previous_signals.items():
            signal.signal(sig, handler)
        os.umask(previous_umask)


def verify(args):
    results = args.results.resolve()
    # Avoid recursively snapshotting this run or accidentally binding the checkout writable.
    if results == ROOT or ROOT in results.parents:
        raise ValueError('--results must be outside the source checkout')
    results.mkdir(parents=True, exist_ok=False, mode=0o700)
    summary = {'gates': {}, 'returncode': 1}
    try:
        with tempfile.TemporaryDirectory(prefix='patrolbot-pre-push-') as temporary:
            temporary = Path(temporary)
            archive = results / 'source.tar'
            summary['snapshot'] = snapshot(archive)
            (results / 'snapshot.json').write_text(json.dumps(summary['snapshot'], indent=2) + '\n')
            source = temporary / 'source'
            source.mkdir()
            with tarfile.open(archive) as bundle:
                bundle.extractall(source, filter='data')
            if args.gate in ('both', 'portable'):
                portable = results / 'portable'
                # The existing runner compiles hardware binaries beside its sources.
                build = temporary / 'portable-build'
                shutil.copytree(source, build)
                env = process_environment(temporary)
                env.update(MAKEFLAGS='-j2', PYTHONDONTWRITEBYTECODE='1',
                           TMPDIR=str(temporary))
                summary['gates']['portable'] = {'returncode': 1}
                summary['gates']['portable'] = run(
                    [str(Path(args.python).absolute()) if '/' in args.python else args.python,
                     '-B', str(build / 'docker/run-portable-review-tests.py')],
                    portable, cwd=build, env=env)
                hardware = build / 'hardware/patrolbot_hw_server/tests'
                if hardware.exists():
                    shutil.copytree(hardware, portable / 'hardware-tests')
            if args.gate in ('both', 'native'):
                native = results / 'native'
                # Never use a workstation's remote Docker context or auth config.
                docker_env = process_environment(temporary)
                docker_env['DOCKER_HOST'] = 'unix:///var/run/docker.sock'
                inspected = run(['docker', 'image', 'inspect', '--format', '{{.Id}}', args.image],
                                native / 'image', env=docker_env)
                if inspected['returncode']:
                    summary['gates']['native'] = inspected
                else:
                    image = (native / 'image/output.log').read_text().strip()
                    if (not image.startswith('sha256:') or len(image) != 71
                            or any(ch not in '0123456789abcdef' for ch in image[7:])):
                        raise ValueError('invalid cached image ID')
                    name = 'patrolbot-pre-push-' + uuid.uuid4().hex
                    summary['gates']['native'] = {'returncode': 1}
                    try:
                        summary['gates']['native'] = run(
                            native_command(image, source, native, name), native, env=docker_env)
                    finally:
                        # docker CLI timeout/interruption must not leave the container running.
                        cleanup = run(['docker', 'rm', '-f', name], native / 'cleanup',
                                      env=docker_env, timeout=30)
                        if cleanup['returncode']:
                            summary['gates']['native']['returncode'] = 1
                            summary['gates']['native']['cleanup_error'] = True
            summary['returncode'] = int(any(
                gate['returncode'] != 0 for gate in summary['gates'].values()))
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError, KeyboardInterrupt) as error:
        summary['error'] = str(error) or type(error).__name__
        print('Verifier failed: ' + summary['error'], file=sys.stderr)
    finally:
        (results / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return summary['returncode']


if __name__ == '__main__':
    sys.exit(main())
