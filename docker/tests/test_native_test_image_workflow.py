"""Check hosted functional policy and real shell failure propagation without Docker."""

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
WORKFLOW = yaml.load(
    (ROOT / '.github/workflows/native-test-image.yml').read_text(),
    Loader=yaml.BaseLoader,
)
JOB = WORKFLOW['jobs']['native-test-image']
CHECKOUT, BUILD, NATIVE, UPLOAD = JOB['steps']


def test_hosted_workflow_is_bounded_read_only_and_covers_stacked_prs():
    assert WORKFLOW['on'] == {'pull_request': '', 'workflow_dispatch': ''}
    assert WORKFLOW['permissions'] == {'contents': 'read'}
    assert list(WORKFLOW['jobs']) == ['native-test-image']
    assert JOB['runs-on'] == 'ubuntu-24.04'
    assert 0 < int(JOB['timeout-minutes']) <= 30
    assert CHECKOUT['uses'] == 'actions/checkout@v4'
    assert CHECKOUT['with'] == {'persist-credentials': 'false'}
    assert UPLOAD['if'] == 'always()'
    assert UPLOAD['uses'] == 'actions/upload-artifact@v4'
    assert UPLOAD['with']['if-no-files-found'] == 'error'
    assert 'native-test-image-evidence/' in UPLOAD['with']['path']
    assert NATIVE['run'].index('chmod 755') < NATIVE['run'].index('sudo chown')


@pytest.mark.parametrize('step,kind', [(BUILD, 'build'), (NATIVE, 'native')])
@pytest.mark.parametrize('exit_code', [0, 37])
def test_commands_keep_exit_status_logs_and_narrow_arguments(tmp_path, step, kind, exit_code):
    # A fake Docker executable exercises the actual workflow shell including tee.
    docker = tmp_path / 'docker'
    docker.write_text(
        f'#!{sys.executable}\n'
        'import json, os, sys\n'
        'print(json.dumps(sys.argv[1:]), flush=True)\n'
        'print("docker stderr", file=sys.stderr, flush=True)\n'
        'sys.exit(int(os.environ["DOCKER_TEST_EXIT"]))\n'
    )
    docker.chmod(0o755)
    sudo = tmp_path / 'sudo'
    sudo.write_text('#!/bin/sh\nexit 0\n')
    sudo.chmod(0o755)
    assert step['shell'] == 'bash'
    result = subprocess.run(
        ['bash', '-c', step['run']], cwd=ROOT, capture_output=True, text=True,
        env={**os.environ, 'PATH': f'{tmp_path}{os.pathsep}{os.environ["PATH"]}',
             'RUNNER_TEMP': str(tmp_path), 'DOCKER_TEST_EXIT': str(exit_code)},
        check=False,
    )
    assert result.returncode == exit_code, result.stderr
    log_path = (tmp_path / 'native-test-image-build.log' if kind == 'build' else
                tmp_path / 'native-test-image-evidence/container.log')
    log = log_path.read_text()
    assert log == result.stdout
    assert 'docker stderr' in log
    args = json.loads(log.splitlines()[0])
    if kind == 'build':
        assert args == ['build', '--pull', '--progress=plain', '--file',
                        'docker/Dockerfile.ros-tests', '--tag',
                        'patrolbot-ros-tests:native', '.']
    else:
        # Exact arguments forbid extra mounts, entrypoint overrides and smoke subsets.
        assert args == [
            'run', '--rm', '--network=none', '--ipc=private', '--cap-drop=ALL',
            '--security-opt=no-new-privileges', '--user', '65534:65534',
            '--read-only', '--tmpfs', '/tmp:rw,nosuid,nodev,size=512m',
            '--mount', f'type=bind,src={tmp_path}/native-test-image-evidence/results,dst=/evidence',
            '--env', 'HOME=/tmp', '--env', 'PATROLBOT_TEST_EVIDENCE=/evidence',
            '--env', 'ROS_DOMAIN_ID=101', '--env', 'ROS_LOCALHOST_ONLY=1',
            '--env', 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST',
            '--env', 'PATROLBOT_RUN_INTEGRATION=1',
            '--env', 'PATROLBOT_RUN_MANUAL_UNDOCK_NATIVE=1',
            'patrolbot-ros-tests:native',
        ]
