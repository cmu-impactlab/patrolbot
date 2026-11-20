"""Check hosted smoke policy and real shell failure propagation without Docker."""

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
CHECKOUT, BUILD, SMOKE, UPLOAD = JOB['steps']
TESTS = [f'ros2_ws/src/patrolbot_bridge/test/test_tuning_{name}.py'
         for name in ('backend', 'recover', 'observation_source', 'map_gate',
                      'runtime_contract')]


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
    assert UPLOAD['with']['path'] == '${{ runner.temp }}/native-test-image-*.log'


@pytest.mark.parametrize('step,kind', [(BUILD, 'build'), (SMOKE, 'smoke')])
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
    assert step['shell'] == 'bash'
    result = subprocess.run(
        ['bash', '-c', step['run']], cwd=ROOT, capture_output=True, text=True,
        env={**os.environ, 'PATH': f'{tmp_path}{os.pathsep}{os.environ["PATH"]}',
             'RUNNER_TEMP': str(tmp_path), 'DOCKER_TEST_EXIT': str(exit_code)},
        check=False,
    )
    assert result.returncode == exit_code, result.stderr
    log = (tmp_path / f'native-test-image-{kind}.log').read_text()
    assert log == result.stdout
    assert 'docker stderr' in log
    args = json.loads(log.splitlines()[0])
    if kind == 'build':
        assert args == ['build', '--pull', '--progress=plain', '--file',
                        'docker/Dockerfile.ros-tests', '--tag',
                        'patrolbot-ros-tests:smoke', '.']
    else:
        # Exact arguments forbid mounts, entrypoint overrides and extra suites.
        assert args == [
            'run', '--rm', '--network=none', '--ipc=private', '--cap-drop=ALL',
            '--security-opt=no-new-privileges', '--user', '65534:65534',
            '--read-only', '--tmpfs', '/tmp:rw,nosuid,nodev,size=256m',
            '--env', 'HOME=/tmp', '--env', 'PATROLBOT_RUN_INTEGRATION=0',
            'patrolbot-ros-tests:smoke', *TESTS,
        ]
        assert all((ROOT / path).is_file() for path in TESTS)
