"""Explicit opt-in local Docker/Unix-socket acceptance, never production names.

The fixture driver selects only its uniquely named disposable container. This
tests real Docker restart/boot ordering plus the shipped helper/startup code,
not the separately gated production image/volume inventory or motion graph.
"""

import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time
from uuid import uuid4

import pytest

from test_tuning_nav2_restart import rig, HOST  # noqa: F401 - shared isolated store fixture
from patrolbot_bridge.tuning_initialize import RuntimeGuard

pytestmark = pytest.mark.skipif(os.environ.get('PATROLBOT_RUN_DOCKER_TUNING_TESTS') != '1',
    reason='requires explicit local disposable Docker fixture approval')


def test_real_docker_restart_consumes_candidate_once_then_ordinary_restart_uses_active(rig):
    root = Path(__file__).resolve().parents[4]
    name = 'patrolbot-tuning-fixture-' + uuid4().hex[:12]
    docker = ['docker', '--host', 'unix:///var/run/docker.sock']
    with RuntimeGuard(rig.store.base):
        pass  # installed launcher retains a shared read-only guard on this existing file
    def run(*arguments):
        completed = subprocess.run([*docker, *arguments], check=True, capture_output=True,
                                  text=True, timeout=90)
        return completed.stdout + (completed.stderr if arguments[0] == 'logs' else '')
    with tempfile.TemporaryDirectory(prefix='nav2-boot-', dir='/tmp/opencode') as temporary:
        directory = Path(temporary)
        boot, output = directory / 'boot', directory / 'output'
        boot.mkdir(); output.mkdir()
        class FixtureDocker:
            def identity(self):
                data = json.loads(run('inspect', name))[0]
                assert data['Name'] == '/' + name and data['HostConfig']['NetworkMode'] == 'none'
                return {'container_id': data['Id'], 'started_at': data['State']['StartedAt'],
                        'image': data['Image'], 'state_directory': str(rig.store.base)}
            def restart(self):
                assert name.startswith('patrolbot-tuning-fixture-')
                run('restart', '--time', '1', name)
        driver = FixtureDocker()
        supervisor = HOST.Supervisor(rig.store, driver, rig.control)
        server = HOST.Server(boot / 'supervisor.sock', supervisor, boot_only=True)
        thread = threading.Thread(target=server.serve_forever)
        thread.start()
        created = False
        def result():
            deadline = time.monotonic() + 15.
            while not (output / 'boot.json').is_file() and time.monotonic() < deadline:
                time.sleep(.02)
            assert (output / 'boot.json').is_file(), run('logs', name)
            return json.loads((output / 'boot.json').read_text())
        try:
            run('create', '--name', name, '--network=none', '--cap-drop=ALL',
                '--user', f'{os.getuid()}:{os.getgid()}',
                '--security-opt=no-new-privileges', '-e', 'PATROLBOT_ISOLATED_TEST=1',
                '-e', 'PATROLBOT_TUNING_ENABLED=1', '-e', 'PATROLBOT_STATE_DIR=/fixture-state',
                '-v', str(root) + ':/work/robot:ro',
                '-v', str(rig.store.base.parent) + ':/fixture-state:ro',
                '-v', str(boot) + ':/run/patrolbot-tuning/boot:ro',
                '-v', str(output) + ':/fixture-output', '--entrypoint', '/bin/bash',
                'patrolbot-ros-tests:save-apply-core-local', '-c',
                'source /opt/ros/jazzy/setup.bash && source /opt/patrolbot/install/setup.bash && '
                'export PYTHONPATH=/work/robot/ros2_ws/src/patrolbot_bridge:$PYTHONPATH && '
                'exec python3 -B /work/robot/docker/tests/tuning_navigation_boot_fixture.py')
            created = True
            run('start', name)
            initial = result()
            assert initial['state'] == 'active' and initial['pf_z'] != 1.2
            (output / 'boot.json').unlink()
            supervisor.restart(rig.value)
            candidate = result()
            assert candidate['state'] == 'candidate' and candidate['pf_z'] == 1.2
            assert supervisor.status(rig.value)['state'] == 'boot_consumed'
            assert supervisor.restart(rig.value)['state'] == 'boot_consumed'  # status, not a restart
            assert rig.store.read_startup()['revision'] == 1
            (output / 'boot.json').unlink()
            driver.restart()  # separate explicit test of an ordinary unsolicited restart
            ordinary = result()
            assert ordinary['state'] == 'active' and ordinary['pf_z'] == initial['pf_z']
        finally:
            if created:
                run('rm', '--force', name)
            server.shutdown()
            thread.join(timeout=3.)
            server.server_close()
