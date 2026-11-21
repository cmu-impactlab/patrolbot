"""Keep the native test recipe's source fixtures and discovery usable offline."""

import ast
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[2]
RECIPE = (ROOT / 'docker/Dockerfile.ros-tests').read_text().replace('\\\n', ' ')
DEPS, IMAGE = RECIPE.split('FROM ros-test-env AS ros-tests\n')
BUILT = ('patrolbot_interfaces', 'patrolbot_bridge', 'patrolbot_dock_manager',
         'patrolbot_navigation')


def copy_sources(recipe, destination):
    """Materialize this recipe's plain COPY inputs without Docker or ROS."""
    for line in recipe.splitlines():
        if not line.startswith('COPY '):
            continue
        *sources, target = shlex.split(line)[1:]
        if target.startswith('/'):
            continue  # The entrypoint is outside the source-fixture tree.
        for source in sources:
            src = ROOT / source
            dst = destination / target
            if src.is_dir():
                shutil.copytree(src, dst, dirs_exist_ok=True)
            else:
                if target.endswith('/'):
                    dst /= src.name
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dst)


def test_selected_source_fixtures_are_copied(tmp_path):
    copy_sources(IMAGE, tmp_path)
    # Cross-package/repository inputs read by the default bridge/navigation suites.
    fixtures = (
        'contracts/robot-tuning.registry.json',
        'contracts/robot-tuning-runtime.json',
        'contracts/web-teleop.json',
        'ros2_ws/src/patrolbot-launch/param/defaults/mux.yaml',
        'ros2_ws/src/patrolbot-launch/param/defaults/smoother.yaml',
        'ros2_ws/src/patrolbot-launch/lifecycle_mgr.py',
        'ros2_ws/src/patrolbot_docking_plugins/src/patrolbot_charging_dock.cpp',
        'hardware/patrolbot_hw_server/patrolbot_server.cpp',
        'hardware/patrolbot_hw_server/maintenance',
        'docker/Dockerfile', 'docker/docker-compose.yml', 'docker/status.sh',
        'docker/source-tree-digest.sh', 'docker/capture-nav-goal.sh',
        'docker/nav2-patches/0018-tuning-navigation-admission.patch',
    )
    for source in fixtures:
        assert (tmp_path / source).exists(), source
    for package in BUILT:
        for source in (ROOT / 'ros2_ws/src' / package).rglob('*.py'):
            assert (tmp_path / source.relative_to(ROOT)).is_file(), source
    # The patch fixture must also survive the restricted Docker build context.
    ignore = (ROOT / '.dockerignore').read_text().splitlines()
    assert '!docker/nav2-patches/' in ignore
    assert '!docker/nav2-patches/**' in ignore


def test_rosdep_sees_built_manifests_and_local_dependencies(tmp_path):
    copy_sources(DEPS, tmp_path)
    manifests = {ET.parse(path).findtext('name'): ET.parse(path).getroot()
                 for path in tmp_path.glob('*/package.xml')}
    assert set(BUILT) <= manifests.keys()
    local = {ET.parse(path).findtext('name')
             for path in (ROOT / 'ros2_ws/src').glob('*/package.xml')}
    for manifest in manifests.values():
        for dependency in manifest:
            if dependency.tag.endswith('depend') and dependency.text in local:
                assert dependency.text in manifests, dependency.text
    assert '--ignore-src' in DEPS
    assert 'python3-pytest-timeout' in shlex.split(DEPS, comments=True)


def test_dock_manager_declares_setuptools_and_exports_ament_python():
    manifest = ET.parse(ROOT / 'ros2_ws/src/patrolbot_dock_manager/package.xml')
    assert manifest.findtext('export/build_type') == 'ament_python'
    dependencies = [node.text for node in manifest.getroot()
                    if node.tag.endswith('depend')]
    assert 'ament_python' not in dependencies
    assert manifest.findtext('buildtool_depend') == 'python3-setuptools'


@pytest.mark.parametrize('sources_exist,failure', [
    (False, ''), (True, ''), (False, 'rosdep init'),
    (False, 'apt-get install'), (True, 'rosdep update'),
    (True, 'apt-get update'), (True, 'rosdep install'), (True, 'rosdep check'),
])
def test_rosdep_run_shell_propagates_failures(tmp_path, sources_exist, failure):
    # Execute the recipe's RUN bodies; only redirect the default-file probe.
    sources = tmp_path / '20-default.list'
    if sources_exist:
        sources.touch()
    log = tmp_path / 'commands.jsonl'
    for name in ('apt-get', 'rosdep', 'rm'):
        executable = tmp_path / name
        executable.write_text(
            f'#!{sys.executable}\n'
            'import json, os, pathlib, sys\n'
            'args = [pathlib.Path(sys.argv[0]).name, *sys.argv[1:]]\n'
            'with open(os.environ["COMMAND_LOG"], "a") as log:\n'
            '    log.write(json.dumps(args) + "\\n")\n'
            'sys.exit(41 if " ".join(args[:2]) == os.environ["FAIL_COMMAND"] else 0)\n'
        )
        executable.chmod(0o755)
    runs = [line.removeprefix('RUN ') for line in DEPS.splitlines()
            if line.startswith('RUN ')]
    assert len(runs) == 2
    for run in runs:
        result = subprocess.run(
            ['/bin/sh', '-c', run.replace(
                '/etc/ros/rosdep/sources.list.d/20-default.list', shlex.quote(str(sources)))],
            cwd=tmp_path, capture_output=True, text=True,
            env={**os.environ, 'PATH': str(tmp_path), 'COMMAND_LOG': str(log),
                 'FAIL_COMMAND': failure}, check=False,
        )
        if result.returncode:
            break
    commands = [json.loads(line) for line in log.read_text().splitlines()]
    names = [' '.join(args[:2]) for args in commands]
    expected = ['apt-get update', 'apt-get install', 'rm -rf']
    if not sources_exist:
        expected.append('rosdep init')
    expected += ['rosdep update', 'apt-get update', 'rosdep install',
                 'rosdep check', 'rm -rf']
    if failure:
        expected = expected[:expected.index(failure) + 1]
    assert result.returncode == (41 if failure else 0), result.stderr
    assert names == expected
    for args in commands:
        if args[:2] in (['rosdep', 'install'], ['rosdep', 'check']):
            expected_args = ['rosdep', args[1], '--from-paths', '.', '--ignore-src']
            if args[1] == 'install':
                expected_args.append('-y')
            assert args == [*expected_args, '--rosdistro', 'jazzy']


def test_native_selection_keeps_fixture_only_packages_unbuilt():
    selection = IMAGE.split('--packages-select ', 1)[1].splitlines()[0]
    assert tuple(shlex.split(selection)) == BUILT
    runner = (ROOT / 'docker/run-ros-tests.sh').read_text()
    helper = (ROOT / 'docker/native_ci_gate.py').read_text()
    for package in BUILT[1:]:
        assert package.removeprefix("patrolbot_") in helper
    assert 'test/test_collision_monitor_integration.py::' in helper
    assert 'PATROLBOT_RUN_MANUAL_UNDOCK_NATIVE=1' in runner
    assert '!docker/native_ci_gate.py' in (ROOT / '.dockerignore').read_text()
    assert 'docker/native_ci_gate.py /usr/local/bin/' in RECIPE
    assert 'no:nose' not in RECIPE + runner


@pytest.mark.parametrize('name', ['backend', 'recover', 'observation_source', 'map_gate'])
def test_test_factories_cannot_be_discovered_as_nose_hooks(name):
    path = ROOT / f'ros2_ws/src/patrolbot_bridge/test/test_tuning_{name}.py'
    module = ast.parse(path.read_text())
    functions = {node.name for node in module.body if isinstance(node, ast.FunctionDef)}
    assert functions.isdisjoint({'setup', 'teardown'})
    assert any(name.startswith('make_') for name in functions)
