"""Keep the native test recipe's source fixtures and discovery usable offline."""

import ast
from pathlib import Path
import shlex
import shutil
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


def test_native_selection_keeps_fixture_only_packages_unbuilt():
    selection = IMAGE.split('--packages-select ', 1)[1].splitlines()[0]
    assert tuple(shlex.split(selection)) == BUILT
    runner = (ROOT / 'docker/run-ros-tests.sh').read_text()
    for package in BUILT[1:]:
        assert f'cd "$workspace/ros2_ws/src/{package}"' in runner
    assert '--ignore=test/test_collision_monitor_integration.py' in runner
    assert 'PATROLBOT_RUN_MANUAL_UNDOCK_NATIVE=1' not in RECIPE + runner
    assert 'no:nose' not in RECIPE + runner


@pytest.mark.parametrize('name', ['backend', 'recover', 'observation_source', 'map_gate'])
def test_test_factories_cannot_be_discovered_as_nose_hooks(name):
    path = ROOT / f'ros2_ws/src/patrolbot_bridge/test/test_tuning_{name}.py'
    module = ast.parse(path.read_text())
    functions = {node.name for node in module.body if isinstance(node, ast.FunctionDef)}
    assert functions.isdisjoint({'setup', 'teardown'})
    assert any(name.startswith('make_') for name in functions)
