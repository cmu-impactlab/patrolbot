"""Deployment recipes must compile every source they claim to replace."""

from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def recipe(name):
    return (ROOT / 'docker' / name).read_text().replace('\\\n', ' ')


def selected(text):
    return set(re.search(r'--packages-select (.*?) --cmake-args', text).group(1).split())


def test_every_native_patch_package_is_extracted_and_compiled():
    patched = set()
    for path in (ROOT / 'docker/nav2-patches').glob('*.patch'):
        patched.update(re.findall(r'^\+\+\+ b/([^/]+)/', path.read_text(), re.MULTILINE))
    text = recipe('Dockerfile.nav2-fixes')
    extracted = set(re.findall(r'navigation2-1\.3\.12/(\w+)', text))
    assert patched <= extracted
    assert patched <= selected(text)
    assert 'nav2_core' in selected(text)


def test_composed_action_input_boundary_is_compiled_into_all_fixed_action_consumers():
    # SimpleActionServer is a template: replacing nav2_util alone would leave
    # old direct-action admission code baked into these consumers' binaries.
    consumers = {'nav2_util', 'nav2_bt_navigator', 'nav2_behaviors', 'nav2_controller',
                 'nav2_planner', 'nav2_smoother', 'nav2_route', 'nav2_waypoint_follower'}
    text = recipe('Dockerfile.nav2-fixes')
    assert consumers <= selected(text)
    assert consumers <= set(re.findall(r'navigation2-1\.3\.12/(\w+)', text))
    # No additional per-node input services or socket/signing protocol is needed.
    patch = (ROOT / 'docker/nav2-patches/0023-tuning-composed-action-input.patch').read_text()
    assert 'tuning_action_gate.cpp' in patch
    assert 'tuning_action_gate.hpp' in patch
    assert 'create_service' not in '\n'.join(line for line in patch.splitlines() if line.startswith('+'))


def test_inherited_native_source_and_build_outputs_are_removed_before_extraction():
    text = recipe('Dockerfile.nav2-fixes')
    remove = text.index('rm -rf ')
    extract = text.index('tar xzf ')
    assert remove < extract < text.index('patch --batch')
    for name in ('src', 'build', 'install', 'log'):
        assert '/opt/patrolbot_nav2/' + name in text[remove:extract]


def test_robot_overlay_rebuilds_every_copied_runtime_package():
    text = recipe('Dockerfile.ros-overlay')
    rebuilt = selected(text)
    expected = {'patrolbot_interfaces', 'patrolbot_bridge', 'patrolbot_dock_manager',
                'patrolbot_docking_plugins', 'patrolbot_navigation', 'patrolbot-launch'}
    cleaned = set(re.search(r'for package in (.*?); do', text).group(1).split())
    assert expected <= rebuilt
    assert expected <= cleaned


def test_robot_overlay_checks_native_provenance_before_replacing_patch_metadata():
    text = recipe('Dockerfile.ros-overlay')
    check = text.index('test -d /opt/patrolbot-nav2-patches')
    replace = text.index('COPY docker/nav2-patches ')
    assert check < replace


def test_native_build_aborts_if_an_earlier_patch_fails(tmp_path):
    patches = tmp_path / 'patches'
    patches.mkdir()
    (patches / '0001.patch').write_text('first')
    (patches / '0002.patch').write_text('second')
    text = recipe('Dockerfile.nav2-fixes')
    loop = re.search(r'for patch_file in .*?; done', text).group(0)
    loop = loop.replace('/opt/patrolbot-nav2-patches', str(patches))
    # An early failure must not be hidden by a later successful patch. No real
    # patch executable or upstream checkout is needed for this shell regression.
    result = subprocess.run(['sh', '-c', 'patch() { read -r name; '
                             '[ "$name" != first ]; }; ' + loop])
    assert result.returncode != 0
