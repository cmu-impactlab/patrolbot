"""This robot has no gimbal: do not ship or deploy support for absent hardware."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ROOT / 'ros2_ws/src'


def test_gimbal_package_and_interfaces_are_absent():
    assert not (PACKAGES / 'patrolbot_gimbal').exists()
    interfaces = PACKAGES / 'patrolbot_interfaces'
    for name in ('GimbalCommand', 'GimbalState'):
        assert not (interfaces / 'msg' / (name + '.msg')).exists()
    assert 'gimbal' not in (interfaces / 'CMakeLists.txt').read_text().lower()


def test_compose_has_no_gimbal_service_profile_device_or_mount():
    files = [
        path for path in (ROOT / 'docker').rglob('*')
        if path.is_file() and 'compose' in path.name
        and path.suffix in ('.yml', '.yaml')
    ]
    assert files
    for path in files:
        assert 'gimbal' not in path.read_text().lower(), str(path.relative_to(ROOT))


def test_image_recipes_and_source_digest_do_not_include_gimbal():
    files = list((ROOT / 'docker').glob('Dockerfile*'))
    files += [
        ROOT / 'docker/source-tree-digest.sh',
        ROOT / 'docker/.env.example',
        ROOT / '.dockerignore',
    ]
    for path in files:
        assert 'gimbal' not in path.read_text().lower(), str(path.relative_to(ROOT))


def test_ros_runtime_sources_do_not_reference_gimbal():
    suffixes = {'.py', '.xml', '.yaml', '.yml', '.msg', '.srv', '.action', '.txt'}
    for path in PACKAGES.rglob('*'):
        relative = path.relative_to(PACKAGES)
        if not path.is_file() or {'test', '__pycache__'} & set(relative.parts):
            continue
        if path.suffix in suffixes:
            assert 'gimbal' not in path.read_text().lower(), str(relative)
