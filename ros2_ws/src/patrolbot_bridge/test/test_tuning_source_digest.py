"""Unit fixtures prove contract edits affect source provenance in both layouts."""

from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = ROOT / 'docker/source-tree-digest.sh'
PACKAGES = ('patrolbot_interfaces', 'patrolbot_bridge', 'patrolbot_dock_manager',
            'patrolbot_docking_plugins', 'patrolbot-launch', 'patrolbot_navigation')


def fixture(root, layout):
    for package in PACKAGES:
        directory = root / layout / package
        directory.mkdir(parents=True)
        (directory / 'source.py').write_text('same source\n')
    hardware = root / 'hardware/patrolbot_hw_server'
    hardware.mkdir(parents=True)
    (hardware / 'protocol.cpp').write_text('same hardware\n')
    (root / 'contracts').mkdir()
    (root / 'contracts/robot-tuning.registry.json').write_text('{"limit":0.3}\n')


def digest(root):
    return subprocess.check_output(['bash', str(SCRIPT), str(root)], text=True).strip()


def test_contract_changes_are_part_of_built_source_provenance(tmp_path):
    checkout, image = tmp_path / 'checkout', tmp_path / 'image'
    fixture(checkout, 'ros2_ws/src')
    fixture(image, 'src')
    assert digest(checkout) == digest(image)
    original = digest(checkout)
    (checkout / 'contracts/robot-tuning.registry.json').write_text('{"limit":0.2}\n')
    assert digest(checkout) != original
    (image / 'contracts/robot-tuning.registry.json').write_text('{"limit":0.2}\n')
    assert digest(checkout) == digest(image)
