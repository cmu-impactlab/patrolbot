from pathlib import Path
import subprocess


def test_digest_ignores_generated_backups_but_tracks_source(tmp_path):
    script = Path(__file__).resolve().parents[1] / 'source-tree-digest.sh'
    workspace = tmp_path / 'backups/candidate'
    for package in ('patrolbot_interfaces', 'patrolbot_bridge', 'patrolbot_dock_manager',
                    'patrolbot_docking_plugins', 'patrolbot-launch', 'patrolbot_navigation'):
        (workspace / 'src' / package).mkdir(parents=True)
    hardware = workspace / 'hardware/patrolbot_hw_server'
    hardware.mkdir(parents=True)
    source = workspace / 'src/patrolbot_bridge/bridge.py'
    source.write_text('original\n')

    def digest():
        return subprocess.check_output(['bash', str(script), str(workspace)], text=True)

    original = digest()
    cache = source.parent / 'backups/pytest-cache'
    cache.mkdir(parents=True)
    (cache / 'nodeids').write_text('generated cache\n')
    source.with_suffix('.py.orig').write_text('patch backup\n')
    source.with_suffix('.py.rej').write_text('rejected patch\n')
    assert digest() == original
    source.write_text('changed\n')
    assert digest() != original
