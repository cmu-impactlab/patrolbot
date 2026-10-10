"""Meaningful rejection cases for completed-soak and backup-retirement gates."""
import copy
import json
from pathlib import Path
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rollout_acceptance import archive_verified, require_soak, retire_verified


def test_soak_gate():
    baseline = {'Image': 'pinned', 'State': {'Status': 'exited', 'Running': False,
                'ExitCode': 0, 'OOMKilled': False}, 'Mounts': [],
                'HostConfig': {'NetworkMode': 'none', 'IpcMode': 'private', 'Devices': [], 'CapDrop': ['ALL']}}
    evidence = [{'event': 'outage_started', 'wall_seconds': 28800},
                {'event': 'outage_acceptance', 'wall_seconds': 28802,
                 'recovery_without_restart': True, 'nonzero_hold': False, 'hold_messages': 288000},
                {'event': 'managed_shutdown_complete'}]
    for case in ('accepted', 'running', 'failed_exit', 'oom', 'wrong_image', 'network',
                 'device', 'privileged', 'capabilities', 'mount', 'fixture', 'short_outage', 'missing_recovery',
                 'nonzero_hold', 'missing_shutdown', 'nan_duration', 'duplicate_result'):
        info, events, fixture = copy.deepcopy(baseline), copy.deepcopy(evidence), b'committed'
        if case == 'running': info['State'].update(Status='running', Running=True)
        if case == 'failed_exit': info['State']['ExitCode'] = 1
        if case == 'oom': info['State']['OOMKilled'] = True
        if case == 'wrong_image': info['Image'] = 'other'
        if case == 'network': info['HostConfig']['NetworkMode'] = 'host'
        if case == 'device': info['HostConfig']['Devices'] = ['robot']
        if case == 'privileged': info['HostConfig']['Privileged'] = True
        if case == 'capabilities': info['HostConfig']['CapDrop'] = []
        if case == 'mount': info['Mounts'] = ['robot']
        if case == 'fixture': fixture = b'changed'
        if case == 'short_outage': events[1]['wall_seconds'] = 60
        if case == 'missing_recovery': events[1]['recovery_without_restart'] = False
        if case == 'nonzero_hold': events[1]['nonzero_hold'] = True
        if case == 'missing_shutdown': events.pop()
        if case == 'nan_duration': events[1]['wall_seconds'] = float('nan')
        if case == 'duplicate_result': events.append(events[1])
        refused = False
        try:
            require_soak(info, '\n'.join(json.dumps(e) for e in events), 'pinned', fixture, b'committed')
        except RuntimeError:
            refused = True
        assert refused == (case != 'accepted'), case
        print('PASS soak', case)


def test_backup_retirement():
    for case in ('verified', 'source_changed', 'archive_changed', 'mode_changed', 'link_changed'):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, archive = root / 'snapshot', root / 'offhost'
            source.mkdir()
            (source / 'env').write_text('private-data')
            (source / 'env').chmod(0o600)
            (source / 'link').symlink_to('env')
            expected = archive_verified(source, archive)
            if case == 'source_changed': (source / 'env').write_text('changed')
            if case == 'archive_changed': (archive / 'env').write_text('changed')
            if case == 'mode_changed': (archive / 'env').chmod(0o644)
            if case == 'link_changed':
                (archive / 'link').unlink()
                (archive / 'link').symlink_to('elsewhere')
            refused = False
            try:
                retire_verified(source, archive, expected)
            except RuntimeError:
                refused = True
            assert refused == (case != 'verified'), case
            assert source.exists() == refused
            assert archive.exists()
            print('PASS backup', case)


if __name__ == '__main__':
    test_soak_gate()
    test_backup_retirement()
