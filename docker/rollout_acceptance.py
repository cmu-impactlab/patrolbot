"""Acceptance and immutable backup checks used before a robot rollout."""
import hashlib
import json
import math
from pathlib import Path
import shutil
import stat


def require_soak(info, output, image_id, fixture, expected_fixture):
    state = info['State']
    if (state.get('Status') != 'exited' or state.get('Running')
            or state.get('ExitCode') != 0 or state.get('OOMKilled')
            or info['Image'] != image_id):
        raise RuntimeError('soak has not completed successfully on the pinned image')
    host = info['HostConfig']
    if (host.get('NetworkMode') != 'none' or host.get('IpcMode') != 'private'
            or host.get('Devices') or host.get('DeviceRequests') or host.get('Privileged')
            or 'ALL' not in (host.get('CapDrop') or []) or info.get('Mounts')):
        raise RuntimeError('soak isolation does not match acceptance requirements')
    if fixture != expected_fixture:
        raise RuntimeError('soak fixture differs from the committed candidate')
    events = []
    for line in output.splitlines():
        try:
            value = json.loads(line)
        except ValueError:
            continue
        if isinstance(value, dict):
            events.append(value)
    starts = [e for e in events if e.get('event') == 'outage_started']
    results = [e for e in events if e.get('event') == 'outage_acceptance']
    if len(starts) != 1 or starts[0].get('wall_seconds') != 28800 or len(results) != 1:
        raise RuntimeError('missing complete eight-hour outage evidence')
    result = results[0]
    duration = result.get('wall_seconds', 0)
    if (not isinstance(duration, (int, float)) or not math.isfinite(duration)
            or duration < 28800 or result.get('recovery_without_restart') is not True
            or result.get('nonzero_hold') is not False
            or result.get('hold_messages', 0) <= 0
            or not any(e.get('event') == 'managed_shutdown_complete' for e in events)):
        raise RuntimeError('soak recovery or managed shutdown acceptance missing')
    return result


def records(root):
    result = {}
    for path in sorted(Path(root).rglob('*')):
        name = str(path.relative_to(root))
        mode = stat.S_IMODE(path.lstat().st_mode)
        if path.is_symlink():
            result[name] = ('link', str(path.readlink()), mode)
        elif path.is_file():
            result[name] = ('file', hashlib.sha256(path.read_bytes()).hexdigest(), mode)
        elif path.is_dir():
            result[name] = ('directory', mode)
        else:
            raise RuntimeError('unsupported backup entry: ' + name)
    if not result:
        raise RuntimeError('empty backup')
    return result


def archive_verified(snapshot, archive):
    before = records(snapshot)
    shutil.copytree(snapshot, archive, symlinks=True)
    if records(snapshot) != before or records(archive) != before:
        raise RuntimeError('backup changed or off-host copy verification failed')
    return before


def retire_verified(snapshot, archive, expected):
    if records(snapshot) != expected or records(archive) != expected:
        raise RuntimeError('refusing to retire an unverified backup')
    shutil.rmtree(snapshot)
