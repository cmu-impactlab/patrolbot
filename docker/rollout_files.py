"""File-only rollout transaction; restart/preflight are separate orchestration gates."""
import hashlib
import json
import os
from pathlib import Path
import shutil


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replace_preserving_mode(source, target):
    staged = target.with_name(target.name + '.patrolbot-stage')
    if staged.exists():
        raise RuntimeError('staging path already exists: ' + str(staged))
    try:
        shutil.copy2(target, staged)
        staged.write_bytes(source.read_bytes())
        os.replace(staged, target)
    finally:
        if staged.exists():
            staged.unlink()


def snapshot(root, backup, changes):
    for change in changes:
        name = change['path']
        if Path(name).is_absolute() or '..' in Path(name).parts:
            raise ValueError('unsafe relative path')
        assert digest(root / name) == change['before'], name
    assert not backup.exists(), 'backup must be new'
    backup.mkdir(parents=True, mode=0o700)
    names = [item['path'] for item in changes] + ['docker/.env']
    for name in names:
        dest = backup / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(root / name, dest)
    records = {name: digest(backup / name) for name in names}
    (backup / 'files.json').write_text(json.dumps(records, sort_keys=True))
    return records


def restore(root, backup):
    records = json.loads((backup / 'files.json').read_text())
    # Verify the entire snapshot before restoring anything.
    for name, expected in records.items():
        assert digest(backup / name) == expected, name
    for name in records:
        replace_preserving_mode(backup / name, root / name)


def install(root, payload, backup, changes, old_image, new_image, verify):
    assert old_image != new_image
    for change in changes:
        assert digest(payload / change['path']) == change['after'], change['path']
    env = root / 'docker/.env'
    original = env.read_text()
    matching = [line for line in original.splitlines() if line.startswith('PATROLBOT_IMAGE=')]
    assert matching == ['PATROLBOT_IMAGE=' + old_image], 'unexpected current image'
    snapshot(root, backup, changes)
    try:
        for change in changes:
            replace_preserving_mode(payload / change['path'], root / change['path'])
        staged_env = backup / 'candidate.env'
        staged_env.write_text('\n'.join(
            'PATROLBOT_IMAGE=' + new_image if line.startswith('PATROLBOT_IMAGE=') else line
            for line in original.splitlines()) + '\n')
        replace_preserving_mode(staged_env, env)
        verify()
    except BaseException:
        restore(root, backup)
        raise
