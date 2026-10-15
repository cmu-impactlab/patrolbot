"""Hash-guarded image-build overlay. Default is read-only verification.

Not a deployment/restart tool. --apply is permitted only inside a build
container with PATROLBOT_SOURCE_OVERLAY_BUILD=1. Parent owns image selection,
rollback, all remote operations and eventual runtime bind changes.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--role', choices=('core', 'web'), required=True)
    parser.add_argument('--root', type=Path, required=True,
                        help='Image filesystem root; never a source checkout')
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    package = Path(__file__).resolve().parent
    manifest = json.loads((package / 'source-manifest.json').read_text())
    selected = [r for r in manifest['modules'] if r['role'] == args.role]
    root = args.root.resolve(strict=True)
    checked = []
    for row in selected:
        target = root / row['installed_path'].lstrip('/')
        source = package / 'source' / row['path']
        if not target.resolve(strict=True).is_relative_to(root):
            raise SystemExit('Target escapes image root: ' + str(target))
        if sha(source) != row['candidate_sha256']:
            raise SystemExit('Candidate hash mismatch: ' + row['path'])
        if sha(target) != row['baseline_sha256']:
            raise SystemExit('Installed baseline mismatch: ' + str(target))
        checked.append((row, source, target))
    # All originals and candidates must match before the first write.
    if args.apply:
        if os.environ.get('PATROLBOT_SOURCE_OVERLAY_BUILD') != '1' or not Path('/.dockerenv').exists():
            raise SystemExit('--apply requires an explicitly scoped image-build container')
        for row, source, target in checked:
            if not row['changed']:
                continue
            staged = target.with_name(target.name + '.image-overlay-new')
            shutil.copyfile(source, staged)
            staged.chmod(target.stat().st_mode & 0o777)
            os.replace(staged, target)
            if sha(target) != row['candidate_sha256']:
                raise SystemExit('Post-copy mismatch: ' + row['path'])
    print(json.dumps({'role': args.role, 'verified': len(checked),
                      'applied': args.apply,
                      'manifest_sha256': sha(package / 'source-manifest.json')}))


if __name__ == '__main__':
    main()
