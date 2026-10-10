#!/usr/bin/env python3
"""Package coordinated local sources with map parity and per-file provenance.

This creates review artifacts only. It never deploys, controls a robot, commits,
pushes, or marks hardware acceptance complete.
"""
import argparse
import hashlib
import json
import runpy
from pathlib import Path
import subprocess
import sys
import tarfile


def git(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args], text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dashboard', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    robot = Path(__file__).resolve().parents[1]
    dashboard = args.dashboard.resolve()
    sys.path.insert(0, str(robot / 'ros2_ws/src/patrolbot_bridge'))
    from patrolbot_bridge.map_catalog import MapCatalog
    canonical = MapCatalog(robot / 'ros2_ws/src/patrolbot_navigation/maps')
    packaged = MapCatalog(dashboard / 'server/maps/catalog')
    revisions = {key: entry['revision'] for key, entry in canonical.maps.items()}
    if revisions != {key: entry['revision'] for key, entry in packaged.maps.items()}:
        raise SystemExit('Robot/dashboard map revision mismatch')
    validator = 'patrolbot_bridge/map_catalog.py'
    if ((robot / 'ros2_ws/src/patrolbot_bridge' / validator).read_bytes() !=
            (dashboard / 'server/app/telemetry/map_catalog.py').read_bytes()):
        raise SystemExit('Robot/dashboard catalog validators differ')
    mock = runpy.run_path(str(dashboard / 'mock-robot/mock_robot/map_context.py'))
    if mock['MAP_REVISIONS'] != revisions:
        raise SystemExit('Mock robot map revisions differ from the canonical catalog')
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = {'schema_version': 1, 'feature_issue': 'https://github.com/cmu-impactlab/patrolbot/issues/66',
                'status': 'candidate; hardware acceptance outstanding',
                'map_context_capability': 'map_context_v1', 'maps': revisions,
                'repositories': {}}
    roots = {
        'robot': (robot, ('ros2_ws/src/', 'hardware/patrolbot_hw_server/', 'docker/',
                         'docs/deployment/floor-maps.md')),
        'dashboard': (dashboard, ('server/', 'frontend/src/', 'frontend/e2e/',
                        'frontend/package', 'frontend/tsconfig', 'frontend/vite',
                        'frontend/index.html', 'frontend/public/', 'shared/', 'mock-robot/',
                        'ros2_ws/src/', 'infrastructure/', 'scripts/verify_floor_maps.py')),
    }
    for label, (root, prefixes) in roots.items():
        listed = subprocess.check_output(['git', '-C', str(root), 'ls-files', '-z',
            '--cached', '--others', '--exclude-standard']).decode().split('\0')
        names = sorted({name for name in listed if name and name.startswith(prefixes)
                        and (root / name).is_file() and not (root / name).is_symlink()})
        files = {name: hashlib.sha256((root/name).read_bytes()).hexdigest() for name in names}
        archive = args.output / (label + '-source.tar.gz')
        with tarfile.open(archive, 'w:gz') as tar:
            for name in names:
                tar.add(root/name, arcname=f'{label}/{name}', recursive=False)
        manifest['repositories'][label] = {
            'head': git(root, 'rev-parse', 'HEAD'), 'dirty': bool(git(root, 'status', '--porcelain')),
            'archive': archive.name, 'archive_sha256': hashlib.sha256(archive.read_bytes()).hexdigest(),
            'files': files,
            'rollback_source_backup': str(root / 'backups/20261007-floor-maps'),
        }
    manifest['deployment_gate'] = ('Capture the actual deployed image digests and state-volume backup '
        'before a separately authorized deployment; no live rollback image was inspected here.')
    path = args.output / 'manifest.json'
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    print(path)


if __name__ == '__main__':
    main()
