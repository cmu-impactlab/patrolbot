#!/usr/bin/env python3
"""Compare a recorded deployment with Git files and read-only remote hashes.

No ROS imports, control requests, container creation, or service changes.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--live', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    manifest = json.loads(args.manifest.read_text())
    failures = []
    for entry in manifest['files']:
        path = root / entry['repository_path']
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != entry['sha256']:
            failures.append('repository mismatch: ' + entry['repository_path'])
    if args.live:
        # Remote values are read only. Arguments come from a reviewed manifest,
        # not from robot messages or browser requests.
        program = '''import hashlib,json,subprocess,sys
m=json.load(sys.stdin)
failures=[]
docker=m.get('docker_command',['docker'])
for c in m.get('containers',[]):
    actual=json.loads(subprocess.check_output(docker+['inspect',c['name']]))[0]
    if actual['Image'] != c['image']:
        failures.append('image mismatch: '+c['name'])
for e in m['files']:
    if e.get('container'):
        data=subprocess.check_output(docker+['exec',e['container'],'cat',e['remote_path']])
    else:
        with open(e['remote_path'],'rb') as f: data=f.read()
    if hashlib.sha256(data).hexdigest()!=e['sha256']:
        failures.append('remote mismatch: '+e['remote_path'])
for e in m.get('executables',[]):
    pid=subprocess.check_output(e['pid_command']).decode().strip()
    with open('/proc/'+pid+'/exe','rb') as f: actual=hashlib.sha256(f.read()).hexdigest()
    if actual!=e['sha256']: failures.append('running executable mismatch: '+e['name'])
print(json.dumps(failures))
'''
        import shlex
        for host in sorted({e['host'] for e in manifest['files']}):
            payload = {key: [e for e in manifest.get(key, []) if e['host'] == host]
                       for key in ('files', 'containers', 'executables')}
            payload['docker_command'] = manifest.get('docker_commands', {}).get(host, ['docker'])
            command = 'python3 -B -c ' + shlex.quote(program)
            result = subprocess.run(
                ['/usr/bin/ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', host, command],
                input=json.dumps(payload), text=True, capture_output=True, timeout=120)
            if result.returncode:
                failures.append('read-only inspection failed: ' + host)
                print(result.stderr, file=sys.stderr)
            else:
                failures.extend(json.loads(result.stdout))
        for asset in manifest.get('assets', []):
            try:
                with urllib.request.urlopen(manifest['public_url'] + asset['path'], timeout=15) as response:
                    digest = hashlib.sha256(response.read()).hexdigest()
                if digest != asset['sha256']:
                    failures.append('public asset mismatch: ' + asset['path'])
            except Exception as error:
                failures.append('public asset unavailable: ' + asset['path'] + ': ' + str(error))
    for failure in failures:
        print(failure, file=sys.stderr)
    print(json.dumps({'passed': not failures, 'files': len(manifest['files']),
                      'live': args.live, 'physical_acceptance': False}))
    return bool(failures)


if __name__ == '__main__':
    raise SystemExit(main())
