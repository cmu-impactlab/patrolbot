"""Docker adapters for an authorized rollout; constructing this class changes nothing."""
import copy
import json
from pathlib import Path
import subprocess
import time


# Runs only in a read-only, network-isolated candidate container.
IMAGE_PROBE = r'''
from pathlib import Path
import hashlib,json,subprocess
root=Path('/opt/patrolbot_ws')
source=subprocess.check_output(['bash',str(root/'source-tree-digest.sh'),str(root)],text=True).strip()
h=hashlib.sha256()
for p in sorted(Path('/opt/patrolbot-nav2-patches').glob('*.patch')):
    h.update(p.name.encode()+b'\0'+p.read_bytes())
pairs=[
 ('patrolbot_dock_manager/patrolbot_dock_manager/dock_manager.py','patrolbot_dock_manager/lib/python3.12/site-packages/patrolbot_dock_manager/dock_manager.py'),
 ('patrolbot_navigation/scripts/patrolbot_joy_teleop.py','patrolbot_navigation/lib/patrolbot_navigation/patrolbot_joy_teleop.py'),
 ('patrolbot_navigation/scripts/patrolbot_controller_feedback.py','patrolbot_navigation/lib/patrolbot_navigation/patrolbot_controller_feedback.py'),
 ('patrolbot_navigation/launch/bringup.launch.py','patrolbot_navigation/share/patrolbot_navigation/launch/bringup.launch.py'),
 ('patrolbot_navigation/config/nav2_params.yaml','patrolbot_navigation/share/patrolbot_navigation/config/nav2_params.yaml')]
installed=all((root/'src'/a).read_bytes()==(root/'install'/b).read_bytes() for a,b in pairs)
print(json.dumps({'source':source,'patches':h.hexdigest(),'installed':installed}))
'''


def command(args, *, input=None, timeout=60):
    # Never echo inspect/config output: it can contain deployment credentials.
    return subprocess.run(args, input=input, text=True, capture_output=True,
                          check=True, timeout=timeout).stdout


class DockerRollout:
    def __init__(self, root, manifest, *, run=command, clock=time.monotonic,
                 sleep=time.sleep):
        self.root = Path(root)
        self.manifest = copy.deepcopy(manifest)
        self.run, self.clock, self.sleep = run, clock, sleep
        if manifest['services'] != ['bringup', 'bridge', 'navigation']:
            raise ValueError('unexpected deployment services')
        self.ids = {}
        self.compose = ['docker', 'compose', '--env-file', str(self.root / 'docker/.env'),
                        '-f', str(self.root / 'docker/docker-compose.yml')]

    def inspect_image(self, image):
        return json.loads(self.run(['docker', 'image', 'inspect', image]))[0]

    def source_digest(self):
        return self.run(['bash', str(self.root / 'docker/source-tree-digest.sh'),
                         str(self.root)]).strip()

    def verify_candidate(self, manifest):
        if manifest != self.manifest:
            raise ValueError('manifest changed')
        m = manifest
        old = self.inspect_image(m['expected_previous_image'])
        new = self.inspect_image(m['candidate_image'])
        if old['Id'] != m['expected_previous_image_id']:
            return False
        labels = new['Config'].get('Labels', {})
        expected = {
            'org.opencontainers.image.revision': m['candidate_revision'],
            'io.patrolbot.source-tree-sha256': m['candidate_source_sha256'],
            'io.patrolbot.nav2-patch-sha256': m['candidate_nav2_patch_sha256'],
        }
        if new['Id'] != m['candidate_image_id'] or any(
                labels.get(k) != value for k, value in expected.items()):
            return False
        if old['Config'].get('Labels', {}).get('io.patrolbot.source-tree-sha256') != m['expected_previous_source_sha256']:
            return False
        configs = [{k: v for k, v in image['Config'].items() if k != 'Labels'}
                   for image in (old, new)]
        if configs[0] != configs[1] or self.source_digest() != m['expected_previous_source_sha256']:
            return False
        probe = json.loads(self.run([
            'docker', 'run', '--rm', '-i', '--network', 'none', '--ipc', 'private',
            '--read-only', '--cap-drop', 'ALL', '--memory', '512m', '--cpus', '1',
            '--entrypoint', 'python3', new['Id'], '-'], input=IMAGE_PROBE))
        if probe != {'source': m['candidate_source_sha256'],
                     'patches': m['candidate_nav2_patch_sha256'], 'installed': True}:
            return False
        self.ids = {m['expected_previous_image']: old['Id'], m['candidate_image']: new['Id']}
        try:
            verified = self.postflight(m['expected_previous_image'])
        except BaseException:
            self.ids = {}
            raise
        if not verified:
            self.ids = {}
        return verified

    def restart(self, image):
        if image not in self.ids or self.inspect_image(image)['Id'] != self.ids[image]:
            raise RuntimeError('unverified or changed image tag')
        resolved = json.loads(self.run(self.compose + ['config', '--format', 'json']))
        if any(resolved['services'][s]['image'] != image for s in self.manifest['services']):
            raise RuntimeError('Compose does not select the verified image')
        # Do not kill a timed-out Compose client and then race an in-flight
        # daemon operation with rollback. Await the mutation's terminal result.
        self.run(self.compose + ['up', '-d', '--no-build', '--no-deps',
                                 *self.manifest['services']], timeout=None)

    def preflight(self, capture_script):
        """Run the reviewed subscriber-only capture immediately before writes."""
        return json.loads(self.run([
            'docker', 'exec', '-i', 'patrolbot-web-bridge', 'bash', '-c',
            'source /opt/patrolbot_ws/install/setup.bash && python3 -'],
            input=Path(capture_script).read_text(), timeout=20))

    def postflight(self, image):
        if image not in self.ids:
            return False
        key = ('candidate_source_sha256' if image == self.manifest['candidate_image']
               else 'expected_previous_source_sha256')
        if self.source_digest() != self.manifest[key]:
            return False
        deadline = self.clock() + 90
        while True:
            containers = json.loads(self.run([
                'docker', 'inspect', *('patrolbot-' + s for s in self.manifest['services'])]))
            if len(containers) != len(self.manifest['services']):
                return False
            if any(c['Image'] != self.ids[image] for c in containers):
                return False
            if all(c['State']['Running'] and not c['State'].get('OOMKilled', False)
                   and c['State'].get('Health', {}).get('Status') == 'healthy'
                   and c.get('RestartCount') == 0 for c in containers):
                return True
            if self.clock() >= deadline:
                return False
            self.sleep(2)
