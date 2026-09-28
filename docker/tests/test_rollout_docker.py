"""Docker adapter contract tests; no Docker or network access."""
import copy
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rollout_docker import DockerRollout, IMAGE_PROBE


def test_adapters():
    compile(IMAGE_PROBE, '<image-probe>', 'exec')
    manifest = dict(services=['bringup', 'bridge', 'navigation'],
                    expected_previous_image='old', candidate_image='new',
                    candidate_image_id='sha256:new', candidate_revision='revision',
                    expected_previous_image_id='sha256:old',
                    candidate_source_sha256='new-source', candidate_nav2_patch_sha256='patches',
                    expected_previous_source_sha256='old-source')
    for case in ('success', 'wrong_previous_id', 'wrong_label', 'wrong_embedded', 'wrong_host_source',
                 'runtime_config_change', 'tag_drift', 'wrong_compose_image',
                 'mixed_containers', 'unhealthy', 'oom', 'restarting', 'no_health'):
        calls = []
        phase = ['before']
        ticks = [0]

        def clock():
            ticks[0] += 30
            return ticks[0]

        def run(args, **kwargs):
            calls.append((args, kwargs))
            if args[:3] == ['docker', 'image', 'inspect']:
                new = args[3] == 'new'
                labels = {'io.patrolbot.source-tree-sha256': 'new-source' if new else 'old-source'}
                if new:
                    labels.update({'org.opencontainers.image.revision': 'revision',
                                   'io.patrolbot.nav2-patch-sha256': 'patches'})
                    if case == 'wrong_label':
                        labels['io.patrolbot.source-tree-sha256'] = 'inherited'
                config = {'Labels': labels, 'Env': ['PRIVATE=opaque']}
                if new and case == 'runtime_config_change':
                    config['Cmd'] = ['different']
                identity = 'sha256:new' if new else 'sha256:old'
                if not new and case == 'wrong_previous_id':
                    identity = 'sha256:unexpected-old'
                if new and phase[0] == 'after' and case == 'tag_drift':
                    identity = 'sha256:replaced'
                return json.dumps([{'Id': identity, 'Config': config}])
            if args[0] == 'bash':
                if case == 'wrong_host_source':
                    return 'unreviewed-source'
                return 'new-source' if phase[0] == 'after' else 'old-source'
            if args[:2] == ['docker', 'run']:
                assert '--read-only' in args
                assert args[args.index('--network') + 1] == 'none'
                assert args[args.index('--ipc') + 1] == 'private'
                assert args[-2:] == ['sha256:new', '-']
                assert kwargs['input'] == IMAGE_PROBE
                return json.dumps({'source': 'bad' if case == 'wrong_embedded' else 'new-source',
                                   'patches': 'patches', 'installed': True})
            if args[:2] == ['docker', 'exec']:
                assert args[2:5] == ['-i', 'patrolbot-web-bridge', 'bash']
                assert kwargs['timeout'] == 20
                assert kwargs['input'] == (Path(__file__).resolve().parents[1] / 'rollout_preflight.py').read_text()
                return json.dumps({'safe': True, 'base_samples': 25, 'odom_samples': 60})
            if args[:2] == ['docker', 'inspect']:
                assert args[2:] == ['patrolbot-bringup', 'patrolbot-bridge', 'patrolbot-navigation']
                c = {'Image': 'sha256:old', 'RestartCount': 0,
                     'State': {'Running': True, 'OOMKilled': False,
                               'Health': {'Status': 'healthy'}}}
                if phase[0] == 'after':
                    c['Image'] = 'sha256:new'
                if case == 'mixed_containers':
                    c['Image'] = 'sha256:other'
                if case == 'unhealthy':
                    c['State']['Health']['Status'] = 'unhealthy'
                if case == 'oom':
                    c['State']['OOMKilled'] = True
                if case == 'restarting':
                    c['RestartCount'] = 1
                if case == 'no_health':
                    del c['State']['Health']
                return json.dumps([c, c, c])
            if 'config' in args:
                image = 'wrong' if case == 'wrong_compose_image' else 'new'
                return json.dumps({'services': {s: {'image': image} for s in manifest['services']}})
            if 'up' in args:
                assert kwargs['timeout'] is None
                assert args[-7:] == ['up', '-d', '--no-build', '--no-deps', 'bringup', 'bridge', 'navigation']
                return ''
            raise AssertionError(args)

        adapter = DockerRollout('/synthetic/root', copy.deepcopy(manifest), run=run,
                                clock=clock, sleep=lambda _: None)
        verified = adapter.verify_candidate(manifest)
        if case in ('success', 'tag_drift', 'wrong_compose_image'):
            assert verified is True
            phase[0] = 'after'
            failed = False
            try:
                adapter.restart('new')
            except RuntimeError:
                failed = True
            assert failed == (case != 'success')
            if case == 'success':
                assert adapter.postflight('new') is True
                assert adapter.preflight(Path(__file__).resolve().parents[1] / 'rollout_preflight.py')['safe'] is True
        else:
            assert verified is False
            try:
                adapter.restart('new')
                raise AssertionError('restart allowed after failed verification')
            except RuntimeError:
                pass
        mutations = [args for args, _ in calls if 'up' in args]
        assert len(mutations) == (1 if case == 'success' else 0)
        print('PASS', case)


if __name__ == '__main__':
    test_adapters()
