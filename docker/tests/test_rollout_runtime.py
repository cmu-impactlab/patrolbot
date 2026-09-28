"""Exercise real file transactions with simulated partial container failures."""
from pathlib import Path
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rollout_files import digest
from rollout_runtime import RollbackFailed, rollout


def test_runtime_transaction():
    cases = ('success', 'candidate_refused', 'preflight_refused',
             'partial_restart', 'postflight_false', 'postflight_exception',
             'rollback_restart_failure', 'rollback_postflight_failure',
             'corrupt_backup')
    for case in cases:
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            root, payload, backup = (work / name for name in ('root', 'payload', 'backup'))
            (root / 'docker').mkdir(parents=True)
            (payload / 'src').mkdir(parents=True)
            (root / 'src').mkdir()
            original_env = 'PATROLBOT_IMAGE=old\nPRIVATE_VALUE=do-not-print\n'
            (root / 'docker/.env').write_text(original_env)
            changes = []
            for name in ('manager.py', 'joy.py', 'launch.py', 'fixture.py'):
                relative = 'src/' + name
                (root / relative).write_text('old:' + name)
                (payload / relative).write_text('new:' + name)
                changes.append({'path': relative, 'before': digest(root / relative),
                                'after': digest(payload / relative)})
            manifest = {'expected_previous_image': 'old', 'candidate_image': 'new',
                        'source_changes': changes}
            events = []
            running = ['old']

            def candidate(_):
                events.append(('candidate',))
                return case != 'candidate_refused'

            def preflight():
                events.append(('preflight',))
                return {'safe': case != 'preflight_refused'}

            def restart(image):
                # Check the actual files seen by the simulated Compose call.
                assert 'PATROLBOT_IMAGE=' + image in (root / 'docker/.env').read_text()
                assert all((root / c['path']).read_text().startswith(image + ':')
                           for c in changes)
                events.append(('restart', image))
                running[0] = image
                if case == 'partial_restart' and image == 'new':
                    raise RuntimeError('one service recreated before failure')
                if case == 'rollback_restart_failure' and image == 'old':
                    raise RuntimeError('old container restart failed')

            def postflight(image):
                events.append(('postflight', image))
                if image == 'old':
                    return case != 'rollback_postflight_failure'
                if case == 'corrupt_backup':
                    (backup / changes[0]['path']).write_text('corrupted')
                if case == 'postflight_exception':
                    raise RuntimeError('health command failed')
                return case == 'success'

            error = None
            try:
                result = rollout(root, payload, backup, manifest,
                                 verify_candidate=candidate, preflight=preflight,
                                 restart=restart, postflight=postflight)
                assert result['image'] == 'new'
            except (RuntimeError, AssertionError) as exc:
                error = exc

            if case == 'success':
                assert error is None and running == ['new']
                assert events == [('candidate',), ('preflight',),
                                  ('restart', 'new'), ('postflight', 'new')]
            elif case in ('candidate_refused', 'preflight_refused'):
                assert error is not None and not backup.exists()
                assert running == ['old']
                assert not any(e[0] == 'restart' for e in events)
                if case == 'candidate_refused':
                    assert events == [('candidate',)]
            elif case in ('rollback_restart_failure', 'rollback_postflight_failure',
                          'corrupt_backup'):
                assert isinstance(error, RollbackFailed)
                assert error.deployment_error is not None
                assert error.rollback_error is not None
                if case == 'corrupt_backup':
                    assert ('restart', 'old') not in events
                    assert running == ['new']
            else:
                assert error is not None and not isinstance(error, RollbackFailed)
                assert running == ['old']
                assert events[-2:] == [('restart', 'old'), ('postflight', 'old')]
            if case not in ('success', 'corrupt_backup'):
                assert (root / 'docker/.env').read_text() == original_env
                assert all(digest(root / c['path']) == c['before'] for c in changes)
            print('PASS', case)


if __name__ == '__main__':
    test_runtime_transaction()
