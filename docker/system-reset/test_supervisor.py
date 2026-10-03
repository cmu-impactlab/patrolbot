import tempfile
import time
import unittest
from pathlib import Path

from supervisor import Store, Supervisor


SAFE = {'safe': True, 'odom_epoch': 'old-origin'}
RECOVERING = {'safe': True, 'odom_epoch': 'new-origin'}


class FakeAdapter:
    def __init__(self, captures):
        self.captures = list(captures)
        self.calls = []

    def capability(self):
        return True

    def capture(self, post=False):
        self.calls.append(('capture', post))
        value = self.captures.pop(0)
        return value

    def restart_sbc(self):
        self.calls.append(('restart_sbc',))

    def restart_pi(self):
        self.calls.append(('restart_pi',))

    def wait_healthy(self):
        self.calls.append(('wait_healthy',))


class SupervisorTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.store = Store(Path(self.tmp.name) / 'state.sqlite3')

    def tearDown(self):
        self.tmp.cleanup()

    def wait_state(self, rid):
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            row = self.store.get(rid)
            if row and row[1] in ('failed', 'succeeded', 'uncertain'):
                return row[1]
            time.sleep(0.01)
        self.fail('request did not reach a terminal state')

    def test_preflight_failure_never_restarts_and_allows_later_retry(self):
        adapter = FakeAdapter([{'safe': False}, SAFE])
        supervisor = Supervisor(adapter, self.store)
        rid = '123e4567-e89b-42d3-a456-426614174000'
        self.assertIn(supervisor.submit(rid)['state'], ('accepted', 'running'))
        self.assertEqual(self.wait_state(rid), 'failed')
        self.assertFalse(any(call[0].startswith('restart') for call in adapter.calls))
        next_id = '123e4567-e89b-42d3-a456-426614174001'
        self.assertIn(supervisor.submit(next_id)['state'], ('accepted', 'running'))
        self.assertEqual(self.wait_state(next_id), 'failed')

    def test_success_requires_post_reset_changed_origin_and_recovery_capture(self):
        adapter = FakeAdapter([SAFE, SAFE, RECOVERING])
        supervisor = Supervisor(adapter, self.store)
        rid = '123e4567-e89b-42d3-a456-426614174002'
        self.assertIn(supervisor.submit(rid)['state'], ('accepted', 'running'))
        self.assertEqual(self.wait_state(rid), 'succeeded')
        self.assertEqual([c[0] for c in adapter.calls],
                         ['capture', 'capture', 'restart_sbc', 'restart_pi',
                          'wait_healthy', 'capture'])
        self.assertIn('fresh operator pose seed', self.store.get(rid)[2])

    def test_ambiguous_outcome_blocks_new_request_ids(self):
        adapter = FakeAdapter([SAFE, SAFE])
        adapter.restart_sbc = lambda: (_ for _ in ()).throw(RuntimeError('lost connection'))
        supervisor = Supervisor(adapter, self.store)
        rid = '123e4567-e89b-42d3-a456-426614174003'
        self.assertIn(supervisor.submit(rid)['state'], ('accepted', 'running'))
        self.assertEqual(self.wait_state(rid), 'uncertain')
        rejected = supervisor.submit('123e4567-e89b-42d3-a456-426614174004')
        self.assertEqual(rejected['state'], 'uncertain')


if __name__ == '__main__':
    unittest.main()


def test_interrupted_supervisor_requires_reconciliation(tmp_path):
    store = Store(tmp_path / 'state.sqlite3')
    rid = '123e4567-e89b-42d3-a456-426614174005'
    store.write(rid, 'running', 'restarting SBC')
    supervisor = Supervisor(FakeAdapter([]), store)
    assert store.get(rid)[1] == 'uncertain'
    assert supervisor.submit('123e4567-e89b-42d3-a456-426614174006')['state'] == 'uncertain'


def test_duplicate_finished_request_does_not_repeat_reset(tmp_path):
    store = Store(tmp_path / 'state.sqlite3')
    rid = '123e4567-e89b-42d3-a456-426614174007'
    store.write(rid, 'succeeded', 'complete')
    adapter = FakeAdapter([])
    supervisor = Supervisor(adapter, store)
    assert supervisor.submit(rid)['state'] == 'succeeded'
    assert adapter.calls == []


def test_pi_reset_restarts_fixed_containers_without_image_or_config_changes():
    from supervisor import ProductionAdapter
    adapter=ProductionAdapter();calls=[]
    adapter.run=lambda argv,**kw:calls.append(argv)
    adapter.restart_pi()
    assert calls==[
        ['/usr/bin/docker','restart','--time','30','patrolbot-bringup','patrolbot-bridge','patrolbot-navigation'],
        ['/usr/bin/docker','restart','--time','30','patrolbot-web-bridge']]


def test_capability_uses_restricted_config_and_legacy_systemd_output(monkeypatch):
    from supervisor import ProductionAdapter, SSH_CONFIG
    monkeypatch.setattr(Path, 'exists', lambda self: True)
    adapter=ProductionAdapter();calls=[]
    def run(argv,**kw):
        calls.append(argv)
        return 'LoadState=loaded\n' if argv[0]=='/usr/bin/ssh' else ''
    adapter.run=run
    assert adapter.capability()
    assert calls[0]==['/usr/bin/ssh','-F',str(SSH_CONFIG),'patrolbot-reset-sbc','status']


def test_sbc_wrapper_rejects_arbitrary_command(tmp_path):
    import os,subprocess
    marker=tmp_path/'must-not-exist'
    result=subprocess.run(['sh',str(Path(__file__).with_name('sbc-command.sh'))],
        env={**os.environ,'SSH_ORIGINAL_COMMAND':f'restart; touch {marker}'},capture_output=True)
    assert result.returncode==64 and not marker.exists()
