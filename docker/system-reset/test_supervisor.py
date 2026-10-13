import tempfile
import threading
import time
import unittest
from pathlib import Path

from supervisor import ProductionAdapter, ResetError, Store, Supervisor, SSH_CONFIG

RID = '123e4567-e89b-42d3-a456-426614174000'
NEXT_ID = '123e4567-e89b-42d3-a456-426614174001'


class FakeAdapter:
    def __init__(self):
        self.calls = []

    def capability(self):
        return True

    def capture(self, post=False):
        raise AssertionError('Emergency reset must not read robot telemetry')

    def wait_healthy(self):
        raise AssertionError('Emergency reset must not wait for robot readiness')

    def stop_pi(self):
        self.calls.append('stop_pi')

    def restart_sbc(self):
        self.calls.append('restart_sbc')

    def restart_pi(self):
        self.calls.append('restart_pi')

    def restart_web_bridge(self):
        self.calls.append('restart_web_bridge')


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
            time.sleep(.01)
        self.fail('request did not reach a terminal state')

    def test_reset_has_no_robot_preconditions_or_postconditions(self):
        adapter = FakeAdapter()
        supervisor = Supervisor(adapter, self.store)
        self.assertEqual(supervisor.submit(RID)['request_id'], RID)
        self.assertEqual(self.wait_state(RID), 'succeeded')
        self.assertEqual(adapter.calls, ['stop_pi', 'restart_sbc', 'restart_pi', 'restart_web_bridge'])

    def test_partial_failure_still_attempts_both_computers_and_allows_explicit_new_reset(self):
        adapter = FakeAdapter()
        def fail():
            adapter.calls.append('restart_sbc')
            raise ResetError('connection lost')
        adapter.restart_sbc = fail
        supervisor = Supervisor(adapter, self.store)
        supervisor.submit(RID)
        self.assertEqual(self.wait_state(RID), 'uncertain')
        self.assertEqual(adapter.calls, ['stop_pi', 'restart_sbc', 'restart_pi', 'restart_web_bridge'])
        self.assertEqual(supervisor.submit(RID)['state'], 'uncertain')
        self.assertEqual(len(adapter.calls), 4)
        supervisor.submit(NEXT_ID)
        self.assertEqual(self.wait_state(NEXT_ID), 'uncertain')
        self.assertEqual(len(adapter.calls), 8)
        self.assertEqual(self.store.get(RID)[1], 'uncertain')

    def test_overlapping_resets_are_rejected(self):
        adapter = FakeAdapter()
        entered, release = threading.Event(), threading.Event()
        def stop():
            adapter.calls.append('stop_pi')
            entered.set()
            release.wait(2)
        adapter.stop_pi = stop
        supervisor = Supervisor(adapter, self.store)
        supervisor.submit(RID)
        self.assertTrue(entered.wait(1))
        try:
            self.assertEqual(supervisor.submit(NEXT_ID)['state'], 'busy')
            self.assertIsNone(self.store.get(NEXT_ID))
        finally:
            release.set()
        self.assertEqual(self.wait_state(RID), 'succeeded')


def test_interrupted_supervisor_records_unknown_without_automatically_retrying(tmp_path):
    store = Store(tmp_path / 'state.sqlite3')
    store.write(RID, 'running', 'restarting SBC')
    adapter = FakeAdapter()
    Supervisor(adapter, store)
    assert store.get(RID)[1] == 'uncertain'
    assert adapter.calls == []


def test_duplicate_finished_request_does_not_repeat_reset(tmp_path):
    store = Store(tmp_path / 'state.sqlite3')
    store.write(RID, 'succeeded', 'complete')
    adapter = FakeAdapter()
    supervisor = Supervisor(adapter, store)
    assert supervisor.submit(RID)['state'] == 'succeeded'
    assert adapter.calls == []


def test_capability_recovers_after_local_management_configuration_becomes_available(tmp_path):
    ready = threading.Event()
    adapter = FakeAdapter()
    adapter.capability = ready.is_set
    supervisor = Supervisor(adapter, Store(tmp_path / 'state.sqlite3'))
    assert supervisor.available is False
    ready.set()
    deadline = time.monotonic() + 6
    while not supervisor.available and time.monotonic() < deadline:
        time.sleep(.01)
    assert supervisor.available is True
    assert adapter.calls == [] and supervisor.store.latest() is None


def test_pi_reset_stops_old_command_publishers_before_starting_existing_containers():
    adapter = ProductionAdapter(); calls = []
    adapter.run = lambda argv, **kw: calls.append(argv)
    adapter.stop_pi(); adapter.restart_pi(); adapter.restart_web_bridge()
    assert calls == [
        ['/usr/bin/docker', 'stop', '--time', '10', 'patrolbot-bridge', 'patrolbot-navigation', 'patrolbot-bringup'],
        ['/usr/bin/docker', 'restart', '--time', '10', 'patrolbot-bringup', 'patrolbot-bridge', 'patrolbot-navigation'],
        ['/usr/bin/docker', 'restart', '--time', '10', 'patrolbot-web-bridge']]


def test_capability_does_not_require_running_ros_or_reachable_sbc(monkeypatch):
    monkeypatch.setattr(Path, 'is_file', lambda self: True)
    adapter = ProductionAdapter()
    adapter.run = lambda *a, **k: (_ for _ in ()).throw(AssertionError('must not probe robot'))
    assert adapter.capability()
    monkeypatch.setattr(Path, 'is_file', lambda self: self != SSH_CONFIG)
    assert not adapter.capability()


def test_sbc_wrapper_rejects_arbitrary_command(tmp_path):
    import os, subprocess
    marker = tmp_path / 'must-not-exist'
    result = subprocess.run(['sh', str(Path(__file__).with_name('sbc-command.sh'))],
        env={**os.environ, 'SSH_ORIGINAL_COMMAND': f'restart; touch {marker}'}, capture_output=True)
    assert result.returncode == 64 and not marker.exists()
