"""Portable checks of the real ROS fixtures' shared lifetime owner."""

from concurrent.futures import ThreadPoolExecutor
from collections import deque
from pathlib import Path
import signal
import subprocess
import sys
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
                       'ros2_ws/src/patrolbot_bridge/test'))
from native_fixture import NativeFixture, fixture_log_path


def test_native_child_logs_use_evidence_directory(monkeypatch, tmp_path):
    evidence = tmp_path / 'evidence'
    fallback = tmp_path / 'pytest'
    monkeypatch.setenv('ROS_LOG_DIR', str(evidence))
    assert fixture_log_path('collision_monitor.log', fallback) == (
        evidence / 'collision_monitor.log')
    assert evidence.is_dir()
    monkeypatch.delenv('ROS_LOG_DIR')
    assert fixture_log_path('collision_monitor.log', fallback) == (
        fallback / 'collision_monitor.log')


class Executor:
    def __init__(self, events, layout='jazzy'):
        self.events = events
        self._executor = ThreadPoolExecutor(max_workers=1)
        if layout == 'legacy':
            self._tasks = []
        else:
            self._pending_tasks = {}
            self._ready_tasks = deque()
        self._futures = []
        self.woken = threading.Event()
        self.spinning = threading.Event()

    def add_node(self, node):
        self.events.append('add')

    def remove_node(self, node):
        assert not any(thread.is_alive() for thread in self._executor._threads)
        self.events.append('remove')

    def spin_once(self, timeout_sec):
        self.spinning.set()
        self.woken.wait(timeout_sec)

    def wake(self):
        self.events.append('wake')
        self.woken.set()

    def shutdown(self, timeout_sec):
        assert not any(thread.is_alive() for thread in self._executor._threads)
        self.events.append('shutdown')
        return True


def resources(events):
    ros = SimpleNamespace(init=lambda: events.append('init'),
                          try_shutdown=lambda: events.append('context'))
    node = SimpleNamespace(destroy_node=lambda: events.append('destroy'))
    return ros, node


@pytest.mark.parametrize('layout', ['legacy', 'jazzy'])
@pytest.mark.parametrize('failure', ['construction', 'readiness', 'test', None])
def test_setup_and_test_failures_always_clean_up(failure, layout):
    events = []
    ros, node = resources(events)
    executor = Executor(events, layout)
    original_hook = threading.excepthook
    primary = RuntimeError(failure)
    try:
        with NativeFixture(ros) as lifetime:
            lifetime.cleanups.append(lambda: events.append('external'))
            lifetime.init()
            lifetime.node(node)
            if failure == 'construction':
                raise primary
            lifetime.start(executor)
            assert executor.spinning.wait(1)
            executor._executor.submit(lambda: events.append('callback')).result(timeout=1)
            if failure:
                raise primary
    except RuntimeError as error:
        assert error is primary
    else:
        assert failure is None
    assert events[-3:] == ['destroy', 'context', 'external']
    if failure != 'construction':
        assert events.index('callback') < events.index('shutdown') < events.index('destroy')
        assert not lifetime.spin_thread.is_alive()
    assert threading.excepthook is original_hook


def test_pending_worker_queue_is_cancelled_before_ros_handles_are_destroyed():
    events = []
    ros, node = resources(events)
    executor = Executor(events)
    entered = threading.Event()
    release = threading.Event()
    draining = threading.Event()
    shutdown = executor._executor.shutdown

    def drain(**kwargs):
        draining.set()
        return shutdown(**kwargs)

    executor._executor.shutdown = drain

    def callback():
        entered.set()
        assert release.wait(2)
        assert 'shutdown' not in events and 'destroy' not in events
        events.append('callback-finished')

    def release_callback():
        assert draining.wait(2)
        release.set()

    releaser = threading.Thread(target=release_callback)
    with NativeFixture(ros) as lifetime:
        lifetime.init()
        lifetime.node(node)
        lifetime.start(executor)
        executor._executor.submit(callback)
        assert entered.wait(1)
        pending = executor._executor.submit(lambda: events.append('queued'))
        releaser.start()
    releaser.join(timeout=2)
    assert not releaser.is_alive()
    assert pending.cancelled()
    assert 'queued' not in events
    assert events.index('callback-finished') < events.index('shutdown')


def test_background_exception_is_a_failure_and_other_cleanup_still_runs():
    events = []
    ros, node = resources(events)
    with pytest.raises(BaseExceptionGroup, match='native fixture cleanup') as caught:
        with NativeFixture(ros) as lifetime:
            lifetime.init()
            lifetime.node(node)
            def fail():
                raise ValueError('worker error')
            worker = threading.Thread(target=fail)
            worker.start()
            worker.join(timeout=1)
    assert any(str(error) == 'worker error' for error in caught.value.exceptions)
    assert events[-2:] == ['destroy', 'context']


def test_cleanup_error_preserves_primary_and_attempts_remaining_resources():
    events = []
    ros, node = resources(events)
    node.destroy_node = Mock(side_effect=ValueError('destroy failed'))
    primary = RuntimeError('readiness failed')
    with pytest.raises(RuntimeError) as caught:
        with NativeFixture(ros) as lifetime:
            lifetime.init()
            lifetime.node(node)
            lifetime.cleanups.append(lambda: events.append('external'))
            raise primary
    assert caught.value is primary
    assert any('destroy failed' in note for note in primary.__notes__)
    assert events[-2:] == ['context', 'external']


@pytest.mark.parametrize('stage', ['spin', 'pool', 'shutdown'])
def test_unfinished_executor_preserves_handles_and_cleans_external_resources(stage):
    events = []
    ros, node = resources(events)
    executor = Executor(events)
    with pytest.raises(BaseExceptionGroup):
        with NativeFixture(ros, timeout=0.1) as lifetime:
            lifetime.init()
            lifetime.node(node)
            lifetime.executor = executor
            lifetime.cleanups.append(lambda: events.append('external'))
            if stage == 'spin':
                lifetime.spin_thread = Mock(name='survivor')
                lifetime.spin_thread.is_alive.return_value = True
            elif stage == 'pool':
                executor._executor.shutdown = Mock(side_effect=ValueError('drain failed'))
            else:
                executor.shutdown = lambda **kwargs: False
    assert 'destroy' not in events and 'context' not in events
    assert events[-1] == 'external'


@pytest.mark.parametrize('storage', ['legacy', 'pending', 'ready', 'futures'])
@pytest.mark.parametrize('state', ['completed', 'cancelled', 'pending'])
def test_callback_exception_is_not_lost_after_spin_stops(storage, state):
    events = []
    ros, node = resources(events)
    executor = Executor(events, 'legacy' if storage == 'legacy' else 'jazzy')
    task = Mock()
    task.done.return_value = state != 'pending'
    task.cancelled.return_value = state == 'cancelled'
    task.exception.return_value = ValueError('callback error')
    if storage == 'legacy':
        executor._tasks = [(task, None, node)]
    elif storage == 'pending':
        executor._pending_tasks[task] = SimpleNamespace(source_node=node)
    elif storage == 'ready':
        executor._ready_tasks.append(task)
    else:
        executor._futures.append(task)

    def run():
        with NativeFixture(ros) as lifetime:
            lifetime.init()
            lifetime.node(node)
            lifetime.start(executor)
    if state == 'completed':
        with pytest.raises(BaseExceptionGroup) as caught:
            run()
        assert [str(error) for error in caught.value.exceptions] == ['callback error']
        task.exception.assert_called_once_with()
    else:
        run()
        task.exception.assert_not_called()
    assert events[-2:] == ['destroy', 'context']


def test_async_task_finishing_during_drain_retains_exception_and_new_tasks():
    events = []
    ros, node = resources(events)
    executor = Executor(events)
    # Like cached Jazzy, this executor has no legacy _tasks attribute.
    assert not hasattr(executor, '_tasks')
    task = Mock()
    task.done.return_value = False
    task.cancelled.return_value = False
    task.exception.return_value = ValueError('async action failed')
    executor._pending_tasks[task] = None
    executor._ready_tasks.append(task)
    executor._futures.append(task)
    shutdown = executor._executor.shutdown

    def drain(**kwargs):
        shutdown(**kwargs)
        task.done.return_value = True
        executor._pending_tasks.clear()
        executor._ready_tasks.clear()
        executor._futures.clear()
        completed = Mock()
        completed.done.return_value = True
        completed.cancelled.return_value = False
        completed.exception.return_value = ValueError('done callback failed')
        executor._pending_tasks[completed] = None

    executor._executor.shutdown = drain
    with pytest.raises(BaseExceptionGroup) as caught:
        with NativeFixture(ros) as lifetime:
            lifetime.init()
            lifetime.node(node)
            lifetime.start(executor)
    assert [str(error) for error in caught.value.exceptions] == [
        'async action failed', 'done callback failed']
    task.exception.assert_called_once_with()
    assert events.index('shutdown') < events.index('destroy') < events.index('context')


@pytest.mark.parametrize('missing', ['_pending_tasks', '_ready_tasks', '_futures', '_executor'])
def test_unsupported_private_api_fails_clearly(missing):
    events = []
    ros, node = resources(events)
    executor = Executor(events)
    pool = executor._executor
    delattr(executor, missing)
    try:
        with pytest.raises(BaseExceptionGroup) as caught:
            with NativeFixture(ros) as lifetime:
                lifetime.init()
                lifetime.node(node)
                lifetime.start(executor)
                lifetime.cleanups.append(lambda: events.append('external'))
        assert any('unsupported test executor' in str(error)
                   for error in caught.value.exceptions)
        assert events[-1] == 'external'
        if missing == '_executor':
            assert 'destroy' not in events and 'context' not in events
    finally:
        pool.shutdown(wait=True, cancel_futures=True)


def test_callback_timeout_retains_handles_and_reports_surviving_drain():
    events = []
    ros, node = resources(events)
    executor = Executor(events)
    entered = threading.Event()
    release = threading.Event()
    before = set(threading.enumerate())
    def callback():
        entered.set()
        assert release.wait(2)
    try:
        with pytest.raises(BaseExceptionGroup) as caught:
            with NativeFixture(ros, timeout=0.02) as lifetime:
                lifetime.init()
                lifetime.node(node)
                lifetime.start(executor)
                executor._executor.submit(callback)
                assert entered.wait(1)
                lifetime.cleanups.append(lambda: events.append('external'))
        assert 'fixture-worker-drain survived' in str(caught.value.exceptions[0])
        assert 'destroy' not in events and 'context' not in events
        assert events[-1] == 'external'
    finally:
        release.set()
        for thread in set(threading.enumerate()) - before:
            thread.join(timeout=2)
            assert not thread.is_alive()


@pytest.mark.parametrize('survives', [False, True])
def test_bridge_workers_quiesce_before_node_destruction(survives):
    events = []
    ros, node = resources(events)
    executor = Executor(events)
    node._stop_event = threading.Event()
    node.running = True
    node._manual_undock = SimpleNamespace(
        destroy=lambda: events.append('manual-stop'), thread=None)
    if survives:
        worker = Mock(name='survivor')
        worker.is_alive.return_value = True
    else:
        def run():
            assert node._stop_event.wait(1)
            assert not node.running and 'destroy' not in events
            assert 'shutdown' in events
            events.append('worker-stopped')
        worker = threading.Thread(target=run)
        worker.start()
    node._worker_threads = [worker]

    def run_fixture():
        with NativeFixture(ros, timeout=0.1) as lifetime:
            lifetime.init()
            lifetime.node(node)
            lifetime.start(executor)
            lifetime.cleanups.append(lambda: events.append('external'))
    if survives:
        with pytest.raises(BaseExceptionGroup):
            run_fixture()
        assert 'destroy' not in events and 'context' not in events
    else:
        run_fixture()
        assert not worker.is_alive()
        assert events.index('worker-stopped') < events.index('destroy')
    assert events[-1] == 'external'


def test_executor_registration_failure_still_drains_and_destroys_nodes():
    events = []
    ros, node = resources(events)
    executor = Executor(events)
    executor.add_node = Mock(side_effect=RuntimeError('registration failed'))
    with pytest.raises(RuntimeError, match='registration failed'):
        with NativeFixture(ros) as lifetime:
            lifetime.init()
            lifetime.node(node)
            lifetime.start(executor)
    assert events[-3:] == ['remove', 'destroy', 'context']


def test_spin_start_failure_still_cleans_registered_resources(monkeypatch):
    events = []
    ros, node = resources(events)
    executor = Executor(events)
    start = threading.Thread.start
    def fail_spin(thread):
        if thread.name == 'fixture-spin':
            raise RuntimeError('thread start failed')
        start(thread)
    monkeypatch.setattr(threading.Thread, 'start', fail_spin)
    with pytest.raises(RuntimeError, match='thread start failed'):
        with NativeFixture(ros) as lifetime:
            lifetime.init()
            lifetime.node(node)
            lifetime.start(executor)
    assert events[-3:] == ['remove', 'destroy', 'context']


@pytest.mark.parametrize('code', [0, -signal.SIGTERM, 1, -signal.SIGKILL])
def test_child_exit_codes_are_explicit(code):
    process = Mock(returncode=code)
    process.poll.return_value = None
    process.wait.return_value = code
    lifetime = NativeFixture(Mock())
    def run():
        with lifetime:
            lifetime.process(process)
    if code in (0, -signal.SIGTERM):
        run()
    else:
        with pytest.raises(BaseExceptionGroup):
            run()
    process.terminate.assert_called_once()
    process.wait.assert_called_once_with(timeout=5.0)


def test_forced_kill_reaps_child_but_fails_cleanup():
    process = Mock()
    process.poll.return_value = None
    process.wait.side_effect = [subprocess.TimeoutExpired('monitor', 5), -signal.SIGKILL]
    with pytest.raises(BaseExceptionGroup) as caught:
        with NativeFixture(Mock()) as lifetime:
            lifetime.process(process)
    process.kill.assert_called_once()
    assert process.wait.call_count == 2
    assert 'forced kill' in str(caught.value.exceptions[0])


def test_early_child_exit_is_a_failure():
    process = Mock(returncode=0)
    process.poll.return_value = 0
    with pytest.raises(BaseExceptionGroup):
        with NativeFixture(Mock()) as lifetime:
            lifetime.process(process)
    process.terminate.assert_not_called()
