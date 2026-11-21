"""Portable checks of the real ROS fixtures' shared lifetime owner."""

from concurrent.futures import ThreadPoolExecutor
from collections import deque
import ast
import inspect
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
                       'ros2_ws/src/patrolbot_bridge/test'))
from native_fixture import NativeFixture, dispose_task, fixture_log_path


def inactive_task():
    task = Mock()
    task.done.return_value = False
    task.cancelled.return_value = False
    task._handler = None
    task._executing = False
    task._task_lock = threading.Lock()
    return task


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
        task = inactive_task()
        async def handler():
            events.append('queued')
        coroutine = task._handler = handler()
        task.side_effect = lambda: coroutine.send(None)
        executor._pending_tasks[task] = None
        pending = executor._executor.submit(task)
        releaser.start()
    releaser.join(timeout=2)
    assert not releaser.is_alive()
    assert pending.cancelled()
    assert inspect.getcoroutinestate(coroutine) == inspect.CORO_CLOSED
    task.cancel.assert_called_once_with()
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
    task = inactive_task()
    task.done.return_value = state == 'completed'
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
    task = inactive_task()
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
        completed = inactive_task()
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
        assert 'destroy' not in events and 'context' not in events
    finally:
        pool.shutdown(wait=True, cancel_futures=True)


@pytest.mark.parametrize('state', ['created', 'suspended', 'cancelled'])
@pytest.mark.parametrize('cancel_closes', [False, True])
def test_inactive_coroutines_are_finalized_before_guard_destruction(state, cancel_closes):
    events = []
    ros, node = resources(events)
    executor = Executor(events)
    task = inactive_task()

    async def handler():
        try:
            await pause()
        finally:
            assert 'shutdown' not in events and 'destroy' not in events
            events.append('finalized')

    class pause:
        def __await__(self):
            yield

    coroutine = task._handler = handler()
    if state == 'suspended':
        coroutine.send(None)
    task.cancelled.return_value = state == 'cancelled'
    if cancel_closes:
        task.cancel.side_effect = lambda: (
            None if task.cancelled() else coroutine.close())
    executor._pending_tasks[task] = None
    executor._ready_tasks.append(task)
    executor._futures.append(task)
    try:
        with NativeFixture(ros) as lifetime:
            lifetime.init()
            lifetime.node(node)
            lifetime.start(executor)
        assert inspect.getcoroutinestate(coroutine) == inspect.CORO_CLOSED
        task.cancel.assert_called_once_with()
        task.assert_not_called()
        if state == 'suspended':
            assert events.index('finalized') < events.index('shutdown')
        assert events[-2:] == ['destroy', 'context']
    finally:
        coroutine.close()


def test_disposal_also_cancels_tasks_scheduled_by_cancellation():
    executor = Executor([])
    first, followup = inactive_task(), inactive_task()
    first.cancel.side_effect = lambda: executor._ready_tasks.append(followup)
    executor._pending_tasks[first] = None
    with NativeFixture(Mock()) as lifetime:
        lifetime.start(executor)
    first.cancel.assert_called_once_with()
    followup.cancel.assert_called_once_with()


@pytest.mark.parametrize('active', ['lock', 'executing'])
def test_active_task_is_never_cancelled_or_closed(active):
    task = inactive_task()
    async def handler():
        pass
    coroutine = task._handler = handler()
    if active == 'lock':
        task._task_lock.acquire()
    else:
        task._executing = True
    try:
        with pytest.raises(RuntimeError, match='still active'):
            dispose_task(task)
        task.cancel.assert_not_called()
        assert inspect.getcoroutinestate(coroutine) == inspect.CORO_CREATED
    finally:
        if active == 'lock':
            task._task_lock.release()
        coroutine.close()


@pytest.mark.parametrize('failure', ['unsupported', 'cancel', 'close'])
def test_disposal_failure_keeps_guards_and_preserves_other_task_errors(failure):
    events = []
    ros, node = resources(events)
    executor = Executor(events)
    task = inactive_task()
    async def handler():
        try:
            await pause()
        finally:
            raise ValueError('close failed')
    class pause:
        def __await__(self):
            yield
    coroutine = task._handler = handler()
    if failure == 'unsupported':
        del task._task_lock
    elif failure == 'cancel':
        task.cancel.side_effect = ValueError('cancel failed')
    else:
        coroutine.send(None)
    completed = inactive_task()
    completed.done.return_value = True
    completed.exception.return_value = ValueError('callback failed')
    executor._ready_tasks.extend([task, completed])
    try:
        with pytest.raises(BaseExceptionGroup) as caught:
            with NativeFixture(ros) as lifetime:
                lifetime.init()
                lifetime.node(node)
                lifetime.start(executor)
                lifetime.cleanups.append(lambda: events.append('external'))
        assert len(caught.value.exceptions) == 2
        assert str(caught.value.exceptions[-1]) == 'callback failed'
        assert 'shutdown' not in events and 'destroy' not in events and 'context' not in events
        assert events[-1] == 'external'
    finally:
        coroutine.close()


@pytest.mark.parametrize('stage', ['callback', 'finalizer'])
def test_callback_timeout_retains_handles_and_reports_surviving_drain(stage):
    events = []
    ros, node = resources(events)
    executor = Executor(events)
    entered = threading.Event()
    release = threading.Event()
    before = set(threading.enumerate())
    def callback():
        entered.set()
        assert release.wait(2)
    class pause:
        def __await__(self):
            yield
    async def handler():
        try:
            await pause()
        finally:
            callback()
    if stage == 'finalizer':
        task = inactive_task()
        task._handler = handler()
        task._handler.send(None)
        executor._ready_tasks.append(task)
    try:
        with pytest.raises(BaseExceptionGroup) as caught:
            with NativeFixture(ros, timeout=0.02) as lifetime:
                lifetime.init()
                lifetime.node(node)
                lifetime.start(executor)
                if stage == 'callback':
                    executor._executor.submit(callback)
                    assert entered.wait(1)
                lifetime.cleanups.append(lambda: events.append('external'))
        assert entered.is_set()
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


@pytest.fixture
def collision_simulation():
    """Run the real harness methods with a clock-driven ROS/DDS stand-in."""
    clock = SimpleNamespace(now=0.0, pending=[], published=[], source=0,
                            graph_at=0.2, transport_at=0.35, reply=True,
                            stopped=False, stop_stamp=0.0, last_sonar=0.0)

    class Twist:
        def __init__(self):
            self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

    def deliver(linear, angular, source=None):
        clock.source += 1
        message = Twist()
        message.linear.x, message.angular.z = linear, angular
        node.receive(message, SimpleNamespace(
            source_timestamp=clock.source if source is None else source))

    def publish(message):
        linear, angular = message.linear.x, message.angular.z
        clock.published.append((clock.now, linear, angular, node.world))
        scan, sonar, healthy = node.world
        stop = ((scan and linear > 0) or (sonar and linear < 0) or
                clock.now - clock.last_sonar > 0.5)
        if stop:
            if not clock.stopped:
                clock.stop_stamp = clock.now
            suppressed = clock.stopped and clock.now - clock.stop_stamp > 1.0
            clock.stopped = True
            linear = angular = 0.0
        else:
            clock.stopped = False
            suppressed = False
        if clock.reply and not suppressed and clock.now >= clock.transport_at:
            clock.pending.append((clock.now + 0.01, linear, angular))

    class Node:
        def __init__(self, name):
            self.default_callback_group = threading.Lock()

        def create_publisher(self, kind, topic, qos):
            return SimpleNamespace(get_subscription_count=lambda: 1, publish=publish)

        def create_subscription(self, *args):
            self.output_group = self.default_callback_group
            return SimpleNamespace(get_publisher_count=lambda: int(
                clock.now >= clock.graph_at))

        def create_timer(self, period, callback, *, callback_group):
            self.sensor_group = callback_group
            return callback

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(
                nanoseconds=int(clock.now * 1e9)))

    def sleep(seconds):
        clock.now += seconds
        node.sensors()
        for item in list(clock.pending):
            if item[0] <= clock.now:
                clock.pending.remove(item)
                deliver(*item[1:])

    source = Path(__file__).resolve().parents[2] / (
        'ros2_ws/src/patrolbot_navigation/test/test_collision_monitor_integration.py')
    tree = ast.parse(source.read_text())
    # Avoid importing ROS; execute the actual class and wait function unchanged.
    tree.body = [item for item in tree.body if getattr(item, 'name', '') in
                 ('CollisionHarness', 'wait_until')]
    namespace = dict(Node=Node, Twist=Twist, math=math, threading=threading,
                     MutuallyExclusiveCallbackGroup=threading.Lock,
                     time=SimpleNamespace(monotonic=lambda: clock.now, sleep=sleep),
                     LaserScan=object, PointCloud2=object, qos_profile_sensor_data=None)
    exec(compile(tree, str(source), 'exec'), namespace)
    node = namespace['CollisionHarness']()
    node.scan = lambda obstacle=False: None
    node.sonar = lambda obstacle=False: setattr(clock, 'last_sonar', clock.now)
    return node, clock, deliver, sleep


@pytest.mark.parametrize('blocked', ['scan', 'sonar', 'command', 'graph'])
def test_collision_receives_while_producer_is_blocked(collision_simulation, blocked):
    node, _, deliver, _ = collision_simulation
    entered, release = threading.Event(), threading.Event()

    def block(*_):
        entered.set()
        assert release.wait(2)
        return 1

    if blocked in ('scan', 'sonar'):
        publisher = getattr(node, blocked + '_pub')
        publisher.publish = block
        setattr(node, blocked, lambda *_: publisher.publish(None))
    elif blocked == 'command':
        node.command_pub.publish = block
    else:
        node.command_pub.get_subscription_count = block

    def produce():
        if blocked in ('scan', 'sonar'):
            # Model rclpy's mutually exclusive dispatch, including group ownership.
            with node.sensor_group:
                node.sensors()
        else:
            node.publish_command(0.05, 0.0, 'scenario')

    def receive():
        with node.output_group:
            deliver(0.05, 0.0, source=42)

    with ThreadPoolExecutor(max_workers=2) as pool:
        producer = pool.submit(produce)
        try:
            assert entered.wait(1)
            assert node.producer_lock.locked()
            assert node.sensor_group is not node.output_group
            pool.submit(receive).result(timeout=1)
            assert not producer.done()
            with node.lock:
                assert len(node.outputs) == len(node.output_sources) == 1
                assert node.outputs[0].linear.x == 0.05
                assert node.output_sources == [42]
                assert node.evidence[-1]['source_ns'] == 42
        finally:
            release.set()
            producer.result(timeout=1)


@pytest.mark.parametrize('blocked', ['scan', 'sonar'])
def test_collision_stopping_sonar_drains_old_sensor_batch(collision_simulation, blocked):
    node, _, _, _ = collision_simulation
    entered, release, transition_waiting = (
        threading.Event(), threading.Event(), threading.Event())
    sonar_worlds = []
    producer_lock = node.producer_lock

    class TrackedLock:
        def __enter__(self):
            if producer_lock.locked():
                transition_waiting.set()
            producer_lock.acquire()

        def __exit__(self, *_):
            producer_lock.release()

    node.producer_lock = TrackedLock()

    def scan(*_):
        if blocked == 'scan':
            entered.set()
            assert release.wait(2)

    def sonar(*_):
        if blocked == 'sonar':
            entered.set()
            assert release.wait(2)
        sonar_worlds.append(node.world)

    node.scan, node.sonar = scan, sonar
    with ThreadPoolExecutor(max_workers=2) as pool:
        producer = pool.submit(node.sensors)
        transition = None
        try:
            assert entered.wait(1)
            transition = pool.submit(node.set_world, publish_sonar=False)
            assert transition_waiting.wait(1)
            assert not transition.done()
            assert node.world == (False, False, True)
        finally:
            release.set()
            producer.result(timeout=1)
            if transition is not None:
                assert transition.result(timeout=1) == 1
    assert sonar_worlds == [(False, False, True)]
    assert node.world == (False, False, False)
    assert [e['event'] for e in node.evidence] == ['sensors', 'world']
    node.sensors()
    assert sonar_worlds == [(False, False, True)]
    assert node.sensor_ticks == 2


def test_collision_callback_keeps_output_and_source_atomic(collision_simulation):
    node, _, deliver, _ = collision_simulation
    entered, release = threading.Event(), threading.Event()

    class PausedAppend(list):
        def append(self, value):
            super().append(value)
            entered.set()
            assert release.wait(2)

    node.outputs = PausedAppend()
    with ThreadPoolExecutor(max_workers=1) as pool:
        callback = pool.submit(deliver, 0.05, 0.0, 42)
        try:
            assert entered.wait(1)
            acquired = node.lock.acquire(blocking=False)
            if acquired:
                node.lock.release()
            assert not acquired, 'observers must not see the half-appended pair'
        finally:
            release.set()
            callback.result(timeout=1)
    with node.lock:
        assert len(node.outputs) == len(node.output_sources) == 1
        assert node.output_sources == [42]
        assert node.evidence[-1]['source_ns'] == 42


def test_collision_discovery_and_motion_precede_single_stop_command(collision_simulation):
    node, clock, deliver, sleep = collision_simulation
    for _ in range(2):
        output = node.command(0.05, scan_obstacle=True)
        assert output.linear.x == output.angular.z == 0.0
        events = node.evidence
        scenario = [e for e in events if e['event'] == 'command' and
                    e['phase'] == 'scenario'][-1]
        ready = [e for e in events if e['event'] == 'precondition-ready'][-1]
        assert ready['monotonic'] < scenario['monotonic']
        assert all(ready['graph'].values())
        assert scenario['world'] == (True, False, True)
        # Already-stopped Nav2 legitimately suppresses further zeros after 1s.
        sleep(1.1)
        count = len(node.outputs)
        stopped_command = type(output)()
        stopped_command.linear.x = 0.05
        node.command_pub.publish(stopped_command)
        sleep(0.02)
        assert len(node.outputs) == count
    assert len([e for e in node.evidence if e['event'] == 'command' and
                e['phase'] == 'scenario']) == 2
    assert any(t < clock.transport_at for t, *_ in clock.published)


def test_collision_stale_sonar_is_not_refreshed_after_fault(collision_simulation):
    node, clock, deliver, sleep = collision_simulation
    output = node.command(-0.05, publish_sonar=False)
    assert output.linear.x == output.angular.z == 0.0
    assert clock.now - clock.last_sonar >= 0.6
    assert node.world == (False, False, False)
    sonar_stamp = clock.last_sonar
    sleep(0.2)
    assert clock.last_sonar == sonar_stamp
    assert node.command(-0.05).linear.x == -0.05


def test_collision_stale_output_cannot_satisfy_scenario(collision_simulation):
    node, clock, deliver, sleep = collision_simulation
    publish = node.command_pub.publish

    def stale_only(message):
        if message.linear.x == 0.037:
            publish(message)
        else:
            # A queued old zero and a delayed clear probe are both insufficient.
            deliver(0.0, 0.0, source=1)
            deliver(*next(iter(node.probes)))
    node.command_pub.publish = stale_only
    with pytest.raises(AssertionError, match='scenario output timed out'):
        node.command(0.05, scan_obstacle=True)
    assert clock.now < 2.0


def test_collision_wrong_current_output_is_returned_without_retry(collision_simulation):
    node, clock, deliver, sleep = collision_simulation
    publish = node.command_pub.publish

    def wrong_velocity(message):
        if message.linear.x == 0.037:
            publish(message)
        else:
            deliver(0.02, 0.0)
    node.command_pub.publish = wrong_velocity
    # The caller's velocity assertion must see the wrong result immediately.
    assert node.command(0.05, scan_obstacle=True).linear.x == 0.02
    assert len([e for e in node.evidence if e['event'] == 'command' and
                e['phase'] == 'scenario']) == 1


@pytest.mark.parametrize('failure', ['command', 'scan', 'sonar', 'output', 'transport'])
def test_collision_readiness_failure_is_bounded_and_never_applies_fault(
        collision_simulation, failure):
    node, clock, deliver, sleep = collision_simulation
    if failure == 'transport':
        clock.reply = False
    elif failure == 'output':
        clock.graph_at = 100.0
    else:
        publisher = getattr(node, failure + '_pub')
        publisher.get_subscription_count = lambda: 0
    with pytest.raises(AssertionError, match='graph not ready|precondition timed out'):
        node.command(0.05, scan_obstacle=True)
    assert clock.now <= 4.02
    assert not any(e['event'] == 'command' and e['phase'] == 'scenario'
                   for e in node.evidence)
    assert node.world == (False, False, True)


@pytest.mark.parametrize('shape', ['dict', 'object'])
def test_collision_receive_accepts_native_metadata(collision_simulation, shape):
    node, _, _, _ = collision_simulation
    message = SimpleNamespace(linear=SimpleNamespace(x=1, y=0, z=0),
                              angular=SimpleNamespace(x=0, y=0, z=0))
    info = {'source_timestamp': 42}
    if shape == 'object':
        info = SimpleNamespace(**info)
    node.receive(message, info)
    assert node.outputs == [message]
    assert node.output_sources == [42]
    assert node.evidence[-1]['source_ns'] == 42


@pytest.mark.parametrize('shape', ['dict', 'object'])
@pytest.mark.parametrize('value', [None, True, False, 0, -1, 1.5, '42', [], {}])
def test_collision_invalid_timestamp_preserves_paired_arrays(
        collision_simulation, shape, value):
    node, _, deliver, _ = collision_simulation
    deliver(0.1, 0.0)
    outputs, sources = node.outputs[:], node.output_sources[:]
    info = {'source_timestamp': value}
    if shape == 'object':
        info = SimpleNamespace(**info)
    with pytest.raises(ValueError, match='positive integer'):
        node.receive(object(), info)
    assert node.outputs == outputs
    assert node.output_sources == sources
    assert len(node.outputs) == len(node.output_sources)
    assert node.evidence[-1]['event'] == 'output-metadata-invalid'
    deliver(0.2, 0.0)
    assert len(node.outputs) == len(node.output_sources) == 2


@pytest.mark.parametrize('info', [None, {}, SimpleNamespace(), [], 42])
def test_collision_missing_metadata_is_recorded_without_output(collision_simulation, info):
    node, _, _, _ = collision_simulation
    with pytest.raises(ValueError, match='positive integer'):
        node.receive(object(), info)
    assert node.outputs == node.output_sources == []
    assert node.evidence[-1]['event'] == 'output-metadata-invalid'


@pytest.mark.parametrize('cleanup_fails', [False, True])
def test_sonar_helper_direct_fixture_startup_and_cleanup(
        monkeypatch, tmp_path, capsys, cleanup_fails):
    root = Path(__file__).resolve().parents[2]
    source = root / 'ros2_ws/src/patrolbot_navigation/test/test_collision_monitor_integration.py'
    tree = ast.parse(source.read_text())
    tree.body = [item for item in tree.body if getattr(item, 'name', '') == 'monitor']
    tree.body[0].decorator_list = []
    scratch_paths = []

    def log_path(name, fallback):
        scratch_paths.append(fallback)
        assert fallback.is_dir()
        return fixture_log_path(name, fallback)

    namespace = dict(fixture_log_path=log_path, rclpy=object(), NativeFixture=Mock(
        side_effect=RuntimeError('native startup reached')))
    exec(compile(tree, str(source), 'exec'), namespace)
    fixture = namespace['monitor']
    if cleanup_fails:
        def fixture(factory):
            log_path('collision_monitor.log', factory.mktemp('collision-monitor')).touch()
            try:
                yield SimpleNamespace(command=lambda _: object())
            finally:
                raise RuntimeError('fixture cleanup failed')

    helper = root / 'docker/tests/integration_sonar_outage.py'
    tree = ast.parse(helper.read_text())
    # Run the actual helper body with ROS startup stubbed; never launch a worker.
    start = next(i for i, item in enumerate(tree.body)
                 if isinstance(item, ast.If))
    tree.body = tree.body[start:]
    logs = tmp_path / 'retained-logs'
    monkeypatch.setenv('ROS_LOG_DIR', str(logs))
    alarms = []
    module = SimpleNamespace(monitor=SimpleNamespace(__wrapped__=fixture),
                             PARAMS_FILE=Path('params.yaml'),
                             assert_velocity=Mock(side_effect=RuntimeError('case reached')))
    namespace = dict(os=os, tempfile=tempfile, Path=Path, SimpleNamespace=SimpleNamespace,
                     module=module, time=__import__('time'), json=json,
                     signal=SimpleNamespace(SIGALRM=signal.SIGALRM,
                                            signal=lambda *_: None, alarm=alarms.append))
    with pytest.raises(RuntimeError, match=(
            'fixture cleanup failed' if cleanup_fails else 'native startup reached')):
        exec(compile(tree, str(helper), 'exec'), namespace)
    assert scratch_paths and all(not path.exists() for path in scratch_paths)
    assert (logs / 'collision_monitor.log').exists()
    assert alarms == [30, 0]
    evidence = json.loads(capsys.readouterr().out)
    assert evidence['passed'] is False
    assert evidence['log_dir'] == str(logs)
    if cleanup_fails:
        assert evidence['cleanup_failure'] == 'fixture cleanup failed'
