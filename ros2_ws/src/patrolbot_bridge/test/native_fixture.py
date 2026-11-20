"""Shared test-only lifetime ownership; no ROS imports for offline checks."""

import signal
import subprocess
import threading
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path


def fixture_log_path(filename, fallback_directory):
    """Keep native child output in uploaded evidence when the runner supplies it."""
    directory = Path(os.environ.get('ROS_LOG_DIR', fallback_directory))
    directory.mkdir(parents=True, exist_ok=True)
    return directory / filename


def executor_tasks(executor):
    """TEST ONLY: snapshot the two known rclpy private task storage layouts."""
    if isinstance(getattr(executor, '_tasks', None), list):
        tasks = [task for task, _entity, _node in executor._tasks]
    elif (isinstance(getattr(executor, '_pending_tasks', None), dict)
          and isinstance(getattr(executor, '_ready_tasks', None), deque)
          and isinstance(getattr(executor, '_futures', None), list)):
        tasks = list(executor._pending_tasks) + list(executor._ready_tasks)
    else:
        raise RuntimeError('unsupported test executor task storage; update TEST ONLY adapter')
    # Completed handlers may already be absent from the pending collection.
    return tasks + list(getattr(executor, '_futures', ()))


class NativeFixture:
    def __init__(self, ros, timeout=5.0):
        self.ros = ros
        self.timeout = timeout
        self.nodes = []
        self.executor = None
        self.spin_thread = None
        self.stop = threading.Event()
        self.errors = []
        self.cleanups = []
        self.initialized = False

    def __enter__(self):
        self.previous_hook = threading.excepthook
        # Include bridge workers and fake scripts, not only the spin thread.
        threading.excepthook = self._thread_error
        return self

    def _thread_error(self, args):
        self.errors.append(args.exc_value)

    def init(self):
        self.ros.init()
        self.initialized = True

    def node(self, node):
        self.nodes.append(node)
        return node

    def start(self, executor):
        self.executor = executor
        for node in self.nodes:
            executor.add_node(node)

        def spin():
            while not self.stop.is_set():
                executor.spin_once(timeout_sec=0.05)

        thread = threading.Thread(target=spin, name='fixture-spin', daemon=True)
        thread.start()
        self.spin_thread = thread

    def process(self, process):
        def close():
            assert process.poll() is None, f'child exited early: {process.returncode}'
            process.terminate()
            try:
                code = process.wait(timeout=self.timeout)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=self.timeout)
                raise AssertionError('child required forced kill')
            assert code in (0, -signal.SIGTERM), f'unexpected child exit: {code}'
        self.cleanups.append(close)
        return process

    def __exit__(self, exc_type, primary, traceback):
        failures = []

        def attempt(action):
            try:
                action()
                return True
            except BaseException as error:
                failures.append(error)
                return False

        def join(thread):
            thread.join(timeout=self.timeout)
            assert not thread.is_alive(), f'{thread.name} survived cleanup'

        safe = True
        try:
            self.stop.set()
            if self.executor is not None:
                attempt(self.executor.wake)
            if self.spin_thread is not None:
                safe = attempt(lambda: join(self.spin_thread))
            if safe and self.executor is not None:
                tasks = []
                attempt(lambda: tasks.extend(executor_tasks(self.executor)))
                # Jazzy may have submitted handlers not yet counted as active work.
                # TEST ONLY: drain the known private pool before destroying guards.
                def drain_workers():
                    pool = getattr(self.executor, '_executor', None)
                    if not isinstance(pool, ThreadPoolExecutor):
                        raise RuntimeError('unsupported test executor worker pool; '
                                           'update TEST ONLY adapter')
                    pool.shutdown(wait=True, cancel_futures=True)

                drained = []
                drain = threading.Thread(
                    target=lambda: drained.append(attempt(drain_workers)),
                    name='fixture-worker-drain', daemon=True)
                safe = attempt(drain.start)
                if safe:
                    safe = attempt(lambda: join(drain)) and drained == [True]
                if safe:
                    attempt(lambda: tasks.extend(executor_tasks(self.executor)))
                    def check_tasks():
                        for task in dict.fromkeys(tasks):
                            if task.done() and not task.cancelled():
                                error = task.exception()
                                if error is not None:
                                    failures.append(error)
                    attempt(check_tasks)
                    def shutdown():
                        assert self.executor.shutdown(timeout_sec=self.timeout), (
                            'executor failed to drain callbacks')
                    safe = attempt(shutdown)
            # Keep handles/context valid if a callback could still be using them.
            if safe:
                nodes_stopped = True
                for node in reversed(self.nodes):
                    def stop_workers():
                        # Bridge workers also use ROS handles outside the executor.
                        workers = list(getattr(node, '_worker_threads', ()))
                        if workers:
                            node._stop_event.set()
                            node.running = False
                            stopped = [attempt(node._manual_undock.destroy)]
                            manual = node._manual_undock.thread
                            if manual is not None:
                                workers.append(manual)
                            for worker in workers:
                                stopped.append(attempt(lambda worker=worker: join(worker)))
                            assert all(stopped), 'bridge workers failed to quiesce'
                    if not attempt(stop_workers):
                        nodes_stopped = False
                        continue
                    if self.executor is not None:
                        attempt(lambda node=node: self.executor.remove_node(node))
                    attempt(node.destroy_node)
                if self.initialized and nodes_stopped:
                    attempt(self.ros.try_shutdown)
            for cleanup in reversed(self.cleanups):
                attempt(cleanup)
            failures.extend(self.errors)
        finally:
            threading.excepthook = self.previous_hook
        if failures:
            error = BaseExceptionGroup('native fixture cleanup failed', failures)
            if primary is not None:
                primary.add_note(str(error))
                for failure in failures:
                    primary.add_note(repr(failure))
            else:
                raise error
        return False
