"""Offline checks of actual method bodies; no ROS graph, sockets or hardware.

Usage: python3 -B -m unittest discover -s tests -v
Override PATROLBOT_TEST_SOURCE_ROOT to demonstrate failures on captured baseline.
"""
import ast
import importlib.util
import importlib
import math
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
from uuid import UUID


ROOT = Path(os.environ.get('PATROLBOT_TEST_SOURCE_ROOT',
                          str(Path(__file__).resolve().parents[1] / 'source')))
BRIDGE = ROOT / 'ros2_ws/src/patrolbot_bridge/patrolbot_bridge'
WEB = ROOT / 'ros2_ws/src/patrolbot_web_bridge/patrolbot_web_bridge/commands.py'
REJECT = 'REJECT'
if os.environ.get('PATROLBOT_TEST_INSTALLED') == '1':
    from rclpy.action import GoalResponse
    REJECT = GoalResponse.REJECT


def body_class(filename, name, extra=None):
    if os.environ.get('PATROLBOT_TEST_INSTALLED') == '1':
        module = importlib.import_module('patrolbot_bridge.' + Path(filename).stem)
        return getattr(module, name)
    tree = ast.parse((BRIDGE / filename).read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
    node.bases = []
    constants = [n for n in tree.body if isinstance(n, ast.Assign)]
    scope = dict(math=math, time=time, threading=threading, Mock=Mock, UUID=UUID,
                 GoalResponse=NS(REJECT='REJECT', ACCEPT='ACCEPT'),
                 EpochRecovery=NS(WAIT_RESET='wait-amcl-reset',
                                  WAIT_SEED='wait-operator-seed',
                                  WAIT_POSE='wait-amcl-pose', READY='ready'))
    scope.update(extra or {})
    code = ast.Module(body=constants + [node], type_ignores=[])
    exec(compile(ast.fix_missing_locations(code), str(BRIDGE / filename), 'exec'), scope)
    return scope[name]


Map = body_class('map_runtime.py', 'MapRuntimeMixin')
Epoch = body_class('epoch_runtime.py', 'EpochRuntime')
for name, value in Map.__dict__.items():
    if name not in ('__dict__', '__weakref__', '__doc__', '__module__'):
        setattr(Epoch, name, value)
Turn = body_class('undock_turn.py', 'UndockTurnRuntime')
spec = importlib.util.spec_from_file_location('manual_backport', BRIDGE / 'manual_active.py')
manual = importlib.util.module_from_spec(spec)
spec.loader.exec_module(manual)
Bridge = body_class('bridge_node.py', 'PatrolBotBridge',
                    {'manual_drive_bypass': manual.manual_drive_bypass})


class Future:
    def __init__(self, result=None, exception=None):
        self.value, self.exception = result, exception
        self.complete, self.reads = True, 0
    def done(self): return self.complete
    def result(self):
        self.reads += 1
        if self.exception: raise self.exception
        return self.value


def runtime():
    obj = Epoch.__new__(Epoch)
    obj.lock = threading.RLock()
    obj.maps = NS(stage='failed', hold=True, fail=Mock())
    obj.maps.token = 'map-request-original'
    obj._map_error = ''
    obj._map_owner_request_id = None
    obj._future = obj._future_kind = obj._map_future = obj._map_goal = None
    obj._lifecycle_mutation_unknown = obj._map_mutation_unknown = False
    obj._unknown_write_owners = []
    obj._failed = True
    obj._navigation_ready = False
    obj.undock_hold = False
    obj._navigation_status = False
    obj.recovery = NS(stage='wait-amcl-reset', token=1, observe_clock=Mock(), required=True)
    obj.guard = NS(ready=True, session=7)
    obj._token = 1
    obj._phase = 'activate'
    obj._deadline = time.monotonic() + 10
    obj.node = NS(get_logger=lambda: Mock(),
                  get_clock=lambda: NS(now=lambda: NS(nanoseconds=123)))
    obj._poll_tf_readiness = Mock()
    obj.poll_navigation_idle = Mock()
    obj.tick_startup_map = Mock()
    return obj


class MutationTests(unittest.TestCase):
    def test_epoch_exception_retains_future_and_holds(self):
        obj = runtime()
        obj.tick_map = Mock(return_value=False)
        future = Future(exception=RuntimeError('lost activate reply'))
        obj._future, obj._future_kind = future, 'mutation'
        obj.tick()
        self.assertIs(obj._future, future)
        self.assertTrue(obj._lifecycle_mutation_unknown)
        self.assertFalse(obj.local_motion_mutations_settled)

    def test_epoch_malformed_responses_hold(self):
        for value in (None, NS(), NS(success=1), NS(success='yes')):
            with self.subTest(value=value):
                obj = runtime()
                obj.tick_map = Mock(return_value=False)
                obj._future, obj._future_kind = Future(value), 'mutation'
                obj.tick()
                self.assertTrue(obj._lifecycle_mutation_unknown)
                self.assertFalse(obj.local_motion_mutations_settled)

    def test_epoch_answered_refusal_is_settled(self):
        obj = runtime()
        obj.tick_map = Mock(return_value=False)
        obj._future, obj._future_kind = Future(NS(success=False)), 'mutation'
        obj.tick()
        self.assertIsNone(obj._future)
        self.assertFalse(obj._lifecycle_mutation_unknown)
        self.assertTrue(obj.local_motion_mutations_settled)

    def test_old_token_does_not_launder_exception(self):
        obj = runtime()
        obj.tick_map = Mock(return_value=False)
        obj._token = -1
        future = Future(exception=RuntimeError('late old write'))
        obj._future, obj._future_kind = future, 'mutation'
        obj.tick()
        self.assertIs(obj._future, future)
        self.assertTrue(obj._lifecycle_mutation_unknown)

    def test_map_exception_and_missing_response_hold(self):
        for future in (Future(exception=RuntimeError('lost resume reply')),
                       Future(None), Future(NS()), Future(NS(success=1))):
            with self.subTest(future=future):
                obj = runtime()
                obj._map_step, obj._map_future = 'resume', future
                with self.assertRaises(Exception):
                    obj._map_call(Mock(), Mock(), 'clear-global', lambda r: r.success is True)
                self.assertIs(obj._map_future, future)
                self.assertTrue(obj._map_mutation_unknown)
                self.assertFalse(obj.local_motion_mutations_settled)

    def test_map_refusal_is_settled(self):
        obj = runtime()
        obj._map_step, obj._map_future = 'resume', Future(NS(success=False))
        with self.assertRaises(RuntimeError):
            obj._map_call(Mock(), Mock(), 'clear-global', lambda r: r.success is True)
        self.assertIsNone(obj._map_future)
        self.assertFalse(obj._map_mutation_unknown)
        self.assertTrue(obj.local_motion_mutations_settled)

    def test_submit_exception_holds(self):
        obj = runtime()
        obj._map_step = 'resume'
        client = NS(service_is_ready=lambda: True,
                    call_async=Mock(side_effect=RuntimeError('transport failure')))
        with self.assertRaises(RuntimeError):
            obj._map_call(client, Mock(), 'clear-global', lambda r: True)
        self.assertTrue(obj._map_mutation_unknown)
        self.assertFalse(obj.local_motion_mutations_settled)

    def test_unknown_does_not_clear_on_new_pose_or_tick(self):
        obj = runtime()
        obj._map_mutation_unknown = True
        self.assertEqual(obj._map_accept(Mock()), REJECT)
        obj.tick()
        self.assertTrue(obj._map_mutation_unknown)
        obj.tick_startup_map.assert_not_called()

    def test_epoch_unknown_does_not_clear_on_new_token_or_tick(self):
        obj = runtime()
        obj._lifecycle_mutation_unknown = True
        obj._token = -1
        self.assertEqual(obj._map_accept(Mock()), REJECT)
        obj.tick()
        self.assertTrue(obj._lifecycle_mutation_unknown)

    def test_failed_map_idle_probe_still_polled(self):
        obj = runtime()
        obj.tick()
        obj.poll_navigation_idle.assert_called_once()
        obj.tick_startup_map.assert_not_called()

    def test_undock_hold_issues_no_map_writes(self):
        obj = runtime()
        obj.undock_hold = True
        obj._tick_map_stage = Mock()
        self.assertTrue(obj.tick_map())
        obj._tick_map_stage.assert_not_called()
        obj.tick()
        obj.tick_startup_map.assert_not_called()

    def test_unrecognized_map_stage_refused_even_without_hold(self):
        obj = runtime()
        obj.maps.stage, obj.maps.hold = 'future-unrecognized-state', False
        self.assertFalse(obj.local_motion_mutations_settled)

    def test_pending_writes_and_goals_refused(self):
        for name in ('_future', '_map_future', '_map_goal'):
            with self.subTest(name=name):
                obj = runtime()
                setattr(obj, name, Future())
                obj._future_kind = 'mutation'
                self.assertFalse(obj.local_motion_mutations_settled)

    def test_execution_rechecks_unknown_mutations(self):
        obj = runtime()
        obj._map_mutation_unknown = True
        turn = Turn.__new__(Turn)
        turn.epoch = obj
        turn.node = Mock()
        reason = turn.hardware_problem(charging=True, stationary=True)
        self.assertIn('unknown', reason)

    def test_localization_callback_reservation_excludes_local_motion(self):
        obj = runtime()
        obj._map_owner_request_id = 'accepted-operator-request'
        self.assertFalse(obj.local_motion_mutations_settled)

    def test_unrelated_epoch_read_and_refusal_cannot_settle_old_write(self):
        for unrelated in (Future(NS(current_state=NS(id=2))),
                          Future(NS(result=NS(successful=False)))):
            with self.subTest(unrelated=unrelated):
                obj = runtime()
                original = Future(exception=RuntimeError('old navigation resume lost'))
                obj._future, obj._future_kind = original, 'mutation'
                with self.assertRaises(RuntimeError): obj._consume_epoch_future()
                owner = obj._unknown_write_owners[0]
                self.assertIs(owner['future'], original)
                self.assertEqual(owner['phase'], 'activate')
                self.assertEqual(owner['token'], 1)
                obj._future, obj._future_kind = unrelated, 'read-only'
                obj._phase, obj.recovery.token = 'inspect', 999
                obj.tick()
                self.assertIs(obj._unknown_write_owners[0]['future'], original)
                self.assertGreater(original.reads, 1)
                self.assertEqual(unrelated.reads, 0)
                self.assertFalse(obj.local_motion_mutations_settled)
                self.assertEqual(obj._map_accept(Mock()), REJECT)

    def test_unrelated_map_read_cannot_settle_old_write(self):
        obj = runtime()
        original = Future(exception=RuntimeError('old map pause lost'))
        obj._map_step, obj._map_future = 'pause', original
        with self.assertRaises(RuntimeError):
            obj._map_call(Mock(), Mock(), 'inspect', lambda r: r.success is True)
        owner = obj._unknown_write_owners[0]
        self.assertIs(owner['future'], original)
        self.assertEqual(owner['token'], 'map-request-original')
        unrelated = Future(NS(current_state=NS(id=2)))
        obj._map_step, obj._map_future = 'inspect', unrelated
        with self.assertRaises(RuntimeError):
            obj._map_call(Mock(), Mock(), 'reset', lambda r: True)
        obj.tick()
        self.assertEqual(unrelated.reads, 0)
        self.assertFalse(obj.local_motion_mutations_settled)
        self.assertEqual(obj._map_accept(Mock()), REJECT)

    def test_pending_old_epoch_write_survives_token_turnover(self):
        obj = runtime()
        obj.tick_map = Mock(return_value=False)
        original = Future(NS(success=True))
        original.complete = False
        obj._future, obj._future_kind = original, 'mutation'
        obj.recovery.token = 999
        obj.tick()
        self.assertIs(obj._future, original)
        self.assertFalse(obj.local_motion_mutations_settled)
        original.complete, original.exception = True, RuntimeError('old activation failed')
        obj.tick()
        self.assertIs(obj._unknown_write_owners[0]['future'], original)
        self.assertEqual(obj._unknown_write_owners[0]['token'], 1)
        self.assertFalse(obj.local_motion_mutations_settled)

    def test_only_same_write_ack_can_settle_abandoned_pending_map_write(self):
        obj = runtime()
        original = Future()
        original.complete = False
        obj._map_step, obj._map_future = 'pause', original
        obj._fail_map_transaction('operator canceled')
        owner = obj._unknown_write_owners[0]
        self.assertEqual(owner['token'], 'map-request-original')
        obj.maps.token, obj.recovery.token = 'different-request', 999
        obj.tick()
        self.assertIs(obj._unknown_write_owners[0]['future'], original)
        self.assertFalse(obj.local_motion_mutations_settled)
        original.value, original.complete = NS(success=False), True
        obj.tick()
        self.assertEqual(obj._unknown_write_owners, [])
        self.assertIsNone(obj._map_future)
        self.assertTrue(obj.local_motion_mutations_settled)
        obj.tick_startup_map.assert_not_called()

    def test_malformed_same_write_reply_stays_owned(self):
        obj = runtime()
        original = Future(NS(success='not a bool'))
        obj._map_step, obj._map_future = 'pause', original
        obj._fail_map_transaction('cancel')
        obj.tick()
        self.assertIs(obj._unknown_write_owners[0]['future'], original)
        self.assertEqual(obj._unknown_write_owners[0]['observation'], 'malformed')
        self.assertFalse(obj.local_motion_mutations_settled)


class ControllerTests(unittest.TestCase):
    def bridge(self):
        b = Bridge.__new__(Bridge)
        b._epoch = runtime()
        b._epoch._map_error = ''
        b._epoch.maps.active = NS()
        b._epoch.node = b
        b._epoch.navigation_idle_problem = Mock(return_value='')
        b._manual_active_at = time.monotonic()
        b._manual_asserted = True
        b._state_lock = threading.RLock()
        b.sock_lock = threading.RLock()
        b._undock_active = False
        b._undock_turn = NS(busy=False)
        b._lidar_ready = Mock(return_value=True)
        b.sock = Mock()
        b.get_publishers_info_by_topic = Mock(return_value=[NS(
            node_name='teleop_velocity_smoother', node_namespace='/',
            topic_type='geometry_msgs/msg/Twist')])
        return b

    def test_settled_failed_map_deadman_reaches_existing_socket(self):
        b = self.bridge()
        b.cmd_vel_callback(NS(linear=NS(x=.1), angular=NS(z=0.)))
        b.sock.sendall.assert_called_once_with(b'DRIVE:0.1:0.0\n')

    def test_release_unknown_idle_or_extra_publisher_sends_nothing(self):
        for change in ('release', 'unknown', 'idle', 'publisher'):
            with self.subTest(change=change):
                b = self.bridge()
                if change == 'release': b._manual_active_cb(NS(data=False))
                if change == 'unknown': b._epoch._map_mutation_unknown = True
                if change == 'idle': b._epoch.navigation_idle_problem.return_value = 'stale'
                if change == 'publisher': b.get_publishers_info_by_topic.return_value = []
                b.cmd_vel_callback(NS(linear=NS(x=.1), angular=NS(z=0.)))
                b.sock.sendall.assert_not_called()


class WordAndScopeTests(unittest.TestCase):
    def test_neutral_wording_and_no_future_interfaces(self):
        text = WEB.read_text()
        self.assertNotIn('started backing up', text)
        self.assertNotIn('Undocking took too long and was stopped.', text)
        self.assertNotIn('navigation_stationary_refresh_v1', text)
        self.assertNotIn('planning_error_code', text)
        self.assertNotIn('localization_pose_stamp_ns', (BRIDGE / 'bridge_node.py').read_text())
        self.assertNotIn('accepted_pose_stamp_ns', (BRIDGE / 'epoch_runtime.py').read_text())


if __name__ == '__main__':
    unittest.main()
