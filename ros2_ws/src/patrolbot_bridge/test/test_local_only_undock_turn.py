"""Production reserve/reverse/local turn/release with isolated lifecycle doubles."""
import ast
from concurrent.futures import Future
from functools import partial
import math
from pathlib import Path
import struct
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock
from uuid import uuid4

import pytest
from patrolbot_bridge.epoch_recovery import EpochRecovery


def done(value):
    future = Future()
    future.set_result(value)
    return future


def twist():
    return NS(linear=NS(x=0.0, y=0.0, z=0.0), angular=NS(x=0.0, y=0.0, z=0.0))


@pytest.fixture
def rig():
    source = Path(__file__).parents[1] / 'patrolbot_bridge/undock_turn.py'
    tree = ast.parse(source.read_text())
    tree.body = [n for n in tree.body if not isinstance(n, (ast.Import, ast.ImportFrom))]
    clock = NS(now=100.0)
    node = Mock()
    clock.monotonic = lambda: clock.now

    def sleep(seconds):
        clock.now += seconds
        for name in (
            '_charge_state_at', '_flags_at', '_pose_at', '_undock_scan_at', '_dock_state_at'
        ):
            setattr(node, name, clock.now)
    clock.sleep = sleep
    namespace = dict(partial=partial, math=math, struct=struct, time=clock, uuid4=uuid4,
                     Twist=twist, GetState=NS(Request=NS),
                     ChangeState=NS(Request=lambda: NS(transition=NS())),
                     Trigger=NS(Request=NS), ManageLifecycleNodes=NS(Request=NS),
                     UndockTurn=NS(Result=type('Result', (NS,), dict(
                         OK=0, CANCELED=1, TIMEOUT=2, MOTION_FAILED=3, SAFETY_FAULT=4)),
                         Feedback=NS), Spin=NS(Goal=lambda: NS(time_allowance=NS()),
                                               Result=NS(NONE=0)),
                     UUID=NS, GoalStatusArray=NS,
                     GoalResponse=NS(ACCEPT=True, REJECT=False), CancelResponse=NS(ACCEPT=True),
                     ActionClient=Mock(), ActionServer=Mock(),
                     qos_profile_action_status_default=None, EpochRecovery=EpochRecovery)
    exec(compile(tree, str(source), 'exec'), namespace)
    recovery = EpochRecovery()
    recovery.observe_origin('00000000-0000-0000-0000-000000000001')
    recovery.reset_complete(recovery.token, 1)
    epoch = NS(lock=threading.RLock(), guard=NS(ready=True, epoch='origin'), recovery=recovery,
               tuning_hold=False, _future=None, _map_future=None, _map_goal=None,
               maps=NS(hold=False), _failed=False, _navigation_ready=False,
               _localization_active=Mock(), _get=Mock(), withdraw=Mock())
    node._epoch = epoch
    node._state_lock = threading.RLock()
    node._pose_lock = threading.RLock()
    node._control_lock = threading.RLock()
    node._hardware_operation_lock = threading.Lock()
    node._pending_controls = {}
    node._link_connected = node._pose_received = node._undock_profile_commissioned = True
    node._estop_pressed = False
    node._fault_flags = 0
    node._px = node._py = node._pth = node._vx = node._vth = 0.0
    node._charge_state = 2
    node._dock_state = 'DOCKED_CONFIRMED'
    node._undock_active = False
    node._motors_enabled = True
    node._session_generation = 7
    node._motor_disable_generation = 0
    node._undock_request_id = None
    node._lidar_ready.return_value = True
    node.get_clock.return_value.now.return_value.nanoseconds = 10
    sleep(0)
    states = dict.fromkeys((*namespace['NAVIGATION_NODES'], namespace['LOCAL_NODES'][0]), 2)

    def client(_kind, path, **kwargs):
        client = Mock()

        def call(request):
            name = path.strip('/').rsplit('/', 1)[0]
            if path.endswith('/get_state'):
                return done(NS(current_state=NS(id=states[name])))
            if path.endswith('/change_state'):
                states[name] = 3 if request.transition.id == 3 else 2
                return done(NS(success=True))
            if path.endswith('/is_active'):
                return done(NS(success=False))
            return done(NS(success=True))
        client.call_async.side_effect = call
        return client
    node.create_client.side_effect = client
    node.get_publishers_info_by_topic.side_effect = lambda topic: [
        NS(node_name=namespace['PATH_PUBLISHERS'][topic], node_namespace='/',
           topic_type='geometry_msgs/msg/Twist')
        for _ in range(4 if topic == '/input/nav_behavior' else 1)]
    runtime = namespace['UndockTurnRuntime'](node)
    runtime.idle_at = clock.now
    return NS(runtime=runtime, node=node, epoch=epoch, clock=clock, states=states,
              names=namespace, hardware_goal=NS(goal_id=NS(uuid=list(range(16))),
                                                is_cancel_requested=False))


def reverse(rig):
    runtime = rig.runtime
    assert runtime.reserve_reverse()
    assert runtime.prepare_reverse(rig.hardware_goal) == ''
    rig.node._charge_state = 0
    rig.node._dock_state = 'CLEAR_CONFIRMED'
    runtime.reverse_result(rig.hardware_goal, NS(success=True, physically_clear=True,
                                                 dock_state='CLEAR_CONFIRMED',
                                                 motors_enabled=True))
    request = NS(hardware_goal_id=rig.hardware_goal.goal_id, session_generation=7,
                 target_yaw=math.pi)
    assert runtime.accept(request)


@pytest.mark.parametrize('map_store', [False, True])
def test_wait_seed_local_turn_releases_owner_without_amcl_or_navigation(rig, map_store):
    if not map_store:
        rig.epoch.maps = None
    reverse(rig)
    runtime = rig.runtime
    goal = Mock(is_cancel_requested=False)
    spin_handle = Mock(accepted=True)
    spin_handle.get_result_async.return_value = done(NS(status=4, result=NS(error_code=0)))

    def send(request, goal_uuid):
        assert request.target_yaw == math.pi
        assert request.time_allowance.sec == 20
        spin_handle.goal_id = goal_uuid
        # Model fresh measured travel delivered by the raw odometry stream.
        rig.node._pth = math.pi
        rig.node._pose_at += 0.001
        return done(spin_handle)
    runtime.spin.send_goal_async.side_effect = send
    result = runtime.execute(goal)
    assert result.success, result.message
    assert result.radians_traveled == math.pi
    assert runtime.phase == 'idle' and not rig.epoch.undock_hold
    assert not rig.node._hardware_operation_lock.locked()
    assert rig.epoch.recovery.stage == EpochRecovery.WAIT_SEED
    assert not rig.epoch._navigation_ready
    assert rig.epoch._accepted_seed_stamp is None
    assert all(value == 2 for value in rig.states.values())
    rig.epoch._localization_active.call_async.assert_not_called()
    rig.epoch._get.call_async.assert_not_called()
    runtime.manager.call_async.assert_not_called()  # no global resume
    for call in rig.node.cmd_vel_callback.call_args_list:
        assert call.args[0].linear.x == call.args[0].angular.z == 0.0
    assert runtime.final_pub.publish.called


@pytest.mark.parametrize('bad', ['stale', 'invalid', 'fault', 'estop', 'origin',
                                 'uncommissioned', 'moving', 'lidar',
                                 'active_action', 'native_unknown',
                                 'map_pending', 'map_held', 'lifecycle_pending',
                                 'lifecycle_failed', 'other_owner'])
def test_reservation_never_bypasses_hardware_or_owner(rig, bad):
    if bad == 'stale':
        rig.node._pose_at -= 1.0
    elif bad == 'invalid':
        rig.node._px = math.nan
    elif bad == 'fault':
        rig.node._fault_flags = 1
    elif bad == 'estop':
        rig.node._estop_pressed = True
    elif bad == 'origin':
        rig.epoch.guard.ready = False
    elif bad == 'uncommissioned':
        rig.node._undock_profile_commissioned = False
    elif bad == 'moving':
        rig.node._vx = 0.1
    elif bad == 'lidar':
        rig.node._lidar_ready.return_value = False
    elif bad == 'active_action':
        rig.runtime.actions['navigate_to_pose'] = True
    elif bad == 'native_unknown':
        rig.runtime.idle_at = None
    elif bad == 'map_pending':
        rig.epoch._map_future = Future()
    elif bad == 'map_held':
        rig.epoch.maps.hold = True
    elif bad == 'lifecycle_pending':
        rig.epoch._future, rig.epoch._future_kind = Future(), 'mutation'
    elif bad == 'lifecycle_failed':
        rig.epoch._failed = True
    elif bad == 'other_owner':
        rig.node._hardware_operation_lock.acquire()
    assert not rig.runtime.reserve_reverse()
    assert rig.runtime.phase == 'idle'


def test_reverse_session_change_cannot_create_turn_token(rig):
    assert rig.runtime.reserve_reverse()
    assert rig.runtime.prepare_reverse(rig.hardware_goal) == ''
    rig.node._session_generation += 1
    reply = NS(success=True, physically_clear=True,
               dock_state='CLEAR_CONFIRMED', motors_enabled=True)
    rig.runtime.reverse_result(rig.hardware_goal, reply)
    assert rig.runtime.token is None
    assert rig.runtime.reason


def test_unknown_reverse_keeps_owner_and_zero_hold(rig):
    assert rig.runtime.reserve_reverse()
    assert rig.runtime.prepare_reverse(rig.hardware_goal) == ''
    rig.runtime.reverse_unknown(rig.hardware_goal, 'no terminal frame')
    assert not rig.runtime.finish()
    assert rig.epoch.undock_hold and rig.node._hardware_operation_lock.locked()
    assert not rig.runtime.drive_allowed


@pytest.mark.parametrize('prior_reason', ['', 'hardware or odometry is stale'])
def test_reverse_result_only_reports_a_preexisting_revocation(rig, prior_reason):
    assert rig.runtime.reserve_reverse()
    assert rig.runtime.prepare_reverse(rig.hardware_goal) == ''
    if prior_reason:
        rig.runtime.revoke(prior_reason)
    assert rig.runtime.reverse_result(rig.hardware_goal, NS(success=False)) == prior_reason


def test_collision_zero_cannot_become_turn_velocity(rig):
    reverse(rig)
    runtime = rig.runtime
    runtime.phase = 'turn'
    runtime.spin_active = runtime.graph_valid = True
    runtime.started = runtime.global_checked_at = rig.clock.now
    runtime.behavior_at = rig.clock.now
    runtime.collision(twist())
    command = runtime.final_pub.publish.call_args.args[0]
    assert command.linear.x == command.angular.z == 0.0
    runtime.manual(twist())
    assert not runtime.reason
    command = twist()
    command.angular.z = 0.1
    runtime.manual(command)
    assert not runtime.drive_allowed
    assert runtime.reason == 'manual command took ownership'


def test_base_state_without_joystick_does_not_cancel_reverse(rig):
    source = Path(__file__).parents[2] / 'patrolbot_navigation/scripts/patrolbot_joy_teleop.py'
    teleop = next(n for n in ast.parse(source.read_text()).body
                  if isinstance(n, ast.ClassDef) and n.name == 'JoyTeleop')
    methods = [n for n in teleop.body if isinstance(n, ast.FunctionDef)
               and n.name in ('_base_state_cb', '_block_undock_input')]
    namespace = dict(Twist=twist, Bool=NS)
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), 'exec'), namespace)
    joy = NS(_last_joy=None, _lidar_generation=None, pub=Mock(), manual_pub=Mock(),
             get_clock=rig.node.get_clock)
    joy._block_undock_input = partial(namespace['_block_undock_input'], joy)
    assert rig.runtime.reserve_reverse()
    assert rig.runtime.prepare_reverse(rig.hardware_goal) == ''
    rig.node._undock_request_id = str(uuid4())
    joy.pub.publish.side_effect = rig.runtime.manual
    for _ in range(3):
        namespace['_base_state_cb'](joy, NS(undock_active=True, link_connected=True,
                                          session_generation=7))
    assert joy._last_joy is None
    assert joy.manual_pub.publish.call_args.args[0].data is False
    assert rig.runtime.reason == '' and rig.runtime.phase == 'reverse'
    assert rig.runtime.token is None and not rig.runtime.drive_allowed
    rig.node._send_undock_cancel.assert_not_called()
    rig.node.get_logger().warn.assert_not_called()


@pytest.mark.parametrize('phase', ['reverse', 'handoff', 'turn'])
def test_neutral_does_not_grant_or_restore_ownership(rig, phase):
    reverse(rig)
    runtime = rig.runtime
    runtime.phase = phase
    runtime.revoke('hardware telemetry expired')
    for _ in range(3):
        runtime.manual(twist())
    assert runtime.reason == 'hardware telemetry expired'
    assert runtime.token is None and not runtime.spin_active and not runtime.drive_allowed
    assert runtime.behavior_at is None and runtime.collision_at is None


@pytest.mark.parametrize('fault,reason', [
    ('_estop_pressed', 'E-stop or hardware fault'),
    ('_fault_flags', 'E-stop or hardware fault'),
    ('_link_connected', 'hardware or odometry is stale'),
    ('_flags_at', 'hardware or odometry is stale'),
])
def test_neutral_does_not_bypass_independent_reverse_guards(rig, fault, reason):
    assert rig.runtime.reserve_reverse()
    command = twist()
    command.linear.x = command.angular.z = -0.0
    rig.runtime.manual(command)
    assert not rig.runtime.reason
    setattr(rig.node, fault, False if fault == '_link_connected' else (
        rig.clock.now - 1.0 if fault == '_flags_at' else 1))
    rig.runtime.tick()
    assert rig.runtime.reason == reason
    rig.runtime.manual(command)
    assert rig.runtime.reason == reason and not rig.runtime.drive_allowed


def test_revocation_log_escapes_untrusted_text(rig):
    assert rig.runtime.reserve_reverse()
    rig.runtime.revoke('failed\nforged log line')
    log = rig.node.get_logger().warn.call_args.args[0]
    assert '\n' not in log
    assert r'failed\nforged log line' in log


@pytest.mark.parametrize('vector', ['linear', 'angular'])
@pytest.mark.parametrize('axis', ['x', 'y', 'z'])
@pytest.mark.parametrize('value', [1e-300, -0.1, math.nan, math.inf, -math.inf, None, '0'])
def test_each_nonzero_or_invalid_component_revokes(rig, vector, axis, value):
    assert rig.runtime.reserve_reverse()
    rig.node._undock_request_id = str(uuid4())
    message = twist()
    setattr(getattr(message, vector), axis, value)
    rig.runtime.manual(message)
    assert rig.runtime.reason == 'manual command took ownership'
    assert rig.runtime.token is None and not rig.runtime.drive_allowed
    rig.node._send_undock_cancel.assert_called_once_with(rig.node._undock_request_id)
    rig.runtime.revoke('later fault')
    rig.node.get_logger().warn.assert_called_once()
    log = rig.node.get_logger().warn.call_args.args[0]
    assert rig.node._undock_request_id in log
    assert "phase='reverse'" in log and 'manual command took ownership' in log
    assert 'later fault' not in log


@pytest.mark.parametrize('message', [None, NS(), NS(linear=NS(x=0.0))])
def test_malformed_manual_message_still_revokes(rig, message):
    assert rig.runtime.reserve_reverse()
    rig.runtime.manual(message)
    assert rig.runtime.reason == 'manual command took ownership'


def test_unknown_turn_acceptance_retains_local_owner(rig):
    reverse(rig)
    rig.runtime.spin.send_goal_async.return_value = Future()
    goal = Mock(is_cancel_requested=False)
    result = rig.runtime.execute(goal)
    assert not result.success
    assert rig.runtime.unknown
    assert rig.epoch.undock_hold
    assert rig.node._hardware_operation_lock.locked()
    assert not rig.runtime.drive_allowed
    goal.succeed.assert_not_called()
