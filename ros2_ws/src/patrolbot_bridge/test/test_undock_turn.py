"""Exercise owned undock persistence, safety stops and lifecycle restoration."""
import math
import threading
import time
from unittest.mock import patch
from types import SimpleNamespace as NS
from unittest.mock import Mock

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from patrolbot_interfaces.action import HardwareUndock, UndockTurn
from unique_identifier_msgs.msg import UUID
import pytest

from patrolbot_bridge.epoch_recovery import EpochRecovery
from patrolbot_bridge.epoch_runtime import EpochRuntime
from patrolbot_bridge.bridge_node import PatrolBotBridge
from patrolbot_bridge.undock_turn import (
    ACTION_NAMES, LOCAL_NODES, NAVIGATION_NODES, PATH_PUBLISHERS,
    UndockTurnRuntime, exclusive_publishers, pure_angular)


ORIGIN = '123e4567-e89b-12d3-a456-426614174000'
GOAL_ID = bytes(range(16))


class Future:
    def __init__(self, result=None, done=True):
        self.value = result
        self.complete = done
        self.cancel = Mock()

    def done(self):
        return self.complete

    def result(self):
        return self.value


def runtime():
    now = time.monotonic()
    recovery = EpochRecovery()
    recovery.observe_origin(ORIGIN)
    epoch = NS(lock=threading.RLock(), guard=NS(ready=True, epoch=ORIGIN),
               recovery=recovery, _future=None, _future_kind=None,
               _map_future=None, _map_goal=None, maps=NS(hold=False),
               _failed=False, undock_hold=False, withdraw=Mock())
    node = NS(_epoch=epoch, _state_lock=threading.Lock(),
              _pose_lock=threading.Lock(), _control_lock=threading.Lock(),
              _hardware_operation_lock=threading.Lock(),
              _pending_controls={}, _link_connected=True, _pose_received=True,
              _charge_state_at=now, _flags_at=now, _pose_at=now,
              _undock_scan_at=now, _dock_state_at=now, _estop_pressed=False,
              _fault_flags=0, _undock_profile_commissioned=True,
              _px=0., _py=0., _pth=0., _vx=0., _vth=0.,
              _charge_state=2, _dock_state='DOCKED_CONFIRMED',
              _undock_active=False, _motors_enabled=False,
              _session_generation=4, _motor_disable_generation=0,
              _undock_request_id='hardware-request',
              _lidar_ready=Mock(return_value=True),
              _send_undock_cancel=Mock(return_value=True),
              cmd_vel_callback=Mock(),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=100)))
    node.get_publishers_info_by_topic = Mock(side_effect=lambda topic: [
        NS(node_name=PATH_PUBLISHERS[topic], node_namespace='/',
           topic_type='geometry_msgs/msg/Twist')
        for _ in range(4 if topic == '/input/nav_behavior' else 1)])
    turn = UndockTurnRuntime.__new__(UndockTurnRuntime)
    turn.node, turn.epoch = node, epoch
    turn.phase, turn.reason, turn.token = 'idle', '', None
    turn.hardware_goal_id = GOAL_ID
    turn.spin_id, turn.spin_handle, turn.spin_active = None, None, False
    turn.pending, turn.unknown, turn.started, turn.bound = None, False, None, None
    turn.radians, turn.last_yaw, turn.last_odom_at = 0., None, None
    turn.target_reached = False
    turn.behavior_at, turn.collision_at = None, None
    turn.graph_valid, turn.global_checked_at, turn.global_probes = True, now, {}
    turn.global_requested_at = now
    turn.idle_at, turn.idle_future, turn.idle_requested_at = now, None, now
    turn.idle_client = NS(service_is_ready=lambda: False)
    turn.actions = {name: False for name in ACTION_NAMES}
    turn.hardware_cancel_sent = turn.spin_cancel_sent = turn.validation_mode = False
    turn.hardware_operation_owned = False
    turn.finishing = False
    turn.docking_pub, turn.final_pub = Mock(), Mock()
    turn.manager_active = NS(call_async=lambda _: Future(NS(success=False)))
    turn.states = {name: NS(service_is_ready=lambda: True,
                          call_async=lambda _: Future(NS(current_state=NS(id=2))))
                   for name in (*NAVIGATION_NODES, LOCAL_NODES[0])}
    turn.changes = {name: Mock() for name in LOCAL_NODES}
    return turn


def handoff(turn):
    assert turn.reserve_reverse()
    goal = NS(goal_id=UUID(uuid=list(GOAL_ID)), is_cancel_requested=False)
    assert turn.prepare_reverse(goal) == ''
    turn.node._dock_state, turn.node._motors_enabled = 'CLEAR_CONFIRMED', True
    reply = NS(success=True, physically_clear=True, dock_state='CLEAR_CONFIRMED',
               motors_enabled=True)
    turn.reverse_result(goal, reply)
    return goal


def request(goal_id=GOAL_ID, session=4):
    goal = UndockTurn.Goal()
    goal.hardware_goal_id = UUID(uuid=list(goal_id))
    goal.session_generation = session
    return goal


def executing(turn):
    handoff(turn)
    assert turn.accept(request()).name == 'ACCEPT'
    turn.phase, turn.spin_id, turn.spin_active = 'turn', GOAL_ID, True
    turn.started = turn.behavior_at = turn.collision_at = time.monotonic()
    turn.spin_handle = Mock()
    turn.last_yaw, turn.last_odom_at = turn.node._pth, turn.node._pose_at
    return turn


@pytest.mark.parametrize('angular_rate', [-.0628319, .767945, .785398, 1.5])
def test_measured_angular_rate_excursions_do_not_cancel_owned_turn(angular_rate):
    turn = executing(runtime())
    turn.node._vth = angular_rate
    assert turn.hardware_problem(turn=True) == ''
    assert turn.drive_allowed
    assert turn.hardware_problem(turn=True, stationary=True) == 'robot must be stationary'


@pytest.mark.parametrize('field,value,reason', [
    ('_vth', float('nan'), 'odometry is invalid'),
    ('_vth', float('inf'), 'odometry is invalid'),
    ('_vx', float('nan'), 'odometry is invalid'),
    ('_pth', float('inf'), 'odometry is invalid'),
    ('_pose_at', 0., 'hardware or odometry is stale'),
    ('_undock_scan_at', 0., 'hardware or odometry is stale'),
    ('_estop_pressed', True, 'E-stop or hardware fault'),
    ('_fault_flags', 1, 'E-stop or hardware fault'),
])
def test_motion_precision_relaxation_preserves_live_feedback_and_fault_checks(field, value, reason):
    turn = executing(runtime())
    setattr(turn.node, field, value)
    assert turn.hardware_problem(turn=True) == reason
    assert not turn.drive_allowed


@pytest.mark.parametrize('linear,angular', [(.011, .785398), (-.03, -.0628319), (.1, 1.5)])
def test_finite_measured_drift_does_not_abort_the_owned_turn(linear, angular):
    turn = executing(runtime())
    turn.node._vx, turn.node._vth = linear, angular
    assert turn.hardware_problem(turn=True) == ''
    assert turn.drive_allowed
    assert turn.hardware_problem(turn=True, stationary=True) == 'robot must be stationary'


def test_cold_no_amcl_admits_only_scoped_streams_and_preserves_reset_barrier():
    turn = runtime()
    original_token = turn.epoch.recovery.token
    assert not EpochRuntime.streams_allowed.fget(turn.epoch)
    handoff(turn)
    assert turn.epoch.recovery.stage == EpochRecovery.WAIT_RESET
    assert turn.epoch.recovery.token == original_token
    assert EpochRuntime.streams_allowed.fget(turn.epoch)
    assert not EpochRuntime.commands_allowed.fget(turn.epoch)
    assert turn.finish()
    assert not EpochRuntime.streams_allowed.fget(turn.epoch)
    assert turn.epoch.recovery.stage == EpochRecovery.WAIT_RESET
    turn.epoch.withdraw.assert_called_once()


@pytest.mark.parametrize('field,value', [
    ('_charge_state', 0), ('_charge_state', -1),
    ('_dock_state', 'CLEAR_CONFIRMED'), ('_estop_pressed', True),
    ('_fault_flags', 1), ('_pose_at', 0.), ('_undock_scan_at', 0.),
    ('_vx', .03), ('_pth', float('nan')), ('_undock_profile_commissioned', False)])
def test_charging_admission_fails_before_any_effect(field, value):
    turn = runtime()
    setattr(turn.node, field, value)
    assert not turn.reserve_reverse()
    assert not turn.epoch.undock_hold
    turn.node.cmd_vel_callback.assert_not_called()


def test_pending_lifecycle_mutation_never_becomes_a_read_only_exception():
    turn = runtime()
    pending = Future(done=False)
    turn.epoch._future, turn.epoch._future_kind = pending, 'mutation'
    assert not turn.reserve_reverse()
    pending.cancel.assert_not_called()
    assert turn.epoch._future is pending


def test_old_reserved_goal_rejection_cannot_mint_a_new_reverse_hold():
    turn = runtime()
    turn.node._undock_turn = turn
    turn.node._undock_goal_lock = threading.Lock()
    turn.node._undock_goal_reserved = True
    turn.node._server_hello = NS(supports_undock=lambda: True)
    turn.node.get_logger = lambda: Mock()
    goal = HardwareUndock.Goal()
    goal.clearance_distance, goal.reverse_speed, goal.maximum_duration = .53, .1, 45.
    assert PatrolBotBridge._undock_goal_callback(turn.node, goal).name == 'REJECT'
    assert turn.phase == 'idle' and not turn.epoch.undock_hold
    assert not turn.node._hardware_operation_lock.locked()


def test_only_explicitly_read_only_epoch_probe_can_be_withdrawn():
    turn = runtime()
    pending = Future(done=False)
    turn.epoch._phase = 'wait-localization-manager'
    turn.epoch._localization_active = Mock()
    turn.epoch._future, turn.epoch._future_kind = pending, 'read-only'
    assert turn.reserve_reverse()
    turn.epoch._localization_active.remove_pending_request.assert_called_once_with(pending)
    pending.cancel.assert_called_once()
    assert turn.epoch._future is None


def test_handoff_is_correlated_and_consumed_once():
    turn = runtime()
    handoff(turn)
    assert turn.accept(request(bytes([99]) * 16)).name == 'REJECT'
    assert turn.accept(request(session=5)).name == 'REJECT'
    assert turn.token is not None
    assert turn.accept(request()).name == 'ACCEPT'
    assert turn.token is None
    assert turn.accept(request()).name == 'REJECT'


def test_clear_hardware_result_is_required_before_turn():
    turn = runtime()
    assert turn.reserve_reverse()
    assert turn.accept(request()).name == 'REJECT'
    goal = NS(is_cancel_requested=False, goal_id=UUID(uuid=list(GOAL_ID)))
    turn.hardware_goal_id = GOAL_ID
    turn.reverse_result(goal, NS(success=False))
    assert turn.token is None
    assert turn.accept(request()).name == 'REJECT'


def test_duplicate_publisher_name_is_still_foreign():
    turn = runtime()
    assert exclusive_publishers(turn.node)
    def endpoints(topic):
        endpoint = NS(node_name=PATH_PUBLISHERS[topic], node_namespace='/',
                      topic_type='geometry_msgs/msg/Twist')
        return ([endpoint, endpoint] if topic == '/cmd_vel' else
                [endpoint] * (4 if topic == '/input/nav_behavior' else 1))
    turn.node.get_publishers_info_by_topic.side_effect = endpoints
    assert not exclusive_publishers(turn.node)


@pytest.mark.parametrize('field,value', [
    ('linear.x', .01), ('linear.y', .01), ('linear.z', .01),
    ('angular.x', .01), ('angular.y', .01), ('angular.z', -.01),
    ('angular.z', .701), ('angular.z', float('nan'))])
def test_every_unowned_component_or_angular_limit_is_rejected(field, value):
    message = Twist()
    vector, axis = field.split('.')
    setattr(getattr(message, vector), axis, value)
    assert not pure_angular(message)


def test_collision_stop_is_preserved_and_stale_output_cannot_move():
    turn = executing(runtime())
    positive = Twist()
    positive.angular.z = .5
    turn.behavior(positive)
    turn.collision(Twist())
    assert turn.final_pub.publish.call_args.args[0].angular.z == 0.
    assert turn.drive_allowed
    turn.collision_at -= .251
    assert not turn.drive_allowed


def test_stale_source_revokes_and_late_fresh_callbacks_cannot_resume():
    turn = executing(runtime())
    turn.behavior_at -= .251
    turn.tick()
    assert turn.reason == 'undock velocity source expired'
    positive = Twist()
    positive.angular.z = .5
    turn.behavior(positive)
    turn.collision(positive)
    assert not turn.drive_allowed
    assert turn.final_pub.publish.call_args.args[0].angular.z == 0.


def test_collision_zero_can_resume_owned_turn_when_fresh_clear_output_returns():
    turn = executing(runtime())
    positive = Twist(); positive.angular.z = .5
    turn.behavior(positive)
    turn.collision(Twist())
    assert turn.reason == '' and turn.final_pub.publish.call_args.args[0].angular.z == 0.
    turn.collision(positive)
    assert turn.reason == '' and turn.final_pub.publish.call_args.args[0].angular.z == .5


@pytest.mark.parametrize('delta', [-.04, .2, .6])
def test_heading_variation_keeps_ownership_and_nominal_pi_still_stops(delta):
    turn = executing(runtime())
    turn.radians = math.pi - .1
    turn.node._pth = delta
    turn.last_odom_at -= .05
    turn.node._pose_at = time.monotonic()
    turn.observe_turn_odom()
    assert turn.reason == ''
    assert turn.radians == pytest.approx(math.pi - .1 + delta)
    assert turn.target_reached is (delta >= .1)
    if turn.target_reached:
        assert turn.phase == 'stopping' and not turn.drive_allowed
        turn.spin_handle.cancel_goal_async.assert_called_once()
        zero = turn.node.cmd_vel_callback.call_args.args[0]
        assert zero.linear.x == zero.angular.z == 0.
    else:
        assert turn.phase == 'turn' and turn.drive_allowed
        turn.spin_handle.cancel_goal_async.assert_not_called()


def test_precision_relaxation_retains_owned_turn_timeout_and_actual_zero():
    turn = executing(runtime())
    turn.started = time.monotonic() - 20.001
    turn.tick()
    assert turn.reason == 'undock turn timed out'
    assert not turn.drive_allowed
    turn.spin_handle.cancel_goal_async.assert_called_once()
    zero = turn.node.cmd_vel_callback.call_args.args[0]
    assert zero.linear.x == zero.angular.z == 0.


def test_measured_pi_stops_immediately_and_cancels_only_the_owned_spin():
    turn = executing(runtime())
    turn.radians = math.pi - .02
    turn.node._pth = .03
    turn.node._pose_at += .05
    turn.observe_turn_odom()
    assert turn.target_reached and turn.phase == 'stopping'
    assert not turn.drive_allowed and turn.reason == ''
    turn.spin_handle.cancel_goal_async.assert_called_once()
    zero = turn.node.cmd_vel_callback.call_args.args[0]
    assert zero.linear.x == zero.angular.z == 0.


def test_global_reply_age_is_measured_from_request_not_callback_receipt():
    turn = executing(runtime())
    turn.global_probes = turn.global_queries()
    turn.global_requested_at = time.monotonic() - .501
    turn.tick()
    assert not turn.drive_allowed
    assert 'stale or changed' in turn.reason


def test_dropped_read_only_idle_probe_expires_and_healthy_reply_can_admit():
    turn = runtime()
    lost = Future(done=False)
    healthy = Future(NS(success=True))
    turn.idle_at = None
    turn.idle_future = lost
    turn.idle_requested_at = time.monotonic() - .501
    turn.idle_client = NS(service_is_ready=lambda: True,
                          remove_pending_request=Mock(),
                          call_async=Mock(return_value=healthy))
    turn.tick()
    turn.idle_client.remove_pending_request.assert_called_once_with(lost)
    lost.cancel.assert_called_once()
    assert turn.idle_future is healthy and turn.idle_at is None
    turn.tick()
    assert turn.reserve_reverse()


def test_every_global_navigation_node_must_stay_inactive():
    turn = executing(runtime())
    turn.states['planner_server'].call_async = lambda _: Future(NS(current_state=NS(id=3)))
    turn.tick()
    turn.tick()
    assert not turn.drive_allowed
    assert 'stale or changed' in turn.reason


def test_manual_centered_sticks_revoke_once_and_send_immediate_real_zero():
    turn = executing(runtime())
    assert turn.drive_allowed
    turn.manual(Twist())
    turn.manual(Twist())
    assert not turn.drive_allowed and turn.token is None
    assert turn.spin_handle.cancel_goal_async.call_count == 1
    zero = turn.node.cmd_vel_callback.call_args.args[0]
    assert pure_angular(zero) and zero.angular.z == 0.
    assert turn.final_pub.publish.call_args.args[0].angular.z == 0.
    assert turn.docking_pub.publish.call_args.args[0].angular.z == 0.


def test_unrelated_action_uuid_releases_no_turn_permission():
    turn = executing(runtime())
    status = GoalStatus()
    status.status = 2
    status.goal_info.goal_id = UUID(uuid=[55] * 16)
    turn.status('spin', GoalStatusArray(status_list=[status]))
    assert not turn.drive_allowed
    assert turn.reason == 'unrelated active action: spin'


def test_old_acceptance_callback_cannot_rebind_a_new_lease():
    turn = runtime()
    assert turn.reserve_reverse()
    old = NS(accepted=True, goal_id=UUID(uuid=list(GOAL_ID)),
             cancel_goal_async=Mock())
    turn.accepted_spin(Future(old), expected_id=GOAL_ID)
    old.cancel_goal_async.assert_called_once()
    assert turn.spin_handle is None
    assert turn.phase == 'reverse' and turn.reason == ''


def test_old_reverse_callbacks_cannot_mutate_a_new_reservation():
    turn = runtime()
    old_goal = handoff(turn)
    assert turn.finish()
    turn.node._dock_state, turn.node._motors_enabled = 'DOCKED_CONFIRMED', False
    assert turn.reserve_reverse()
    turn.reverse_unknown(old_goal, 'old terminal lost')
    turn.reverse_result(old_goal, NS(success=True))
    assert not turn.unknown and turn.reason == ''
    assert turn.phase == 'reverse' and turn.hardware_goal_id is None
    assert turn.token is None


@pytest.mark.parametrize('change', ['session', 'origin', 'disable', 'recovery'])
def test_handoff_token_cannot_survive_generation_changes(change):
    turn = runtime()
    handoff(turn)
    if change == 'session':
        turn.node._session_generation += 1
    elif change == 'origin':
        turn.epoch.guard.epoch = 'aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee'
    elif change == 'disable':
        turn.node._motor_disable_generation += 1
    else:
        turn.epoch.recovery.observe_clock(100)
        turn.epoch.recovery.observe_clock(90)
    assert turn.accept(request()).name == 'REJECT'


def test_unknown_late_transition_preserves_hold_and_prevents_restoration_race():
    turn = executing(runtime())
    pending = Future(done=False)
    with pytest.raises(RuntimeError, match='timed out'):
        turn.wait(pending, 0., mutation=True)
    assert turn.unknown and turn.pending is pending
    assert not turn.finish()
    assert turn.epoch.undock_hold
    assert not EpochRuntime.commands_allowed.fget(turn.epoch)
    assert not any(client.call_async.called for client in turn.changes.values())


def test_confirmed_restoration_deactivates_only_borrowed_subset():
    turn = executing(runtime())
    states = {name: 3 if name in LOCAL_NODES else 2
              for name in (*NAVIGATION_NODES, LOCAL_NODES[0])}
    transitions = []
    for name in states:
        turn.states[name].call_async = lambda _, name=name: Future(NS(current_state=NS(id=states[name])))
    for name in LOCAL_NODES:
        def transition(request, name=name):
            transitions.append((name, request.transition.id))
            states[name] = 2
            return Future(NS(success=True))
        turn.changes[name].call_async.side_effect = transition
    assert turn.finish()
    assert transitions == [(name, 4) for name in reversed(LOCAL_NODES)]
    assert turn.phase == 'idle' and not turn.epoch.undock_hold
    assert not turn.drive_allowed
    assert states['bt_navigator'] == 2
    assert all(states[name] == 2 for name in NAVIGATION_NODES)


@pytest.mark.parametrize('delta,late_fault', [(-.4, False), (.121, False), (.35, False),
                                            (.35, True)])
def test_final_heading_difference_does_not_abort_but_late_safety_fault_does(delta, late_fault):
    turn = runtime()
    handoff(turn)
    assert turn.accept(request()).name == 'ACCEPT'
    turn.transition = Mock()
    turn.inactive = Mock()
    goal = NS(is_cancel_requested=False, publish_feedback=Mock(), succeed=Mock(),
              abort=Mock(), canceled=Mock())
    wrapped = NS(status=4, result=NS(error_code=0))
    def spin_send(_goal, goal_uuid):
        turn.radians = math.pi
        handle = NS(accepted=True, goal_id=goal_uuid,
                    get_result_async=lambda: Future(wrapped), cancel_goal_async=Mock())
        future = Future(handle)
        future.add_done_callback = lambda cb: cb(future)
        return future
    turn.spin = NS(wait_for_server=lambda **_: True, send_goal_async=spin_send)
    def restore():
        # A new odometry sample arrives while the borrowed nodes deactivate.
        turn.node._pth = delta
        turn.node._pose_at += .05
        turn.observe_turn_odom()
        if late_fault:
            turn.revoke('E-stop or hardware fault')
        return True
    turn.finish = restore
    with patch('patrolbot_bridge.undock_turn.time.sleep', return_value=None):
        result = turn.execute(goal)
    assert result.radians_traveled == pytest.approx(math.pi + delta)
    assert result.success is (not late_fault)
    assert goal.succeed.called is (not late_fault)
    assert goal.abort.called is late_fault
    if late_fault:
        assert result.code == result.MOTION_FAILED
        assert 'E-stop or hardware fault' in result.message
    else:
        assert result.code == result.OK
    zero = turn.node.cmd_vel_callback.call_args.args[0]
    assert zero.linear.x == zero.angular.z == 0.


def test_pending_mutation_exception_is_an_unknown_outcome_with_hold():
    turn = executing(runtime())
    class BrokenFuture(Future):
        def result(self):
            raise RuntimeError('transport failed after request')
    pending = BrokenFuture()
    with pytest.raises(RuntimeError, match='transport failed'):
        turn.wait(pending, 1., mutation=True)
    assert turn.unknown and turn.pending is pending
    assert not turn.finish() and turn.epoch.undock_hold


@pytest.mark.parametrize('status,error', [(6, 0), (4, 1)])
def test_failed_spin_terminal_stops_and_does_not_start_a_blind_retry(status, error):
    turn = runtime()
    handoff(turn)
    assert turn.accept(request()).name == 'ACCEPT'
    turn.transition = Mock(); turn.inactive = Mock(); turn.finish = Mock(return_value=True)
    goal = NS(is_cancel_requested=False, publish_feedback=Mock(), succeed=Mock(),
              abort=Mock(), canceled=Mock())
    wrapped = NS(status=status, result=NS(error_code=error))
    def spin_send(_goal, goal_uuid):
        handle = NS(accepted=True, goal_id=goal_uuid,
                    get_result_async=lambda: Future(wrapped), cancel_goal_async=Mock())
        future = Future(handle)
        future.add_done_callback = lambda cb: cb(future)
        return future
    send = Mock(side_effect=spin_send)
    turn.spin = NS(wait_for_server=lambda **_: True, send_goal_async=send)
    result = turn.execute(goal)
    assert not result.success and 'Spin failed' in result.message
    send.assert_called_once()
    goal.succeed.assert_not_called(); goal.abort.assert_called_once()
    zero = turn.node.cmd_vel_callback.call_args.args[0]
    assert zero.linear.x == zero.angular.z == 0.


@pytest.mark.parametrize('operator_cancel', [False, True])
def test_only_own_measured_target_cancel_can_complete_successfully(operator_cancel):
    turn = runtime()
    handoff(turn)
    assert turn.accept(request()).name == 'ACCEPT'
    turn.transition = Mock()
    turn.inactive = Mock()
    turn.finish = Mock(return_value=True)
    goal = NS(is_cancel_requested=False, publish_feedback=Mock(), succeed=Mock(),
              abort=Mock(), canceled=Mock())
    wrapped = NS(status=5, result=NS(error_code=0))
    def spin_send(_goal, goal_uuid):
        turn.radians = math.pi
        turn.target_reached = True
        if operator_cancel:
            goal.is_cancel_requested = True
            turn.cancel(goal)
        handle = NS(accepted=True, goal_id=goal_uuid,
                    get_result_async=lambda: Future(wrapped), cancel_goal_async=Mock())
        future = Future(handle)
        future.add_done_callback = lambda cb: cb(future)
        return future
    turn.spin = NS(wait_for_server=lambda **_: True, send_goal_async=spin_send)
    with patch('patrolbot_bridge.undock_turn.time.sleep', return_value=None):
        result = turn.execute(goal)
    assert result.success is (not operator_cancel)
    assert goal.succeed.called is (not operator_cancel)
    assert goal.canceled.called is operator_cancel
