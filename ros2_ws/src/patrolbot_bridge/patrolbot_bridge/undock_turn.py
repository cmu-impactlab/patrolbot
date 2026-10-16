"""One-shot, odometry-based undock turn through the existing Nav2 safety chain."""

from functools import partial
import math
import time
from uuid import uuid4

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import Spin
from nav2_msgs.srv import ManageLifecycleNodes
from patrolbot_interfaces.action import UndockTurn
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.qos import qos_profile_action_status_default
from std_srvs.srv import Trigger
from unique_identifier_msgs.msg import UUID

from .epoch_recovery import EpochRecovery


NAVIGATION_NODES = (
    'controller_server', 'smoother_server', 'planner_server', 'route_server',
    'behavior_server', 'velocity_smoother', 'collision_monitor',
    'bt_navigator', 'waypoint_follower')
LOCAL_NODES = ('local_costmap/local_costmap', 'behavior_server',
               'velocity_smoother', 'collision_monitor')
ACTION_NAMES = ('navigate_to_pose', 'navigate_through_poses', 'follow_waypoints',
                'follow_path', 'spin', 'wait', 'drive_on_heading',
                'assisted_teleop', 'backup')
PATH_PUBLISHERS = {
    '/input/nav_controller': 'controller_server',
    '/input/nav_behavior': 'behavior_server',
    '/input/docking': 'patrolbot_bridge',
    '/cmd_vel_nav': 'nav_command_mux',
    '/cmd_vel_smoothed': 'velocity_smoother',
    '/input/navi': 'collision_monitor',
    '/input/undock_turn': 'patrolbot_bridge',
    '/cmd_vel_out': 'cmd_vel_mux',
    '/cmd_vel': 'teleop_velocity_smoother',
}


def pure_angular(message):
    values = (message.linear.x, message.linear.y, message.linear.z,
              message.angular.x, message.angular.y, message.angular.z)
    return (all(math.isfinite(v) for v in values)
            and all(v == 0. for v in values[:5])
            and 0. <= message.angular.z <= .7 + 1e-6)


def exclusive_publishers(node):
    """Count endpoints too: two publishers with the same node name are foreign."""
    for topic, name in PATH_PUBLISHERS.items():
        infos = node.get_publishers_info_by_topic(topic)
        count = 4 if topic == '/input/nav_behavior' else 1
        if (len(infos) != count or any(
                info.node_name != name or info.node_namespace != '/'
                or info.topic_type != 'geometry_msgs/msg/Twist' for info in infos)):
            return False
    return True


class UndockTurnRuntime:
    def __init__(self, node):
        self.node = node
        self.epoch = node._epoch
        self.phase = 'idle'
        self.reason = ''
        self.token = None
        self.hardware_goal_id = None
        self.spin_id = None
        self.spin_handle = None
        self.spin_active = False
        self.pending = None
        self.unknown = False
        self.started = None
        self.bound = None
        self.radians = 0.
        self.target_yaw = math.pi
        self.target_reached = False
        self.last_yaw = None
        self.last_odom_at = None
        self.behavior_at = self.collision_at = None
        self.graph_valid = False
        self.global_checked_at = None
        self.global_probes = {}
        self.global_requested_at = None
        self.idle_at = None
        self.idle_future = None
        self.idle_requested_at = None
        self.actions = {name: False for name in ACTION_NAMES}
        self.hardware_cancel_sent = False
        self.spin_cancel_sent = False
        self.validation_mode = False
        self.hardware_operation_owned = False
        self.finishing = False
        self.docking_pub = node.create_publisher(Twist, '/input/docking', 10)
        self.final_pub = node.create_publisher(Twist, '/input/undock_turn', 10)
        node.create_subscription(Twist, '/input/nav_behavior', self.behavior, 10,
                                 callback_group=node._callbacks)
        node.create_subscription(Twist, '/input/navi', self.collision, 10,
                                 callback_group=node._callbacks)
        for name in ('/input/joy', '/input/teleop', '/input/switch'):
            node.create_subscription(Twist, name, self.manual, 10,
                                     callback_group=node._callbacks)
        node.create_subscription(Twist, '/input/nav_controller', self.controller,
                                 10, callback_group=node._callbacks)
        self.status_subs = [node.create_subscription(
            GoalStatusArray, '/' + name + '/_action/status',
            partial(self.status, name), qos_profile_action_status_default,
            callback_group=node._callbacks) for name in ACTION_NAMES]
        self.idle_client = node.create_client(Trigger, '/bt_navigator/navigation_idle',
                                              callback_group=node._callbacks)
        self.manager_active = node.create_client(
            Trigger, '/lifecycle_manager_navigation/is_active',
            callback_group=node._callbacks)
        self.manager = node.create_client(
            ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes',
            callback_group=node._callbacks)
        self.states = {name: node.create_client(
            GetState, '/' + name + '/get_state', callback_group=node._callbacks)
            for name in (*NAVIGATION_NODES, LOCAL_NODES[0])}
        self.changes = {name: node.create_client(
            ChangeState, '/' + name + '/change_state', callback_group=node._callbacks)
            for name in LOCAL_NODES}
        self.spin = ActionClient(node, Spin, '/spin', callback_group=node._callbacks)
        self.server = ActionServer(
            node, UndockTurn, '/patrolbot/undock_turn',
            goal_callback=self.accept, cancel_callback=self.cancel,
            execute_callback=self.execute, callback_group=node._callbacks)
        node.create_timer(.05, self.tick, callback_group=node._callbacks)

    @property
    def busy(self):
        return self.phase != 'idle'

    def hardware_problem(self, *, charging=False, turn=False, stationary=False,
                         allow_uncommissioned=False):
        now = time.monotonic()
        if not self.epoch.guard.ready or not self.node._lidar_ready():
            return 'fresh origin and LiDAR required'
        with self.node._state_lock, self.node._pose_lock:
            ages = (now - self.node._charge_state_at,
                    now - self.node._flags_at, now - self.node._pose_at,
                    now - self.node._undock_scan_at)
            if (not self.node._link_connected or not self.node._pose_received
                    or not all(math.isfinite(v) and 0. <= v <= .5 for v in ages)
                    or not 0. <= now - self.node._dock_state_at <= 1.):
                return 'hardware or odometry is stale'
            if self.node._estop_pressed or self.node._fault_flags:
                return 'E-stop or hardware fault'
            if (not self.node._undock_profile_commissioned
                    and not (allow_uncommissioned or
                             (self.phase == 'reverse' and self.validation_mode))):
                return 'undock hardware profile is not commissioned'
            if not all(math.isfinite(v) for v in (
                    self.node._px, self.node._py, self.node._pth,
                    self.node._vx, self.node._vth)):
                return 'odometry is invalid'
            if stationary and (abs(self.node._vx) > .005
                               or abs(self.node._vth) > .01):
                return 'robot must be stationary'
            if charging and (self.node._charge_state <= 0
                             or self.node._dock_state != 'DOCKED_CONFIRMED'
                             or self.node._undock_active):
                return 'robot is not confirmed charging'
            if turn and (self.node._dock_state != 'CLEAR_CONFIRMED'
                         or self.node._undock_active
                         or not self.node._motors_enabled):
                return 'hardware handoff is not clear and live'
            if self.bound is not None and self.bound != (
                    self.node._session_generation, self.epoch.guard.epoch,
                    self.node._motor_disable_generation, self.epoch.recovery.token):
                return 'undock session or origin changed'
        return ''

    def spin_hold_problem(self):
        """While spinning, only E-stop or contact evidence may stop the turn.

        Adopted 2026-10-16 with the bridge-owned turn: per-bumper signals
        are unresolved on this platform (masks default to 0), so contact
        evidence is the firmware fault/stall flags plus the hardware motion
        cutoff behind them. Degraded warnings veto as well; narrow to
        mapped stall bits once #9 commissions them. Liveness supervision
        (publishers, stamps, cancel, timeouts) continues in tick() and
        drive_allowed(); the full hardware suite still applies outside
        the spin itself.
        """
        if self.node._estop_pressed:
            return 'E-stop asserted during undock turn'
        if self.node._fault_flags:
            return 'hardware fault during undock turn'
        return ''

    def reserve_reverse(self, validation_mode=False):
        with self.epoch.lock:
            if self.busy or self.hardware_problem(
                    charging=True, stationary=True,
                    allow_uncommissioned=validation_mode):
                return False
            if ((self.epoch._future is not None
                 and self.epoch._future_kind != 'read-only')
                    or self.epoch._map_future is not None
                    or self.epoch._map_goal is not None
                    or (self.epoch.maps and self.epoch.maps.hold)
                    or self.epoch._failed
                    or self.epoch.recovery.stage not in (
                        EpochRecovery.WAIT_RESET, EpochRecovery.WAIT_SEED,
                        EpochRecovery.READY)
                    or any(self.actions.values()) or self.idle_at is None
                    or not 0. <= time.monotonic() - self.idle_at <= .5):
                return False
            with self.node._control_lock:
                if self.node._pending_controls:
                    return False
            if not self.node._hardware_operation_lock.acquire(blocking=False):
                return False
            self.hardware_operation_owned = True
            if self.epoch._future is not None:
                client = (self.epoch._localization_active if
                          self.epoch._phase == 'wait-localization-manager'
                          else self.epoch._get)
                client.remove_pending_request(self.epoch._future)
                self.epoch._future.cancel()
                self.epoch._future = None
                self.epoch._future_kind = None
            self.phase = 'reverse'
            self.validation_mode = validation_mode
            self.reason = ''
            self.unknown = False
            self.pending = None
            self.hardware_goal_id = None
            self.spin_id = None
            self.spin_handle = None
            self.spin_active = False
            self.started = None
            self.target_yaw = math.pi
            self.target_reached = False
            self.behavior_at = self.collision_at = None
            self.global_probes = {}
            self.global_requested_at = self.global_checked_at = None
            self.bound = (self.node._session_generation, self.epoch.guard.epoch,
                          self.node._motor_disable_generation, self.epoch.recovery.token)
            self.epoch.undock_hold = True
            self.hardware_cancel_sent = False
            return True

    def wait(self, future, timeout, goal=None, *, mutation=False):
        if mutation:
            self.pending = future
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            if goal is not None and goal.is_cancel_requested:
                self.revoke('operator canceled undock turn')
            time.sleep(.02)
        if not future.done():
            if mutation:
                self.unknown = True
            raise RuntimeError('undock lifecycle/action response timed out')
        try:
            result = future.result()
        except Exception:
            if mutation:
                self.unknown = True
            raise
        if mutation:
            self.pending = None
        return result

    def state(self, name):
        client = self.states[name]
        if not client.service_is_ready():
            raise RuntimeError('lifecycle state unavailable: ' + name)
        result = self.wait(client.call_async(GetState.Request()), 2.)
        return result.current_state.id

    def inactive(self):
        active = self.wait(self.manager_active.call_async(Trigger.Request()), 2.)
        if active.success or any(self.state(name) != 2
                                 for name in (*NAVIGATION_NODES, LOCAL_NODES[0])):
            raise RuntimeError('navigation lifecycle is not confirmed inactive')
        self.global_checked_at = time.monotonic()

    def prepare_reverse(self, goal):
        with self.epoch.lock:
            if self.phase != 'reverse' or self.hardware_goal_id is not None:
                return 'hardware reverse ownership changed'
            self.hardware_goal_id = bytes(goal.goal_id.uuid)
        try:
            states = [self.state(name) for name in NAVIGATION_NODES]
            if all(state == 3 for state in states):
                request = ManageLifecycleNodes.Request()
                request.command = request.PAUSE
                response = self.wait(self.manager.call_async(request), 10., mutation=True)
                if not response.success:
                    raise RuntimeError('navigation pause refused')
            elif not all(state == 2 for state in states):
                raise RuntimeError('mixed navigation lifecycle state')
            self.inactive()
            if self.reason or goal.is_cancel_requested:
                raise RuntimeError(self.reason or 'operator canceled undock')
            problem = self.hardware_problem(charging=True, stationary=True)
            if problem:
                raise RuntimeError(problem)
            # Reverse changes the pose without requiring a map. Global goals
            # remain held until an operator supplies a new accepted seed.
            with self.epoch.lock:
                if self.epoch.recovery.stage != EpochRecovery.WAIT_RESET:
                    self.epoch.recovery.require_seed(self.node.get_clock().now().nanoseconds)
                self.bound = (*self.bound[:3], self.epoch.recovery.token)
                self.epoch._token = self.epoch.recovery.token
                self.epoch._navigation_ready = False
                self.epoch._accepted_seed_stamp = None
                self.epoch._pending_pose = None
                self.epoch._pose_anchor = None
            return ''
        except Exception as exc:
            self.revoke(str(exc))
            return self.reason

    def reverse_result(self, goal, reply):
        with self.epoch.lock:
            if (self.phase != 'reverse' or self.hardware_goal_id != bytes(goal.goal_id.uuid)
                    or self.bound is None):
                return
            if (reply.success and not self.reason and not goal.is_cancel_requested
                    and self.bound == (self.node._session_generation, self.epoch.guard.epoch,
                                       self.node._motor_disable_generation, self.epoch.recovery.token)
                    and reply.physically_clear and reply.dock_state == 'CLEAR_CONFIRMED'
                    and reply.motors_enabled):
                self.token = (self.hardware_goal_id, *self.bound, time.monotonic())
                self.phase = 'handoff'
                return
            self.revoke('hardware undock did not produce a clear handoff')
        self.finish()

    def reverse_unknown(self, goal, reason):
        with self.epoch.lock:
            if self.phase != 'reverse' or self.hardware_goal_id != bytes(goal.goal_id.uuid):
                return
            self.unknown = True
            self.revoke(reason)

    def accept(self, request):
        with self.epoch.lock:
            target = float(getattr(request, 'target_yaw', math.pi))
            if (self.phase != 'handoff' or self.token is None or self.reason
                    or bytes(request.hardware_goal_id.uuid) != self.token[0]
                    or request.session_generation != self.token[1]
                    or not 0. <= time.monotonic() - self.token[-1] <= 8.
                    or not 0.0 < target <= math.pi
                    or self.hardware_problem(turn=True, stationary=True)
                    or any(self.actions.values())):
                return GoalResponse.REJECT
            self.token = None
            self.target_yaw = target
            self.phase = 'turn-preparing'
            return GoalResponse.ACCEPT

    def cancel(self, _goal):
        self.revoke('operator canceled undock turn')
        return CancelResponse.ACCEPT

    def transition(self, name, transition, expected):
        request = ChangeState.Request()
        request.transition.id = transition
        response = self.wait(self.changes[name].call_async(request), 5., mutation=True)
        if not response.success or self.state(name) != expected:
            raise RuntimeError('local lifecycle transition refused: ' + name)

    def execute(self, goal):
        result = UndockTurn.Result()
        try:
            self.inactive()
            if not exclusive_publishers(self.node):
                raise RuntimeError('undock command publishers are not exclusive')
            for name in LOCAL_NODES:
                if self.reason or goal.is_cancel_requested:
                    raise RuntimeError(self.reason or 'operator canceled undock turn')
                self.transition(name, 3, 3)
            if not self.spin.wait_for_server(timeout_sec=2.):
                raise RuntimeError('Nav2 Spin action unavailable')
            requested_at = time.monotonic()
            probes = self.global_queries()
            self.check_global(probes, requested_at)
            self.global_checked_at = requested_at
            with self.epoch.lock, self.node._pose_lock:
                self.started = time.monotonic()
                self.last_yaw = self.node._pth
                self.last_odom_at = self.node._pose_at
                self.radians = 0.
                self.spin_cancel_sent = False
                self.spin_id = uuid4().bytes
                self.phase = 'turn'
            spin = Spin.Goal()
            spin.target_yaw = float(self.target_yaw)
            spin.time_allowance.sec = 20
            send = self.spin.send_goal_async(spin, goal_uuid=UUID(uuid=list(self.spin_id)))
            send.add_done_callback(partial(self.accepted_spin, expected_id=self.spin_id))
            handle = self.wait(send, 2., goal, mutation=True)
            if handle is None or not handle.accepted:
                raise RuntimeError('Nav2 Spin goal rejected')
            self.spin_handle = handle
            terminal = handle.get_result_async()
            deadline = self.started + 20.
            while not terminal.done() and time.monotonic() < deadline:
                if goal.is_cancel_requested:
                    self.revoke('operator canceled undock turn')
                if self.reason:
                    break
                feedback = UndockTurn.Feedback()
                feedback.radians_traveled = float(self.radians)
                goal.publish_feedback(feedback)
                time.sleep(.05)
            if not terminal.done():
                self.revoke(self.reason or 'undock turn timed out')
                if not self.spin_cancel_sent:
                    handle.cancel_goal_async()
                    self.spin_cancel_sent = True
                wrapped = self.wait(terminal, 2., mutation=True)
            else:
                wrapped = terminal.result()
            with self.epoch.lock:
                self.phase = 'stopping'
                self.spin_active = False
                self.zeros()
                self.node.cmd_vel_callback(Twist())
            own_target_cancel = bool(self.target_reached and not goal.is_cancel_requested
                                     and not self.reason and wrapped is not None
                                     and wrapped.status == 5)
            if (wrapped is None or (not own_target_cancel and (
                    wrapped.status != 4 or wrapped.result.error_code != Spin.Result.NONE))):
                raise RuntimeError(self.reason or 'collision-checked Spin failed')
            if self.reason:
                raise RuntimeError(self.reason)
            deadline = time.monotonic() + 1.5
            stationary_since = None
            while time.monotonic() < deadline:
                if not self.hardware_problem(turn=True, stationary=True):
                    stationary_since = stationary_since or time.monotonic()
                    if time.monotonic() - stationary_since >= .25:
                        break
                else:
                    stationary_since = None
                time.sleep(.02)
            else:
                raise RuntimeError('stationary clear handoff not confirmed after turn')
            with self.epoch.lock:
                self.observe_turn_odom()
                if self.reason:
                    raise RuntimeError(self.reason)
            result.success = True
            result.code = result.OK
            result.message = 'Undock turn complete'
        except Exception as exc:
            self.revoke(str(exc))
            result.success = False
            result.code = (result.CANCELED if goal.is_cancel_requested else
                           result.TIMEOUT if self.unknown else result.MOTION_FAILED)
            result.message = self.reason
        finally:
            restored = self.finish()
            if not restored:
                result.success = False
                result.code = result.SAFETY_FAULT
                result.message = (self.reason + '; local lifecycle restoration unconfirmed')
            elif result.success and self.reason:
                result.success = False
                result.code = result.MOTION_FAILED
                result.message = self.reason
            result.radians_traveled = float(self.radians)
        if result.success:
            goal.succeed()
        elif goal.is_cancel_requested:
            goal.canceled()
        else:
            goal.abort()
        return result

    def accepted_spin(self, future, expected_id):
        try:
            handle = future.result()
            if handle is not None and handle.accepted:
                with self.epoch.lock:
                    if (self.phase != 'turn' or expected_id != self.spin_id or
                            bytes(handle.goal_id.uuid) != expected_id):
                        handle.cancel_goal_async()
                        return
                    self.spin_handle = handle
                    if (self.reason or self.unknown) and not self.spin_cancel_sent:
                        handle.cancel_goal_async()
                        self.spin_cancel_sent = True
        except Exception:
            if expected_id == self.spin_id:
                self.revoke('Spin acceptance failed')

    def finish(self):
        with self.epoch.lock:
            self.zeros()
            if self.phase == 'idle':
                return True
            if self.unknown or self.finishing:
                return False
            self.finishing = True
            self.phase = 'restoring'
        try:
            for name in reversed(LOCAL_NODES):
                state = self.state(name)
                if state == 3:
                    self.transition(name, 4, 2)
                elif state != 2:
                    raise RuntimeError('unexpected local lifecycle state: ' + name)
            self.inactive()
        except Exception as exc:
            self.revoke(str(exc))
            self.phase = 'held'
            return False
        else:
            with self.epoch.lock:
                self.phase = 'idle'
                self.token = None
                self.bound = None
                self.spin_id = None
                self.spin_handle = None
                self.spin_active = False
                self.behavior_at = self.collision_at = None
                self.epoch.undock_hold = False
                self.global_probes = {}
                if self.hardware_operation_owned:
                    self.node._hardware_operation_lock.release()
                    self.hardware_operation_owned = False
                if self.epoch.recovery.stage == EpochRecovery.WAIT_RESET:
                    self.epoch.withdraw()
        finally:
            self.finishing = False
        return True

    def global_queries(self):
        queries = {name: self.states[name].call_async(GetState.Request())
                   for name in NAVIGATION_NODES if name not in LOCAL_NODES}
        queries['manager'] = self.manager_active.call_async(Trigger.Request())
        return queries

    def check_global(self, probes, requested_at):
        for name, future in probes.items():
            result = self.wait(future, max(0., .5 - (time.monotonic() - requested_at)))
            if (not 0. <= time.monotonic() - requested_at <= .5 or
                    (result.success if name == 'manager' else result.current_state.id != 2)):
                raise RuntimeError('global navigation inactive state is stale or changed')

    def observe_turn_odom(self):
        with self.node._pose_lock:
            if self.last_yaw is not None and self.last_odom_at != self.node._pose_at:
                delta = math.remainder(self.node._pth - self.last_yaw, 2. * math.pi)
                self.last_yaw = self.node._pth
                self.last_odom_at = self.node._pose_at
                self.radians += delta
                if self.phase == 'turn' and self.radians >= self.target_yaw:
                    # Stop on measured travel rather than letting pipeline
                    # latency extend an already-completed turn.
                    self.target_reached = True
                    self.phase = 'stopping'
                    self.spin_active = False
                    self.zeros()
                    self.node.cmd_vel_callback(Twist())
                    if self.spin_handle is not None and not self.spin_cancel_sent:
                        self.spin_handle.cancel_goal_async()
                        self.spin_cancel_sent = True

    def revoke(self, reason):
        with self.epoch.lock:
            if not self.busy:
                return
            self.reason = self.reason or reason
            self.token = None
            self.spin_active = False
            self.zeros()
            if self.spin_handle is not None and not self.spin_cancel_sent:
                self.spin_handle.cancel_goal_async()
                self.spin_cancel_sent = True
            if self.phase == 'reverse' and not self.hardware_cancel_sent:
                request_id = self.node._undock_request_id
                if request_id is not None:
                    self.hardware_cancel_sent = self.node._send_undock_cancel(request_id)
            self.node.cmd_vel_callback(Twist())

    def zeros(self):
        self.docking_pub.publish(Twist())
        self.final_pub.publish(Twist())

    @property
    def drive_allowed(self):
        now = time.monotonic()
        return bool(self.phase == 'turn' and self.spin_active and not self.reason
                    and self.graph_valid and not self.spin_hold_problem()
                    and self.global_checked_at is not None
                    and 0. <= now - self.global_checked_at <= .5
                    and self.started is not None and 0. <= now - self.started <= 20.
                    and all(stamp is not None and 0. <= now - stamp <= .25
                            for stamp in (self.behavior_at, self.collision_at)))

    def behavior(self, message):
        with self.epoch.lock:
            if self.phase != 'turn' or self.reason:
                return
            if not pure_angular(message):
                self.revoke('foreign or invalid behavior velocity')
            elif self.spin_active and self.graph_valid and exclusive_publishers(self.node):
                self.behavior_at = time.monotonic()
                self.docking_pub.publish(message)

    def collision(self, message):
        with self.epoch.lock:
            if self.phase != 'turn' or self.reason:
                return
            if not pure_angular(message):
                self.revoke('foreign or invalid collision output')
                return
            if not exclusive_publishers(self.node):
                self.revoke('collision output publishers changed')
                return
            self.collision_at = time.monotonic()
            self.final_pub.publish(message if self.drive_allowed else Twist())

    def manual(self, _message):
        self.revoke('manual command took ownership')

    def controller(self, message):
        if (message.linear.x != 0. or message.angular.z != 0.):
            self.revoke('unrelated controller command during undock')

    def status(self, name, message):
        with self.epoch.lock:
            active = [s for s in message.status_list if s.status in (1, 2, 3)]
            own = [s for s in active if name == 'spin'
                   and bytes(s.goal_info.goal_id.uuid) == self.spin_id]
            self.actions[name] = len(active) != len(own)
            if name == 'spin':
                self.spin_active = any(s.status == 2 for s in own)
            if self.busy and self.actions[name]:
                self.revoke('unrelated active action: ' + name)

    def tick(self):
        finish_handoff = False
        with self.epoch.lock:
            now = time.monotonic()
            if self.idle_future is not None and self.idle_future.done():
                try:
                    valid = (self.idle_future.result().success
                             and not any(self.actions.values())
                             and 0. <= now - self.idle_requested_at <= .5)
                    self.idle_at = self.idle_requested_at if valid else None
                except Exception:
                    self.idle_at = None
                self.idle_future = None
            elif (self.idle_future is not None
                  and now - self.idle_requested_at > .5):
                self.idle_client.remove_pending_request(self.idle_future)
                self.idle_future.cancel()
                self.idle_future = None
                self.idle_at = None
            if (self.idle_future is None and self.idle_client.service_is_ready()
                    and (self.idle_requested_at is None or now - self.idle_requested_at >= .1)):
                self.idle_requested_at = now
                self.idle_future = self.idle_client.call_async(Trigger.Request())
            if not self.busy:
                return
            self.graph_valid = exclusive_publishers(self.node)
            if self.phase == 'turn':
                # Authorized 2026-10-16: while spinning, only E-stop or
                # contact evidence may stop the turn. Preparation and
                # stopping keep the full hardware suite.
                problem = self.spin_hold_problem()
            else:
                problem = self.hardware_problem(turn=self.phase in ('turn-preparing', 'stopping'))
            if problem or not self.graph_valid:
                self.revoke(problem or 'undock command publisher changed')
            if self.phase == 'handoff' and (self.token is None or now - self.token[-1] > 8.):
                self.revoke('hardware handoff expired')
                finish_handoff = True
            if self.phase == 'turn' and self.started is not None:
                source_ages = [now - stamp for stamp in (self.behavior_at, self.collision_at)
                               if stamp is not None]
                if (any(not 0. <= age <= .25 for age in source_ages) or
                        (now - self.started > 1. and len(source_ages) < 2)):
                    self.revoke('undock velocity source expired')
                if not self.global_probes:
                    self.global_requested_at = now
                    self.global_probes = self.global_queries()
                elif all(f.done() for f in self.global_probes.values()):
                    try:
                        self.check_global(self.global_probes, self.global_requested_at)
                        self.global_checked_at = self.global_requested_at
                    except Exception as exc:
                        self.revoke(str(exc))
                    self.global_probes = {}
                if self.global_checked_at is None or now - self.global_checked_at > .5:
                    self.revoke('global navigation state is stale during undock')
                if now - self.started > 20.:
                    self.revoke('undock turn timed out')
            if self.phase in ('turn', 'stopping', 'restoring'):
                self.observe_turn_odom()
            if not self.drive_allowed:
                self.zeros()
        if finish_handoff:
            self.finish()

    def destroy(self):
        self.revoke('bridge stopped')
        self.server.destroy()
        self.spin.destroy()
