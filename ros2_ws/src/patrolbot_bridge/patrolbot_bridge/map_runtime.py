"""Map action adapter for the existing EpochRuntime lifecycle/motion owner."""
import math
import os
import time
from pathlib import Path

from action_msgs.msg import GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.srv import GetMap
from nav2_msgs.srv import LoadMap, ManageLifecycleNodes, ClearEntireCostmap
from patrolbot_interfaces.action import LocalizeOnMap
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_srvs.srv import Trigger, Empty

from .epoch_recovery import EpochRecovery
from .map_catalog import ActiveMapStore, MapCatalog
from .map_transaction import MapTransaction


def map_identity(grid):
    stamp = grid.info.map_load_time
    return f'{stamp.sec}:{stamp.nanosec}:{grid.info.width}:{grid.info.height}'


class MapRuntimeMixin:
    def init_maps(self):
        self._map_error = ''
        self.maps = None
        self._map_goal = None
        self._map_future = None
        self._map_step = None
        self._map_identity = None
        self._map_deadline = None
        self.map_activated_stamp_ns = 0
        self._navigation_status = False
        self._navigation_states = {}
        self._navigation_idle_at = None
        self._navigation_probe = None
        self._navigation_probe_at = 0.
        self._navigation_idle = self.node.create_client(Trigger, '/bt_navigator/navigation_idle')
        try:
            catalog = MapCatalog(Path(get_package_share_directory('patrolbot_navigation')) / 'maps')
            self._map_store = ActiveMapStore(os.environ.get('PATROLBOT_STATE_DIR', '/var/lib/patrolbot'), catalog)
            self._startup_map = self._map_store.load()
            self.maps = MapTransaction(catalog)
        except Exception as exc:
            self._map_error = str(exc)
            self.node.get_logger().error('Map state unavailable; motion held: ' + str(exc))
        self._load_map = self.node.create_client(LoadMap, '/map_server/load_map')
        self._get_map = self.node.create_client(GetMap, '/map_server/map')
        self._amcl_map = self.node.create_client(Trigger, '/amcl/map_consumed')
        self._global_map = self.node.create_client(Trigger, '/global_costmap/global_costmap/static_layer/map_consumed')
        self._clear_global = self.node.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self._refresh_localization = self.node.create_client(Empty, '/request_nomotion_update')
        self._clear_local = self.node.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self._map_seed_pub = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self._nav_status_subs = [self.node.create_subscription(
            GoalStatusArray, '/' + name + '/_action/status',
            lambda msg, name=name: self._map_nav_status(msg, name), qos)
            for name in ('navigate_to_pose', 'navigate_through_poses', 'patrolbot/undock')]

        self._map_server = ActionServer(self.node, LocalizeOnMap, '/patrolbot/localize_on_map',
            goal_callback=self._map_accept, cancel_callback=self._map_cancel,
            execute_callback=self._map_execute, callback_group=self.node._callbacks)

    def _map_nav_status(self, msg, name="navigate_to_pose"):
        with self.lock:
            self._navigation_states[name] = any(s.status in (1, 2, 3) for s in msg.status_list)
            self._navigation_status = any(self._navigation_states.values())
            if self._navigation_status and self.maps and self.maps.hold:
                self.maps.fail('navigation requested during map recovery')

    def poll_navigation_idle(self):
        now = time.monotonic()
        if self._navigation_probe is not None:
            if self._navigation_probe.done():
                try:
                    valid = (self._navigation_probe.result().success and
                             not self._navigation_status and
                             0 <= now - self._navigation_probe_at <= .5)
                except Exception:
                    valid = False
                self._navigation_idle_at = self._navigation_probe_at if valid else None
                self._navigation_probe = None
            elif now - self._navigation_probe_at > .5:
                self._navigation_idle.remove_pending_request(self._navigation_probe)
                self._navigation_probe.cancel()
                self._navigation_probe = None
                self._navigation_idle_at = None
        if (self._navigation_probe is None and now-self._navigation_probe_at >= .1
                and self._navigation_idle.service_is_ready()):
            self._navigation_probe_at = now
            self._navigation_probe = self._navigation_idle.call_async(Trigger.Request())

    @property
    def map_hold(self):
        return bool(self._map_error or self.maps is None or self.maps.active is None or self.maps.hold)

    def _map_accept(self, request):
        with self.lock:
            if (self._map_error or self.maps is None or self._map_goal is not None
                    or self._future is not None or self._map_future is not None or not self.guard.ready
                    or self.recovery.stage not in (EpochRecovery.READY, EpochRecovery.WAIT_SEED)
                    or self._navigation_status or self._navigation_idle_at is None
                    or not 0 <= time.monotonic() - self._navigation_idle_at <= .5):
                return GoalResponse.REJECT
            pose = request.pose.pose
            q = pose.orientation
            values = (pose.position.x, pose.position.y, pose.position.z, q.x, q.y, q.z, q.w,
                      *request.pose.covariance)
            if (not all(math.isfinite(v) for v in values)
                    or abs(sum(v*v for v in (q.x,q.y,q.z,q.w)) - 1) > 1e-3
                    or any(request.pose.covariance[i] <= 0 for i in (0,7,35))):
                return GoalResponse.REJECT
            now = time.monotonic()
            with self.node._state_lock, self.node._pose_lock, self.node._control_lock, self.node._undock_goal_lock:
                age = max(now-self.node._pose_at, now-self.node._flags_at,
                          now-self.node._charge_state_at)
                try:
                    self.maps.begin(request.request_id, request.map_id, request.map_revision,
                        (pose.position.x, pose.position.y, 2*math.atan2(q.z,q.w)),
                        request.operator_authorized, self.recovery.token, self.guard.session,
                        age, self.node._vx, self.node._vth, self._navigation_status,
                        self.node._undock_active or self.node._undock_goal_reserved,
                        bool(self.node._pending_controls))
                except ValueError:
                    return GoalResponse.REJECT
            # The shared cmd_vel guard reads this hold under this same lock.
            self._navigation_ready = False
            self._accepted_seed_stamp = None
            self._pending_pose = None
            self._map_request = request
            self._map_deadline = now + 110.
            self._map_refresh_requested = False
            self._map_step = 'pause'
            self.node.cmd_vel_callback(Twist())
            self.maps.advance(self.maps.token, 'hold')
            return GoalResponse.ACCEPT

    def _map_cancel(self, goal):
        with self.lock:
            if self.maps and self.maps.token and self.maps.token.request_id == goal.request.request_id:
                self.maps.fail('operator canceled map localization')
        return CancelResponse.ACCEPT

    def _map_execute(self, goal):
        with self.lock:
            self._map_goal = goal
        while self.node.context.ok() and not self.node._stop_event.is_set():
            with self.lock:
                tx = self.maps
                if time.monotonic() > self._map_deadline:
                    tx.fail('map localization timed out')
                feedback = LocalizeOnMap.Feedback()
                feedback.request_id = goal.request.request_id
                feedback.stage = tx.stage
                feedback.map_activated = tx.activated
                feedback.active_map_id = tx.active['map_id'] if tx.active else ''
                feedback.active_map_revision = tx.active['revision'] if tx.active else ''
                goal.publish_feedback(feedback)
                if tx.stage in ('ready', 'failed'):
                    result = LocalizeOnMap.Result()
                    result.request_id = goal.request.request_id
                    result.code = (result.OK if tx.stage == 'ready' else
                                   result.CANCELED if goal.is_cancel_requested else
                                   result.TIMEOUT if tx.error == 'map localization timed out' else result.FAILED)
                    result.message = tx.error or 'Map activated and localization verified'
                    result.map_activated = tx.activated
                    result.localized = tx.stage == 'ready'
                    result.active_map_id = feedback.active_map_id
                    result.active_map_revision = feedback.active_map_revision
                    if goal.is_cancel_requested: goal.canceled()
                    elif tx.stage == 'ready': goal.succeed()
                    else: goal.abort()
                    self._map_goal = None
                    return result
            time.sleep(.1)
        with self.lock:
            self.maps.fail('bridge shutdown')
        goal.abort()
        result = LocalizeOnMap.Result(); result.code = result.FAILED
        result.request_id = goal.request.request_id
        result.map_activated = self.maps.activated
        result.active_map_id = self.maps.active['map_id'] if self.maps.active else ''
        result.active_map_revision = self.maps.active['revision'] if self.maps.active else ''
        result.message = 'bridge shutdown; motion hold retained'
        return result

    def _map_call(self, client, request, next_step, check):
        if self._map_future is None:
            if not client.service_is_ready(): return False
            self._map_future = client.call_async(request)
            return False
        if not self._map_future.done(): return False
        future, self._map_future = self._map_future, None
        result = future.result()
        if not check(result): raise RuntimeError('map stage readback failed: ' + self._map_step)
        self._map_step = next_step
        return True

    def tick_map(self):
        """Return True while this transaction owns all lifecycle clients.

        A pending timed-out mutation is never canceled/retried. Failure keeps
        this owner and its hold until an explicit operator stack recovery.
        """
        tx = self.maps
        if tx is None or tx.stage in ('idle', 'ready'): return False
        if tx.stage == 'failed': return True
        if (not self.guard.ready or self.guard.session != tx.token.session
                or self.recovery.token != tx.token.origin):
            tx.fail('origin/transport changed during map recovery'); return True
        if time.monotonic() > self._map_deadline:
            tx.fail('map localization timed out'); return True
        now = time.monotonic()
        with self.node._state_lock, self.node._pose_lock:
            if (not self.node._link_connected or self.node._fault_flags or
                    self.node._undock_active or
                    any(not 0 <= now-stamp <= .5 for stamp in (
                        self.node._pose_at, self.node._flags_at, self.node._charge_state_at)) or
                    not math.isfinite(self.node._vx) or not math.isfinite(self.node._vth) or
                    abs(self.node._vx) > .005 or abs(self.node._vth) > .01):
                tx.fail('fresh stationary base telemetry lost during map recovery')
                return True
        try:
            self._tick_map_stage()
        except Exception as exc:
            tx.fail(str(exc))
        return True

    def _tick_map_stage(self):
        tx = self.maps; token = tx.token; step = self._map_step
        ok = lambda r: r.success is True
        if step in ('pause', 'resume'):
            request = ManageLifecycleNodes.Request()
            request.command = request.PAUSE if step == 'pause' else request.RESUME
            if self._map_call(self._navigation, request, 'inspect' if step == 'pause' else 'clear-global', ok):
                if step == 'pause': tx.advance(token, 'pause-navigation')
        elif step == 'inspect':
            def inspected(r):
                self._map_reset_step = {1:'load', 2:'cleanup', 3:'deactivate'}[r.current_state.id]
                return True
            if self._map_call(self._get, GetState.Request(), 'reset', inspected):
                self._map_step = self._map_reset_step
        elif step in ('deactivate','cleanup','configure','activate'):
            request = ChangeState.Request(); request.transition.id = {'deactivate':4,'cleanup':2,'configure':1,'activate':3}[step]
            next_step = {'deactivate':'cleanup','cleanup':'load','configure':'activate','activate':'readback'}[step]
            if self._map_call(self._change, request, next_step, ok) and step == 'activate':
                # Keep the same origin generation; only localization evidence is reset.
                self.recovery.stage = EpochRecovery.WAIT_RESET
                self.recovery.reset_complete(self.recovery.token, self.node.get_clock().now().nanoseconds)
                self.withdraw(reset_odometry=False)
        elif step == 'load':
            if tx.stage == 'reset-amcl': tx.advance(token, 'reset-amcl')
            request = LoadMap.Request(); request.map_url = tx.target['path']
            def loaded(r):
                if r.result != 0: return False
                self._map_identity = map_identity(r.map)
                return self._map_geometry(r.map, tx.target)
            if self._map_call(self._load_map, request, 'parameters', loaded):
                tx.advance(token, 'load-map')
        elif step == 'parameters':
            from rcl_interfaces.srv import SetParametersAtomically
            from rclpy.parameter import Parameter
            request = SetParametersAtomically.Request()
            request.parameters = [Parameter('set_initial_pose', value=False).to_parameter_msg(),
                Parameter('always_reset_initial_pose', value=True).to_parameter_msg()]
            self._map_call(self._parameters, request, 'configure', lambda r:r.result.successful)
        elif step == 'readback':
            self._map_call(self._get_map, GetMap.Request(), 'amcl-map',
                lambda r:map_identity(r.map) == self._map_identity and self._map_geometry(r.map, tx.target))
        elif step in ('amcl-map', 'costmap-map'):
            client = self._amcl_map if step == 'amcl-map' else self._global_map
            if self._map_future is None:
                if client.service_is_ready(): self._map_future = client.call_async(Trigger.Request())
            elif self._map_future.done():
                r = self._map_future.result(); self._map_future = None
                if r.success and r.message == self._map_identity:
                    if step == 'amcl-map':
                        self.map_activated_stamp_ns = self.node.get_clock().now().nanoseconds
                        tx.advance(token, 'verify-map')
                        self._map_store.save(tx.target['map_id'], tx.target['revision'])
                        tx.advance(token, 'persist-map'); self._map_step = 'tf'
                    elif self._map_localization_fresh():
                        tx.advance(token, 'verify-costmaps')
                        self._navigation_ready = True
                    elif not self._map_refresh_requested:
                        # Nav2 activation/inflation can outlast pose freshness.
                        # Ask for one new scan update, never another pose seed.
                        self._map_refresh_requested = True
                        self._map_step = 'refresh-localization'
        elif step == 'refresh-localization':
            self._map_call(self._refresh_localization, Empty.Request(), 'costmap-map', lambda r: True)
        elif step == 'tf':
            if self.seed_tf_ready:
                tx.advance(token, 'wait-tf'); self._map_step = 'seed'
        elif step == 'seed':
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'; msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.pose = self._map_request.pose
            if tx.seeded(token, self.stamp_ns(msg)):
                self.seed(msg)
                self._map_seed_pub.publish(msg)
                self._map_step = 'localization'
        elif step == 'localization':
            if self._map_localization_fresh():
                tx.advance(token, 'wait-localization'); self._map_step = 'resume'
        elif step in ('clear-global','clear-local'):
            client = self._clear_global if step == 'clear-global' else self._clear_local
            if self._map_call(client, ClearEntireCostmap.Request(),
                    'clear-local' if step == 'clear-global' else 'costmap-map', lambda r:True):
                if step == 'clear-local': tx.advance(token, 'refresh-costmaps')

    def _map_localization_fresh(self):
        pose = self._pending_pose
        return (pose is not None and self._accepted_seed_stamp == self.maps.seed_stamp
                and self.recovery.stage == EpochRecovery.READY
                and self.stamp_ns(pose) > (self.maps.seed_stamp or 0)
                and 0 <= self.node.get_clock().now().nanoseconds-self.stamp_ns(pose) <= 1_000_000_000
                and all(math.isfinite(pose.pose.covariance[i]) and
                        0 < pose.pose.covariance[i] <= .25 for i in (0, 7, 35)))

    @staticmethod
    def _map_geometry(grid, entry):
        info=grid.info; origin=info.origin
        return (grid.header.frame_id == 'map' and info.width == entry['width']
            and info.height == entry['height'] and abs(info.resolution-entry['resolution']) < 1e-7
            and abs(origin.position.x-entry['origin'][0]) < 1e-7
            and abs(origin.position.y-entry['origin'][1]) < 1e-7
            and abs(origin.position.z) < 1e-7
            and abs(origin.orientation.x) < 1e-7
            and abs(origin.orientation.y) < 1e-7
            and abs(origin.orientation.z-math.sin(entry['origin'][2]/2)) < 1e-7
            and abs(origin.orientation.w-math.cos(entry['origin'][2]/2)) < 1e-7)

    def tick_startup_map(self):
        if (self.maps is None or self.maps.active is not None or self._map_error
                or self.recovery.stage not in (EpochRecovery.WAIT_SEED, EpochRecovery.READY)):
            return
        try:
            if self._map_step is None:
                self._map_step = 'startup-readback'
            if self._map_step == 'startup-readback':
                def readback(r):
                    if not self._map_geometry(r.map, self._startup_map): return False
                    self._map_identity = map_identity(r.map)
                    return True
                self._map_call(self._get_map, GetMap.Request(), 'startup-amcl', readback)
            elif self._map_step == 'startup-amcl':
                if self._map_future is None:
                    if self._amcl_map.service_is_ready():
                        self._map_future = self._amcl_map.call_async(Trigger.Request())
                elif self._map_future.done():
                    r = self._map_future.result(); self._map_future = None
                    if r.success and r.message == self._map_identity:
                        self.maps.active = self._startup_map
                        self.map_activated_stamp_ns = self.node.get_clock().now().nanoseconds
                        self._map_step = None
        except Exception as exc:
            self._map_error = str(exc)
