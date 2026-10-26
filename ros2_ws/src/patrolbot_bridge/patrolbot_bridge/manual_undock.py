"""Trusted local Start input for an SBC-owned, Nav2-independent undock.

No velocity publisher, pose injection, Nav2 service, or motor-enable request
exists here. The SBC owns release/reverse/turn and every hardware safety stop.
"""

import math
import threading
import time
from uuid import uuid4

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

from .control_protocol import COMMAND_MOTOR_DISABLE, format_request
from .local_source import LocalSource
from .manual_telemetry import ManualTelemetry
from .manual_undock_policy import StartGesture
from .undock_protocol import format_manual_undock_request, SESSION_CHANGED


class ManualUndockRuntime:
    """Share the existing exclusive hardware-operation owner and wire tracker."""

    def __init__(self, node):
        self.node = node
        self.gesture = StartGesture()
        self.raw = ManualTelemetry()
        self.joy_source = LocalSource('/joy', 'joy_node', 'sensor_msgs/msg/Joy', .4)
        self.web_source = LocalSource('/patrolbot/web_teleop_session_active',
                                      'patrolbot_web_bridge', 'std_msgs/msg/Bool', .5)
        self.busy = False
        self.unknown = False
        self.cancel_requested = False
        self.input_session = None
        self.web_session_active = False
        self.input_after_ns = 0
        self.thread = None
        self.publisher = node.create_publisher(Bool, '/patrolbot/manual_undock_active', 10)
        self.sensor_pub = node.create_publisher(Bool, '/patrolbot/manual_sensors_ready', 10)
        node.create_subscription(Joy, '/joy', self.joy, 10, callback_group=node._callbacks)
        node.create_subscription(Bool, '/patrolbot/web_teleop_session_active',
                                 self.web_session, 10, callback_group=node._callbacks)
        node.create_timer(.1, self.publish, callback_group=node._callbacks)

    def publish(self):
        self.publisher.publish(Bool(data=self.busy))
        self.sensor_pub.publish(Bool(data=self.sensors_ready()))

    def sensors_ready(self):
        n, e = self.node, self.node._epoch
        with e.lock:
            return bool(e.guard.ready and n._link_connected and n._lidar_ready()
                        and self.raw.ready(now=time.monotonic(), session=n._session_generation,
                                           epoch=e.guard.epoch))

    def input_is_new(self, info):
        """Reject pre-handoff deadman evidence after a completed operation."""
        if not self.input_after_ns:
            return True
        stamp = (info.get('source_timestamp') if isinstance(info, dict)
                 else getattr(info, 'source_timestamp', None))
        now = self.node.get_clock().now().nanoseconds
        return (type(stamp) is int and stamp > self.input_after_ns
                and 0 <= now - stamp <= 500_000_000)

    def observe(self, line, received, session):
        e = self.node._epoch
        if e.guard.ready and session == e.guard.session:
            self.raw.observe(line, received, now=time.monotonic(), session=session,
                             epoch=e.guard.epoch)

    def web_session(self, message, info):
        """Keep an active web session busy until an explicit local release."""
        with self.node._epoch.lock:
            if self.web_source.accepts(self.node, info):
                self.web_session_active = bool(message.data)

    def joy(self, message, info):
        """Only the sole local joy_node endpoint can initiate or cancel a job."""
        try:
            now_ns = self.node.get_clock().now().nanoseconds
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            with self.node._epoch.lock:
                if not self.joy_source.accepts(self.node, info):
                    self.gesture.reset()
                    return
                session = (self.node._session_generation, self.node._motor_disable_generation)
                if session != self.input_session:
                    self.input_session = session
                    self.gesture.reset()
                intent = self.gesture.update(message.axes, message.buttons,
                                             now=now_ns / 1e9, stamp=stamp_ns / 1e9,
                                             busy=self.busy)
                if intent == 'cancel' and self.busy:
                    self.cancel_requested = True
                elif intent == 'start':
                    self.start()
        except (AttributeError, TypeError, ValueError, OverflowError):
            self.gesture.reset()

    def admission_problem(self):
        n, e = self.node, self.node._epoch
        now = time.monotonic()
        if (self.busy or n._undock_turn.busy or e.undock_hold or e.tuning_hold
                or self.web_session_active):
            return 'another undock or tuning operation owns the robot'
        if n._stop_event.is_set() or not self.sensors_ready():
            return 'bridge stopping or LiDAR unavailable'
        if (e._failed or e._map_error or e._map_goal is not None or e._map_future is not None
                or (e._future is not None and e._future_kind != 'read-only')
                or (e.maps and e.maps.hold and e.maps.stage != 'failed')
                or e._navigation_status or any(n._undock_turn.actions.values())):
            return 'navigation or a hardware/map/lifecycle mutation is active or unknown'
        with n._state_lock, n._pose_lock:
            hello = n._server_hello
            if hello is None or 'MANUAL_UNDOCK_V1' not in hello.features:
                return 'SBC does not advertise MANUAL_UNDOCK_V1; matched update required'
            if (not n._link_connected or not e.guard.ready
                    or not n._undock_profile_commissioned
                    or n._dock_state != 'DOCKED_CONFIRMED' or n._charge_state <= 0
                    or n._estop_pressed or n._fault_flags or n._undock_active):
                return 'fresh, safe, stationary charging hardware required'
            sample = self.raw.sample
            ages = (now - n._flags_at, now - n._charge_state_at, now - n._dock_state_at)
            if (not all(math.isfinite(v) for v in sample.values())
                    or not all(0 <= age <= .5 for age in ages)
                    or abs(sample.linear_x) > .005 or abs(sample.angular_z) > .01):
                return 'fresh finite odometry and stationary hardware required'
        with n._control_lock:
            if n._pending_controls:
                return 'hardware control is pending'
        # Session-bound web/physical manual evidence must be released. Missing
        # Nav2 idle services are deliberately NOT a prerequisite for this mode.
        if (0 <= now - n._web_teleop_active_at <= .5
                or 0 <= now - n._manual_active_at <= .5):
            return 'release all manual driving input first'
        return ''

    def start(self):
        n, e = self.node, self.node._epoch
        # Caller holds epoch.lock; ordinary control admission uses that same lock.
        with n._undock_goal_lock:
            problem = self.admission_problem()
            if problem or n._undock_goal_reserved:
                n.get_logger().warning('Manual undock refused: ' + (problem or 'undock reserved'))
                return
            if not n._hardware_operation_lock.acquire(blocking=False):
                n.get_logger().warning('Manual undock refused: hardware operation pending')
                return
            self.busy, self.unknown, self.cancel_requested = True, False, False
            self.bound = (n._session_generation, e.guard.epoch, n._motor_disable_generation)
            n._undock_goal_reserved = True
            e.undock_hold = True
            # Preserve the actual origin and any pending AMCL reset. Never claim
            # AMCL reset succeeded merely to make this manual operation available.
            e._navigation_ready = False
            e._accepted_seed_stamp = e._pending_pose = e._pose_anchor = None
            if e.recovery.stage != e.recovery.WAIT_RESET:
                e.recovery.require_seed(n.get_clock().now().nanoseconds)
                e._token = e.recovery.token
            self.publish()
            self.thread = threading.Thread(target=self.run, name='manual-undock', daemon=True)
            self.thread.start()

    def emergency_disable(self):
        """One explicit disable, never a motor-enable or blind mutation retry."""
        n = self.node
        try:
            with n.sock_lock:
                if n.sock is not None:
                    n.sock.sendall(format_request(COMMAND_MOTOR_DISABLE, str(uuid4())))
        except (OSError, ValueError):
            pass

    def run(self):
        n, e = self.node, self.node._epoch
        request_id = str(uuid4())
        session, epoch, motor_generation = self.bound
        known_terminal = False
        try:
            with n._undock_lock:
                n._undock_request_id = request_id
                n._undock_result = n._undock_feedback = None
                n._undock_accepted = False
                n._undock_session = session
                n._undock_event.clear()
            with e.lock, n.sock_lock:
                if (n.sock is None or n._session_generation != session
                        or n._motor_disable_generation != motor_generation
                        or e.guard.epoch != epoch or not self.sensors_ready()
                        or n._estop_pressed or n._dock_state != 'DOCKED_CONFIRMED'
                        or n._charge_state <= 0 or e.tuning_hold or n._stop_event.is_set()):
                    raise RuntimeError('hardware ownership changed before dispatch')
                n.sock.sendall(format_manual_undock_request(request_id))
            n.get_logger().info('Manual undock dispatched once: ' + request_id)
            deadline = time.monotonic() + 60.
            cancel_sent = False
            phase = None
            while not n._undock_event.wait(.05):
                if (time.monotonic() >= deadline or n._stop_event.is_set()
                        or n._session_generation != session):
                    n._send_undock_cancel(request_id)
                    raise RuntimeError('manual undock terminal outcome unknown')
                if self.cancel_requested and not cancel_sent:
                    cancel_sent = True
                    if not n._send_undock_cancel(request_id):
                        raise RuntimeError('manual undock cancellation send failed')
                with n._undock_lock:
                    feedback = n._undock_feedback
                if feedback is not None and feedback.phase_name != phase:
                    phase = feedback.phase_name
                    n.get_logger().info('Manual undock: ' + phase)
            with n._undock_lock:
                result = n._undock_result
            if (result is None or result.code == SESSION_CHANGED
                    or n._session_generation != session):
                raise RuntimeError('manual undock hardware session/outcome unknown')
            # Terminal means the SBC has stopped its exclusive controller. Still
            # require fresh stationary telemetry before admitting manual escape.
            settled = None
            deadline = time.monotonic() + 3.
            while time.monotonic() < deadline:
                now = time.monotonic()
                with e.lock:
                    raw_ready = self.sensors_ready()
                    sample = self.raw.sample
                    with n._state_lock:
                        safe_stop = (
                            n._session_generation == session and n._link_connected
                            and not n._undock_active and raw_ready
                            and 0 <= now - n._flags_at <= .5
                            and abs(sample.linear_x) <= .005 and abs(sample.angular_z) <= .01)
                settled = (settled or now) if safe_stop else None
                if settled is not None and now - settled >= .25:
                    known_terminal = True
                    break
                time.sleep(.02)
            if not known_terminal:
                raise RuntimeError('stationary terminal handoff unconfirmed')
            n.get_logger().info(
                f'Manual undock terminal: {result.message}; '
                f'motors={result.motors_enabled}; dock={result.dock_state}')
        except Exception as exc:
            self.unknown = True
            self.emergency_disable()
            n.get_logger().error(f'Manual undock held: {exc}; explicit recovery required')
        finally:
            with e.lock:
                if known_terminal and not self.unknown:
                    with n._undock_lock:
                        n._undock_request_id = None
                        n._undock_feedback = None
                    with n._undock_goal_lock:
                        n._undock_goal_reserved = False
                    # Inputs generated before completion never re-arm manual
                    # bypass. The joystick also requires a fresh released report.
                    n._manual_active_at = n._web_teleop_active_at = 0.
                    self.input_after_ns = n.get_clock().now().nanoseconds
                    e.undock_hold = False
                    self.busy = False
                    n._hardware_operation_lock.release()
                # Unknown outcomes retain their owner/hold; never silently retry.
                self.publish()

    def destroy(self):
        if self.busy:
            self.emergency_disable()
        if self.thread is not None:
            self.thread.join(timeout=2.)
