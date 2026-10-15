import math
import queue
import socket
import struct
import threading
import time
from uuid import uuid4

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from patrolbot_bridge.control_protocol import (
    COMMAND_CHARGE_RELEASE,
    COMMAND_DOCK_RESET,
    COMMAND_MOTOR_ENABLE, COMMAND_MOTOR_DISABLE,
    COMMAND_LIDAR_ON, COMMAND_LIDAR_OFF,
    ControlReply,
    format_request,
    motor_enable_allowed,
    parse_reply,
    valid_request_id,
)
from patrolbot_bridge.epoch_runtime import EpochRuntime
from patrolbot_bridge.manual_active import manual_drive_bypass
from patrolbot_bridge.lidar_state import LidarState
from patrolbot_bridge.odom_covariance import OdomCovariance
from patrolbot_bridge.odom_validation import OdomSample, OdomValidator
from patrolbot_bridge.scan_timing import parse_laser_age, compute_scan_stamp_ns
from rclpy.time import Time
from patrolbot_bridge.safety_telemetry import (
    battery_diagnostic,
    BATTERY_MIN_OPERATING_VOLTS,
    BATTERY_RECHARGE_VOLTS,
    battery_supply_status,
    decode_fault_flags,
)
from patrolbot_bridge.undock_protocol import (
    CANCELED,
    format_undock_cancel,
    format_undock_request,
    LINK_UNAVAILABLE,
    parse_hello,
    parse_undock_event,
    parse_undock_result,
    SESSION_CHANGED,
    TIMEOUT,
    UndockResult,
)
from patrolbot_bridge.undock_turn import UndockTurnRuntime, pure_angular
from patrolbot_interfaces.action import HardwareUndock
from patrolbot_interfaces.msg import BaseState
from patrolbot_interfaces.srv import ChargeRelease, DockReset, MotorEnable, MotorDisable, SetLidarPower
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


class PendingControl:
    """Thread-safe rendezvous between a ROS service and the TCP reader."""

    def __init__(self, command, request_id, session_generation):
        self.command = command
        self.request_id = request_id
        self.session_generation = session_generation
        self.accepted = False
        self.reply = None
        self.event = threading.Event()


def get_quaternion_from_euler(yaw):
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


class PatrolBotBridge(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__(
            'patrolbot_bridge', parameter_overrides=parameter_overrides)

        # Candidate near-field topic for a controlled sensor study. It is
        # disabled by default and deliberately not consumed by Nav2 or Collision
        # Monitor until self-occlusion and stopping behavior are measured.
        self.declare_parameter('publish_unfiltered_scan_candidate', False)
        self.declare_parameter(
            'unfiltered_scan_topic', '/scan_unfiltered_candidate')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        # LaserScan is high-rate, lossy sensor data. BEST_EFFORT prefers a fresh
        # sample over retransmitting an old one to a stalled DDS reader.
        self.scan_pub = self.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self.unfiltered_scan_pub = None
        self.unfiltered_scan_topic = str(
            self.get_parameter('unfiltered_scan_topic').value)
        if bool(self.get_parameter(
                'publish_unfiltered_scan_candidate').value):
            self.unfiltered_scan_pub = self.create_publisher(
                LaserScan, self.unfiltered_scan_topic,
                qos_profile_sensor_data)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        # Deadman evidence from the joystick teleop (see manual_active).
        self._manual_active_at = 0.0
        self.manual_sub = self.create_subscription(
            Bool, "/patrolbot/manual_active", self._manual_active_cb, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Auxiliary telemetry (sonar / battery / diagnostics). Parsed from a separate
        # AUX line, fully decoupled from the nav-critical /odom and /scan path so a
        # malformed reading here can never disturb navigation. See _parse_aux.
        # Sonar is also direct, high-churn sensor data. In particular, do not let
        # a remote RViz PointCloud2 display turn it into a reliable DDS writer
        # that can wait for acknowledgements over Wi-Fi.
        self.sonar_pub = self.create_publisher(
            PointCloud2, '/sonar', qos_profile_sensor_data)
        self.battery_pub = self.create_publisher(BatteryState, '/battery', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.base_state_pub = self.create_publisher(
            BaseState, '/patrolbot/base_state', 10)

        self.charge_release_srv = self.create_service(
            ChargeRelease, '/patrolbot/charge_release',
            self._charge_release_callback)
        self.motor_enable_srv = self.create_service(
            MotorEnable, '/patrolbot/motor_enable',
            self._motor_enable_callback)
        self.motor_disable_srv = self.create_service(
            MotorDisable, '/patrolbot/motor_disable',
            self._motor_disable_callback, callback_group=ReentrantCallbackGroup())
        # The only way out of the SBC's FAULT_STOPPED dock state. Without it a
        # failed undock leaves the robot unable to move with no recovery path.
        self.lidar_power_srv = self.create_service(
            SetLidarPower, '/patrolbot/set_lidar_power',
            self._set_lidar_power_callback, callback_group=ReentrantCallbackGroup())
        self.dock_reset_srv = self.create_service(
            DockReset, '/patrolbot/dock_reset',
            self._dock_reset_callback)

        self.declare_parameter('server_ip', '10.0.0.1')
        self.declare_parameter('server_port', 7272)
        self.declare_parameter('connect_timeout', 5.0)
        self.declare_parameter('receive_timeout', 3.0)
        self.declare_parameter('reconnect_delay', 3.0)
        self.declare_parameter('control_timeout', 6.0)
        # How stale the last SBC pose may get before odom->base_link is
        # withdrawn. /odom arrives at ~22.7 Hz, so 0.5 s is about eleven
        # missed samples: long enough not to flicker on a scheduling hiccup,
        # and inside Nav2's transform_tolerance of 1.0 s so consumers see the
        # transform disappear before they would have accepted a stale one.
        self.declare_parameter('odom_tf_timeout', 0.5)
        # config/bridge_params.yaml records these as an operator MEASUREMENT
        # (2026-08-18), not a tuning knob.
        self.declare_parameter('odom_sigma_xy_per_m', 0.016)
        self.declare_parameter('odom_sigma_yaw_per_rad', 0.014)
        self.declare_parameter('odom_sigma_yaw_per_m', 0.004)
        self.declare_parameter('odom_sigma_vx', 0.02)
        self.declare_parameter('odom_sigma_vyaw', 0.02)
        self.declare_parameter('odom_min_variance', 1.0e-6)
        self.declare_parameter('odom_max_variance', 1.0)
        # These are protocol/timing contracts, not estimated physical values.
        # patrolbot_server rejects DRIVE above 0.30 m/s or 1.20 rad/s and emits
        # telemetry every 50 ms. The validator derives each continuity
        # allowance from elapsed time plus one sample period.
        self.declare_parameter('odom_max_linear_speed', 0.30)
        self.declare_parameter('odom_max_angular_speed', 1.20)
        self.declare_parameter('odom_expected_period', 0.05)
        self.server_ip = str(self.get_parameter('server_ip').value)
        self.server_port = int(self.get_parameter('server_port').value)
        self.CONNECT_TIMEOUT = float(
            self.get_parameter('connect_timeout').value)
        self.RECONNECT_DELAY = float(
            self.get_parameter('reconnect_delay').value)

        # The SBC streams telemetry at 20 Hz (every 50 ms). If no data arrives for
        # this long the link is dead — silent link loss can leave no FIN/RST, so
        # a blocking recv() would otherwise hang forever and never reconnect. A read
        # timeout converts that silent death into a socket.timeout we can recover from.
        self.RECV_TIMEOUT = float(
            self.get_parameter('receive_timeout').value)

        # Near-range measurement floor, not a robot-footprint filter. Passive
        # dock telemetry on 2026-10-10 contains real forward returns at 0.12-
        # 0.25 m; deleting those blinds localization and obstacle consumers.
        # Retain them with a 0.10 m floor. This is a software acceptance cutoff,
        # not a claim of calibrated sensor accuracy at every shorter distance.
        # Body reflections must be characterized separately; never classify
        # all nearby objects as self-occlusion using the navigation radius.
        # Historical ~0.15 m frame reflections near -75 degrees caused DWB
        # footprint deadlock; lowering this floor requires live verification
        # of those bearings, not an invented angular mask.
        self.SCAN_RANGE_MIN = 0.10

        self.sock = None
        self.sock_lock = threading.Lock()
        self.running = True
        self._stop_event = threading.Event()
        self._workers_stopped = False
        self.data_buffer = ''

        # Hardware state is valid only when both BATT and FLAGS have arrived in
        # the current TCP session. A reconnect increments the generation and
        # invalidates all prior telemetry and outstanding control operations.
        self._state_lock = threading.Lock()
        self._session_generation = 0
        self._link_connected = False
        self._charge_state = -1
        self._charge_state_at = 0.0
        self._flags_at = 0.0
        self._motors_enabled = False
        self._estop_pressed = False
        self._flags = 0
        self._fault_flags = 0
        self._stall_value = 0
        # Dock ownership, published by the SBC in the AUX DOCK= section. The
        # bridge holds no dock policy of its own; it mirrors the SBC's observer.
        self._dock_state = ''
        self._dock_state_at = 0.0
        self._dock_phase = 0
        self._dock_phase_name = 'IDLE'
        self._undock_active = False
        self._undock_release_attempts = 0
        self._minimum_rear_range = 0.0
        self._rear_sonar_usable = False
        self._redock_inhibited = False
        self._redock_inhibit_remaining = 0.0
        self._undock_profile_commissioned = False
        # Capability banner. Undock goals are refused until an SBC advertises
        # FEATURES=UNDOCK, so an older server fails closed instead of hanging.
        self._server_hello = None
        self._lidar = LidarState()
        self._lidar_pending = False
        self._lidar_hold = False
        self._lidar_cutoff = 0.
        self._hardware_operation_lock = threading.Lock()
        self._motor_disable_generation = 0

        # The stall word's physical bumper mapping is unresolved on this robot.
        # Keep it invalid until controlled testing supplies explicit masks.
        self.declare_parameter('bumpers_mapped', False)
        self.declare_parameter('front_bumper_mask', 0)
        self.declare_parameter('rear_bumper_mask', 0)

        self._control_lock = threading.Lock()
        self._pending_controls = {}
        self.CONTROL_TIMEOUT = float(
            self.get_parameter('control_timeout').value)

        # One hardware undock at a time, tracked separately from the zero-motion
        # controls because it is long-running and streams progress events.
        self._undock_lock = threading.Lock()
        self._undock_request_id = None
        self._undock_event = threading.Event()
        self._undock_result = None
        self._undock_feedback = None
        self._undock_accepted = False
        self._undock_session = 0
        self._undock_goal_lock = threading.Lock()
        self._undock_goal_reserved = False
        self._callbacks = ReentrantCallbackGroup()
        self._undock_server = ActionServer(
            self, HardwareUndock, '/patrolbot/hardware_undock',
            goal_callback=self._undock_goal_callback,
            cancel_callback=self._undock_cancel_callback,
            execute_callback=self._undock_execute_callback,
            callback_group=self._callbacks)

        # Never publish ROS messages from the socket reader. DDS publish() can
        # wait behind a stalled reader; when that happened here, the Pi
        # stopped draining TCP, the SBC's send buffer filled for ~3 s, and the
        # SBC correctly disconnected its apparently-unresponsive client. Keep a
        # latest-only queue for each path so TCP is always drained and auxiliary
        # visualization can never hold up navigation telemetry.
        self._nav_queue = queue.Queue(maxsize=1)
        self._aux_queue = queue.Queue(maxsize=1)
        self._queue_drops = {'navigation': 0, 'auxiliary': 0}
        self._last_queue_warning = 0.0

        # Latest pose, updated by the navigation publisher worker, read by TF.
        self._pose_lock = threading.Lock()
        self._px, self._py, self._pth = 0.0, 0.0, 0.0
        self._vx, self._vth = 0.0, 0.0
        self._pose_received = False
        self._pose_at = 0.0
        self._undock_scan_at = 0.0
        self._odom_tf_timeout = float(
            self.get_parameter('odom_tf_timeout').value)
        self._odom_covariance = OdomCovariance(
            sigma_xy_per_m=float(
                self.get_parameter('odom_sigma_xy_per_m').value),
            sigma_yaw_per_rad=float(
                self.get_parameter('odom_sigma_yaw_per_rad').value),
            sigma_yaw_per_m=float(
                self.get_parameter('odom_sigma_yaw_per_m').value),
            sigma_vx=float(self.get_parameter('odom_sigma_vx').value),
            sigma_vyaw=float(self.get_parameter('odom_sigma_vyaw').value),
            min_variance=float(
                self.get_parameter('odom_min_variance').value),
            max_variance=float(
                self.get_parameter('odom_max_variance').value),
        )
        self._odom_validator = OdomValidator(
            max_linear_speed=float(
                self.get_parameter('odom_max_linear_speed').value),
            max_angular_speed=float(
                self.get_parameter('odom_max_angular_speed').value),
            expected_period=float(
                self.get_parameter('odom_expected_period').value),
        )
        self._odom_rejections = {}
        self._last_odom_rejection_warning = 0.0
        self._odom_tf_stale = False
        self._tf_send_lock = threading.Lock()

        self._epoch = EpochRuntime(self)
        self._undock_turn = UndockTurnRuntime(self)

        # Publish odom→base_link TF at 50 Hz from the spin thread.
        # This decouples TF from scan delivery — TF entries are always in the
        # buffer BEFORE any scan arrives at the costmap message_filter.
        self.create_timer(0.02, self._tf_timer)
        self.create_timer(0.1, self._base_state_timer)

        self.nav_publish_thread = threading.Thread(
            target=self._publish_loop,
            args=(self._nav_queue, self._parse_telemetry, 'navigation'),
            daemon=True)
        self.aux_publish_thread = threading.Thread(
            target=self._publish_loop,
            args=(self._aux_queue, self._parse_aux, 'auxiliary'),
            daemon=True)
        self.recv_thread = threading.Thread(
            target=self._connect_loop, daemon=True)
        self._worker_threads = (
            self.nav_publish_thread,
            self.aux_publish_thread,
            self.recv_thread,
        )
        for thread in self._worker_threads:
            thread.start()

        self.get_logger().info(
            f'Bridge node started. Connecting to '
            f'{self.server_ip}:{self.server_port}...')

    # ------------------------------------------------------------------
    # TF timer — runs at 50 Hz in the spin thread
    # ------------------------------------------------------------------

    def _tf_timer(self):
        with self._epoch.lock:
            if self._epoch.streams_allowed:
                self._tf_timer_guarded()

    def _tf_timer_guarded(self):
        """
        Broadcast odom->base_link, but only while the pose is still real.

        This timer runs at 50 Hz off the node's own clock, independent of the
        SBC link, and it stamps each transform with `now`. Left ungated it
        therefore republishes the last pose it ever saw, permanently fresh by
        its header and increasingly wrong in fact, for as long as the node
        lives. Every consumer that guards on TF age -- the costmaps, the
        collision monitor, AMCL -- is satisfied by that stamp and told the
        robot's position is current when nothing is measuring it.

        Withdrawing the transform instead turns a silent wrong answer into a
        TF lookup failure, which is a failure those consumers already know how
        to handle.
        """
        with self._pose_lock:
            if not self._pose_received:
                return
            age = time.monotonic() - self._pose_at
            x, y, th = self._px, self._py, self._pth

        if age > self._odom_tf_timeout:
            if not self._odom_tf_stale:
                self._odom_tf_stale = True
                self.get_logger().warning(
                    f'odom->base_link withdrawn: no SBC pose for {age:.2f} s '
                    f'(limit {self._odom_tf_timeout:.2f} s). Consumers will '
                    f'report TF lookup failures until telemetry resumes.')
            return
        if self._odom_tf_stale:
            self._odom_tf_stale = False
            self.get_logger().info(
                'odom->base_link resumed; SBC pose is fresh again.')

        self._broadcast_odom_tf(x, y, th)

    def _broadcast_odom_tf(self, x, y, th):
        with self._epoch.lock:
            if self._epoch.streams_allowed:
                self._broadcast_odom_tf_guarded(x, y, th)

    def _broadcast_odom_tf_guarded(self, x, y, th):
        """
        Send one odom->base_link, stamped now.

        Called from the 50 Hz timer in the spin thread and, on the first pose
        after a gap, from the telemetry worker. The lock is what keeps those
        two from interleaving into an out-of-order pair of transforms.
        """
        q = get_quaternion_from_euler(th)
        t = TransformStamped()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        with self._tf_send_lock:
            t.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(t)
            if self._epoch.first_tf_ns is None:
                self._epoch.first_tf_ns = (t.header.stamp.sec * 1_000_000_000 +
                                           t.header.stamp.nanosec)

    # ------------------------------------------------------------------
    # Connection management — background thread
    # ------------------------------------------------------------------

    def _connect_loop(self):
        while self.running:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                sock.settimeout(self.CONNECT_TIMEOUT)
                sock.connect((self.server_ip, self.server_port))
                if not self.running:
                    break
                # Detect a silently-dead peer at the OS level too (defence in depth).
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                # Read timeout (not blocking forever): recv() raises socket.timeout if
                # the SBC stops streaming, so _receive_loop can break and reconnect.
                sock.settimeout(self.RECV_TIMEOUT)
                with self.sock_lock:
                    self.sock = sock
                    self.data_buffer = ''
                self._epoch.begin_session()
                with self._state_lock:
                    self._session_generation += 1
                    self._link_connected = True
                    self._charge_state_at = 0.0
                    self._flags_at = 0.0
                    self._dock_state = ''
                    self._dock_state_at = 0.0
                    self._undock_active = False
                    # A new session proves nothing about the dock, and the SBC
                    # re-announces its capabilities on every accept.
                    self._server_hello = None
                    self._lidar.reset()
                    session = self._session_generation
                # TCP session identity is not an ARIA odometry epoch. Keep
                # both the covariance and continuity reference across link
                # reconnects; only explicit ODOM_EPOCH changes reset them.
                # A disconnect must never manufacture renewed confidence.
                self.get_logger().info('Connected to SBC server.')
                self._receive_loop(sock)
            except Exception as e:
                if self.running:
                    self.get_logger().warn(
                        f'SBC connection failed: {e}. Retrying in '
                        f'{self.RECONNECT_DELAY:g}s...')
            finally:
                with self.sock_lock:
                    if self.sock is sock:
                        self.sock = None
                with self._state_lock:
                    self._link_connected = False
                    self._lidar.reset()
                    session = self._session_generation
                self._epoch.disconnect(session)
                self._fail_pending_controls(
                    session, 'hardware session disconnected')
                self._fail_pending_undock(session)
                try:
                    sock.close()
                except Exception:
                    pass
            if self.running:
                self._stop_event.wait(self.RECONNECT_DELAY)

    def _receive_loop(self, sock):
        while self.running:
            try:
                raw = sock.recv(65536)
            except socket.timeout:
                # No telemetry for RECV_TIMEOUT s — SBC is gone/hung. Break so
                # _connect_loop closes this socket and reconnects.
                self.get_logger().warn('SBC telemetry timed out. Reconnecting...')
                break
            except Exception as e:
                self.get_logger().warn(f'SBC receive error: {e}')
                break
            if not raw:
                self.get_logger().warn('SBC closed connection.')
                break
            self.data_buffer += raw.decode('utf-8', errors='ignore')
            while '\n' in self.data_buffer:
                line, self.data_buffer = self.data_buffer.split('\n', 1)
                if not line:
                    continue
                received_at = time.monotonic()
                if line.startswith('AUX:'):
                    self._queue_latest(
                        self._aux_queue, (line, received_at, self._session_generation),
                        'auxiliary')
                elif line.startswith('ACK:') or line.startswith('RESULT:'):
                    self._handle_control_reply(line)
                elif line.startswith('UNDOCK_EVENT:'):
                    self._handle_undock_event(line)
                elif line.startswith('UNDOCK_RESULT:'):
                    self._handle_undock_result(line)
                elif line.startswith('HELLO:'):
                    self._handle_hello(line)
                else:
                    self._queue_latest(
                        self._nav_queue, (line, received_at, self._session_generation),
                        'navigation')

    def _handle_control_reply(self, line):
        """Deliver an SBC ACK/RESULT without contaminating odometry parsing."""
        try:
            reply = parse_reply(line)
        except ValueError as exc:
            self.get_logger().error(f'Invalid SBC control reply: {exc}')
            return
        if reply is None:
            return
        key = (reply.command, reply.request_id)
        with self._control_lock:
            pending = self._pending_controls.get(key)
            if pending is None:
                self.get_logger().warn(
                    f'Ignoring unmatched {reply.kind} for '
                    f'{reply.command}:{reply.request_id}')
                return
            if pending.session_generation != self._session_generation:
                return
            if reply.kind == 'ACK':
                pending.accepted = True
                return
            pending.accepted = True
            pending.reply = reply
            pending.event.set()

    def _fail_pending_controls(self, session_generation, message):
        """Wake callers immediately when their hardware session disappears."""
        with self._control_lock:
            pending = list(self._pending_controls.values())
        for operation in pending:
            if operation.session_generation != session_generation:
                continue
            operation.reply = ControlReply(
                kind='RESULT', command=operation.command,
                request_id=operation.request_id, accepted=operation.accepted,
                success=False, code=6, message=message)
            operation.event.set()

    def _queue_latest(self, target_queue, item, path_name):
        """Enqueue without ever blocking the TCP receive thread."""
        try:
            target_queue.put_nowait(item)
            return
        except queue.Full:
            pass

        # A newer sensor sample supersedes an unprocessed one. This is bounded
        # latest-value behavior, not an unbounded backlog of stale robot state.
        try:
            target_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            target_queue.put_nowait(item)
        except queue.Full:
            # The worker raced us and immediately filled the single slot again.
            pass

        self._queue_drops[path_name] += 1
        now = time.monotonic()
        if now - self._last_queue_warning >= 5.0:
            self.get_logger().warn(
                f'ROS publish back-pressure: replaced queued {path_name} '
                f'telemetry (navigation drops='
                f'{self._queue_drops["navigation"]}, auxiliary drops='
                f'{self._queue_drops["auxiliary"]}).')
            self._last_queue_warning = now

    def _publish_loop(self, source_queue, parser, path_name):
        """Publish one path independently while the socket reader keeps draining."""
        while self.running:
            try:
                item = source_queue.get(timeout=0.5)
                line, received_at = item[:2]
            except queue.Empty:
                continue
            if not self.running:
                break

            age = time.monotonic() - received_at
            if age > 0.5:
                self.get_logger().warn(
                    f'Discarding {age:.3f} s old {path_name} telemetry after '
                    'ROS publish stall.')
                continue
            if path_name == 'navigation':
                parser(line, received_at=received_at, session=item[2])
            else:
                with self._epoch.lock:
                    if (item[2] == self._epoch.guard.session and
                            self._epoch.guard.ready):
                        parser(line, received_at=received_at)

    def _publish_with_timing(self, publisher, message, topic):
        """Publish and make any unexpected middleware stall visible in logs."""
        started_at = time.monotonic()
        publisher.publish(message)
        elapsed = time.monotonic() - started_at
        if elapsed > 0.05:
            self.get_logger().warn(
                f'DDS publish stall on {topic}: publish() blocked for '
                f'{elapsed:.3f} s.')

    # ------------------------------------------------------------------
    # Telemetry parsing — called from the navigation publisher worker
    # ------------------------------------------------------------------

    def _record_odom_rejection(self, reason, detail=''):
        self._odom_rejections[reason] = (
            self._odom_rejections.get(reason, 0) + 1)
        now = time.monotonic()
        if now - self._last_odom_rejection_warning < 5.0:
            return
        self._last_odom_rejection_warning = now
        counts = ', '.join(
            f'{name}={count}'
            for name, count in sorted(self._odom_rejections.items()))
        suffix = f' ({detail})' if detail else ''
        self.get_logger().error(
            f'Rejected SBC navigation frame: {reason}{suffix}; '
            f'odometry rejection counts: {counts}.')

        status = DiagnosticStatus()
        status.name = 'patrolbot/odometry'
        status.hardware_id = 'pioneer_patrolbot'
        status.level = DiagnosticStatus.ERROR
        status.message = f'odometry frame rejected: {reason}'
        status.values = [
            KeyValue(key='rejected_total',
                     value=str(sum(self._odom_rejections.values()))),
        ] + [
            KeyValue(key=f'rejected_{name}', value=str(count))
            for name, count in sorted(self._odom_rejections.items())
        ]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self._publish_with_timing(
            self.diag_pub, diagnostics, '/diagnostics')

    def _parse_telemetry(self, line, received_at=None, session=None):
        with self._epoch.lock:
            if session is None:
                session = self._epoch.guard.session
            decision = self._epoch.guard.check_frame(session, line)
            if not decision.accepted:
                if decision.reason == 'frame-epoch-mismatch':
                    self._epoch.invalidate_transport(decision.reason)
                return
            self._parse_telemetry_guarded(
                line, received_at, publish=self._epoch.streams_allowed)

    def _parse_telemetry_guarded(self, line, received_at=None, *, publish=True):
        try:
            receipt_clock = self.get_clock().now()
            queue_age = 0. if received_at is None else time.monotonic() - received_at
            if not math.isfinite(queue_age) or not 0 <= queue_age <= self._odom_tf_timeout:
                self._record_odom_rejection('stale-or-future-queued-odometry')
                return
            queue_age_ns = int(queue_age * 1e9)
            if '|' not in line or 'ODOM:' not in line or 'LASER:' not in line:
                return

            parts = line.split('|')
            odom_fields = parts[0].replace('ODOM:', '').split(',')
            laser_fields = parts[1].replace('LASER:', '').strip().split(',')

            x = float(odom_fields[0])
            y = float(odom_fields[1])
            th = float(odom_fields[2])
            vx = float(odom_fields[3])
            vth = float(odom_fields[4])

            now = time.monotonic()
            validation = self._odom_validator.check_and_accept(
                OdomSample(x, y, th, vx, vth), received_at if received_at is not None else now)
            if not validation.accepted:
                self._record_odom_rejection(validation.reason)
                return

            with self._pose_lock:
                # Was odom->base_link absent immediately before this pose --
                # either never published, or withdrawn as stale?
                tf_was_absent = (
                    not self._pose_received or
                    now - self._pose_at > self._odom_tf_timeout)
                self._px, self._py, self._pth = x, y, th
                self._vx, self._vth = vx, vth
                self._pose_received = True
                self._pose_at = received_at if received_at is not None else now
                self._odom_covariance.update(x, y, th)
                pose_covariance = self._odom_covariance.pose_covariance()
                twist_covariance = self._odom_covariance.twist_covariance()

            if self._undock_turn.phase in ('turn', 'stopping', 'restoring'):
                self._undock_turn.observe_turn_odom()

            if publish and tf_was_absent:
                # Publish the transform here rather than waiting up to 20 ms
                # for the next timer tick. /scan below is stamped from the same
                # instant as /odom, so leaving it to the timer would let the
                # first scan after a reconnect reach the costmap message filter
                # with no transform covering it -- exactly the ordering this
                # node publishes TF at 50 Hz to avoid. The timer keeps the
                # transform alive from here on; this only leads it back in.
                if self._odom_tf_stale:
                    self._odom_tf_stale = False
                    self.get_logger().info(
                        'odom->base_link resumed; SBC pose is fresh again.')
                self._broadcast_odom_tf(x, y, th)

            q = get_quaternion_from_euler(th)
            stamp = rclpy.time.Time(nanoseconds=receipt_clock.nanoseconds-queue_age_ns).to_msg()
            self._epoch._odom_history.add(
                stamp.sec*1_000_000_000+stamp.nanosec, (x, y, th), self._epoch.guard.epoch)

            # /odom — receipt-time estimate; acquisition time is not on the wire
            odom_msg = Odometry()
            odom_msg.header.stamp = stamp
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.angular.z = vth
            odom_msg.pose.covariance = pose_covariance
            odom_msg.twist.covariance = twist_covariance
            if publish:
                self._publish_with_timing(self.odom_pub, odom_msg, '/odom')

            # /scan — preserve reported acquisition age plus local queue age.
            # After startup/reconnect, older scans may precede TF history and
            # must wait/drop in consumers; never re-stamp them to hide the gap.
            # Preserve close external returns down to the measurement floor.
            # A navigation footprint is not a valid radial sensor mask: the
            # scanner's offset and actual chassis geometry differ from it.
            raw_ranges = []
            ranges = []
            # patrolbot_server.cpp writes every reading with a trailing comma;
            # discard that single wire terminator rather than inventing a beam.
            if laser_fields and not laser_fields[-1].strip():
                laser_fields = laser_fields[:-1]
            for r in laser_fields:
                r = r.strip()
                try:
                    val = float(r)
                    raw_ranges.append(
                        val if math.isfinite(val) and val >= 0.0
                        else float('inf'))
                    ranges.append(
                        val if math.isfinite(val) and
                        val >= self.SCAN_RANGE_MIN else float('inf'))
                except ValueError:
                    # Index is bearing; dropping this slot re-aims the rest
                    # of the scan, so keep it as an invalid return.
                    raw_ranges.append(float('inf'))
                    ranges.append(float('inf'))

            # One beam has no honest angle_increment for the 180-degree sweep,
            # while zero beams is the normal laser-down path and stays silent.
            if len(ranges) >= 2:
                age = parse_laser_age(line)
                if not age.accepted:
                    self.get_logger().warning(
                        f'Dropping laser scan: {age.reason}.',
                        throttle_duration_sec=10.0)
                    return
                if age.legacy:
                    self.get_logger().warning(
                        'SBC scan has no acquisition age; legacy arrival-time '
                        'stamping is unverified for localization.',
                        throttle_duration_sec=60.0)
                measured_age_ns = (age.age_ms or 0) * 1000000 + queue_age_ns
                if measured_age_ns > 500000000:
                    self.get_logger().warning(
                        'Dropping laser scan: acquisition plus queue age '
                        'exceeds 0.5 s.', throttle_duration_sec=10.0)
                    return
                scan_stamp = Time(nanoseconds=compute_scan_stamp_ns(
                    receipt_clock.nanoseconds, measured_age_ns)).to_msg()
                scan_msg = self._laser_scan_message(
                    scan_stamp, ranges, self.SCAN_RANGE_MIN)
                if not age.legacy:
                    with self._pose_lock:
                        self._undock_scan_at = (received_at if received_at is not None
                                                else now) - (age.age_ms or 0) / 1000.
                if publish:
                    self._publish_with_timing(self.scan_pub, scan_msg, '/scan')
                if publish and self.unfiltered_scan_pub is not None:
                    candidate = self._laser_scan_message(
                        scan_stamp, raw_ranges, 0.0)
                    self._publish_with_timing(
                        self.unfiltered_scan_pub, candidate,
                        self.unfiltered_scan_topic)
            elif ranges:
                self.get_logger().warning(
                    f'Dropping laser scan with {len(ranges)} range: no honest '
                    'angle_increment exists for a single beam.',
                    throttle_duration_sec=10.0)

        except (IndexError, ValueError) as exc:
            self._record_odom_rejection(
                'malformed-telemetry', type(exc).__name__)
        except Exception as exc:
            self._record_odom_rejection(
                'navigation-frame-error', type(exc).__name__)

    @staticmethod
    def _laser_scan_message(stamp, ranges, range_min):
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = -math.pi / 2.0
        msg.angle_max = math.pi / 2.0
        msg.angle_increment = math.pi / (len(ranges) - 1)
        msg.time_increment = 0.0
        msg.scan_time = 0.05
        msg.range_min = range_min
        msg.range_max = 8.0
        msg.ranges = ranges
        return msg

    # ------------------------------------------------------------------
    # Auxiliary telemetry parsing (sonar / battery / flags)
    # ------------------------------------------------------------------

    def _parse_aux(self, line, received_at=None):
        # AUX:SONAR=x,y;x,y;...|BATT=v,soc,cs,temp|FLAGS=flags,fault,stall,motors,estop
        #     |DOCK=state,phase,phase_name,active,attempts,rear_m,rear_usable,
        #           inhibited,inhibit_remaining_ms,profile_ok
        # DOCK is emitted only by protocol 2 servers. Unknown sections are
        # skipped, so this stays compatible in both directions.
        # Each section is parsed and published in isolation: a malformed or missing
        # section only skips its own topic and never affects the others or nav data.
        sections = {}
        for part in line[4:].split('|'):
            if '=' in part:
                key, val = part.split('=', 1)
                sections[key] = val
        stamp = self.get_clock().now().to_msg()
        lidar_sections = [p[6:] for p in line[4:].split('|') if p.startswith('LIDAR=')]
        self._record_lidar_section(lidar_sections[0] if len(lidar_sections) == 1 else '', received_at)

        if 'SONAR' in sections:
            try:
                self._publish_sonar(sections['SONAR'], stamp)
            except Exception:
                pass
        # Record DOCK before publishing BATT even though BATT appears first on
        # the wire. This makes one AUX frame internally consistent: battery
        # charging status is derived from the dock observer in that same frame,
        # never the previous frame or raw charge state alone.
        if 'DOCK' in sections:
            try:
                self._record_dock_section(sections['DOCK'])
            except Exception:
                pass
        if 'BATT' in sections:
            try:
                self._publish_battery(sections['BATT'], stamp)
            except Exception:
                pass
        if 'FLAGS' in sections:
            try:
                self._publish_flags(sections['FLAGS'], stamp)
            except Exception:
                pass

    def _record_lidar_section(self, data, received_at=None):
        received_at = time.monotonic() if received_at is None else received_at
        with self._state_lock:
            if received_at <= self._lidar_cutoff:
                return
            try:
                self._lidar.update(data, received_at)
            except ValueError:
                self._lidar.reset()

    def _lidar_ready(self):
        # Callers serialize admission/hold changes through the epoch lock.
        with self._state_lock:
            return (self._link_connected and not self._lidar_pending and
                    not self._lidar_hold and self._lidar.ready(time.monotonic()))

    def _record_dock_section(self, data):
        """Mirror the SBC's dock observer; the bridge holds no dock policy."""
        f = data.split(',')
        if len(f) < 10:
            raise ValueError('short DOCK section')
        state = f[0]
        phase = int(f[1])
        phase_name = f[2]
        active = bool(int(f[3]))
        attempts = int(f[4])
        rear_range = float(f[5])
        rear_usable = bool(int(f[6]))
        inhibited = bool(int(f[7]))
        inhibit_remaining = float(f[8]) / 1000.0
        commissioned = bool(int(f[9]))
        with self._state_lock:
            self._dock_state = state
            self._dock_state_at = time.monotonic()
            self._dock_phase = phase
            self._dock_phase_name = phase_name
            self._undock_active = active
            self._undock_release_attempts = attempts
            self._minimum_rear_range = rear_range
            self._rear_sonar_usable = rear_usable
            self._redock_inhibited = inhibited
            self._redock_inhibit_remaining = inhibit_remaining
            self._undock_profile_commissioned = commissioned

    def _publish_sonar(self, data, stamp):
        points = []
        for pair in data.split(';'):
            if not pair:
                continue
            xy = pair.split(',')
            if len(xy) >= 2:
                points.append((float(xy[0]), float(xy[1]), 0.0))

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True
        buf = bytearray()
        for (x, y, z) in points:
            buf += struct.pack('<fff', x, y, z)
        msg.data = bytes(buf)
        self._publish_with_timing(self.sonar_pub, msg, '/sonar')

    def _publish_battery(self, data, stamp):
        f = data.split(',')
        volt = float(f[0])
        soc = float(f[1])
        charge_state = int(float(f[2]))
        temp = float(f[3])

        with self._state_lock:
            self._charge_state = charge_state
            self._charge_state_at = time.monotonic()
            dock_state = self._dock_state
            dock_age = (
                time.monotonic() - self._dock_state_at
                if self._dock_state_at else math.inf)
            dock_state_valid = bool(
                self._link_connected and dock_state and dock_age <= 1.0)

        msg = BatteryState()
        msg.header.stamp = stamp
        msg.voltage = volt
        msg.percentage = (soc / 100.0) if soc >= 0.0 else float('nan')
        msg.temperature = temp if temp > -100.0 else float('nan')
        msg.present = True
        # Raw charge state 3 can re-latch after a proven departure. The fresh
        # SBC dock observer is authoritative for physical charger contact.
        msg.power_supply_status = battery_supply_status(
            charge_state, dock_state, dock_state_valid)
        self._publish_with_timing(self.battery_pub, msg, '/battery')

        level, message = battery_diagnostic(
            volt, charge_state, dock_state, dock_state_valid)
        status = DiagnosticStatus()
        status.name = 'patrolbot/battery'
        status.hardware_id = 'pioneer_patrolbot'
        status.level = {
            0: DiagnosticStatus.OK,
            1: DiagnosticStatus.WARN,
            2: DiagnosticStatus.ERROR,
        }[level]
        status.message = message
        status.values = [
            KeyValue(key='voltage', value=f'{volt:.3f}'),
            KeyValue(key='charge_state', value=str(charge_state)),
            KeyValue(key='dock_state', value=(
                dock_state if dock_state_valid else 'UNKNOWN')),
            KeyValue(
                key='power_supply_status',
                value=str(msg.power_supply_status)),
            KeyValue(
                key='recharge_threshold_volts',
                value=str(BATTERY_RECHARGE_VOLTS)),
            KeyValue(
                key='minimum_operating_volts',
                value=str(BATTERY_MIN_OPERATING_VOLTS)),
        ]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = stamp
        diagnostics.status = [status]
        self._publish_with_timing(
            self.diag_pub, diagnostics, '/diagnostics')

    def _publish_flags(self, data, stamp):
        f = data.split(',')
        flags = int(f[0])
        fault = int(f[1])
        stall = int(f[2])
        motors = int(f[3])
        # e-stop field added 2026-06-28; tolerate older servers that omit it.
        estop = int(f[4]) if len(f) > 4 else 0

        with self._state_lock:
            self._flags = flags
            self._fault_flags = fault
            self._stall_value = stall
            self._motors_enabled = bool(motors)
            self._estop_pressed = bool(estop)
            self._flags_at = time.monotonic()

        # ARIA stall value: high byte = left wheel, low byte = right wheel.
        # bit 0 of each byte = motor stall (reliable). The remaining bits nominally
        # carry bumper segments, but on this PatrolBot-SH a constant high bit shows
        # even when idle/healthy (no bumpers wired / reserved bit), so they are
        # reported raw for reference only — they do NOT drive the alarm level.
        left_stall = bool(stall & 0x0100)
        right_stall = bool(stall & 0x0001)
        left_bumper_bits = (stall >> 9) & 0x7F
        right_bumper_bits = (stall >> 1) & 0x7F
        active_faults = decode_fault_flags(fault)

        st = DiagnosticStatus()
        st.name = 'patrolbot/base'
        st.hardware_id = 'pioneer_patrolbot'
        # Level driven only by well-defined signals: e-stop, fault flags, stalls.
        # A latched e-stop is the most common reason the robot won't move, so it
        # gets its own explicit, top-priority message.
        if estop:
            st.level = DiagnosticStatus.ERROR
            st.message = 'E-STOP ENGAGED — release the red button to move'
        elif fault or left_stall or right_stall:
            st.level = DiagnosticStatus.ERROR
            if active_faults:
                st.message = 'hardware fault: ' + ', '.join(active_faults)
            else:
                st.message = 'motor stall'
        elif not motors:
            st.level = DiagnosticStatus.WARN
            st.message = 'motors disabled'
        else:
            st.level = DiagnosticStatus.OK
            st.message = 'OK'
        st.values = [
            KeyValue(key='estop_pressed', value=str(bool(estop))),
            KeyValue(key='motors_enabled', value=str(bool(motors))),
            KeyValue(key='flags', value=str(flags)),
            KeyValue(key='fault_flags', value=str(fault)),
            KeyValue(
                key='fault_flags_hex', value=f'0x{fault & 0xFFFF:04x}'),
            KeyValue(
                key='active_faults',
                value=','.join(active_faults) if active_faults else 'none'),
            KeyValue(key='stall_value', value=str(stall)),
            KeyValue(key='left_motor_stall', value=str(left_stall)),
            KeyValue(key='right_motor_stall', value=str(right_stall)),
            KeyValue(key='left_bumper_bits_raw', value=str(left_bumper_bits)),
            KeyValue(key='right_bumper_bits_raw', value=str(right_bumper_bits)),
        ]
        arr = DiagnosticArray()
        arr.header.stamp = stamp
        arr.status = [st]
        self._publish_with_timing(self.diag_pub, arr, '/diagnostics')

    def _base_state_timer(self):
        with self._epoch.lock:
            self._base_state_timer_guarded()

    def _base_state_timer_guarded(self):
        """Publish typed, session-aware state; stale fields are never valid."""
        now = time.monotonic()
        with self._state_lock:
            generation = self._session_generation
            connected = self._link_connected
            charge_state = self._charge_state
            charge_age = (
                now - self._charge_state_at
                if self._charge_state_at else math.inf)
            flags_age = now - self._flags_at if self._flags_at else math.inf
            motors = self._motors_enabled
            estop = self._estop_pressed
            flags = self._flags
            fault_flags = self._fault_flags
            stall_value = self._stall_value
            dock_state = self._dock_state
            dock_age = (
                now - self._dock_state_at if self._dock_state_at else math.inf)
            dock_phase = self._dock_phase
            dock_phase_name = self._dock_phase_name
            undock_active = self._undock_active
            release_attempts = self._undock_release_attempts
            minimum_rear_range = self._minimum_rear_range
            rear_sonar_usable = self._rear_sonar_usable
            redock_inhibited = self._redock_inhibited
            redock_remaining = self._redock_inhibit_remaining
            profile_commissioned = self._undock_profile_commissioned

        telemetry_age = max(charge_age, flags_age)
        state_valid = connected and telemetry_age <= 0.5
        bumpers_mapped = bool(
            self.get_parameter('bumpers_mapped').value)
        front_mask = int(self.get_parameter('front_bumper_mask').value)
        rear_mask = int(self.get_parameter('rear_bumper_mask').value)
        bumpers_valid = (
            state_valid and bumpers_mapped and front_mask != 0 and rear_mask != 0)

        with self._epoch.lock:
            origin = self._epoch.guard.epoch or ''
            origin_valid = self._epoch.guard.ready
            recovery_required = not self._epoch.commands_allowed
            recovery_stage = self._epoch.status_stage
            origin_generation = self._epoch.guard.generation
            seed_stamp = self._epoch.recovery._seed_ns or 0
        msg = BaseState()
        msg.localization_seed_stamp_ns = seed_stamp
        msg.localization_pose_stamp_ns = self._epoch.accepted_pose_stamp_ns
        with self._epoch.lock:
            active_map = self._epoch.maps.active if self._epoch.maps else None
            msg.map_id = active_map['map_id'] if active_map else ''
            msg.map_revision = active_map['revision'] if active_map else ''
            msg.map_activated_stamp_ns = self._epoch.map_activated_stamp_ns
        msg.odom_epoch = origin
        msg.odom_epoch_generation = origin_generation
        msg.odom_epoch_valid = origin_valid
        msg.localization_recovery_required = recovery_required
        msg.localization_recovery_stage = recovery_stage
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.session_generation = generation
        msg.link_connected = connected
        msg.telemetry_age = float(
            telemetry_age if math.isfinite(telemetry_age) else 1e6)
        msg.hardware_state_valid = state_valid
        with self._state_lock:
            lidar_state, supported, scan_age = self._lidar.snapshot(now)
            msg.lidar_state = (
                "STARTING" if self._lidar_pending else
                "FAULT" if self._lidar_hold else lidar_state) if connected else "UNKNOWN"
            msg.lidar_power_supported = supported and connected
            msg.lidar_scan_age = float(scan_age)
        msg.charge_state = charge_state
        msg.charge_state_valid = connected and charge_age <= 0.5
        msg.motors_enabled = motors
        msg.estop_pressed = estop
        msg.flags = flags
        msg.fault_flags = fault_flags
        msg.stall_value = stall_value
        msg.bumpers_valid = bumpers_valid
        msg.front_bumper_pressed = (
            bool(stall_value & front_mask) if bumpers_valid else False)
        msg.rear_bumper_pressed = (
            bool(stall_value & rear_mask) if bumpers_valid else False)
        # A stale or absent DOCK section is never reported as a dock state: an
        # older SBC does not emit one, and its absence must not read as CLEAR.
        dock_valid = bool(connected and dock_state and dock_age <= 1.0)
        msg.dock_state = dock_state if dock_valid else 'UNKNOWN'
        msg.dock_state_valid = dock_valid
        msg.dock_phase = dock_phase if dock_valid else 0
        msg.dock_phase_name = dock_phase_name if dock_valid else 'IDLE'
        msg.undock_active = undock_active if dock_valid else False
        with self._epoch.lock:
            msg.undock_turn_active = self._undock_turn.drive_allowed
            msg.undock_turn_goal_id = (
                self._undock_turn.spin_id.hex() if msg.undock_turn_active else '')
        msg.undock_release_attempts = int(release_attempts)
        msg.minimum_rear_range = float(minimum_rear_range)
        msg.rear_sonar_usable = bool(rear_sonar_usable and dock_valid)
        # Fail closed: without a fresh DOCK section, treat docking as inhibited
        # and the profile as uncommissioned.
        msg.redock_inhibited = bool(redock_inhibited) if dock_valid else True
        msg.redock_inhibit_remaining = float(redock_remaining)
        msg.undock_profile_commissioned = (
            bool(profile_commissioned) if dock_valid else False)
        self._publish_with_timing(
            self.base_state_pub, msg, '/patrolbot/base_state')

    # ------------------------------------------------------------------
    # Idempotent, zero-motion hardware-control services
    # ------------------------------------------------------------------

    def _charge_release_callback(self, request, response):
        return self._control_service(
            COMMAND_CHARGE_RELEASE, request.request_id, response)

    def _motor_enable_callback(self, request, response):
        if self._undock_turn.busy:
            response.request_id = request.request_id
            return self._fill_control_response(
                response, None, 3, 'undock owns motor state', time.monotonic())
        return self._control_service(
            COMMAND_MOTOR_ENABLE, request.request_id, response)

    def _motor_disable_callback(self, request, response):
        if valid_request_id(request.request_id):
            self._undock_turn.revoke('motors disabled by operator')
            with self._epoch.lock, self.sock_lock:
                self._motor_disable_generation = getattr(self, '_motor_disable_generation', 0) + 1
        return self._control_service_guarded(
            COMMAND_MOTOR_DISABLE, request.request_id, response)

    def _dock_reset_callback(self, request, response):
        if valid_request_id(request.request_id):
            self._undock_turn.revoke('dock reset requested')
        return self._control_service(
            COMMAND_DOCK_RESET, request.request_id, response)

    def _set_lidar_power_callback(self, request, response):
        if valid_request_id(request.request_id):
            self._undock_turn.revoke('LiDAR power change requested')
        return self._control_service(
            COMMAND_LIDAR_ON if request.enabled else COMMAND_LIDAR_OFF,
            request.request_id, response)

    def _control_service(self, command, request_id, response):
        response.request_id = request_id
        if not self._hardware_operation_lock.acquire(blocking=False):
            return self._fill_control_response(
                response, None, 3, 'another hardware operation is pending', time.monotonic())
        power = command in (COMMAND_LIDAR_ON, COMMAND_LIDAR_OFF)
        try:
            return self._control_service_guarded(command, request_id, response)
        finally:
            if power:
                with self._epoch.lock, self._state_lock:
                    self._lidar_pending = False
            self._hardware_operation_lock.release()

    def _control_service_guarded(self, command, request_id, response):
        started = time.monotonic()
        power = command in (COMMAND_LIDAR_ON, COMMAND_LIDAR_OFF)
        response.request_id = request_id
        with self._epoch.lock:
            motor_generation = getattr(self, '_motor_disable_generation', 0)
            if command not in (COMMAND_MOTOR_DISABLE, COMMAND_MOTOR_ENABLE) and self._epoch.maps and self._epoch.maps.hold:
                return self._fill_control_response(response, None, 3, 'map localization owns the robot', started)
        if not valid_request_id(request_id):
            return self._fill_control_response(
                response, None, 1, 'request_id must be a canonical UUID',
                started)

        with self._state_lock:
            generation = self._session_generation
            connected = self._link_connected
            charge_age = (
                started - self._charge_state_at
                if self._charge_state_at else math.inf)
            flags_age = (
                started - self._flags_at if self._flags_at else math.inf)
            charge_state = self._charge_state
            motors = self._motors_enabled
            estop = self._estop_pressed
            fault_flags = self._fault_flags
            dock_state = self._dock_state
            undock_active = self._undock_active
        with self._pose_lock:
            linear = self._vx
            angular = self._vth
            odom_age = started - self._pose_at if self._pose_at else math.inf

        if not connected:
            return self._fill_control_response(
                response, None, 2, 'hardware link unavailable', started)
        if command == COMMAND_MOTOR_ENABLE and not motor_enable_allowed(dock_state, estop):
            return self._fill_control_response(
                response, None, 3, 'motor enable refused while E-stop asserted', started)
        if command not in (COMMAND_MOTOR_DISABLE, COMMAND_MOTOR_ENABLE):
            if not all(0 <= age <= .5 for age in (charge_age, flags_age)):
                return self._fill_control_response(
                    response, None, 3, 'hardware telemetry is stale', started)
            if not 0 <= odom_age <= .5:
                return self._fill_control_response(
                    response, None, 3, 'odometry is stale', started)
            if (not all(math.isfinite(v) for v in (linear, angular)) or
                    abs(linear) > 0.005 or abs(angular) > 0.01):
                return self._fill_control_response(
                    response, None, 3, 'robot must be stationary', started)
            if fault_flags:
                return self._fill_control_response(
                    response, None, 3, 'hardware fault flags are nonzero', started)
            if undock_active:
                return self._fill_control_response(
                    response, None, 3, 'the undock state machine owns motion',
                    started)
            if command == COMMAND_DOCK_RESET and estop:
                return self._fill_control_response(
                    response, None, 3, 'dock reset refused while E-stop asserted',
                    started)
            if command == COMMAND_CHARGE_RELEASE:
                if charge_state < 0:
                    return self._fill_control_response(
                        response, None, 3, 'charge state is invalid', started)
                # Manual dock charging can leave the motor-enabled flag set. The
                # SBC release operation owns a zero-motion disable-confirm-release
                # sequence in that state. Already-released enabled motors remain a
                # precondition failure and are never changed by an idempotent call.
                if motors and charge_state == 0:
                    return self._fill_control_response(
                        response, None, 3,
                        'motors enabled while charge is already released', started)
            if not power and not self._lidar_ready():
                return self._fill_control_response(
                    response, None, 3, 'fresh ready LiDAR is required', started)
        key = (command, request_id)
        pending = PendingControl(command, request_id, generation)
        with self._control_lock:
            if key in self._pending_controls:
                return self._fill_control_response(
                    response, None, 1, 'duplicate request already in progress',
                    started)
            self._pending_controls[key] = pending

        try:
            payload = format_request(command, request_id)
            with self._epoch.lock, self.sock_lock:
                if self._session_generation != generation:
                    self._discard_pending_control(key, pending)
                    return self._fill_control_response(
                        response, None, 6, 'hardware session changed', started)
                if command == COMMAND_MOTOR_ENABLE and motor_generation != getattr(self, '_motor_disable_generation', 0):
                    self._discard_pending_control(key, pending)
                    return self._fill_control_response(
                        response, None, 3, 'canceled by motor disable', started)
                if power:
                    with self._state_lock, self._undock_goal_lock:
                        state, supported, _ = self._lidar.snapshot(time.monotonic())
                        hello = self._server_hello
                        busy = self._undock_goal_reserved or self._undock_active
                    idle_at = self._epoch._navigation_idle_at
                    if (hello is None or 'LIDAR_POWER' not in hello.features or
                            not supported or state in ('UNKNOWN', 'STARTING') or busy or
                            (self._epoch.maps and self._epoch.maps.hold) or
                            self._epoch._map_goal is not None or
                            self._epoch._future is not None or self._epoch._map_future is not None or
                            self._epoch._navigation_status or idle_at is None or
                            not 0 <= time.monotonic() - idle_at <= .5):
                        self._discard_pending_control(key, pending)
                        return self._fill_control_response(
                            response, None, 3,
                            'power control requires commissioned fresh LiDAR state and idle robot', started)
                    with self._state_lock:
                        self._lidar_pending = True
                        self._lidar_hold = True
                        self._lidar_cutoff = time.monotonic()
                    self._epoch.require_lidar_seed()
                    if self.sock is not None:
                        self.sock.sendall(b'DRIVE:0.0:0.0\n')
                elif command not in (COMMAND_MOTOR_DISABLE, COMMAND_MOTOR_ENABLE) and (
                        not self._lidar_ready() or
                        not self._epoch.commands_allowed):
                    self._discard_pending_control(key, pending)
                    return self._fill_control_response(
                        response, None, 3, 'odometry origin recovery required', started)
                if self.sock is None:
                    self._discard_pending_control(key, pending)
                    return self._fill_control_response(
                        response, None, 2, 'hardware link unavailable', started)
                self.sock.sendall(payload)
            self.get_logger().info(
                f'Sent zero-motion {command} request {request_id} '
                f'on hardware session {generation}.')
        except Exception as exc:
            self._discard_pending_control(key, pending)
            return self._fill_control_response(
                response, None, 2, f'hardware send failed: {exc}', started)

        if not pending.event.wait(self.CONTROL_TIMEOUT):
            self._discard_pending_control(key, pending)
            return self._fill_control_response(
                response, None, 5, 'hardware operation timed out', started,
                accepted=pending.accepted)
        reply = pending.reply
        self._discard_pending_control(key, pending)
        if reply is None:
            return self._fill_control_response(
                response, None, 7, 'missing terminal result', started,
                accepted=pending.accepted)
        if reply.code == 6:
            return self._fill_control_response(
                response, reply, 6, reply.message, started,
                accepted=reply.accepted)
        if power and reply.success and pending.session_generation == self._session_generation:
            with self._epoch.lock, self._state_lock:
                self._lidar_cutoff = time.monotonic()
                self._lidar.update(
                    ('STARTING' if command == COMMAND_LIDAR_ON else 'OFF') + ',1,-1',
                    self._lidar_cutoff)
                self._lidar_hold = False
        self.get_logger().info(
            f'{command} request {request_id} completed: '
            f'success={reply.success}, code={reply.code}, message={reply.message}.')
        return self._fill_control_response(
            response, reply, reply.code, reply.message, started,
            accepted=reply.accepted)

    def _discard_pending_control(self, key, pending):
        with self._control_lock:
            if self._pending_controls.get(key) is pending:
                self._pending_controls.pop(key, None)

    def _fill_control_response(
            self, response, reply, code, message, started, accepted=False):
        """Populate either generated service response with the same contract."""
        with self._state_lock:
            generation = self._session_generation
            charge_state = self._charge_state
            motors = self._motors_enabled
            estop = self._estop_pressed
        response.accepted = accepted or bool(reply and reply.accepted)
        response.success = bool(reply and reply.success)
        response.code = int(code)
        response.message = message
        response.charge_state = (
            reply.charge_state if reply is not None else charge_state)
        response.motors_enabled = (
            reply.motors_enabled if reply is not None else motors)
        response.estop_pressed = (
            reply.estop_pressed if reply is not None else estop)
        response.session_generation = generation
        response.elapsed_seconds = float(time.monotonic() - started)
        if hasattr(response, 'state'):
            with self._state_lock:
                response.state = ('FAULT' if self._lidar_hold else
                                  self._lidar.snapshot(time.monotonic())[0])
        # request_id is initialized by _control_service before any return.
        return response

    # ------------------------------------------------------------------
    # Hardware undock action
    # ------------------------------------------------------------------

    def _handle_hello(self, line):
        """Record the SBC capability banner for this session."""
        try:
            hello = parse_hello(line)
        except ValueError as exc:
            self.get_logger().error(f'Invalid SBC HELLO: {exc}')
            with self._epoch.lock:
                self._epoch.invalidate_transport('malformed HELLO')
            return
        if hello is None:
            return
        self._epoch.announce(self._session_generation, hello.odom_epoch)
        with self._state_lock:
            self._server_hello = hello
        self.get_logger().info(
            f'SBC protocol {hello.protocol}, features '
            f'{sorted(hello.features) or ["none"]}.')

    def _handle_undock_event(self, line):
        """Record an undock progress frame for the executing goal."""
        try:
            event = parse_undock_event(line)
        except ValueError as exc:
            self.get_logger().error(f'Invalid UNDOCK_EVENT: {exc}')
            return
        if event is None:
            return
        with self._undock_lock:
            if event.request_id != self._undock_request_id:
                return
            self._undock_accepted = True
            self._undock_feedback = event

    def _handle_undock_result(self, line):
        """Deliver the terminal undock frame and wake the executing goal."""
        try:
            result = parse_undock_result(line)
        except ValueError as exc:
            self.get_logger().error(f'Invalid UNDOCK_RESULT: {exc}')
            return
        if result is None:
            return
        with self._undock_lock:
            if result.request_id != self._undock_request_id:
                self.get_logger().warn(
                    f'Ignoring unmatched UNDOCK_RESULT for {result.request_id}')
                return
            self._undock_result = result
        self._undock_event.set()

    def _fail_pending_undock(self, session_generation):
        """Wake an executing undock immediately when its session disappears."""
        with self._undock_lock:
            if self._undock_request_id is None:
                return
            if self._undock_session != session_generation:
                return
            if self._undock_result is not None:
                return
            self._undock_result = UndockResult(
                request_id=self._undock_request_id, success=False,
                code=SESSION_CHANGED, message='hardware session disconnected',
                phase=0, phase_name='IDLE', distance_traveled=0.0,
                lateral_error=0.0, heading_error=0.0, charge_state=-1,
                motors_enabled=False, estop_pressed=True,
                minimum_rear_range=0.0, release_attempts=0,
                dock_state='UNKNOWN', physically_clear=False)
        self._undock_event.set()

    def _undock_goal_callback(self, request):
        """Admit one undock goal at a time, against an SBC that supports it."""
        for name, value in (
                ('clearance_distance', request.clearance_distance),
                ('reverse_speed', request.reverse_speed),
                ('maximum_duration', request.maximum_duration)):
            if not math.isfinite(value) or value <= 0.0:
                self.get_logger().error(
                    f'Hardware undock rejected: {name} must be finite and '
                    f'positive.')
                return GoalResponse.REJECT
        with self._state_lock:
            connected = self._link_connected
            hello = self._server_hello
        if not connected:
            self.get_logger().error(
                'Hardware undock rejected: no SBC link.')
            return GoalResponse.REJECT
        if hello is None or not hello.supports_undock():
            # Fail closed rather than sending a command this server will never
            # answer and then waiting out the whole timeout.
            self.get_logger().error(
                'Hardware undock rejected: this SBC does not advertise '
                'FEATURES=UNDOCK.')
            return GoalResponse.REJECT
        with self._epoch.lock, self._undock_goal_lock:
            if self._undock_goal_reserved:
                self.get_logger().error(
                    'Hardware undock rejected: another goal is active.')
                return GoalResponse.REJECT
            if not self._undock_turn.reserve_reverse(bool(request.validation_mode)):
                return GoalResponse.REJECT
            self._undock_goal_reserved = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _undock_cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def _undock_execute_callback(self, goal_handle):
        started = time.monotonic()
        request_id = str(uuid4())
        with self._state_lock:
            session = self._session_generation
        with self._undock_lock:
            self._undock_request_id = request_id
            self._undock_result = None
            self._undock_feedback = None
            self._undock_accepted = False
            self._undock_session = session
        self._undock_event.clear()
        try:
            problem = self._undock_turn.prepare_reverse(goal_handle)
            if problem:
                return self._undock_abort(
                    goal_handle, LINK_UNAVAILABLE, problem, started, session)
            payload = format_undock_request(
                request_id, float(goal_handle.request.clearance_distance),
                float(goal_handle.request.reverse_speed),
                float(goal_handle.request.maximum_duration),
                bool(goal_handle.request.validation_mode))
            with self._epoch.lock, self.sock_lock:
                if self._session_generation != session:
                    return self._undock_abort(
                        goal_handle, SESSION_CHANGED,
                        'hardware session changed', started, session)
                if (self._undock_turn.reason or
                        self._undock_turn.hardware_problem(charging=True, stationary=True)):
                    return self._undock_abort(
                        goal_handle, LINK_UNAVAILABLE,
                        'odometry origin recovery required', started, session)
                if self.sock is None:
                    return self._undock_abort(
                        goal_handle, LINK_UNAVAILABLE,
                        'hardware link unavailable', started, session)
                self.sock.sendall(payload)
            self.get_logger().info(
                f'Sent UNDOCK {request_id} on hardware session {session}.')

            # The SBC owns the operation timeout; this is only a backstop so a
            # lost terminal frame cannot block the goal forever.
            deadline = started + float(
                goal_handle.request.maximum_duration) + 15.0
            cancel_sent = False
            while time.monotonic() < deadline:
                if self._undock_event.wait(0.05):
                    break
                if goal_handle.is_cancel_requested and not cancel_sent:
                    cancel_sent = self._send_undock_cancel(request_id)
                with self._state_lock:
                    current_session = self._session_generation
                if current_session != session:
                    self._fail_pending_undock(session)
                    break
                self._publish_undock_feedback(goal_handle)

            with self._undock_lock:
                result = self._undock_result
            if result is None:
                self._send_undock_cancel(request_id)
                self._undock_turn.reverse_unknown(goal_handle, 'no terminal result from the SBC')
                return self._undock_abort(
                    goal_handle, TIMEOUT,
                    'no terminal result from the SBC', started, session)
            self._undock_turn.reverse_result(goal_handle, result)
            return self._undock_finish(goal_handle, result, started, session)
        finally:
            with self._undock_lock:
                self._undock_request_id = None
                self._undock_feedback = None
            with self._undock_goal_lock:
                self._undock_goal_reserved = False

    def _send_undock_cancel(self, request_id):
        """Ask the SBC to cancel; it reports CANCELED only once it has stopped."""
        try:
            with self.sock_lock:
                if self.sock is None:
                    return False
                self.sock.sendall(format_undock_cancel(request_id))
        except Exception as exc:
            self.get_logger().warn(f'UNDOCK_CANCEL send failed: {exc}')
            return False
        self.get_logger().info(f'Sent UNDOCK_CANCEL for {request_id}.')
        return True

    def _publish_undock_feedback(self, goal_handle):
        with self._undock_lock:
            event = self._undock_feedback
        if event is None:
            return
        feedback = HardwareUndock.Feedback()
        feedback.phase = int(event.phase)
        feedback.phase_name = event.phase_name
        feedback.distance_traveled = float(event.distance_traveled)
        feedback.lateral_error = float(event.lateral_error)
        feedback.heading_error = float(event.heading_error)
        feedback.charge_state = int(event.charge_state)
        feedback.motors_enabled = bool(event.motors_enabled)
        feedback.minimum_rear_range = float(event.minimum_rear_range)
        feedback.release_attempts = int(event.release_attempts)
        feedback.dock_state = event.dock_state
        goal_handle.publish_feedback(feedback)

    def _undock_finish(self, goal_handle, reply, started, session):
        result = HardwareUndock.Result()
        result.success = bool(reply.success)
        result.code = int(reply.code)
        result.message = reply.message
        result.distance_traveled = float(reply.distance_traveled)
        result.lateral_error = float(reply.lateral_error)
        result.heading_error = float(reply.heading_error)
        result.release_attempts = int(reply.release_attempts)
        result.final_charge_state = int(reply.charge_state)
        result.motors_enabled = bool(reply.motors_enabled)
        result.final_dock_state = reply.dock_state
        result.physically_clear = bool(reply.physically_clear)
        result.elapsed_seconds = float(time.monotonic() - started)
        result.session_generation = session
        if reply.success:
            goal_handle.succeed()
        elif reply.code == CANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self.get_logger().info(
            f'Hardware undock finished: success={reply.success} '
            f'code={reply.code} message={reply.message} '
            f'distance={reply.distance_traveled:.3f} '
            f'attempts={reply.release_attempts} '
            f'dock_state={reply.dock_state} motors={reply.motors_enabled}')
        return result

    def _undock_abort(self, goal_handle, code, message, started, session):
        self._undock_turn.revoke(message)
        self._undock_turn.finish()
        if goal_handle.is_cancel_requested:
            code = CANCELED
        elif self._session_generation != session or not self._epoch.guard.ready:
            code = SESSION_CHANGED
        result = HardwareUndock.Result()
        result.success = False
        result.code = int(code)
        result.message = message
        result.final_dock_state = 'UNKNOWN'
        result.elapsed_seconds = float(time.monotonic() - started)
        result.session_generation = session
        if code == CANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self.get_logger().error(f'Hardware undock aborted: {message}')
        return result

    # ------------------------------------------------------------------
    # Velocity forwarding
    # ------------------------------------------------------------------

    def _manual_active_cb(self, msg):
        if bool(getattr(msg, "data", False)):
            self._manual_active_at = time.monotonic()

    def _manual_drive_allowed(self):
        # Scoped bypass: fresh deadman AND recovery is the blocker.
        # Autonomy never asserts manual_active, so it stays blocked.
        return manual_drive_bypass(
            time.monotonic() - self._manual_active_at,
            undock_hold=self._epoch.undock_hold,
            lidar_ready=self._lidar_ready(),
            map_hold=self._epoch.map_hold,
            guard_ready=self._epoch.guard.ready,
            recovery_required=self._epoch.recovery.required)

    def cmd_vel_callback(self, msg):
        # Defence in depth. The SBC refuses every nonzero DRIVE while its undock
        # state machine owns motion; not sending them at all keeps the log clean
        # and removes any chance of a race on the socket.
        with self._state_lock:
            if self._undock_active:
                return
        try:
            # Serialize all writes so DRIVE bytes can never interleave with an
            # idempotent hardware-control request on the TCP stream.
            with self._epoch.lock, self.sock_lock:
                if msg.linear.x != 0.0 or msg.angular.z != 0.0:
                    if self._undock_turn.busy:
                        if not (self._undock_turn.drive_allowed and pure_angular(msg)):
                            return
                    elif not self._epoch.commands_allowed or not self._lidar_ready():
                        # 20261014: deadman-held manual DRIVE stays available
                        # during localization recovery; autonomy (no manual
                        # evidence) remains blocked by commands_allowed.
                        if not self._manual_drive_allowed():
                            return
                        return
                if self.sock is None:
                    return
                self.sock.sendall(
                    f'DRIVE:{msg.linear.x}:{msg.angular.z}\n'.encode('utf-8'))
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self):
        if self._workers_stopped:
            return
        self._undock_turn.destroy()
        try:
            self._undock_server.destroy()
        except Exception:
            pass
        self.running = False
        self._stop_event.set()
        with self.sock_lock:
            if self.sock:
                try:
                    self.sock.shutdown(socket.SHUT_RDWR)
                except Exception:
                    pass
                try:
                    self.sock.close()
                except Exception:
                    pass
        # Publishers, loggers, and queues remain valid until every worker has
        # exited. This prevents a shutdown/restart race where a background
        # publisher enters rclpy after Node.destroy_node() frees its handle.
        deadline = time.monotonic() + max(
            self.CONNECT_TIMEOUT, self.RECV_TIMEOUT, 0.5) + 1.0
        for thread in self._worker_threads:
            remaining = max(0.0, deadline - time.monotonic())
            thread.join(timeout=remaining)
        alive = [thread.name for thread in self._worker_threads
                 if thread.is_alive()]
        if alive:
            self.get_logger().error(
                'Bridge worker shutdown timed out: ' + ', '.join(alive))
        self._workers_stopped = True
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolBotBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
