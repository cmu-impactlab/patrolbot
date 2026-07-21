import math
import queue
import socket
import struct
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from patrolbot_bridge.control_protocol import (
    COMMAND_CHARGE_RELEASE,
    COMMAND_MOTOR_ENABLE,
    ControlReply,
    format_request,
    parse_reply,
    valid_request_id,
)
from patrolbot_bridge.safety_telemetry import (
    battery_diagnostic,
    BATTERY_MIN_OPERATING_VOLTS,
    BATTERY_RECHARGE_VOLTS,
    decode_fault_flags,
)
from patrolbot_interfaces.msg import BaseState
from patrolbot_interfaces.srv import ChargeRelease, MotorEnable
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, LaserScan, PointCloud2, PointField
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

        self.declare_parameter('server_ip', '10.0.0.1')
        self.declare_parameter('server_port', 7272)
        self.declare_parameter('connect_timeout', 5.0)
        self.declare_parameter('receive_timeout', 3.0)
        self.declare_parameter('reconnect_delay', 3.0)
        self.declare_parameter('control_timeout', 6.0)
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

        # Laser returns closer than this are dropped (footprint-clearance filter).
        # Set just ABOVE the 0.22 m robot_radius so the only returns removed are
        # ones at/inside the robot's own body envelope (true self-occlusion, or an
        # object already in collision) — these can never be a navigable obstacle
        # and, left in, they sit permanently inside the footprint and deadlock
        # DWB's ObstacleFootprintCritic ("No valid trajectories out of 20").
        # 0.25 = 0.22 footprint + 0.03 margin for the laser's 0.037 m forward
        # mount offset. DO NOT raise much higher: real obstacles can legitimately
        # be at 0.25-0.40 m (verified live 2026-06-27 — symmetric ~0.25 m returns
        # that looked like self-occlusion turned out to be REAL nearby objects;
        # they vanished when the robot was repositioned). A larger value would
        # blind the robot to real close obstacles.
        self.SCAN_RANGE_MIN = 0.25

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

        # The stall word's physical bumper mapping is unresolved on this robot.
        # Keep it invalid until controlled testing supplies explicit masks.
        self.declare_parameter('bumpers_mapped', False)
        self.declare_parameter('front_bumper_mask', 0)
        self.declare_parameter('rear_bumper_mask', 0)

        self._control_lock = threading.Lock()
        self._pending_controls = {}
        self.CONTROL_TIMEOUT = float(
            self.get_parameter('control_timeout').value)

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
        with self._pose_lock:
            if not self._pose_received:
                return
            x, y, th = self._px, self._py, self._pth

        q = get_quaternion_from_euler(th)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

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
                with self._state_lock:
                    self._session_generation += 1
                    self._link_connected = True
                    self._charge_state_at = 0.0
                    self._flags_at = 0.0
                    session = self._session_generation
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
                    session = self._session_generation
                self._fail_pending_controls(
                    session, 'hardware session disconnected')
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
                        self._aux_queue, (line, received_at), 'auxiliary')
                elif line.startswith('ACK:') or line.startswith('RESULT:'):
                    self._handle_control_reply(line)
                else:
                    self._queue_latest(
                        self._nav_queue, (line, received_at), 'navigation')

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
                line, received_at = source_queue.get(timeout=0.5)
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
            parser(line)

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

    def _parse_telemetry(self, line):
        try:
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

            with self._pose_lock:
                self._px, self._py, self._pth = x, y, th
                self._vx, self._vth = vx, vth
                self._pose_received = True
                self._pose_at = time.monotonic()

            q = get_quaternion_from_euler(th)
            stamp = self.get_clock().now().to_msg()

            # /odom — accurate timestamp from current clock
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
            self._publish_with_timing(self.odom_pub, odom_msg, '/odom')

            # /scan — TF timer has been publishing 50 Hz in parallel, so the
            # TF buffer already has entries before this scan is processed.
            # Returns below SCAN_RANGE_MIN are dropped to +inf (invalid): the SICK
            # grazes the robot's own frame near -75 deg (~0.15 m, fixed self-
            # occlusion), which otherwise paints a permanent phantom obstacle
            # inside the 0.22 m footprint and blocks Nav2. Anything closer than the
            # robot radius is inside the footprint anyway.
            raw_ranges = []
            ranges = []
            for r in laser_fields:
                r = r.strip()
                if r:
                    try:
                        val = float(r)
                        raw_ranges.append(
                            val if math.isfinite(val) and val >= 0.0
                            else float('inf'))
                        ranges.append(
                            val if math.isfinite(val) and
                            val >= self.SCAN_RANGE_MIN else float('inf'))
                    except ValueError:
                        pass

            if ranges:
                scan_msg = self._laser_scan_message(
                    stamp, ranges, self.SCAN_RANGE_MIN)
                self._publish_with_timing(self.scan_pub, scan_msg, '/scan')
                if self.unfiltered_scan_pub is not None:
                    candidate = self._laser_scan_message(
                        stamp, raw_ranges, 0.0)
                    self._publish_with_timing(
                        self.unfiltered_scan_pub, candidate,
                        self.unfiltered_scan_topic)

        except Exception:
            pass

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

    def _parse_aux(self, line):
        # AUX:SONAR=x,y;x,y;...|BATT=v,soc,cs,temp|FLAGS=flags,fault,stall,motors,estop
        # Each section is parsed and published in isolation: a malformed or missing
        # section only skips its own topic and never affects the others or nav data.
        sections = {}
        for part in line[4:].split('|'):
            if '=' in part:
                key, val = part.split('=', 1)
                sections[key] = val
        stamp = self.get_clock().now().to_msg()

        if 'SONAR' in sections:
            try:
                self._publish_sonar(sections['SONAR'], stamp)
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

        msg = BatteryState()
        msg.header.stamp = stamp
        msg.voltage = volt
        msg.percentage = (soc / 100.0) if soc >= 0.0 else float('nan')
        msg.temperature = temp if temp > -100.0 else float('nan')
        msg.present = True
        # ARIA charge state >0 = charging/charged on the dock; <=0 = on battery.
        if charge_state > 0:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self._publish_with_timing(self.battery_pub, msg, '/battery')

        level, message = battery_diagnostic(volt, charge_state)
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

        telemetry_age = max(charge_age, flags_age)
        state_valid = connected and telemetry_age <= 0.5
        bumpers_mapped = bool(
            self.get_parameter('bumpers_mapped').value)
        front_mask = int(self.get_parameter('front_bumper_mask').value)
        rear_mask = int(self.get_parameter('rear_bumper_mask').value)
        bumpers_valid = (
            state_valid and bumpers_mapped and front_mask != 0 and rear_mask != 0)

        msg = BaseState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.session_generation = generation
        msg.link_connected = connected
        msg.telemetry_age = float(
            telemetry_age if math.isfinite(telemetry_age) else 1e6)
        msg.hardware_state_valid = state_valid
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
        self._publish_with_timing(
            self.base_state_pub, msg, '/patrolbot/base_state')

    # ------------------------------------------------------------------
    # Idempotent, zero-motion hardware-control services
    # ------------------------------------------------------------------

    def _charge_release_callback(self, request, response):
        return self._control_service(
            COMMAND_CHARGE_RELEASE, request.request_id, response)

    def _motor_enable_callback(self, request, response):
        return self._control_service(
            COMMAND_MOTOR_ENABLE, request.request_id, response)

    def _control_service(self, command, request_id, response):
        started = time.monotonic()
        response.request_id = request_id
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
        with self._pose_lock:
            linear = self._vx
            angular = self._vth
            odom_age = started - self._pose_at if self._pose_at else math.inf

        if not connected:
            return self._fill_control_response(
                response, None, 2, 'hardware link unavailable', started)
        if max(charge_age, flags_age) > 0.5:
            return self._fill_control_response(
                response, None, 3, 'hardware telemetry is stale', started)
        if odom_age > 0.5:
            return self._fill_control_response(
                response, None, 3, 'odometry is stale', started)
        if abs(linear) > 0.005 or abs(angular) > 0.01:
            return self._fill_control_response(
                response, None, 3, 'robot must be stationary', started)
        if fault_flags:
            return self._fill_control_response(
                response, None, 3, 'hardware fault flags are nonzero', started)
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
        if command == COMMAND_MOTOR_ENABLE:
            if charge_state != 0:
                return self._fill_control_response(
                    response, None, 3, 'motor enable refused while charging',
                    started)
            if estop:
                return self._fill_control_response(
                    response, None, 3, 'motor enable refused while E-stop asserted',
                    started)

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
            with self.sock_lock:
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
        # request_id is initialized by _control_service before any return.
        return response

    # ------------------------------------------------------------------
    # Velocity forwarding
    # ------------------------------------------------------------------

    def cmd_vel_callback(self, msg):
        try:
            # Serialize all writes so DRIVE bytes can never interleave with an
            # idempotent hardware-control request on the TCP stream.
            with self.sock_lock:
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
