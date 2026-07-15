import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_ros import TransformBroadcaster
import socket
import threading
import time
import math
import struct

def get_quaternion_from_euler(yaw):
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

class PatrolBotBridge(Node):
    def __init__(self):
        super().__init__('patrolbot_bridge')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Auxiliary telemetry (sonar / battery / diagnostics). Parsed from a separate
        # AUX line, fully decoupled from the nav-critical /odom and /scan path so a
        # malformed reading here can never disturb navigation. See _parse_aux.
        self.sonar_pub = self.create_publisher(PointCloud2, '/sonar', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.server_ip = '10.0.0.1'
        self.server_port = 7272

        # The SBC streams telemetry at 20 Hz (every 50 ms). If no data arrives for
        # this long the link is dead — silent link loss can leave no FIN/RST, so
        # a blocking recv() would otherwise hang forever and never reconnect. A read
        # timeout converts that silent death into a socket.timeout we can recover from.
        self.RECV_TIMEOUT = 3.0

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
        self.data_buffer = ""

        # Latest pose, updated by receive thread, read by TF timer.
        self._pose_lock = threading.Lock()
        self._px, self._py, self._pth = 0.0, 0.0, 0.0
        self._vx, self._vth = 0.0, 0.0
        self._pose_received = False

        # Publish odom→base_link TF at 50 Hz from the spin thread.
        # This decouples TF from scan delivery — TF entries are always in the
        # buffer BEFORE any scan arrives at the costmap message_filter.
        self.create_timer(0.02, self._tf_timer)

        self.recv_thread = threading.Thread(target=self._connect_loop, daemon=True)
        self.recv_thread.start()

        self.get_logger().info(f'Bridge node started. Connecting to {self.server_ip}:{self.server_port}...')

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
                sock.settimeout(5.0)
                sock.connect((self.server_ip, self.server_port))
                # Detect a silently-dead peer at the OS level too (defence in depth).
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                # Read timeout (not blocking forever): recv() raises socket.timeout if
                # the SBC stops streaming, so _receive_loop can break and reconnect.
                sock.settimeout(self.RECV_TIMEOUT)
                with self.sock_lock:
                    self.sock = sock
                    self.data_buffer = ""
                self.get_logger().info('Connected to SBC server.')
                self._receive_loop(sock)
            except Exception as e:
                self.get_logger().warn(f'SBC connection failed: {e}. Retrying in 3s...')
            finally:
                with self.sock_lock:
                    if self.sock is sock:
                        self.sock = None
                try:
                    sock.close()
                except Exception:
                    pass
            if self.running:
                time.sleep(3)

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
                if line.startswith('AUX:'):
                    self._parse_aux(line)
                else:
                    self._parse_telemetry(line)

    # ------------------------------------------------------------------
    # Telemetry parsing — called from receive thread
    # ------------------------------------------------------------------

    def _parse_telemetry(self, line):
        try:
            if '|' not in line or 'ODOM:' not in line or 'LASER:' not in line:
                return

            parts = line.split('|')
            odom_fields = parts[0].replace('ODOM:', '').split(',')
            laser_fields = parts[1].replace('LASER:', '').strip().split(',')

            x   = float(odom_fields[0])
            y   = float(odom_fields[1])
            th  = float(odom_fields[2])
            vx  = float(odom_fields[3])
            vth = float(odom_fields[4])

            with self._pose_lock:
                self._px, self._py, self._pth = x, y, th
                self._vx, self._vth = vx, vth
                self._pose_received = True

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
            self.odom_pub.publish(odom_msg)

            # /scan — TF timer has been publishing 50 Hz in parallel, so the
            # TF buffer already has entries before this scan is processed.
            # Returns below SCAN_RANGE_MIN are dropped to +inf (invalid): the SICK
            # grazes the robot's own frame near -75 deg (~0.15 m, fixed self-
            # occlusion), which otherwise paints a permanent phantom obstacle
            # inside the 0.22 m footprint and blocks Nav2. Anything closer than the
            # robot radius is inside the footprint anyway.
            ranges = []
            for r in laser_fields:
                r = r.strip()
                if r:
                    try:
                        val = float(r)
                        ranges.append(val if val >= self.SCAN_RANGE_MIN else float('inf'))
                    except ValueError:
                        pass

            if ranges:
                scan_msg = LaserScan()
                scan_msg.header.stamp = stamp
                scan_msg.header.frame_id = 'laser_frame'
                scan_msg.angle_min = -math.pi / 2.0
                scan_msg.angle_max = math.pi / 2.0
                scan_msg.angle_increment = math.pi / (len(ranges) - 1)
                scan_msg.time_increment = 0.0
                scan_msg.scan_time = 0.05
                scan_msg.range_min = self.SCAN_RANGE_MIN
                scan_msg.range_max = 8.0
                scan_msg.ranges = ranges
                self.scan_pub.publish(scan_msg)

        except Exception:
            pass

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
        self.sonar_pub.publish(msg)

    def _publish_battery(self, data, stamp):
        f = data.split(',')
        volt = float(f[0])
        soc = float(f[1])
        charge_state = int(float(f[2]))
        temp = float(f[3])

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
        self.battery_pub.publish(msg)

    def _publish_flags(self, data, stamp):
        f = data.split(',')
        flags = int(f[0])
        fault = int(f[1])
        stall = int(f[2])
        motors = int(f[3])
        # e-stop field added 2026-06-28; tolerate older servers that omit it.
        estop = int(f[4]) if len(f) > 4 else 0

        # ARIA stall value: high byte = left wheel, low byte = right wheel.
        # bit 0 of each byte = motor stall (reliable). The remaining bits nominally
        # carry bumper segments, but on this PatrolBot-SH a constant high bit shows
        # even when idle/healthy (no bumpers wired / reserved bit), so they are
        # reported raw for reference only — they do NOT drive the alarm level.
        left_stall = bool(stall & 0x0100)
        right_stall = bool(stall & 0x0001)
        left_bumper_bits = (stall >> 9) & 0x7F
        right_bumper_bits = (stall >> 1) & 0x7F

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
            st.message = 'motor stall / fault'
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
            KeyValue(key='stall_value', value=str(stall)),
            KeyValue(key='left_motor_stall', value=str(left_stall)),
            KeyValue(key='right_motor_stall', value=str(right_stall)),
            KeyValue(key='left_bumper_bits_raw', value=str(left_bumper_bits)),
            KeyValue(key='right_bumper_bits_raw', value=str(right_bumper_bits)),
        ]
        arr = DiagnosticArray()
        arr.header.stamp = stamp
        arr.status = [st]
        self.diag_pub.publish(arr)

    # ------------------------------------------------------------------
    # Velocity forwarding
    # ------------------------------------------------------------------

    def cmd_vel_callback(self, msg):
        with self.sock_lock:
            s = self.sock
        if s is None:
            return
        try:
            s.sendall(f'DRIVE:{msg.linear.x}:{msg.angular.z}\n'.encode('utf-8'))
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.running = False
        with self.sock_lock:
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolBotBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
