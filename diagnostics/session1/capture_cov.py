import csv, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped

DURATION = 60.0

class Cap(Node):
    def __init__(self):
        super().__init__("session1_cov_capture")
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.rows = []
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.cb, qos)
    def cb(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        c = msg.pose.covariance
        self.rows.append([
            "%d.%09d" % (msg.header.stamp.sec, msg.header.stamp.nanosec),
            "%.6f" % p.position.x, "%.6f" % p.position.y, "%.6f" % yaw,
            "%.6e" % c[0], "%.6e" % c[7], "%.6e" % c[35],
            "%.6e" % c[1], "%.6e" % c[6], "%.6e" % c[5], "%.6e" % c[30],
        ])
        self.get_logger().info("sample %d cov0=%.6e cov7=%.6e cov35=%.6e"
                               % (len(self.rows), c[0], c[7], c[35]))

rclpy.init()
n = Cap()
t0 = time.monotonic()
while time.monotonic() - t0 < DURATION:
    rclpy.spin_once(n, timeout_sec=0.5)
with open("/home/ubuntu/session1_capture/amcl_cov_before.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["stamp","x","y","yaw","cov0_xx","cov7_yy","cov35_yawyaw",
                "cov1_xy","cov6_yx","cov5_xyaw","cov30_yawx"])
    w.writerows(n.rows)
print("samples: %d" % len(n.rows))
neg = [r for r in n.rows if float(r[4])<=0 or float(r[5])<=0 or float(r[6])<=0]
print("non-positive-diagonal samples: %d" % len(neg))
if neg:
    print("first offender:", neg[0])
rclpy.shutdown()
