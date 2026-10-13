"""Stationary seed must not consume scans captured before it; isolated ROS only."""
import json, math, os, signal, subprocess, time
from pathlib import Path
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from lifecycle_msgs.srv import ChangeState
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
assert os.environ.get('PATROLBOT_ISOLATED_TEST') == '1'
assert {p.name for p in Path('/sys/class/net').iterdir()} == {'lo'}
log_path=Path('/tmp/amcl-seed-scan.log'); process=None
rclpy.init();node=rclpy.create_node('isolated_seed_scan_barrier')
def ns(stamp):return stamp.sec*10**9+stamp.nanosec
poses=[];acks=[]
node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',poses.append,10)
node.create_subscription(PoseWithCovarianceStamped,'/amcl/accepted_initial_pose',acks.append,10)
seed_pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
scan_pub=node.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
map_pub=node.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
tf_pub=TransformBroadcaster(node);static_pub=StaticTransformBroadcaster(node)
tf=TransformStamped();tf.header.frame_id='base_link';tf.child_frame_id='laser_frame';tf.transform.rotation.w=1.;static_pub.sendTransform(tf)
def spin(seconds,scan_stamp=None):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='base_link';tf.header.stamp=node.get_clock().now().to_msg();tf.transform.rotation.w=1.;tf_pub.sendTransform(tf)
  if scan_stamp is not None:
   scan=LaserScan();scan.header.frame_id='laser_frame';scan.header.stamp=scan_stamp
   scan.angle_min=-math.pi/2;scan.angle_max=math.pi/2;scan.angle_increment=math.pi/180
   scan.range_min=.05;scan.range_max=20.;scan.ranges=[4.5]*181;scan_pub.publish(scan)
  rclpy.spin_once(node,timeout_sec=.02)
def transition(number):
 client=node.create_client(ChangeState,'/amcl/change_state');assert client.wait_for_service(timeout_sec=10)
 req=ChangeState.Request();req.transition.id=number;future=client.call_async(req)
 deadline=time.monotonic()+10
 while not future.done() and time.monotonic()<deadline:spin(.02)
 assert future.done() and future.result().success,number
 node.destroy_client(client)
try:
 with log_path.open('w') as log:
  process=subprocess.Popen(['ros2','run','nav2_amcl','amcl','--ros-args','--params-file','/opt/patrolbot_ws/src/patrolbot_navigation/config/nav2_params.yaml'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  transition(1);transition(3)
  grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=.1;grid.info.width=grid.info.height=100;grid.info.origin.orientation.w=1.
  grid.data=[100 if x in (0,99) or y in (0,99) else 0 for y in range(100) for x in range(100)]
  map_pub.publish(grid);spin(1.)
  assert seed_pub.get_subscription_count()==1
  seed=PoseWithCovarianceStamped();seed.header.frame_id='map';seed.header.stamp=node.get_clock().now().to_msg()
  seed.pose.pose.position.x=seed.pose.pose.position.y=5.;seed.pose.pose.orientation.w=1.
  seed.pose.covariance[0]=seed.pose.covariance[7]=.0009;seed.pose.covariance[35]=.00030461742
  seed_pub.publish(seed);spin(.3)
  assert any(ns(m.header.stamp)==ns(seed.header.stamp) for m in acks),'seed was not accepted'
  applied=[json.loads(line.split('Applied initial pose: ',1)[1]) for line in log_path.read_text().splitlines() if 'Applied initial pose: ' in line]
  assert len(applied)==1,'missing or duplicate pf_init application diagnostic'
  diagnostic=applied[0]
  assert diagnostic['request_stamp_ns']==ns(seed.header.stamp),'diagnostic lost original request correlation'
  assert diagnostic['application_stamp_ns']>=ns(seed.header.stamp)
  assert diagnostic['mean']==[5.,5.,0.] and diagnostic['covariance']==[.0009,0.,0.,0.,.0009,0.,0.,0.,.00030461742]
  assert math.hypot(diagnostic['sample_mean'][0]-5.,diagnostic['sample_mean'][1]-5.)<=.01
  assert abs(diagnostic['sample_mean'][2])<=math.radians(.3)
  assert diagnostic['within_envelope']/diagnostic['sample_count']>=.99
  assert diagnostic['sample_count']==4000,'unexpected initialization particle count'
  assert list(acks[0].pose.covariance)==list(seed.pose.covariance),'ACK changed original seed covariance'
  old=(rclpy.time.Time.from_msg(seed.header.stamp)-Duration(seconds=.15)).to_msg()
  spin(.4,old)
  assert not poses, 'pre-seed scan consumed the first stationary update: '+str([ns(m.header.stamp)-ns(seed.header.stamp) for m in poses])
  spin(.2,seed.header.stamp)
  assert not poses, 'scan at seed timestamp must not satisfy the post-seed barrier'
  fresh=node.get_clock().now().to_msg();spin(.8,fresh)
  assert poses and all(ns(m.header.stamp)>ns(seed.header.stamp) for m in poses),'no post-seed estimate without motion or refresh'
  print(json.dumps({'passed':['pre_seed_scan_rejected','equal_stamp_scan_rejected','stationary_post_seed_estimate_without_refresh'],'seed_ns':ns(seed.header.stamp),'pose_stamps':[ns(m.header.stamp) for m in poses],'applied_seed':diagnostic}),flush=True)
finally:
 if process is not None and process.poll() is None:
  os.killpg(process.pid,signal.SIGINT)
  try:process.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
 print(log_path.read_text() if log_path.exists() else '',flush=True)
 node.destroy_node();rclpy.try_shutdown()
