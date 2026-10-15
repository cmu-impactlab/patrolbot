"""Bounded read-only ROS evidence recorder and analyzer-input conversion.

No publishers, services, actions, or robot commands. Run beside the deployed
ROS graph; keep the output outside the source checkout. Missing inputs remain
explicit, and no dashboard request ID is inferred from a ROS pose stamp.
"""
import argparse
import json
import math
from pathlib import Path
import time


def stamp_ns(message):
    stamp=message['header']['stamp']
    return stamp['sec']*1_000_000_000+stamp['nanosec']


def yaw(q):
    return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))


def planar_pose(message):
    pose=message['pose']['pose']
    return [pose['position']['x'],pose['position']['y'],yaw(pose['orientation'])]


def clean(value):
    if isinstance(value,float) and not math.isfinite(value): return None
    if isinstance(value,dict): return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [clean(v) for v in value]
    return value


def convert(records,identity,request=None):
    latest={};selected=None;seed=None;ack=None;applied=None;poses=[]
    for record in records:
        topic=record.get('topic');message=record.get('message',{})
        if topic=='/initialpose': seed=message;ack=None;applied=None
        if topic=='/amcl/accepted_initial_pose': ack=message
        if topic=='/rosout' and 'Applied initial pose' in message.get('msg',''): applied=message
        if topic=='/amcl_pose': poses.append(planar_pose(message))
        if topic: latest[topic]=message
        if topic=='/scan':
            selected=(record,dict(latest),seed,ack,applied,list(poses))
    # Stationary AMCL need not update on every scan; pair by acquisition stamp,
    # including an estimate delivered after its scan callback.
    estimate=latest.get('/amcl_pose')
    if estimate:
        matched=next((r for r in reversed(records) if r.get('topic')=='/scan'
                      and stamp_ns(r['message'])==stamp_ns(estimate)),None)
        if matched and (seed is None or stamp_ns(estimate)>stamp_ns(seed)):
            odometry=[r['message'] for r in records if r.get('topic')=='/odom']
            paired=dict(latest)
            if odometry:
                paired['/odom']=min(odometry,key=lambda m:abs(stamp_ns(m)-stamp_ns(estimate)))
            selected=(matched,paired,seed,ack,applied,list(poses))
    result={'complete':False,'missing':[],'map':identity,'identity':{
        'command_id':request.get('data',request).get('command_id') if request else None,
        'dashboard_request':request,'command_note':'Exact request supplied separately' if request else
        'ROS initialpose mean/covariance only; no dashboard command ID available'}}
    if selected:
        record,latest,seed,ack,applied,poses=selected
    for name,value in [('seed',seed),('accepted_seed',ack),('applied_seed',applied),
                       ('scan',selected),('amcl_pose',latest.get('/amcl_pose')),
                       ('odom',latest.get('/odom')),('particles',latest.get('/particle_cloud')),
                       ('active_recovery_state',latest.get('/patrolbot/base_state'))]:
        if value is None:result['missing'].append(name)
    if not selected:
        if request is None:result['missing'].append('exact_dashboard_request')
        return result
    scan=record['message'];transforms=record.get('transforms',[])
    laser=next((t for t in transforms if t['parent']=='base_link' and t['child']=='laser_frame'),None)
    result['scan']={k:scan[k] for k in ('angle_min','angle_increment','ranges','range_min','range_max') if k in scan}
    if laser:
        result['laser_transform']=laser['pose']
        # A pi roll reverses laser Y; yaw alone cannot represent that transform.
        qx,qy,qz,qw=laser['quaternion']
        roll=math.atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy))
        pitch=math.asin(max(-1,min(1,2*(qw*qy-qz*qx))))
        if abs(pitch)>.01 or min(abs(roll),abs(abs(roll)-math.pi))>.01:
            result['missing'].append('planar_laser_transform')
        elif abs(roll)>math.pi/2:
            result['scan']['angle_min']*=-1;result['scan']['angle_increment']*=-1
    else:result['missing'].append('laser_transform')
    if seed:result.update(supplied_pose=planar_pose(seed),supplied_covariance=seed['pose']['covariance'])
    if request is None:
        result['missing'].append('exact_dashboard_request')
    else:
        body=request.get('data',request)
        goal=body.get('goal',{})
        requested=[goal.get('x'),goal.get('y'),goal.get('yaw',0.)]
        if (body.get('command')!='set_initial_pose' or
                not all(isinstance(v,(float,int)) and math.isfinite(v) for v in requested)):
            result['missing'].append('valid_dashboard_request')
        else:
            result['supplied_pose']=requested
            if seed:
                published=planar_pose(seed)
                if (math.dist(requested[:2],published[:2])>1e-6 or
                        abs(math.remainder(requested[2]-published[2],2*math.pi))>1e-6):
                    result['missing'].append('request_seed_mismatch')
            if goal.get('map_id')!=identity['map_id'] or goal.get('map_revision')!=identity['revision']:
                result['missing'].append('matching_request_map')
    if latest.get('/amcl_pose'):
        result['amcl_pose']=planar_pose(latest['/amcl_pose'])
        if seed and stamp_ns(latest['/amcl_pose'])<=stamp_ns(seed):
            result['missing'].append('scan_derived_pose_after_seed')
    result['evidence']={'now_ns':record['now_ns'],'scan_stamp_ns':stamp_ns(scan),
        'transforms':transforms,'poses':poses}
    if latest.get('/amcl_pose'):
        result['evidence']['amcl_stamp_ns']=stamp_ns(latest['/amcl_pose'])
        if result['evidence']['amcl_stamp_ns']!=stamp_ns(scan):
            result['missing'].append('amcl_scan_timestamp_mismatch')
    if latest.get('/odom'):result['evidence']['odom_stamp_ns']=stamp_ns(latest['/odom'])
    if len(transforms)!=4:result['missing'].append('timestamped_tf_chain')
    if seed and ack and stamp_ns(seed)!=stamp_ns(ack):result['missing'].append('matching_seed_ack')
    if applied:
        try:
            diagnostic=json.loads(applied['msg'].split('Applied initial pose:',1)[1].strip())
            result['applied_seed']=diagnostic
            if seed and diagnostic.get('request_stamp_ns')!=stamp_ns(seed):
                result['missing'].append('matching_applied_seed')
        except (ValueError,KeyError):result['missing'].append('structured_applied_seed')
    state=latest.get('/patrolbot/base_state')
    if state and (state.get('map_id')!=identity['map_id'] or state.get('map_revision')!=identity['revision']):
        result['missing'].append('matching_active_map')
    result['seed_ack']=ack;result['applied_diagnostic']=applied
    result['state']=latest.get('/patrolbot/base_state')
    result['complete']=not result['missing']
    return result


def record_ros(args,identity):
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from rclpy.time import Time
    from rosidl_runtime_py.convert import message_to_ordereddict
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from nav2_msgs.msg import ParticleCloud
    from sensor_msgs.msg import LaserScan
    from diagnostic_msgs.msg import DiagnosticArray
    from patrolbot_interfaces.msg import BaseState
    from rcl_interfaces.msg import Log
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformListener
    rclpy.init();node=Node('localization_read_only_capture')
    buffer=Buffer(cache_time=Duration(seconds=20));listener=TransformListener(buffer,node)
    records=[];pending_scans=[]
    output=args.output.open('w')
    def receive(topic,message,lookup_ready=False):
        if topic=='/scan' and not lookup_ready:
            pending_scans.append((time.monotonic(),message));return
        item={'topic':topic,'now_ns':node.get_clock().now().nanoseconds,
              'received_monotonic_ns':time.monotonic_ns(),'message':message_to_ordereddict(message)}
        if topic=='/scan':
            item['transforms']=[];item['tf_errors']=[]
            for parent,child in [('map','odom'),('odom','base_link'),('base_link','laser_frame'),('map','base_link')]:
                try:
                    tf=buffer.lookup_transform(parent,child,Time.from_msg(message.header.stamp))
                    t=tf.transform.translation;q=tf.transform.rotation
                    item['transforms'].append({'parent':parent,'child':child,
                        'stamp_ns':tf.header.stamp.sec*1_000_000_000+tf.header.stamp.nanosec,
                        'static':parent=='base_link' and child=='laser_frame' and tf.header.stamp.sec==0 and tf.header.stamp.nanosec==0,
                        'pose':[t.x,t.y,yaw({'x':q.x,'y':q.y,'z':q.z,'w':q.w})],
                        'quaternion':[q.x,q.y,q.z,q.w]})
                except Exception as exc:item['tf_errors'].append({'parent':parent,'child':child,'error':str(exc)})
        item=clean(item)
        output.write(json.dumps(item,allow_nan=False)+'\n');output.flush()
        # Raw particle arrays stay on disk; replay conversion only needs their presence.
        if topic=='/particle_cloud':
            item={**item,'message':{'header':item['message']['header'],
                'particle_count':len(item['message'].get('particles',[]))}}
        records.append(item)
    def drain_scans():
        while pending_scans and time.monotonic()-pending_scans[0][0]>=.12:
            _,message=pending_scans.pop(0)
            receive('/scan',message,True)
    scan_timer=node.create_timer(.025,drain_scans)
    subscriptions=[]
    topics=[('/initialpose',PoseWithCovarianceStamped),('/amcl/accepted_initial_pose',PoseWithCovarianceStamped),
            ('/amcl_pose',PoseWithCovarianceStamped),('/particle_cloud',ParticleCloud),('/scan',LaserScan),
            ('/scan_unfiltered_candidate',LaserScan),('/odom',Odometry),('/diagnostics',DiagnosticArray),
            ('/patrolbot/base_state',BaseState),('/rosout',Log),('/tf',TFMessage),('/tf_static',TFMessage)]
    for topic,kind in topics:
        qos=QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic=='/tf_static' else qos_profile_sensor_data
        subscriptions.append(node.create_subscription(kind,topic,lambda message,t=topic:receive(t,message),qos))
    deadline=time.monotonic()+args.seconds
    try:
        while rclpy.ok() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        output.close();node.destroy_node();rclpy.try_shutdown()
    return records


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=int,default=180,choices=range(120,301),metavar='120..300')
    parser.add_argument('--output',type=Path,required=True,help='raw JSONL output')
    parser.add_argument('--analyzer-input',type=Path,required=True)
    parser.add_argument('--map-yaml',type=Path,required=True)
    parser.add_argument('--map-id',required=True);parser.add_argument('--map-revision',required=True)
    parser.add_argument('--request-json',type=Path);parser.add_argument('--replay',type=Path)
    args=parser.parse_args()
    from geometry import MapRaster
    identity=MapRaster(args.map_yaml,args.map_id,args.map_revision).identity
    records=[json.loads(line) for line in args.replay.read_text().splitlines()] if args.replay else record_ros(args,identity)
    request=json.loads(args.request_json.read_text()) if args.request_json else None
    result=convert(records,identity,request)
    args.analyzer_input.write_text(json.dumps(clean(result),indent=2,allow_nan=False)+'\n')
    print(json.dumps({'complete':result['complete'],'missing':result['missing'],'records':len(records)}))

if __name__=='__main__':main()
