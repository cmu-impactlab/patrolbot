"""NEW actual Jazzy DDS/Trigger test. Inert epoch/plant; no robot connection.

Run only in network-none/private-IPC nonproduction container. Verifies genuine
callback dictionaries and graph endpoint GIDs, real services/subscriptions,
installed bridge methods, and zero/denial across invalidated offers.
Not the original native 22 scenarios; not physical acceptance.
"""
import json
import os
import time
from types import SimpleNamespace as NS
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from patrolbot_bridge.manual_active import ManualDrain
from patrolbot_bridge.bridge_node import PatrolBotBridge
import test_manual_handshake as contracts

assert os.environ.get('ROS_DOMAIN_ID') == '187'
rclpy.init()
f=contracts.Fixture(); f.close()  # initialize inert collaborators; UNPATCH real time
b=f.b
Node.__init__(b,'patrolbot_bridge')
del b.get_clock, b.get_publishers_info_by_topic
b._manual_context=b._manual_context_key()
b._manual_drain=ManualDrain()
records=[]
b.sock=NS(sendall=lambda data:records.append(data.decode().strip()))
joy=Node('patrolbot_joy_teleop'); mux=Node('cmd_vel_mux'); final=Node('teleop_velocity_smoother')
jp=joy.create_publisher(Twist,'/input/joy',10)
bp=joy.create_publisher(Bool,'/patrolbot/manual_active',10)
mp=mux.create_publisher(Twist,'/cmd_vel_out',10)
fp=final.create_publisher(Twist,'/cmd_vel',10)
metadata=[]
def observe(message,info):
    metadata.append(dict(info)); b.cmd_vel_callback(message,info)
b.create_subscription(Twist,'/input/joy',b._manual_joy_cb,10)
b.create_subscription(Twist,'/cmd_vel_out',b._manual_mux_cb,10)
b.create_subscription(Twist,'/cmd_vel',observe,10)
b.create_subscription(Bool,'/patrolbot/manual_active',b._manual_active_cb,10)
reply_delay=[0.]
def ready_service(request,response):
    if reply_delay[0]: time.sleep(reply_delay[0])
    return b._manual_ready_cb(request,response)
b.create_service(Trigger,'/patrolbot/manual_ready',ready_service)
client=joy.create_client(Trigger,'/patrolbot/manual_ready')
executor=SingleThreadedExecutor()
for node in (b,joy,mux,final): executor.add_node(node)
def spin(seconds):
    until=time.monotonic()+seconds
    while time.monotonic()<until: executor.spin_once(timeout_sec=min(.001,max(0.,until-time.monotonic())))
def request():
    start=time.monotonic(); future=client.call_async(Trigger.Request())
    while not future.done() and time.monotonic()-start < .2: executor.spin_once(timeout_sec=.001)
    assert future.done(),'actual Trigger did not complete'
    return future.result(),time.monotonic()-start
def zeros(seconds=.57):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        jp.publish(Twist()); mp.publish(Twist()); fp.publish(Twist()); spin(.02)
def count(): return sum(row!='DRIVE:0.0:0.0' for row in records)
summary={'new_graph_checks':[], 'physical_acceptance':False,'original_22_rerun':0}
try:
    assert client.wait_for_service(timeout_sec=2.)
    spin(.5); zeros()
    response,latency=request(); assert response.success,response.message
    assert latency < 1./30.,f'Trigger roundtrip exceeded one tick: {latency}'
    moving=Twist(); moving.linear.x=.1
    fp.publish(moving); spin(.005)
    assert count()==0,'readiness query alone opened motion'
    summary['new_graph_checks'].append('service_only_no_motion')
    # A current Bool consumes the offer. Renew after the read-only probe above.
    response,_=request(); assert response.success
    bp.publish(Bool(data=True)); spin(.005)
    response,_=request(); assert response.success and response.message.startswith('manual-owned-v1:')
    jp.publish(moving); mp.publish(moving); spin(.003)
    fp.publish(moving); spin(.005)
    assert count()==1,('positive current handoff did not forward exactly one',records[-8:])
    summary['new_graph_checks'].append('current_trigger_bool_one_accepted_drive')
    bp.publish(Bool(data=False)); spin(.005)
    assert records[-1]=='DRIVE:0.0:0.0','release did not stop existing socket'
    fp.publish(moving); spin(.005); assert count()==1
    summary['new_graph_checks'].append('false_release_immediate_same_socket_stop')
    zeros(); response,_=request(); assert response.success
    spin(.04); bp.publish(Bool(data=True)); spin(.005)
    fp.publish(moving); spin(.005); assert count()==1
    summary['new_graph_checks'].append('expired_offer_no_nonzero')
    bp.publish(Bool(data=False)); spin(.003); zeros()
    response,_=request(); assert response.success
    b._epoch._manual_mutation_generation=1  # accepted/settled intervening owner
    bp.publish(Bool(data=True)); spin(.005)
    fp.publish(moving); spin(.005); assert count()==1
    summary['new_graph_checks'].append('intervening_owner_generation_no_nonzero')
    for change in ('unknown_write','action_owner','lidar','raw_origin'):
        bp.publish(Bool(data=False)); spin(.003); zeros()
        b._manual_context=b._manual_context_key()
        b._epoch.local_motion_mutations_settled=True; b._undock_turn.actions={}
        b._lidar_ready=lambda:True; b._manual_raw_ready=lambda:True
        if change=='unknown_write': b._epoch.local_motion_mutations_settled=False
        if change=='action_owner': b._undock_turn.actions={'dock':True}
        if change=='lidar': b._lidar_ready=lambda:False
        if change=='raw_origin': b._manual_raw_ready=lambda:False
        response,_=request(); assert not response.success,change
        bp.publish(Bool(data=True)); fp.publish(moving); spin(.005); assert count()==1,change
        fp.publish(Twist()); spin(.003); assert records[-1]=='DRIVE:0.0:0.0'
        summary['new_graph_checks'].append(change+'_no_nonzero_zero_persists')
    assert metadata and isinstance(metadata[0],dict)
    assert 'source_timestamp' in metadata[0]
    assert 'publication_sequence_number' in metadata[0]
    assert not any('gid' in key.lower() for key in metadata[0])
    summary['actual_jazzy_message_info_keys']=sorted(metadata[0])
    summary['actual_jazzy_message_info_sample']=metadata[0]
    summary['endpoint_gids']={topic:[list(i.endpoint_gid) for i in b.get_publishers_info_by_topic(topic)]
                              for topic in ('/input/joy','/patrolbot/manual_active','/cmd_vel_out','/cmd_vel')}
    summary['socket_nonzero_count']=count(); summary['socket_zero_count']=len(records)-count()
    summary['first_trigger_roundtrip_s']=latency
    # Exercise the actual installed teleop tick/client as well, not just a
    # hand-issued Bool. This is a NEW native integration test, not an old case.
    bp.publish(Bool(data=False)); spin(.005)
    b._epoch.local_motion_mutations_settled=True; b._undock_turn.actions={}
    b._lidar_ready=lambda:True; b._manual_raw_ready=lambda:True
    t=f.t
    t.get_clock=joy.get_clock
    t.get_publishers_info_by_topic=joy.get_publishers_info_by_topic
    t.pub=jp; t.manual_pub=bp; t.bridge_ready=client
    t._manual_drain=ManualDrain()
    t._bridge_ready_future=t._bridge_ready_context=None
    t.lin_cmd=t.ang_cmd=0.; t.lin_tgt=.3; t.ang_tgt=0.
    latest=[Twist()]
    joy.create_subscription(Twist,'/input/joy',lambda msg:latest.__setitem__(0,msg),10)
    joy.create_subscription(Twist,'/cmd_vel_out',
                            lambda msg,info:t._drain_output_cb('/cmd_vel_out','mux',msg,info),10)
    joy.create_subscription(Twist,'/cmd_vel',
                            lambda msg,info:t._drain_output_cb('/cmd_vel','final',msg,info),10)
    before=count(); next_tick=time.monotonic()
    for _ in range(120):
        spin(max(0.,next_tick-time.monotonic()))
        t._last_joy=joy.get_clock().now().nanoseconds/1e9
        t.tick()
        spin(.002)
        mp.publish(latest[0]); fp.publish(latest[0]); spin(.002)
        next_tick+=t.dt
    summary['actual_teleop_120_ticks_nonzero_delta']=count()-before
    assert count()-before>=60,('actual teleop handoff starved on real Trigger graph',count()-before)
    summary['new_graph_checks'].append('actual_installed_teleop_client_tick_completes_handoff')
    # Actual delayed ROS service, not a mocked Future. Even a fresh server stamp
    # cannot rescue a response whose arrival exceeded the request deadline.
    spin(.01); reply_delay[0]=.04
    t._last_joy=joy.get_clock().now().nanoseconds/1e9; t.tick()
    spin(.055)
    t._last_joy=joy.get_clock().now().nanoseconds/1e9; t.tick()
    assert t.lin_cmd==0. and t.ang_cmd==0.,'delayed actual Trigger reply ramped'
    spin(.055)
    assert not b._manual_asserted,'delayed actual Trigger retained true Bool'
    denied_before=count(); fp.publish(moving); spin(.005)
    assert count()==denied_before,'delayed actual Trigger opened nonzero socket'
    fp.publish(Twist()); spin(.005); assert records[-1]=='DRIVE:0.0:0.0'
    summary['new_graph_checks'].append('actual_delayed_trigger_zero_false_no_socket_nonzero')
    print(json.dumps(summary,indent=2))
finally:
    for node in (b,joy,mux,final):
        # Do not call production Bridge.destroy_node: no production workers were
        # constructed. These are DDS harness nodes only.
        Node.destroy_node(node)
    executor.shutdown(); rclpy.shutdown()
