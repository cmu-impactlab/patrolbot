"""NEW deterministic contracts, not the lost original 22-case native suite.

Offline uses actual candidate class bodies with inert ROS-shaped collaborators.
HANDSHAKE_INSTALLED=1 uses actual installed imports with the same tests.
"""
import ast
import importlib
import importlib.util
import math
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parent
SOURCE = Path(os.environ.get('HANDSHAKE_SOURCE', ROOT))
BRIDGE = SOURCE/'ros2_ws/src/patrolbot_bridge/patrolbot_bridge'
NAV = SOURCE/'ros2_ws/src/patrolbot_navigation/scripts'

class Twist:
    def __init__(self, x=0., z=0.):
        self.linear = NS(x=x, y=0., z=0.)
        self.angular = NS(x=0., y=0., z=z)

class Bool:
    def __init__(self, data=False): self.data = data

if os.environ.get('HANDSHAKE_INSTALLED'):
    import patrolbot_bridge.manual_active as manual
    from patrolbot_bridge.bridge_node import PatrolBotBridge as Bridge
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger
    import sys
    sys.path.insert(0, '/opt/undock_test_ws/install/patrolbot_navigation/lib/patrolbot_navigation')
    from patrolbot_joy_teleop import JoyTeleop as Teleop
else:
    spec = importlib.util.spec_from_file_location('candidate_manual', BRIDGE/'manual_active.py')
    manual = importlib.util.module_from_spec(spec); spec.loader.exec_module(manual)
    Trigger = NS(Request=NS)
    def load(path, name):
        tree = ast.parse(path.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
        cls.bases = []
        scope = dict(vars(manual), math=math, time=time, Twist=Twist, Bool=Bool,
                     Trigger=Trigger, ramp_toward=lambda a,b,c:a+max(-c,min(c,b-a)),
                     JOY_TIMEOUT=.5, NAV_CANCEL_RETRY=.25,
                     GoalStatus=NS(STATUS_ACCEPTED=1,STATUS_EXECUTING=2,STATUS_CANCELING=3))
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),str(path),'exec'),scope)
        return scope[name]
    Bridge = load(BRIDGE/'bridge_node.py', 'PatrolBotBridge')
    Teleop = load(NAV/'patrolbot_joy_teleop.py', 'JoyTeleop')

def twist(x=0., z=0.):
    t=Twist(); t.linear.x=x; t.angular.z=z; return t

class Future:
    def __init__(self, value=None, done=True, exc=None):
        self.value,self.complete,self.exc=value,done,exc
    def done(self): return self.complete
    def add_done_callback(self, callback):
        if self.complete: callback(self)
    def result(self):
        if self.exc: raise self.exc
        return self.value

class Fixture:
    def __init__(self):
        self.now=10.
        self.b=Bridge.__new__(Bridge)
        b=self.b
        b._epoch=NS(lock=threading.RLock(),guard=NS(session=1,epoch='origin',generation=1,ready=True),
                    recovery=NS(token=1,stage='ready',required=True),_token=1,_tf_generation=1,
                    local_motion_mutations_settled=True,_map_error='',undock_hold=False,
                    map_hold=True,maps=NS(stage='failed',hold=True,active={'map_id':'m'}),
                    _navigation_status={},navigation_idle_problem=lambda:'',commands_allowed=False)
        b._session_generation=1; b._motor_disable_generation=0
        b._manual_drain=manual.ManualDrain(); b._manual_writer_ids=None
        b._manual_context=b._manual_context_key()
        b._manual_asserted=False; b._manual_active_at=0.; b._manual_forwarded=False
        b._manual_offer=b._manual_handoff=None
        b._undock_turn=NS(busy=False,actions={},drive_allowed=False)
        b._state_lock=threading.RLock(); b.sock_lock=threading.RLock()
        b._undock_active=False; b.sock=Mock(); b._lidar_ready=lambda:True
        b._manual_raw_ready=lambda:True
        b.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(self.now*1e9)))
        names={'/input/joy':'patrolbot_joy_teleop','/patrolbot/manual_active':'patrolbot_joy_teleop',
               '/cmd_vel_out':'cmd_vel_mux','/cmd_vel':'teleop_velocity_smoother'}
        self.endpoints={k:[NS(node_name=v,node_namespace='/',endpoint_gid=bytes([i+1])*16,
                             topic_type='std_msgs/msg/Bool' if 'manual' in k else 'geometry_msgs/msg/Twist')]
                        for i,(k,v) in enumerate(names.items())}
        b.get_publishers_info_by_topic=lambda topic:self.endpoints.get(topic,[])
        self.t=Teleop.__new__(Teleop); t=self.t
        t._manual_drain=manual.ManualDrain(); t._takeover_generation=1
        t._lidar_generation=1; t._lidar_chord=NS(block_drive=False,reset=Mock())
        t._nav_status_received=True; t._nav_goal_active=False
        t._bridge_ready_future=t._bridge_ready_context=None
        t._bridge_ready_received=None
        t._takeover_pending=False; t._cancel_accepted=False; t._cancel_in_flight=False
        t._cancel_unavailable_logged=False; t._request_navigation_cancel=Mock(); t._try_lidar_toggle=Mock()
        t.active=True; t._last_joy=self.now; t.dt=1./30.
        t.lin_cmd=t.ang_cmd=0.; t.lin_tgt=.3; t.ang_tgt=0.
        t.accel_lin=.7; t.accel_ang=2.; t.was_pub=False
        t.manual_pub=Mock(); t.pub=Mock(); t.get_clock=b.get_clock
        t.get_logger=lambda:Mock()
        t.bridge_ready=Mock(); t.bridge_ready.service_is_ready.return_value=True
        t.bridge_ready.call_async.side_effect=lambda _:Future(b._manual_ready_cb(None,NS()))
        self.patcher=patch.object(time,'monotonic',lambda:self.now); self.patcher.start()
    def close(self): self.patcher.stop()
    def info(self, age=0.):
        # Actual Jazzy callback dictionary has timestamps/sequences, NO GID.
        return dict(source_timestamp=int((self.now-age)*1e9),received_timestamp=int(self.now*1e9),
                    publication_sequence_number=int(self.now*1e6),reception_sequence_number=int(self.now*1e6))
    def zeros(self, now):
        self.now=now; b=self.b
        b._manual_joy_cb(twist(),self.info()); b._manual_mux_cb(twist(),self.info())
        b.cmd_vel_callback(twist(),self.info())
    def drain(self):
        self.zeros(10.)
        for i in range(1,18): self.zeros(10.+i*.033)
        t=self.t
        for stage in ('joy','mux','final'):
            t._manual_drain.observe(stage,True,self.now-.501,1)
            t._manual_drain.observe(stage,True,self.now,int(self.now*1e9))
        t._last_joy=self.now
    def arm(self):
        self.assert_ready=self.b._manual_ready_cb(None,NS()).success
        self.b._manual_active_cb(Bool(data=True),self.info())
    def nonzero(self):
        self.now+=.0001
        self.b.cmd_vel_callback(twist(.1),self.info())
    def sent_nonzero(self):
        return [c.args[0] for c in self.b.sock.sendall.call_args_list if c.args[0]!=b'DRIVE:0.0:0.0\n']

class HandshakeTests(unittest.TestCase):
    def setUp(self): self.f=Fixture(); self.addCleanup(self.f.close)
    def test_boundary_0533_0534_and_positive_control(self):
        f=self.f; t=f.t; b=f.b
        # Teleop zero started at .033; bridge's final zero started at .035.
        # At .533/.534 their durations differ: .500 vs .499, valid DDS stamps.
        for stage in ('joy','mux','final'):
            t._manual_drain.observe(stage,True,10.033,10_033_000_000)
        f.zeros(10.035)
        for i in range(1,15): f.zeros(10.035+i*.033)
        for stage in ('joy','mux','final'):
            t._manual_drain.observe(stage,True,10.532,10_532_000_000)
        f.now=10.533; t._last_joy=f.now; t.tick()
        self.assertEqual(t.lin_cmd,0.); self.assertFalse(t.manual_pub.publish.call_args.args[0].data)
        f.zeros(10.534)
        self.assertFalse(b._manual_ready_cb(None,NS()).success)
        self.assertFalse(b._manual_drain.owned)
        # Continue zeros, NOT the old first-nonzero destruction of zero history.
        f.zeros(10.568); t._last_joy=f.now
        t._manual_drain.observe('mux',True,f.now,int(f.now*1e9))
        t._manual_drain.observe('final',True,f.now,int(f.now*1e9))
        t.tick()  # false reply from boundary; issue current request
        f.now=10.588; t._last_joy=f.now; t.tick()  # READY: Bool, still zero
        self.assertTrue(t.manual_pub.publish.call_args.args[0].data)
        self.assertEqual(t.lin_cmd,0.)
        b._manual_active_cb(Bool(data=True),f.info())
        f.now=10.589; t._last_joy=f.now; t.tick()  # consume READY, issue OWNED query
        f.now=10.609; t._last_joy=f.now; t.tick()
        self.assertGreater(t.lin_cmd,0.)
        b._manual_active_cb(Bool(data=True),f.info())
        b._manual_joy_cb(t.pub.publish.call_args.args[0],f.info())
        f.nonzero(); self.assertEqual(len(f.sent_nonzero()),1)
    def test_original_separate_drains_reproduce_indefinite_unowned(self):
        d=manual.ManualDrain()
        for stage in ('joy','mux','final'): d.observe(stage,True,.035,35_000_000)
        for i in range(1,15):
            for stage in ('joy','mux','final'): d.observe(stage,True,.035+i*.033,int((.035+i*.033)*1e9))
        d.observe('joy',False,.534,534_000_000)
        for i in range(1,80):
            for stage in ('joy','mux','final'):
                d.observe(stage,False,.534+i*.033,int((.534+i*.033)*1e9))
            self.assertFalse(d.ready(.534+i*.033))
        self.assertFalse(d.owned)
    def test_service_alone_never_grants(self):
        f=self.f; f.drain(); self.assertTrue(f.b._manual_ready_cb(None,NS()).success)
        f.nonzero(); self.assertFalse(f.b._manual_asserted); self.assertFalse(f.sent_nonzero())
    def test_no_service_no_bypass_even_with_ready_true_bool(self):
        f=self.f; f.drain(); f.b._manual_active_cb(Bool(data=True),f.info()); f.nonzero()
        self.assertFalse(f.sent_nonzero())
    def test_expired_one_shot_cannot_arm(self):
        f=self.f; f.drain(); f.b._manual_ready_cb(None,NS()); f.now+=.034
        f.b._manual_active_cb(Bool(data=True),f.info()); f.nonzero(); self.assertFalse(f.sent_nonzero())
    def test_stale_bool_cannot_consume_current_offer(self):
        f=self.f; f.drain(); f.b._manual_ready_cb(None,NS()); f.now+=.001
        f.b._manual_active_cb(Bool(data=True),f.info(age=.01)); f.nonzero()
        self.assertFalse(f.sent_nonzero())
    def test_false_release_immediately_stops_socket(self):
        f=self.f; f.drain(); f.arm(); f.nonzero(); self.assertEqual(len(f.sent_nonzero()),1)
        f.b._manual_active_cb(Bool(data=False),f.info())
        self.assertEqual(f.b.sock.sendall.call_args.args[0],b'DRIVE:0.0:0.0\n')
        f.nonzero(); self.assertEqual(len(f.sent_nonzero()),1)
    def test_current_owner_mutation_action_origin_and_lidar_rechecked(self):
        changes=[lambda b:setattr(b._epoch,'local_motion_mutations_settled',False),
                 lambda b:setattr(b._epoch,'local_motion_mutations_settled',None),
                 lambda b:setattr(b._epoch,'_navigation_status',{'goal':True}),
                 lambda b:setattr(b._undock_turn,'actions',{'dock':True}),
                 lambda b:setattr(b._undock_turn,'busy',True),
                 lambda b:setattr(b._epoch.guard,'generation',2),
                 lambda b:setattr(b._epoch.recovery,'token',2),
                 lambda b:setattr(b._epoch,'_tf_generation',2),
                 lambda b:setattr(b,'_lidar_ready',lambda:False),
                 lambda b:setattr(b,'_manual_raw_ready',lambda:False)]
        for change in changes:
            with self.subTest(change=change):
                f=Fixture()
                try:
                    f.drain(); f.arm(); change(f.b); f.nonzero()
                    self.assertFalse(f.sent_nonzero())
                    f.b.cmd_vel_callback(twist(),f.info())
                    self.assertEqual(f.b.sock.sendall.call_args.args[0],b'DRIVE:0.0:0.0\n')
                finally: f.close()
    def test_new_foreign_nonzero_invalidates_offer_and_lease(self):
        f=self.f; f.drain(); f.b._manual_ready_cb(None,NS())
        f.b._manual_foreign_cb(twist(.1)); f.b._manual_active_cb(Bool(data=True),f.info())
        f.nonzero(); self.assertFalse(f.sent_nonzero())
    def test_changed_gid_invalidates_offer_and_lease(self):
        f=self.f; f.drain(); f.arm()
        f.endpoints['/cmd_vel'][0].endpoint_gid=b'changed'*3
        f.nonzero(); self.assertFalse(f.sent_nonzero())
    def test_stale_final_and_zero_buffer_cannot_admit(self):
        f=self.f; f.drain(); f.arm(); f.now+=.51
        f.b.cmd_vel_callback(twist(.1),f.info(age=.51)); self.assertFalse(f.sent_nonzero())
        f.b.cmd_vel_callback(twist(),f.info(age=.51))
        self.assertFalse(f.b._manual_drive_allowed())
    def test_async_delay_exception_canceled_unavailable_and_wrong_generation_hold_zero(self):
        for mode in ('delayed','expired','exception','cancelled','unavailable','generation','false'):
            with self.subTest(mode=mode):
                f=self.f; f.drain(); t=f.t; now=f.now
                future=Future(NS(success=True,message=f'{manual.MANUAL_OWNED}:{int(now*1e9)}'))
                if mode=='delayed': future.complete=False
                if mode in ('exception','cancelled'): future.exc=RuntimeError(mode)
                if mode=='false': future.value.success=False
                if mode=='unavailable':
                    t.bridge_ready.service_is_ready.return_value=False; future=None
                t._bridge_ready_future=future
                t._bridge_ready_context=((t._takeover_generation,t._manual_drain.generation,t._lidar_generation),now)
                t._bridge_ready_received=(future,now)
                if mode=='generation': t._takeover_generation+=1
                f.now+=.034 if mode=='expired' else .01; t._last_joy=f.now
                t.tick()
                self.assertEqual(t.lin_cmd,0.); self.assertFalse(t.manual_pub.publish.call_args.args[0].data)
                self.assertTrue(manual.zero_twist(t.pub.publish.call_args.args[0]))
    def test_cancel_ack_keeps_zero_through_canceling_after_release(self):
        f=self.f; t=f.t; f.drain(); t._takeover_pending=True; t._cancel_in_flight=True
        t._navigation_cancel_done(Future(NS(return_code=0)),t._takeover_generation)
        self.assertTrue(t._takeover_pending)
        t._navigation_status(NS(status_list=[NS(status=3)])); self.assertTrue(t._takeover_pending)
        t.active=False; t.tick(); self.assertTrue(manual.zero_twist(t.pub.publish.call_args.args[0]))
        t._navigation_status(NS(status_list=[NS(status=5)])); self.assertFalse(t._takeover_pending)
    def test_late_arrival_cannot_hide_behind_fresh_server_timestamp(self):
        f=self.f; f.drain(); t=f.t; issued=f.now
        future=Future(NS(success=True,message=f'{manual.MANUAL_OWNED}:{int((issued+.04)*1e9)}'))
        context=((t._takeover_generation,t._manual_drain.generation,t._lidar_generation),issued)
        t._bridge_ready_future=future; t._bridge_ready_context=context
        f.now+=.04
        t._bridge_reply_done(future,context)
        self.assertIsNone(t._bridge_ready_received)
        t._last_joy=f.now; t.tick()
        self.assertFalse(t.manual_pub.publish.call_args.args[0].data)
        self.assertTrue(manual.zero_twist(t.pub.publish.call_args.args[0]))
    def test_response_timestamp_is_not_request_tick_and_never_extended(self):
        f=self.f; f.drain(); t=f.t; issued=f.now
        # Timely response generated/received 2 ms after the request. A tick just
        # over dt since request is still INSIDE dt since response, not a widened
        # reply timeout. The bridge independently enforces the same deadline.
        f.now+=.002
        future=Future(NS(success=True,message=f'{manual.MANUAL_READY}:{int(f.now*1e9)}'))
        context=((t._takeover_generation,t._manual_drain.generation,t._lidar_generation),issued)
        t._bridge_ready_future=future; t._bridge_ready_context=context
        t._bridge_reply_done(future,context)
        f.now=issued+t.dt+.0001
        self.assertEqual(t._bridge_readiness(f.now,True),manual.MANUAL_READY)
        t._bridge_ready_future=future; t._bridge_ready_context=context
        t._bridge_ready_received=(future,issued+.002)
        f.now=issued+.002+t.dt+.0001
        self.assertEqual(t._bridge_readiness(f.now,True),'')
    def test_old_future_cannot_overwrite_new_request_receipt(self):
        f=self.f; f.drain(); t=f.t
        old=Future(); current=Future()
        context=((t._takeover_generation,t._manual_drain.generation,t._lidar_generation),f.now)
        t._bridge_ready_future=current; t._bridge_ready_context=context
        t._bridge_reply_done(old,context)
        self.assertIsNone(t._bridge_ready_received)
    def test_service_disappears_or_throws_even_with_previous_fresh_reply(self):
        for mode in ('unavailable','call_exception'):
            with self.subTest(mode=mode):
                f=self.f; f.drain(); t=f.t; now=f.now
                future=Future(NS(success=True,message=f'{manual.MANUAL_OWNED}:{int(now*1e9)}'))
                t._bridge_ready_future=future
                t._bridge_ready_context=((t._takeover_generation,t._manual_drain.generation,t._lidar_generation),now)
                t._bridge_ready_received=(future,now)
                t.bridge_ready.service_is_ready.return_value=mode!='unavailable'
                if mode=='call_exception': t.bridge_ready.call_async.side_effect=RuntimeError('lost service')
                f.now+=.001; t._last_joy=f.now; t.tick()
                self.assertFalse(t.manual_pub.publish.call_args.args[0].data)
                self.assertTrue(manual.zero_twist(t.pub.publish.call_args.args[0]))
    def test_no_matching_cancel_waits_for_new_idle_not_cached_status(self):
        t=self.f.t; t._takeover_pending=True; t._cancel_in_flight=True
        t._navigation_cancel_done(Future(NS(return_code=1)),t._takeover_generation)
        self.assertTrue(t._takeover_pending)
        t._navigation_status(NS(status_list=[]))
        self.assertFalse(t._takeover_pending)
    def test_intervening_settled_mutation_invalidates_offer(self):
        f=self.f; f.drain(); f.b._manual_ready_cb(None,NS())
        f.b._epoch._manual_mutation_generation=1
        f.b._manual_active_cb(Bool(data=True),f.info()); f.nonzero()
        self.assertFalse(f.sent_nonzero())
    def test_ordinary_navigation_remains_held_in_recovery(self):
        f=self.f; f.drain(); f.nonzero(); self.assertFalse(f.sent_nonzero())

if __name__=='__main__': unittest.main(verbosity=2)
