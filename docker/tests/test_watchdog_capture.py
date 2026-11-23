"""Offline checks for watchdog discovery and recovery fault injection."""
import ast
from pathlib import Path
from types import SimpleNamespace
import threading
import time
import math
import unittest

ROOT=Path(__file__).resolve().parents[2]


def function(path,name):
    source=ast.parse(path.read_text())
    for node in ast.walk(source):
        if isinstance(node,ast.FunctionDef) and node.name==name:
            namespace={}
            exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),namespace)
            return namespace[name]
    raise AssertionError('missing function '+name)


class WatchdogCaptureTests(unittest.TestCase):
    def test_silence_without_all_endpoints_cannot_establish_release(self):
        matched=function(ROOT/'docker/tests/integration_watchdog_outage.py','endpoints_matched')
        self.assertFalse(matched({}))
        self.assertFalse(matched({'/scan':1,'/input/safety_controller':0}))
        self.assertFalse(matched({'/scan':0,'/input/safety_controller':1}))
        self.assertTrue(matched({'/scan':1,'/input/safety_controller':1}))

    def test_first_hold_onset_is_zero_and_within_channel_budget(self):
        check=function(ROOT/'docker/tests/integration_watchdog_outage.py','hold_onset')
        zero=SimpleNamespace(linear=SimpleNamespace(x=0.),angular=SimpleNamespace(z=0.))
        nonzero=SimpleNamespace(linear=SimpleNamespace(x=.1),angular=SimpleNamespace(z=0.))
        self.assertAlmostEqual(check([(10.55,zero)],10.,.5,.60),.55)
        for samples in [[],[(10.61,zero)],[(10.49,zero)],[(10.55,nonzero)]]:
            with self.assertRaises(AssertionError):check(samples,10.,.5,.60)

    def test_localization_recovery_preserves_manual_hardware_validity(self):
        callback=function(ROOT/'ros2_ws/src/patrolbot_navigation/scripts/patrolbot_safety_watchdog.py','_base_state_cb')
        watchdog=SimpleNamespace(_now=lambda:1.)
        message=SimpleNamespace(hardware_state_valid=True,odom_epoch_valid=True,
            localization_recovery_required=False,fault_flags=0,estop_pressed=False,
            bumpers_valid=True,front_bumper_pressed=False,rear_bumper_pressed=False)
        callback(watchdog,message)
        self.assertTrue(watchdog._hardware_state_valid)
        message.localization_recovery_required=True
        callback(watchdog,message)
        self.assertTrue(watchdog._hardware_state_valid)
        message.localization_recovery_required=False
        callback(watchdog,message)
        self.assertTrue(watchdog._hardware_state_valid)
        # Manual recovery still requires real hardware and a valid odom epoch.
        for field in ('hardware_state_valid', 'odom_epoch_valid'):
            setattr(message, field, False)
            callback(watchdog,message)
            self.assertFalse(watchdog._hardware_state_valid)
            setattr(message, field, True)

    def test_localization_recovery_still_gates_autonomy(self):
        allowed=function(ROOT/'ros2_ws/src/patrolbot_bridge/patrolbot_bridge/epoch_runtime.py',
                         'commands_allowed').fget
        allowed.__globals__.update(time=time, math=math)
        source=ROOT/'ros2_ws/src/patrolbot_bridge/patrolbot_bridge/map_runtime.py'
        class EpochDouble(SimpleNamespace):
            mutation_unresolved=function(source, 'mutation_unresolved')
            tuning_gate_unresolved=function(source, 'tuning_gate_unresolved')

        runtime=EpochDouble(lock=threading.RLock(), tuning_hold=False, map_hold=False, undock_hold=False,
            guard=SimpleNamespace(ready=True), recovery=SimpleNamespace(required=False),
            _navigation_ready=True, first_tf_ns=1, _lifecycle_outcome_unknown=False, _future=None,
            node=SimpleNamespace(_lidar_ready=lambda:True, _pose_lock=threading.Lock(),
                _pose_at=time.monotonic(), _pose_received=True, _odom_tf_timeout=10.))
        self.assertTrue(allowed(runtime))
        runtime.recovery.required=True
        self.assertFalse(allowed(runtime))
        runtime.recovery.required=False
        runtime._lifecycle_outcome_unknown=True
        self.assertFalse(allowed(runtime))

    def test_floor_capture_sensor_qos_accepts_best_effort_and_latched_tf(self):
        source=(ROOT/'docker/tests/integration_floor_maps.py').read_text()
        tree=ast.parse(source)
        loop=next(node for node in ast.walk(tree) if isinstance(node,ast.For)
                  and isinstance(node.target,ast.Tuple)
                  and [n.id for n in node.target.elts if isinstance(n,ast.Name)]==['topic','kind'])
        calls=[]
        namespace={'topic':'/scan','kind':object(),'captures':[],
            'node':SimpleNamespace(create_subscription=lambda *args:calls.append(args)),
            'capture':lambda *args:None,'qos_profile_sensor_data':'best_effort',
            'QoSProfile':lambda **kwargs:kwargs,
            'DurabilityPolicy':SimpleNamespace(TRANSIENT_LOCAL='transient_local')}
        code=compile(ast.Module(body=loop.body,type_ignores=[]),'<capture qos>','exec')
        exec(code,namespace)
        self.assertEqual(calls[-1][-1],'best_effort')
        namespace['topic']='/tf_static';exec(code,namespace)
        self.assertEqual(calls[-1][-1]['durability'],'transient_local')

if __name__=='__main__':unittest.main()
