"""Exercise real bridge evidence producer with no ROS/odom/localization fixture."""

import ast
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import pytest

from patrolbot_bridge.tuning_activation import evaluate_evidence
from patrolbot_bridge.tuning_runtime import TuningObservations

SOURCE = Path(__file__).resolve().parents[1] / 'patrolbot_bridge/bridge_node.py'


def method(name, now):
    tree = ast.parse(SOURCE.read_text())
    klass = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'PatrolBotBridge')
    definition = next(node for node in klass.body if isinstance(node, ast.FunctionDef) and node.name == name)
    namespace = {'time': NS(monotonic=lambda: now)}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[definition], type_ignores=[])), str(SOURCE), 'exec'), namespace)
    return namespace[name]


def bridge(now=100.):
    observations = TuningObservations(clock=lambda: now)
    observations.healthy_publishers(('hardware-1', 'publisher-1'))
    for channel in ('physical', 'web', 'web_session'):
        observations.deadman(channel, False)
    for topic in ('/cmd_vel', '/cmd_vel_out'):
        observations.velocity(topic, 0., 0.)
    epoch = NS(lock=threading.RLock(), guard=NS(ready=True), _navigation_status=False,
               navigation_idle_problem=lambda: '', _tuning_nav_pending=True,
               _map_error='', maps=NS(stage='idle', hold=False, active=None),
               _map_goal=None, _map_future=None, _tuning_map_probe=None,
               _failed=False, _future=None, maintenance_reserved=False,
               recovery=NS(stage='wait-operator-seed', READY='ready', WAIT_SEED='wait-operator-seed', WAIT_POSE='wait-amcl-pose'))
    parameters = {'front_bumper_mask': 1, 'rear_bumper_mask': 2, 'bumpers_mapped': True}
    return NS(_state_lock=threading.RLock(), _session_generation=1, _link_connected=True,
              _flags_at=now, _charge_state_at=now, _fault_flags=0, _estop_pressed=False,
              _stall_value=0, _dock_state_at=now, _dock_state='CLEAR_CONFIRMED',
              _undock_active=False, _undock_goal_reserved=False, _epoch=epoch,
              _hardware_operation_lock=threading.Lock(), _lidar_pending=False,
              get_parameter=lambda name: NS(value=parameters[name]), _tuning_observations=observations)


@pytest.mark.parametrize('motors', [False, True])
@pytest.mark.parametrize('stage', ['ready', 'wait-operator-seed', 'wait-amcl-pose'])
def test_real_evidence_never_reads_odom_pose_motor_or_map_readiness(motors, stage):
    node = bridge()
    node._motors_enabled = motors
    node._epoch.recovery.stage = stage
    # No _pose_lock, _pose_at, _vx, _vth or localization values exist. A read
    # of any of them would fail this fixture rather than create fake freshness.
    evidence = method('_tuning_evidence', 100.)(node)
    assert evidence.raw_odom_fresh is None and evidence.stationary is None
    assert evaluate_evidence(evidence) == (True, '')
    assert node._epoch.maps.active is None and node._epoch._tuning_nav_pending


@pytest.mark.parametrize('condition', ['link', 'fault', 'dock', 'input', 'hardware-owner', 'map-owner', 'lifecycle', 'navigator'])
def test_non_odom_unknown_or_busy_evidence_still_refuses(condition):
    node = bridge()
    if condition == 'link': node._flags_at = 99.
    if condition == 'fault': node._fault_flags = 1
    if condition == 'dock': node._dock_state = 'UNKNOWN'
    if condition == 'input': node._tuning_observations.invalidate(channel='physical')
    if condition == 'hardware-owner': node._hardware_operation_lock.acquire()
    if condition == 'map-owner': node._epoch.maps.stage = 'load-map'
    if condition == 'lifecycle': node._epoch._future = object()
    if condition == 'navigator': node._epoch.navigation_idle_problem = lambda: 'unknown'
    assert evaluate_evidence(method('_tuning_evidence', 100.)(node))[0] is False
