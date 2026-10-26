"""Exercise real manual operation ownership with a recording socket, no ROS."""

import importlib.util
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace as NS

import pytest


@pytest.fixture
def runtime(monkeypatch):
    for name, attrs in {
        'sensor_msgs': {}, 'sensor_msgs.msg': {'Joy': object},
        'std_msgs': {}, 'std_msgs.msg': {'Bool': lambda **kw: NS(**kw)},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).parents[1] / 'patrolbot_bridge' / 'manual_undock.py'
    spec = importlib.util.spec_from_file_location('patrolbot_bridge.manual_undock_tested', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    clock = NS(t=100.)

    def sleep(dt):
        clock.t += dt
    monkeypatch.setattr(module, 'time', NS(monotonic=lambda: clock.t, sleep=sleep))

    class Thread:
        def __init__(self, target, **kw):
            self.target = target

        def start(self):
            pass
    monkeypatch.setattr(module, 'threading', NS(Thread=Thread))
    recovery = NS(stage='ready', WAIT_RESET='wait-reset', token='old')

    def require_seed(stamp):
        recovery.stage, recovery.token = 'wait-seed', 'new'
    recovery.require_seed = require_seed
    epoch = NS(
        lock=threading.RLock(), undock_hold=False, tuning_hold=False,
        _failed=False, _map_error='', _map_goal=None, _map_future=None,
        _future=None, _future_kind=None, maps=NS(hold=False, stage='ready'),
        _navigation_status={},
        guard=NS(ready=True, epoch='11111111-2222-3333-4444-555555555555', session=1),
        recovery=recovery)
    messages, logs, writes = [], [], []
    reply = NS(code=0, success=True, message='complete', motors_enabled=True,
               dock_state='CLEAR_CONFIRMED')
    node = NS(
        _epoch=epoch, _callbacks=object(),
        _state_lock=threading.Lock(), _pose_lock=threading.Lock(),
        _control_lock=threading.Lock(), _undock_lock=threading.Lock(),
        _undock_goal_lock=threading.Lock(), _hardware_operation_lock=threading.Lock(),
        sock_lock=threading.Lock(), _stop_event=threading.Event(),
        _undock_turn=NS(busy=False, actions={}), _undock_goal_reserved=False,
        _server_hello=NS(features={'MANUAL_UNDOCK_V1'}), _link_connected=True,
        _lidar_ready=lambda: True, _pose_received=True, _undock_profile_commissioned=True,
        _dock_state='DOCKED_CONFIRMED', _charge_state=3, _estop_pressed=False,
        _fault_flags=0, _undock_active=False, _px=0., _py=0., _pth=0., _vx=0., _vth=0.,
        _pose_at=100., _flags_at=100., _charge_state_at=100., _dock_state_at=100.,
        _undock_scan_at=100., _pending_controls={}, _manual_active_at=0.,
        _web_teleop_active_at=0., _session_generation=1, _motor_disable_generation=0,
        _undock_result=None, _undock_feedback=None,
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=int(clock.t * 1e9))),
        create_publisher=lambda *a: NS(publish=messages.append),
        create_subscription=lambda *a, **kw: None, create_timer=lambda *a, **kw: None,
        get_logger=lambda: NS(info=logs.append, warning=logs.append, error=logs.append))
    node.sock = NS(sendall=writes.append)
    node._send_undock_cancel = lambda identity: writes.append(('cancel', identity)) or True

    class Event:
        lost = False

        def clear(self):
            pass

        def wait(self, dt):
            if self.lost:
                clock.t += dt
                return False
            node._undock_result = reply
            return True
    node._undock_event = Event()
    obj = module.ManualUndockRuntime(node)
    obj.raw.observe('ODOM:0,0,0,0,0|LASER:1,1,|LASER_AGE_MS:0|ODOM_EPOCH:' + epoch.guard.epoch,
                    clock.t, now=clock.t, session=1, epoch=epoch.guard.epoch)
    return NS(obj=obj, node=node, module=module, clock=clock, writes=writes,
              logs=logs, reply=reply, messages=messages)


def test_no_amcl_or_nav2_services_needed_for_admission_and_terminal_release(runtime):
    r, n = runtime.obj, runtime.node
    # The fake has no Nav2 clients at all and has an absent/settled-failed map.
    n._epoch.maps = NS(hold=True, stage='failed')
    n._epoch.recovery.stage = n._epoch.recovery.WAIT_RESET
    n._pose_received = False  # Public /odom may still be held pending AMCL reset.
    assert r.admission_problem() == ''
    with n._epoch.lock:
        r.start()
    assert r.busy and n._epoch.undock_hold and n._undock_goal_reserved
    r.thread.target()
    assert len(runtime.writes) == 1
    assert runtime.writes[0].startswith(b'MANUAL_UNDOCK:')
    assert not r.busy and not n._epoch.undock_hold and not n._undock_goal_reserved
    assert not n._hardware_operation_lock.locked()
    assert n._epoch.recovery.stage == n._epoch.recovery.WAIT_RESET
    assert not n._epoch._navigation_ready


@pytest.mark.parametrize('block', [
    'capability', 'tuning', 'unknown-mutation', 'map',
    'navigation', 'web-session', 'web-input', 'stale-odom', 'estop', 'moving', 'hardware-owner'])
def test_conflicts_do_not_dispatch_or_steal_ownership(runtime, block):
    r, n = runtime.obj, runtime.node
    if block == 'capability':
        n._server_hello.features.clear()
    if block == 'tuning':
        n._epoch.tuning_hold = True
    if block == 'unknown-mutation':
        n._epoch._failed = True
    if block == 'map':
        n._epoch._map_future = object()
    if block == 'navigation':
        n._epoch._navigation_status = {'active': True}
    if block == 'web-session':
        r.web_session_active = True
    if block == 'web-input':
        n._web_teleop_active_at = runtime.clock.t
    if block == 'stale-odom':
        r.raw.received -= 1.
    if block == 'estop':
        n._estop_pressed = True
    if block == 'moving':
        r.raw.sample = NS(values=lambda: (0., 0., 0., .1, 0.), linear_x=.1, angular_z=0.)
    if block == 'hardware-owner':
        n._hardware_operation_lock.acquire()
    with n._epoch.lock:
        r.start()
    assert not r.busy and not n._undock_goal_reserved
    assert runtime.writes == []


def test_unknown_outcome_disables_once_and_retains_owner_without_retries(runtime):
    r, n = runtime.obj, runtime.node
    n._undock_event.lost = True
    with n._epoch.lock:
        r.start()
    r.thread.target()
    assert r.unknown and r.busy and n._epoch.undock_hold and n._undock_goal_reserved
    assert n._hardware_operation_lock.locked()
    assert sum(isinstance(w, bytes) and w.startswith(b'MANUAL_UNDOCK:')
               for w in runtime.writes) == 1
    assert sum(isinstance(w, bytes) and w.startswith(b'MOTOR_DISABLE:')
               for w in runtime.writes) == 1


def test_confirmed_recoverable_failure_never_enables_or_disables_motors(runtime):
    runtime.reply.code, runtime.reply.success = 11, False
    with runtime.node._epoch.lock:
        runtime.obj.start()
    runtime.obj.thread.target()
    assert not runtime.obj.busy
    assert len(runtime.writes) == 1  # Only dispatch; SBC owns the motor state.
    assert not runtime.obj.input_is_new(None)
    assert not runtime.obj.input_is_new({'source_timestamp': int((runtime.clock.t - .1) * 1e9)})
    runtime.clock.t += .05
    assert runtime.obj.input_is_new({'source_timestamp': int(runtime.clock.t * 1e9)})


def test_wrong_or_missing_source_cannot_initiate(runtime):
    runtime.node.get_publishers_info_by_topic = lambda _: []
    for _ in range(30):
        runtime.obj.joy(NS(), NS())
    assert not runtime.obj.busy and runtime.writes == []


def test_session_changes_between_reservation_and_dispatch_never_move(runtime):
    r, n = runtime.obj, runtime.node
    with n._epoch.lock:
        r.start()
    n._session_generation += 1
    r.thread.target()
    assert r.unknown and r.busy
    assert not any(w.startswith(b'MANUAL_UNDOCK:') for w in runtime.writes if isinstance(w, bytes))


def test_controller_loss_does_not_set_cancellation_but_new_start_edge_does(runtime):
    r, n = runtime.obj, runtime.node
    endpoints = [NS(node_name='joy_node', node_namespace='/', topic_type='sensor_msgs/msg/Joy',
                    endpoint_gid=bytes([1] * 16))]
    n.get_publishers_info_by_topic = lambda _: endpoints
    with n._epoch.lock:
        r.start()

    def joy(start):
        t = runtime.clock.t
        ns = int(t * 1e9)
        msg = NS(header=NS(stamp=NS(sec=ns // 10**9, nanosec=ns % 10**9)),
                 axes=[0., 0., 1., 0., 0., 1., 0., 0.], buttons=[0] * 11)
        msg.buttons[7] = int(start)
        r.joy(msg, {'source_timestamp': ns})
    joy(False)
    runtime.clock.t += .5
    joy(False)
    assert not r.cancel_requested
    runtime.clock.t += .05
    joy(True)
    assert r.cancel_requested
