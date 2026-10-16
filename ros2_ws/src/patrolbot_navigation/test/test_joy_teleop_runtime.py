"""Exercise the real teleop callbacks without ROS or a robot connection."""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from uuid import UUID

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))


def stamp(seconds):
    nanos = int(seconds * 1e9)
    return SimpleNamespace(sec=nanos // 10**9, nanosec=nanos % 10**9)


def joy(now, lb=False, rb=False, lt=False, rt=False, linear=0.0):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp(now)),
        buttons=[0, 0, 0, 0, int(lb), int(rb)],
        axes=[0.0, linear, -1.0 if lt else 1.0, 0.0, 0.0,
              -1.0 if rt else 1.0, 0.0, 0.0])


def state(now, lidar='READY', generation=1, **changes):
    values = dict(
        header=SimpleNamespace(stamp=stamp(now)),
        session_generation=generation, link_connected=True,
        hardware_state_valid=True, telemetry_age=0.0,
        lidar_state=lidar, lidar_power_supported=True,
        lidar_scan_age=0.0 if lidar == 'READY' else -1.0)
    values.update(changes)
    return SimpleNamespace(**values)


class Future:
    def __init__(self):
        self.callback = None

    def add_done_callback(self, callback):
        self.callback = callback

    def finish(self, result=None, error=None):
        self.value, self.error = result, error
        self.callback(self)

    def result(self):
        if self.error:
            raise self.error
        return self.value


class Client:
    def __init__(self):
        self.ready, self.calls, self.futures = True, [], []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.calls.append(request)
        future = Future()
        self.futures.append(future)
        return future


@pytest.fixture
def teleop(monkeypatch):
    class Node:
        def __init__(self, name):
            self.now, self.parameters, self.clients, self.messages = 1.0, {}, {}, []

        def declare_parameter(self, name, value):
            self.parameters[name] = value

        def get_parameter(self, name):
            return SimpleNamespace(value=self.parameters[name])

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(self.now * 1e9)))

        def get_logger(self):
            return SimpleNamespace(info=lambda text: None, warn=lambda text: None,
                                   error=lambda text: None)

        def create_subscription(self, *args):
            pass

        def create_publisher(self, *args):
            return SimpleNamespace(publish=self.messages.append)

        def create_client(self, service, name):
            client = Client()
            self.clients[name] = client
            return client

        def create_timer(self, *args):
            pass

    class Twist:
        def __init__(self):
            self.linear = SimpleNamespace(x=0.0)
            self.angular = SimpleNamespace(z=0.0)

    modules = {
        'rclpy': {}, 'rclpy.node': {'Node': Node},
        'rclpy.executors': {'ExternalShutdownException': type('Shutdown', (Exception,), {})},
        'rclpy.qos': {'qos_profile_action_status_default': object()},
        'action_msgs': {}, 'action_msgs.msg': {
            'GoalStatus': SimpleNamespace(STATUS_ACCEPTED=1, STATUS_EXECUTING=2,
                                          STATUS_CANCELING=3),
            'GoalStatusArray': object},
        'action_msgs.srv': {'CancelGoal': SimpleNamespace(Request=SimpleNamespace)},
        'geometry_msgs': {}, 'geometry_msgs.msg': {'Twist': Twist},
        'sensor_msgs': {}, 'sensor_msgs.msg': {'Joy': object},
        'patrolbot_interfaces': {}, 'patrolbot_interfaces.msg': {'BaseState': object},
        'patrolbot_interfaces.srv': {'SetLidarPower': SimpleNamespace(Request=SimpleNamespace)},
    }
    for name, attributes in modules.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location('teleop_under_test', SCRIPTS / 'patrolbot_joy_teleop.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.JoyTeleop()


def chord(teleop):
    teleop.joy_cb(joy(teleop.now))
    teleop.joy_cb(joy(teleop.now, True, True, True, True, linear=1.0))


def cancel_done(teleop):
    teleop.nav_cancel.futures[-1].finish(SimpleNamespace(return_code=0))


def power_client(teleop):
    return teleop.clients['/patrolbot/set_lidar_power']


@pytest.mark.parametrize('current,target', [('READY', False), ('OFF', True)])
def test_complete_chord_waits_for_cancel_then_sends_one_explicit_target(teleop, current, target):
    teleop._base_state_cb(state(teleop.now, current))
    chord(teleop)
    assert not power_client(teleop).calls
    cancel_done(teleop)
    assert len(power_client(teleop).calls) == 1
    request = power_client(teleop).calls[0]
    assert request.enabled is target
    assert str(UUID(request.request_id)) == request.request_id
    teleop.joy_cb(joy(teleop.now, True, True, True, True))
    teleop.tick()
    assert len(power_client(teleop).calls) == 1


def test_chord_clears_existing_ramp_and_blocks_drive_until_every_control_releases(teleop):
    teleop._base_state_cb(state(teleop.now))
    teleop.joy_cb(joy(teleop.now))
    teleop.joy_cb(joy(teleop.now, rb=True, linear=1.0))
    teleop.tick()
    assert teleop.messages[-1].linear.x > 0.0
    teleop.joy_cb(joy(teleop.now, lb=True, rb=True, linear=1.0))
    teleop.tick()
    assert teleop.messages[-1].linear.x == 0.0
    teleop.joy_cb(joy(teleop.now, rb=True, linear=1.0))
    teleop.tick()
    assert teleop.messages[-1].linear.x == 0.0
    teleop.joy_cb(joy(teleop.now))
    teleop.joy_cb(joy(teleop.now, rb=True, linear=1.0))
    teleop.tick()
    assert teleop.messages[-1].linear.x > 0.0


@pytest.mark.parametrize('abandon', ['release', 'timeout', 'stale', 'malformed', 'generation', 'disconnect'])
def test_delayed_cancel_cannot_toggle_an_abandoned_chord(teleop, abandon):
    teleop._base_state_cb(state(teleop.now))
    chord(teleop)
    if abandon == 'release':
        teleop.joy_cb(joy(teleop.now))
    elif abandon == 'timeout':
        teleop.now += 0.5
    elif abandon == 'stale':
        teleop.joy_cb(joy(0.0, True, True, True, True))
    elif abandon == 'malformed':
        message = joy(teleop.now, True, True, True, True)
        message.axes[2] = float('nan')
        teleop.joy_cb(message)
    elif abandon == 'generation':
        teleop._base_state_cb(state(teleop.now, generation=2))
    else:
        teleop._base_state_cb(state(teleop.now, link_connected=False))
    cancel_done(teleop)
    teleop.tick()
    assert not power_client(teleop).calls


@pytest.mark.parametrize('changes', [
    {'lidar_state': 'UNKNOWN'}, {'lidar_state': 'STARTING'}, {'lidar_state': 'FAULT'},
    {'lidar_power_supported': False}, {'telemetry_age': 1.0},
    {'lidar_scan_age': 1.0}, {'lidar_scan_age': float('nan')},
    {'hardware_state_valid': False}, {'link_connected': False},
])
def test_toggle_requires_fresh_known_commissioned_state(teleop, changes):
    teleop._base_state_cb(state(teleop.now, **changes))
    chord(teleop)
    cancel_done(teleop)
    assert not power_client(teleop).calls


def test_toggle_refuses_missing_or_aged_base_state(teleop):
    chord(teleop)
    cancel_done(teleop)
    assert not power_client(teleop).calls
    teleop._base_state_cb(state(teleop.now))
    teleop.now += 0.6
    chord(teleop)
    cancel_done(teleop)
    assert not power_client(teleop).calls


@pytest.mark.parametrize('failure', ['exception', 'refused', 'unavailable'])
def test_no_blind_retry_after_service_failure_or_unavailability(teleop, failure):
    teleop._base_state_cb(state(teleop.now))
    client = power_client(teleop)
    if failure == 'unavailable':
        client.ready = False
    chord(teleop)
    cancel_done(teleop)
    if failure == 'exception':
        client.futures[-1].finish(error=RuntimeError('lost result'))
    elif failure == 'refused':
        client.futures[-1].finish(SimpleNamespace(accepted=False, success=False, message='busy'))
    client.ready = True
    for _ in range(3):
        teleop.tick()
        teleop.joy_cb(joy(teleop.now, True, True, True, True))
    assert len(client.calls) == (0 if failure == 'unavailable' else 1)


def test_busy_service_requires_another_complete_chord_after_result(teleop):
    teleop._base_state_cb(state(teleop.now))
    client = power_client(teleop)
    chord(teleop)
    cancel_done(teleop)
    chord(teleop)
    cancel_done(teleop)
    assert len(client.calls) == 1
    client.futures[0].finish(SimpleNamespace(accepted=True, success=True, message='sent'))
    teleop.tick()
    assert len(client.calls) == 1
    chord(teleop)
    cancel_done(teleop)
    assert len(client.calls) == 2


def test_reconnect_or_joy_timeout_cannot_rearm_a_held_chord(teleop):
    teleop._base_state_cb(state(teleop.now))
    teleop.joy_cb(joy(teleop.now))
    teleop.now += 0.5
    teleop.tick()
    teleop.joy_cb(joy(teleop.now, True, True, True, True))
    cancel_done(teleop)
    assert not power_client(teleop).calls
    teleop._base_state_cb(state(teleop.now, generation=2))
    teleop.joy_cb(joy(teleop.now, True, True, True, True))
    teleop.tick()
    assert not power_client(teleop).calls


def test_existing_rb_takeover_retries_cancel_and_holds_zero_after_release(teleop):
    teleop.nav_cancel.ready = False
    teleop.joy_cb(joy(teleop.now, rb=True, linear=1.0))
    teleop.tick()
    assert teleop.messages[-1].linear.x > 0.0
    teleop.joy_cb(joy(teleop.now))
    for _ in range(20):
        teleop.tick()
    assert teleop.messages[-1].linear.x == 0.0
    assert teleop._takeover_pending
    teleop.nav_cancel.ready = True
    teleop.now += 0.3
    teleop.tick()
    cancel_done(teleop)
    assert not teleop._takeover_pending


def test_manual_evidence_is_not_kept_alive_by_the_takeover_zero_hold(teleop):
    # An unconfirmable takeover latches _takeover_pending, which keeps
    # publishing priority-8 zero. That fail-closed zero hold is not manual
    # intent and must not keep the bridge's manual bypass open.
    teleop.nav_cancel.ready = False
    teleop.joy_cb(joy(teleop.now, rb=True, linear=1.0))
    teleop.tick()
    evidence = [m for m in teleop.messages if hasattr(m, 'data')]
    assert evidence and evidence[-1].data is True
    teleop.joy_cb(joy(teleop.now))
    for _ in range(30):
        teleop.tick()
    assert teleop._takeover_pending
    evidence = [m for m in teleop.messages if hasattr(m, 'data')]
    assert evidence[-1].data is False
    twists = [m for m in teleop.messages if hasattr(m, 'linear')]
    assert twists[-1].linear.x == 0.0


def test_gap_without_timer_cannot_preserve_an_old_pending_chord(teleop):
    teleop._base_state_cb(state(teleop.now))
    chord(teleop)
    teleop.now += 0.45
    teleop.joy_cb(joy(teleop.now, True, True, True, True))
    cancel_done(teleop)
    assert not power_client(teleop).calls


def test_input_acquisition_age_is_rechecked_in_delayed_cancel_callback(teleop):
    teleop._base_state_cb(state(teleop.now))
    teleop.joy_cb(joy(teleop.now - 0.36))
    teleop.joy_cb(joy(teleop.now - 0.35, True, True, True, True))
    teleop.now += 0.1
    cancel_done(teleop)
    assert not power_client(teleop).calls


def test_old_cancel_callback_requires_a_new_cancel_for_a_new_chord(teleop):
    teleop._base_state_cb(state(teleop.now))
    chord(teleop)
    old_cancel = teleop.nav_cancel.futures[-1]
    chord(teleop)
    old_cancel.finish(SimpleNamespace(return_code=0))
    assert not power_client(teleop).calls
    assert len(teleop.nav_cancel.calls) == 2
    cancel_done(teleop)
    assert len(power_client(teleop).calls) == 1


def test_base_session_change_clears_cached_drive_intent(teleop):
    teleop._base_state_cb(state(teleop.now))
    teleop.joy_cb(joy(teleop.now, rb=True, linear=1.0))
    teleop.tick()
    assert teleop.messages[-1].linear.x > 0.0
    teleop._base_state_cb(state(teleop.now, generation=2))
    teleop.tick()
    assert teleop.messages[-1].linear.x == 0.0
    assert teleop.lin_tgt == 0.0


def test_base_state_stamp_and_acquisition_age_both_gate_toggle(teleop):
    teleop._base_state_cb(state(teleop.now - 0.6))
    chord(teleop)
    cancel_done(teleop)
    assert not power_client(teleop).calls
    teleop._base_state_cb(state(teleop.now, telemetry_age=0.45))
    chord(teleop)
    teleop.now += 0.1
    cancel_done(teleop)
    assert not power_client(teleop).calls


@pytest.mark.parametrize('abandon', ['timeout', 'malformed', 'generation'])
def test_abandoned_chord_cannot_restore_rb_drive_before_full_release(teleop, abandon):
    teleop._base_state_cb(state(teleop.now))
    chord(teleop)
    if abandon == 'timeout':
        teleop.now += 0.5
        teleop.tick()
    elif abandon == 'malformed':
        message = joy(teleop.now)
        message.axes = []
        teleop.joy_cb(message)
    else:
        teleop._base_state_cb(state(teleop.now, generation=2))
    teleop.joy_cb(joy(teleop.now, rb=True, linear=1.0))
    teleop.tick()
    assert teleop.messages[-1].linear.x == 0.0
    teleop.joy_cb(joy(teleop.now))
    teleop.joy_cb(joy(teleop.now, rb=True, linear=1.0))
    teleop.tick()
    assert teleop.messages[-1].linear.x > 0.0


def test_cached_joystick_drive_is_cleared_before_power_service_dispatch(teleop):
    teleop._base_state_cb(state(teleop.now))
    teleop.joy_cb(joy(teleop.now))
    teleop.joy_cb(joy(teleop.now, rb=True, linear=1.0))
    cancel_done(teleop)
    teleop.tick()
    assert teleop.lin_cmd > 0.0
    client = power_client(teleop)
    original = client.call_async

    def dispatch(request):
        assert (teleop.lin_tgt, teleop.ang_tgt, teleop.lin_cmd, teleop.ang_cmd) == (0.0,) * 4
        return original(request)

    client.call_async = dispatch
    teleop.joy_cb(joy(teleop.now, True, True, True, True, linear=1.0))
    assert len(client.calls) == 1


def test_clock_rollback_drops_deadman_and_cached_drive_target(teleop):
    teleop.joy_cb(joy(teleop.now, rb=True, linear=1.0))
    teleop.tick()
    previous = teleop.messages[-1].linear.x
    assert previous > 0.0
    teleop.now -= 0.1
    teleop.tick()
    assert not teleop.active
    assert teleop.lin_tgt == 0.0
    assert teleop.messages[-1].linear.x < previous


@pytest.mark.parametrize('field', ['lidar_scan_age', 'telemetry_age'])
def test_queued_base_state_age_is_added_to_measured_age_before_toggle(teleop, field):
    teleop._base_state_cb(state(teleop.now - .48, **{field: .1}))
    teleop.now += .01
    chord(teleop)
    cancel_done(teleop)
    assert not power_client(teleop).calls
