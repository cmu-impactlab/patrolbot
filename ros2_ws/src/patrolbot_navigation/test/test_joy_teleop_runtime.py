"""Exercise the real teleop callbacks without ROS or a robot connection."""

import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from uuid import UUID

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(SCRIPTS.parents[1] / 'patrolbot_bridge'))


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

        def create_service(self, *args):
            pass

        def add_on_set_parameters_callback(self, callback):
            self.validate_parameters = callback

        def add_post_set_parameters_callback(self, callback):
            self.commit_parameters = callback

    class Twist:
        def __init__(self):
            self.linear = SimpleNamespace(x=0.0)
            self.angular = SimpleNamespace(z=0.0)

    class Bool:
        def __init__(self, data=False):
            self.data = data

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
        'std_msgs': {}, 'std_msgs.msg': {'Bool': Bool},
        'std_srvs': {}, 'std_srvs.srv': {'Trigger': SimpleNamespace(Request=SimpleNamespace)},
        'rcl_interfaces': {}, 'rcl_interfaces.msg': {'SetParametersResult': SimpleNamespace},
        'patrolbot_interfaces': {}, 'patrolbot_interfaces.msg': {'BaseState': object},
        'patrolbot_interfaces.srv': {'SetLidarPower': SimpleNamespace(Request=SimpleNamespace),
                                    'TuningMaintenance': SimpleNamespace(Request=SimpleNamespace)},
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


def maintenance(teleop, operation='acquire', **changes):
    values = dict(operation=operation, job_id='11111111-1111-4111-8111-111111111111',
                  digest='sha256:' + 'a' * 64, plan_hash='sha256:' + 'b' * 64)
    values.update(changes)
    return teleop._tuning_maintenance(SimpleNamespace(**values), SimpleNamespace())


def test_replacement_can_restore_only_lower_trims_under_the_exact_held_job(teleop):
    payload = json.dumps({'max_linear': .1, 'max_angular': .2})
    assert not maintenance(teleop, 'restore_trims', payload_json=payload).accepted
    teleop._tuning_hold_cb(SimpleNamespace(data=True))
    assert maintenance(teleop, 'block').accepted
    assert not maintenance(teleop, 'restore_trims', payload_json=payload,
                           digest='sha256:' + 'c' * 64).accepted
    assert maintenance(teleop, 'restore_trims', payload_json=payload).accepted
    assert (teleop.max_lin, teleop.max_ang) == (.1, .2)
    assert teleop._operator_linear_trim and teleop._operator_angular_trim
    assert maintenance(teleop, 'restore_trims', payload_json=json.dumps({'max_linear': .3})).accepted
    assert teleop.max_lin == .1
    assert teleop.get_parameter('max_linear').value == .3
    assert not teleop.active and teleop._tuning_require_release


@pytest.mark.parametrize('payload', ['{}', '{"path":"arbitrary"}', '{"max_linear":true}',
                                    '{"max_angular":-1}', '{"max_linear":NaN}'])
def test_trim_restoration_rejects_non_policy_payloads_without_mutating_a_cache(teleop, payload):
    teleop._tuning_hold_cb(SimpleNamespace(data=True))
    assert maintenance(teleop, 'block').accepted
    assert not maintenance(teleop, 'restore_trims', payload_json=payload).accepted
    assert (teleop.max_lin, teleop.max_ang) == (.3, .7)


def test_maintenance_requires_an_explicit_fresh_complete_released_controller(teleop):
    assert not maintenance(teleop).accepted  # Never received is not released.
    teleop.joy_cb(joy(teleop.now))
    teleop.now += .401
    assert not maintenance(teleop).accepted
    teleop.joy_cb(joy(teleop.now))
    assert maintenance(teleop).accepted
    assert teleop._tuning_hold
    teleop.joy_cb(SimpleNamespace(header=SimpleNamespace(stamp=stamp(teleop.now)), buttons=[], axes=[]))
    inspected = maintenance(teleop, 'inspect')
    assert inspected.accepted
    assert json.loads(inspected.evidence_json)['idle'] is False
    assert teleop._tuning_hold  # Unknown input never expires ownership.


@pytest.mark.parametrize('kind', ['stale', 'future', 'malformed', 'held', 'partial_chord', 'trim'])
def test_maintenance_never_treats_invalid_or_held_input_as_release(teleop, kind):
    message = joy(teleop.now)
    if kind == 'stale':
        message.header.stamp = stamp(teleop.now - .401)
    elif kind == 'future':
        message.header.stamp = stamp(teleop.now + .01)
    elif kind == 'malformed':
        message.axes[0] = float('nan')
    elif kind == 'held':
        message.buttons[5] = 1
    elif kind == 'partial_chord':
        message.buttons[4] = 1
    else:
        message.buttons[0] = 1
    teleop.joy_cb(message)
    assert not maintenance(teleop).accepted
    assert not teleop._tuning_hold


def test_maintenance_is_exact_job_digest_plan_and_release_requires_new_deadman(teleop):
    teleop.joy_cb(joy(teleop.now))
    assert maintenance(teleop).accepted
    assert maintenance(teleop).accepted  # Idempotent exact acquisition.
    for operation in ('acquire', 'inspect', 'release'):
        for field in ('job_id', 'digest', 'plan_hash'):
            foreign = ('22222222-2222-4222-8222-222222222222' if field == 'job_id'
                       else 'sha256:' + 'c' * 64)
            assert not maintenance(teleop, operation, **{field: foreign}).accepted
            assert teleop._tuning_hold
    teleop._tuning_hold_cb(SimpleNamespace(data=False))
    assert teleop._tuning_hold  # Legacy false cannot drop a job's owner.
    teleop.joy_cb(joy(teleop.now, rb=True, linear=1.))
    assert maintenance(teleop, 'release').accepted
    assert maintenance(teleop, 'release').accepted
    assert not maintenance(teleop).accepted  # Released job cannot reacquire.
    teleop.joy_cb(joy(teleop.now, rb=True, linear=1.))
    teleop.tick()
    assert not teleop.active and teleop.lin_cmd == 0.
    teleop.now += .001
    teleop.joy_cb(joy(teleop.now))
    teleop.joy_cb(joy(teleop.now, rb=True, linear=1.))
    teleop.tick()
    assert teleop.active and teleop.lin_cmd > 0.


def test_inspect_reports_actual_consumed_caps_minima_trims_and_zero_buffers(teleop):
    message = joy(teleop.now)
    message.buttons[0] = 1
    teleop.joy_cb(message)
    teleop.joy_cb(joy(teleop.now))
    assert maintenance(teleop).accepted
    inspected = json.loads(maintenance(teleop, 'inspect').evidence_json)
    assert inspected['policy']['max_linear'] == .25
    assert teleop.get_parameter('max_linear').value == .30  # Parameter-only proof is insufficient.
    assert inspected['policy']['max_linear_cap'] == .30
    assert inspected['policy']['min_linear'] == .05
    assert inspected['operator_linear_trim'] is True
    assert inspected['release_required'] is True
    assert set(inspected['buffers'].values()) == {0.0}
    assert inspected['policy_version'] == 1
    assert not power_client(teleop).calls  # Maintenance is not a hardware command.


def test_unknown_corrupt_cache_evidence_retains_maintenance_hold(teleop):
    teleop.joy_cb(joy(teleop.now))
    assert maintenance(teleop).accepted
    teleop.lin_cap = float('nan')
    assert not maintenance(teleop, 'release').accepted
    assert teleop._tuning_hold and teleop._tuning_lease.held


def test_blocked_input_is_not_falsely_called_released_and_does_not_abort_policy_install(teleop):
    assert maintenance(teleop, 'observe').accepted
    assert not maintenance(teleop, 'block').accepted  # core closure not observed
    teleop._tuning_hold_cb(SimpleNamespace(data=True))
    teleop.joy_cb(joy(teleop.now, rb=True, linear=1.))
    assert maintenance(teleop, 'block').accepted
    inspected = json.loads(maintenance(teleop, 'inspect').evidence_json)
    assert inspected['input_blocked'] and not inspected['input_released']
    result = teleop._validate_tuning_parameters([SimpleNamespace(name='max_linear', value=.2)])
    assert result.successful
    assert not teleop.active and teleop.lin_tgt == 0. and teleop.lin_cmd == 0.


def test_queued_release_press_and_trim_cannot_replay_after_the_last_hold_ends(teleop):
    teleop._tuning_hold_cb(SimpleNamespace(data=True))
    assert maintenance(teleop, 'block').accepted
    old_release = joy(teleop.now)
    old_press = joy(teleop.now, rb=True, linear=1.)
    old_trim = joy(teleop.now)
    old_trim.buttons[0] = 1
    assert maintenance(teleop, 'release').accepted
    assert teleop._tuning_hold  # core hold still blocks; no premature release barrier
    teleop.now += .001
    teleop._tuning_hold_cb(SimpleNamespace(data=False))
    for sample in (old_release, old_press, old_trim):
        teleop.joy_cb(sample)
    teleop.tick()
    assert not teleop.active and teleop.lin_cmd == 0.
    assert teleop.max_lin == .30 and teleop._tuning_require_release
    assert not teleop._tuning_input_released
    assert teleop.nav_cancel.calls == []
    teleop.now += .001
    teleop.joy_cb(joy(teleop.now))
    teleop.now += .001
    teleop.joy_cb(joy(teleop.now, rb=True, linear=1.))
    assert teleop.active and teleop.lin_tgt > 0.


@pytest.mark.parametrize('field', ['active', '_takeover_pending', '_cancel_in_flight',
                                 '_lidar_toggle_pending', '_lidar_in_flight', 'lin_tgt', 'ang_cmd'])
def test_job_hold_never_excuses_a_live_buffer_or_pending_producer_operation(teleop, field):
    teleop._tuning_hold_cb(SimpleNamespace(data=True))
    assert maintenance(teleop, 'block').accepted
    setattr(teleop, field, True if field not in ('lin_tgt', 'ang_cmd') else .1)
    assert not teleop._validate_tuning_parameters([SimpleNamespace(name='max_linear', value=.2)]).successful


def test_release_does_not_clear_unrelated_legacy_hold(teleop):
    teleop.joy_cb(joy(teleop.now))
    assert maintenance(teleop).accepted
    teleop._tuning_hold_cb(SimpleNamespace(data=True))
    assert maintenance(teleop, 'release').accepted
    assert teleop._tuning_hold


def test_tuning_hold_requires_release_then_a_new_deadman_press(teleop):
    teleop._tuning_hold_cb(SimpleNamespace(data=True))
    teleop.joy_cb(joy(teleop.now, rb=True, linear=1.))
    teleop.tick()
    assert not teleop.active
    assert teleop.lin_cmd == 0
    assert not teleop._tuning_idle(None, SimpleNamespace()).success
    teleop._tuning_hold_cb(SimpleNamespace(data=False))
    teleop.joy_cb(joy(teleop.now, rb=True, linear=1.))
    teleop.tick()
    assert not teleop.active
    assert teleop.lin_cmd == 0
    teleop.now += .001
    teleop.joy_cb(joy(teleop.now, rb=False))
    teleop.joy_cb(joy(teleop.now, rb=True, linear=1.))
    teleop.tick()
    assert teleop.active
    assert 0 < teleop.lin_cmd < teleop.max_lin


def test_tuning_enabled_restart_does_not_replay_an_already_held_controller(teleop, monkeypatch):
    monkeypatch.setenv('PATROLBOT_TUNING_ENABLED', '1')
    restarted = type(teleop)()
    restarted.joy_cb(joy(restarted.now, rb=True, linear=1.))
    restarted.tick()
    assert not restarted.active and restarted.lin_cmd == 0
    # A replacement remains closed until the core publishes its current hold.
    restarted._tuning_hold_cb(SimpleNamespace(data=False))
    restarted.now += .001
    restarted.joy_cb(joy(restarted.now, rb=False))
    restarted.joy_cb(joy(restarted.now, rb=True, linear=1.))
    restarted.tick()
    assert restarted.active and restarted.lin_cmd > 0


def test_tuning_policy_updates_cached_caps_only_under_acknowledged_idle_hold(teleop):
    updates = [SimpleNamespace(name=name, value=value) for name, value in {
        'max_linear': .15, 'max_linear_cap': .15, 'min_linear': .05,
        'max_angular': .35, 'max_angular_cap': .35, 'min_angular': .1,
    }.items()]
    assert not teleop.validate_parameters(updates).successful
    teleop.joy_cb(joy(teleop.now))
    teleop._tuning_hold_cb(SimpleNamespace(data=True))
    assert teleop._tuning_idle(None, SimpleNamespace()).success
    assert teleop.validate_parameters(updates).successful
    for parameter in updates:
        teleop.parameters[parameter.name] = parameter.value
    teleop.commit_parameters(updates)
    assert teleop.lin_cap == .15 and teleop.ang_cap == .35
    assert teleop.max_lin <= .15 and teleop.max_ang <= .35
    assert not teleop.active and teleop.lin_cmd == teleop.ang_cmd == 0
    updates[0].value = .31
    assert not teleop.validate_parameters(updates).successful


def test_lower_then_raise_updates_real_cached_targets_but_never_rearms(teleop):
    teleop.joy_cb(joy(teleop.now))
    teleop._tuning_hold_cb(SimpleNamespace(data=True))
    for linear, angular in ((.15, .35), (.30, .70)):
        updates = [SimpleNamespace(name=name, value=value) for name, value in {
            'max_linear': linear, 'max_linear_cap': linear, 'min_linear': .05,
            'max_angular': angular, 'max_angular_cap': angular, 'min_angular': .1,
        }.items()]
        assert teleop.validate_parameters(updates).successful
        for parameter in updates:
            teleop.parameters[parameter.name] = parameter.value
        teleop.commit_parameters(updates)
        assert teleop.max_lin == linear and teleop.max_ang == angular
        assert teleop.lin_cmd == teleop.ang_cmd == 0
        assert not teleop.active


def test_administrator_cap_raise_preserves_explicit_lower_operator_trim(teleop):
    message = joy(teleop.now)
    message.buttons[0] = 1  # A: lower the temporary linear trim.
    teleop.joy_cb(message)
    assert teleop.max_lin == .25 and teleop._operator_linear_trim
    teleop.joy_cb(joy(teleop.now))
    teleop._tuning_hold_cb(SimpleNamespace(data=True))
    for linear in (.20, .30):
        updates = [SimpleNamespace(name=name, value=value) for name, value in {
            'max_linear': linear, 'max_linear_cap': linear, 'min_linear': .05,
        }.items()]
        assert teleop.validate_parameters(updates).successful
        for parameter in updates:
            teleop.parameters[parameter.name] = parameter.value
        teleop.commit_parameters(updates)
    assert teleop.lin_cap == .30
    assert teleop.max_lin == .20
    assert not teleop.active and teleop.lin_cmd == 0


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
