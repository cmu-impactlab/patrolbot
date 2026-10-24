"""Native F710 service/parameter callback evidence, isolated graph and no device."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from patrolbot_interfaces.srv import TuningMaintenance

pytestmark = pytest.mark.skipif(
    os.environ.get('PATROLBOT_RUN_INTEGRATION') != '1'
    or int(os.environ.get('ROS_DOMAIN_ID', '0')) < 90,
    reason='requires isolated integration domain >=90')


@pytest.mark.parametrize('admission', ['acquire', 'block'])
def test_native_f710_exact_job_inspects_consumed_policy_not_parameter_only(admission):
    scripts = Path(__file__).resolve().parents[1] / 'scripts'
    sys.path.insert(0, str(scripts))
    spec = importlib.util.spec_from_file_location('native_joy_teleop', scripts / 'patrolbot_joy_teleop.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init()
    producer = module.JoyTeleop()
    client_node = Node('tuning_producer_native_client')
    executor = SingleThreadedExecutor()
    executor.add_node(producer)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    client = client_node.create_client(TuningMaintenance, '/patrolbot/joy_tuning_maintenance')
    hold = client_node.create_publisher(Bool, '/patrolbot/tuning_hold', 10)
    owner = dict(job_id='11111111-1111-4111-8111-111111111111',
                 digest='sha256:' + 'a' * 64, plan_hash='sha256:' + 'b' * 64)

    def call(operation, **changes):
        future = client.call_async(TuningMaintenance.Request(operation=operation, **dict(owner, **changes)))
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        assert done.wait(3.)
        return future.result()

    # A timer-generated fresh controller release (never a held RB or a command).
    released = [None]
    def sample():
        if released[0] is None:
            return
        message = Joy()
        message.header.stamp = producer.get_clock().now().to_msg()
        message.axes = [0., 0., 1., 0., 0., 1., 0., 0.]
        message.buttons = [0] * 6
        message.buttons[5] = 0 if released[0] else 1
        producer.joy_cb(message)
    timer = producer.create_timer(.02, sample)
    try:
        assert client.wait_for_service(timeout_sec=3.)
        observed = json.loads(call('observe').evidence_json)
        assert observed['owner'] is None and not observed['input_blocked']
        assert not call('acquire').accepted
        released[0] = True
        deadline = time.monotonic() + 3.
        while not producer._tuning_command_idle() and time.monotonic() < deadline:
            time.sleep(.01)
        if admission == 'block':
            # The real hold topic closes producer ingress. A new held RB is
            # discarded, not relabelled as an explicit release or replayed.
            deadline = time.monotonic() + 3.
            while hold.get_subscription_count() != 1 and time.monotonic() < deadline:
                time.sleep(.01)
            assert hold.get_subscription_count() == 1
            hold.publish(Bool(data=True))
            deadline = time.monotonic() + 3.
            while not json.loads(call('observe').evidence_json)['input_blocked'] and time.monotonic() < deadline:
                time.sleep(.01)
            assert json.loads(call('observe').evidence_json)['input_blocked']
            released[0] = False
            deadline = time.monotonic() + 3.
            while producer._tuning_input_released and time.monotonic() < deadline:
                time.sleep(.01)
            assert not producer._tuning_input_released
        assert call(admission).accepted
        assert not call('release', plan_hash='sha256:' + 'c' * 64).accepted

        # Execute the native post-set callback in its owner executor, not in the
        # test thread. Preserve a lower operator trim independent of ROS getters.
        changed, results = threading.Event(), []
        def update():
            if changed.is_set():
                return
            producer.max_lin = .10
            producer._operator_linear_trim = True
            results.append(producer.set_parameters_atomically([
                Parameter('max_linear', value=.15), Parameter('max_linear_cap', value=.15),
                Parameter('min_linear', value=.05)]))
            changed.set()
        update_timer = producer.create_timer(.02, update)
        assert changed.wait(3.)
        producer.destroy_timer(update_timer)
        assert results[0].successful
        inspected = call('inspect')
        assert inspected.accepted
        data = json.loads(inspected.evidence_json)
        assert data['policy_version'] == 1
        assert data['policy']['max_linear_cap'] == .15
        assert data['policy']['max_linear'] == .10
        assert producer.get_parameter('max_linear').value == .15
        assert data['operator_linear_trim'] is True
        assert data['input_blocked'] and data['release_required']
        assert data['input_released'] is (admission == 'acquire')
        assert set(data['buffers'].values()) == {0.0}
        assert not call('restore_trims', digest='sha256:' + 'c' * 64,
                        payload_json=json.dumps({'max_linear': .08})).accepted
        assert call('restore_trims', payload_json=json.dumps({'max_linear': .08})).accepted
        restored = json.loads(call('inspect').evidence_json)
        assert restored['policy']['max_linear'] == .08 and restored['operator_linear_trim']
        assert producer.get_parameter('max_linear').value == .15  # generated policy is not rewritten
        assert call('restore_trims', payload_json=json.dumps({'max_linear': .20})).accepted
        assert json.loads(call('inspect').evidence_json)['policy']['max_linear'] == .08  # never raises a trim
        assert call('release').accepted
        assert not producer.active
        if admission == 'block':
            hold.publish(Bool(data=False))
            deadline = time.monotonic() + 3.
            while json.loads(call('observe').evidence_json)['input_blocked'] and time.monotonic() < deadline:
                time.sleep(.01)
            assert not json.loads(call('observe').evidence_json)['input_blocked']
            assert not producer.active and producer._tuning_require_release
        assert not call('acquire').accepted
    finally:
        producer.destroy_timer(timer)
        executor.shutdown()
        thread.join(timeout=3.)
        client_node.destroy_node()
        producer.destroy_node()
        rclpy.shutdown()
