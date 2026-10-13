"""Network-none, installed Collision Monitor sonar-loss timing acceptance."""
import importlib.util
import json
import os
import signal
from pathlib import Path
import time

assert os.environ.get('PATROLBOT_ISOLATED_TEST') == '1'
assert {p.name for p in Path('/sys/class/net').iterdir()} == {'lo'}
os.environ['PATROLBOT_RUN_INTEGRATION'] = '1'
assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 90
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist

source = Path('/validation/ros2_ws/src/patrolbot_navigation/test/test_collision_monitor_integration.py')
spec = importlib.util.spec_from_file_location('collision_harness', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.PARAMS_FILE = Path(get_package_share_directory('patrolbot_navigation')) / 'config/nav2_params.yaml'
fixture = module.monitor.__wrapped__()
evidence = {'physical_acceptance': False, 'passed': False, 'params': str(module.PARAMS_FILE)}
started = time.monotonic()
def timed_out(_signal, _frame):
    raise TimeoutError('30-second isolated case budget exhausted')
signal.signal(signal.SIGALRM, timed_out)
signal.alarm(30)
try:
    monitor = next(fixture)
    module.assert_velocity(monitor.command(-.05), -.05)
    samples = []
    sub = monitor.create_subscription(Twist, '/input/navi',
        lambda msg: samples.append((time.monotonic(), msg.linear.x, msg.angular.z)), 10)
    assert module.wait_until(lambda: sub.get_publisher_count() == 1)
    # Continue scan and commands; sonar alone stops arriving.
    monitor.sonar()
    last_sonar = time.monotonic()
    command = Twist(); command.linear.x = -.05
    while time.monotonic() - last_sonar < .60:
        monitor.scan()
        monitor.command_pub.publish(command)
        time.sleep(.01)
    moving = [t for t, linear, angular in samples if linear < -.001]
    zeros = [t for t, linear, angular in samples if linear == 0. and angular == 0.]
    assert moving, 'no nonzero flow during fresh-sonar portion'
    assert zeros, 'no zero output by .60s sonar-loss deadline'
    onset = zeros[0] - last_sonar
    assert .5 <= onset <= .60, ('sonar-loss hold outside existing budget', onset)
    assert all(linear == 0. and angular == 0. for t, linear, angular in samples if t >= zeros[0]), 'hold released while sonar stayed stale'
    module.assert_velocity(monitor.command(-.05), -.05)
    evidence.update(passed=True, zero_hold_onset_s=onset, recovery_without_restart=True,
                    fresh_nonzero_samples=len(moving), stale_zero_samples=len(zeros))
except Exception as error:
    evidence['failure'] = str(error)
    raise
finally:
    fixture.close()
    signal.alarm(0)
    evidence['elapsed_s'] = time.monotonic() - started
    print(json.dumps({'event': 'sonar_outage_evidence', **evidence}), flush=True)
    assert evidence['elapsed_s'] <= 30., '30-second isolated case budget exhausted'
