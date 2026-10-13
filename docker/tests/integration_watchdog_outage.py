"""Isolated installed-watchdog sensor-loss timing case.

Run in a network-none container after sourcing the candidate ROS overlay:
PATROLBOT_ISOLATED_TEST=1 python3 /validation/docker/tests/integration_watchdog_outage.py
Only synthetic publishers are used. Sonar is not monitored by this watchdog;
this case provides no sonar-loss acceptance.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, BatteryState
from patrolbot_interfaces.msg import BaseState
from rclpy.qos import qos_profile_sensor_data
assert os.environ.get('PATROLBOT_ISOLATED_TEST') == '1'
assert {p.name for p in Path('/sys/class/net').iterdir()} == {'lo'}, 'requires network-none isolation'
rclpy.init()
node = rclpy.create_node('isolated_watchdog_outage_test')
watchdog_script = os.environ.get('PATROLBOT_WATCHDOG_SCRIPT')
command = ([sys.executable, watchdog_script] if watchdog_script else
           ['ros2', 'run', 'patrolbot_navigation', 'patrolbot_safety_watchdog.py'])
process = subprocess.Popen(command, start_new_session=True)
seen = []
sub = node.create_subscription(Twist, '/input/safety_controller', lambda msg: seen.append((time.monotonic(), msg)), 10)
pubs = {t: node.create_publisher(typ, t, qos_profile_sensor_data if t == '/scan' else 10)
        for t, typ in [('/scan', LaserScan), ('/odom', Odometry), ('/battery', BatteryState), ('/patrolbot/base_state', BaseState)]}
scan = LaserScan(); scan.header.frame_id = 'laser_frame'; scan.ranges = [1.0, 1.0, 1.0]
odom = Odometry(); odom.pose.pose.orientation.w = 1.0
battery = BatteryState(); battery.voltage = 24.0
base = BaseState(); base.hardware_state_valid = True; base.bumpers_valid = True; base.link_connected = True
base.odom_epoch_valid = True
base.localization_recovery_required = False
messages = {'/scan': scan, '/odom': odom, '/battery': battery, '/patrolbot/base_state': base}
def discovery_counts():
    return {**{topic: pub.get_subscription_count() for topic,pub in pubs.items()},
            '/input/safety_controller': sub.get_publisher_count()}


def endpoints_matched(counts):
    return bool(counts) and all(count>0 for count in counts.values())


def hold_onset(samples,last_publication,timeout,budget):
    assert samples, 'no zero hold observed before sensor-loss deadline'
    assert all(message.linear.x==0 and message.angular.z==0 for _,message in samples), 'watchdog hold was nonzero'
    onset=samples[0][0]-last_publication
    assert timeout<=onset<=budget, ('hold onset outside sensor-loss window',onset,timeout,budget)
    return onset


last_publication={}
def spin(seconds, missing=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for topic, msg in messages.items():
            if topic == missing:
                continue
            msg.header.stamp = node.get_clock().now().to_msg()
            last_publication[topic]=time.monotonic()
            pubs[topic].publish(msg)
        rclpy.spin_once(node, timeout_sec=.025)
evidence={'physical_acceptance':False,'phases':[]}
phase='startup'
try:
    # Spend the original startup budget proving DDS discovery, so silence
    # cannot be mistaken for release before this test receives any holds.
    startup_deadline=time.monotonic()+2.0
    while time.monotonic()<startup_deadline and not (endpoints_matched(discovery_counts()) and seen):
        rclpy.spin_once(node,timeout_sec=.025)
    evidence['startup_endpoints']=discovery_counts()
    evidence['startup_hold_messages']=len(seen)
    assert endpoints_matched(discovery_counts()) and seen, (
        'watchdog startup/discovery incomplete within existing 2s budget',discovery_counts(),len(seen))
    assert all(m.linear.x==0 and m.angular.z==0 for _,m in seen), 'startup hold was nonzero'
    spin(max(0.,startup_deadline-time.monotonic()))
    seen.clear()
    spin(.5)
    assert not seen, 'watchdog failed to leave startup hold with fresh inputs'
    evidence['phases'].append({'stage':'fresh_release','holds':len(seen)})
    phase='epoch_recovery'
    base.localization_recovery_required = True
    recovery_started=time.monotonic()
    spin(.4)
    assert seen, 'epoch recovery did not assert safety hold'
    evidence['phases'].append({'stage':phase,'holds':len(seen),
        'first_zero_hold_after_change_s':seen[0][0]-recovery_started})
    assert all(m.linear.x == 0 and m.angular.z == 0 for _, m in seen)
    base.localization_recovery_required = False
    spin(.3)
    seen.clear()
    spin(.3)
    assert not seen, 'epoch recovery completion retained hold'
    # Effective watchdog timeout plus one 10 Hz evaluation period; no grace extension.
    for missing,timeout,budget in [('/scan',.5,.60),('/odom',.5,.60),
                                   ('/patrolbot/base_state',.75,.85),('/battery',1.,1.10)]:
        phase='sensor_loss:'+missing
        seen.clear()
        loss_started=time.monotonic()
        last_sent=last_publication[missing]
        spin(max(0.,budget-(loss_started-last_sent)),missing=missing)
        entry={'stage':phase,'missing_topic':missing,'timeout_s':timeout,'budget_s':budget,
               'last_publication_monotonic':last_sent,'holds':len(seen),
               'first_zero_hold_monotonic':seen[0][0] if seen else None,
               'other_channel_last_publication':{t:last_publication[t] for t in messages if t!=missing}}
        evidence['phases'].append(entry)
        entry['zero_hold_onset_s']=hold_onset(seen,last_sent,timeout,budget)
        assert all(last_publication[t]>=loss_started and
                   time.monotonic()-last_publication[t]<.1 for t in messages if t!=missing), (
                   'another channel was not replayed fresh',entry)
        spin(.5)
        seen.clear()
        spin(.5)
        assert not seen, 'sensor recovery required a restart or retained hold: '+missing
        entry['recovery_without_restart']=True
    evidence['passed']=True
    evidence['not_tested']={'sonar':'Not monitored by this watchdog; no sonar-loss acceptance.'}
except Exception as error:
    evidence.update(passed=False,failed_phase=phase,failure=str(error),endpoints=discovery_counts())
    raise
finally:
    print(json.dumps({'event':'watchdog_evidence',**evidence}),flush=True)
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL); process.wait()
    node.destroy_node(); rclpy.shutdown()
    assert process.returncode == 0, 'watchdog shutdown failed'
