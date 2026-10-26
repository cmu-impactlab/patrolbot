"""Execute the real watchdog predicate without ROS or hardware imports."""

import ast
from pathlib import Path
import sys
from types import MethodType, SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from safety_policy import evaluate_safety  # noqa: E402


def watchdog():
    path = Path(__file__).parents[1] / 'scripts' / 'patrolbot_safety_watchdog.py'
    tree = ast.parse(path.read_text())
    klass = next(n for n in tree.body
                 if isinstance(n, ast.ClassDef) and n.name == 'SafetyWatchdog')
    methods = [n for n in klass.body if isinstance(n, ast.FunctionDef) and n.name == '_reasons']
    namespace = {'evaluate_safety': evaluate_safety}
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), namespace)
    n = NS(now=10., _last_scan=None, _last_odom=None, _manual_sensor_at=None,
           _last_battery=10., _battery_voltage=25., _last_base_state=10.,
           _hardware_state_valid=True, _fault_flags=0, _estop_pressed=False,
           bumpers_enabled=True, _bumpers_valid=True, _front_bumper_pressed=False,
           _rear_bumper_pressed=False, _desired_linear=0., _desired_angular=0.,
           scan_timeout=.5, odom_timeout=.5, battery_timeout=1., base_state_timeout=.75,
           battery_stop_voltage=23., stop_on_faults=True, stop_on_estop=True)
    n._now = lambda: n.now
    n._reasons = MethodType(namespace['_reasons'], n)
    return n


def test_raw_hardware_evidence_does_not_require_amcl_gated_navigation_topics():
    n = watchdog()
    assert 'scan_stale' in n._reasons() and 'odom_stale' in n._reasons()
    n._manual_sensor_at = 10.
    assert n._reasons() == []
    n.now += .251
    assert 'scan_stale' in n._reasons() and 'odom_stale' in n._reasons()


def test_raw_evidence_never_waives_other_physical_checks():
    n = watchdog()
    n._manual_sensor_at = 10.
    n._estop_pressed = True
    n._fault_flags = 4
    n._rear_bumper_pressed = True
    n._desired_linear = -.1
    reasons = n._reasons()
    assert 'estop_asserted' in reasons
    assert 'hardware_fault_0x0004' in reasons
    assert 'rear_bumper_motion_block' in reasons
    n._hardware_state_valid = False
    assert 'hardware_state_invalid' in n._reasons()
