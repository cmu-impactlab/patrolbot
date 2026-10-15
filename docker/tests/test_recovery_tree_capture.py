import importlib.util
from pathlib import Path
import pytest


def contract():
    spec = importlib.util.spec_from_file_location('recovery_tree_capture', Path(__file__).with_name('recovery_tree_capture.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.validate


def test_complete_production_recovery_exhaustion():
    check = contract()
    capture = dict(mode='exhaust', status=6, follow_calls=20,
        recovery_actions=['backup','clear','spin','wait']*2+['backup'], elapsed_s=30., contextual_clear_calls=10, navigator_exit_code=0)
    check(capture)
    for key,value in [('status',4),('follow_calls',18),('recovery_actions',['clear']),('elapsed_s',121.),('contextual_clear_calls',0),('navigator_exit_code',-6)]:
        with pytest.raises(AssertionError): check(dict(capture,**{key:value}))


def test_fault_clear_requires_complete_cycle_then_success():
    check = contract()
    capture = dict(mode='fault-clears', status=4, follow_calls=9,
        recovery_actions=['backup','clear','spin','wait'], elapsed_s=30., contextual_clear_calls=4, navigator_exit_code=0)
    check(capture)
    for key,value in [('status',6),('follow_calls',1),('recovery_actions',[])]:
        with pytest.raises(AssertionError): check(dict(capture,**{key:value}))


def test_planner_failure_clears_once_and_never_starts_motion_recovery():
    check = contract()
    capture = dict(mode='planner-exhaust', status=6, follow_calls=0, plan_calls=2,
        recovery_actions=[], elapsed_s=30.,
        planner_contextual_clear_calls=1, navigator_exit_code=0)
    check(capture)
    for key,value in [('status',4),('follow_calls',1),('plan_calls',20),
                      ('recovery_actions',['backup']),('planner_contextual_clear_calls',0)]:
        with pytest.raises(AssertionError): check(dict(capture,**{key:value}))
