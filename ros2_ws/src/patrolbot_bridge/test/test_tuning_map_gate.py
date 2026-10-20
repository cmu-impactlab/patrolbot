"""Test the shipped post-tuning map gate without a ROS graph or motion."""

import ast
from pathlib import Path
from types import SimpleNamespace as NS


SOURCE = Path(__file__).resolve().parents[1] / 'patrolbot_bridge/map_runtime.py'


class Future:
    def __init__(self, response=None):
        self.response = response

    def done(self):
        return self.response is not None

    def result(self):
        return self.response


class Client:
    def __init__(self):
        self.calls = []
        self.future = None

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.calls.append(request)
        self.future = Future()
        return self.future


def setup():
    tree = ast.parse(SOURCE.read_text())
    klass = next(item for item in tree.body if isinstance(item, ast.ClassDef))
    method = next(item for item in klass.body if isinstance(item, ast.FunctionDef)
                  and item.name == 'tick_tuning_map_gate')
    namespace = {'EpochRecovery': NS(READY='ready'), 'Trigger': NS(Request=NS),
                 'SetBool': NS(Request=NS)}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])),
                 str(SOURCE), 'exec'), namespace)
    owner = NS(_tuning_nav_pending=True, _tuning_nav_reserved=True,
               node=NS(_tuning_navigation_released=lambda: None),
               guard=NS(ready=True), recovery=NS(stage='wait-seed'),
               _navigation_ready=False, _tuning_map_probe=None, _tuning_map_phase='global-map',
               _tuning_map_identity='1:2:100:200', _global_map=Client(),
               _tuning_nav_release=Client())

    def tick():
        namespace['tick_tuning_map_gate'](owner)

    return owner, tick


def test_no_navigation_release_before_new_seed_and_exact_global_map_ack():
    owner, tick = setup()
    tick()
    assert not owner._global_map.calls and not owner._tuning_nav_release.calls
    owner.recovery.stage = 'ready'
    tick()
    assert not owner._global_map.calls
    owner._navigation_ready = True
    tick()
    owner._global_map.future.response = NS(success=True, message='different-map')
    tick()
    assert owner._tuning_nav_pending and not owner._tuning_nav_release.calls
    tick()
    owner._global_map.future.response = NS(success=True, message='1:2:100:200')
    tick()
    assert owner._tuning_nav_release.calls[-1].data is False
    assert owner._tuning_nav_pending
    owner._tuning_nav_release.future.response = NS(success=True)
    tick()
    assert not owner._tuning_nav_pending


def test_hardware_or_origin_loss_prevents_even_a_late_release_ack():
    owner, tick = setup()
    owner.recovery.stage = 'ready'
    owner._navigation_ready = True
    tick()
    owner._global_map.future.response = NS(success=True, message='1:2:100:200')
    owner.guard.ready = False
    tick()
    assert owner._tuning_nav_pending
    assert not owner._tuning_nav_release.calls


def test_bridge_restart_reacquires_persisted_navigation_gate_before_new_seed():
    owner, tick = setup()
    owner._tuning_nav_reserved = False
    tick()
    assert owner._tuning_nav_release.calls[-1].data is True
    owner._tuning_nav_release.future.response = NS(success=True)
    tick()
    assert owner._tuning_nav_reserved
    assert owner._tuning_nav_pending
    assert not owner._global_map.calls
