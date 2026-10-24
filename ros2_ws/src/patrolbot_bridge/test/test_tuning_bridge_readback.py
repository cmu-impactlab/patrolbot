"""Actual bridge readback worker charges native RPC latency to sample age."""

import ast
from pathlib import Path
import threading
from types import MethodType, SimpleNamespace as NS

import pytest

SOURCE = Path(__file__).resolve().parents[1] / 'patrolbot_bridge/bridge_node.py'


def bridge(*, delay, fault=None):
    now = [100.]
    tree = ast.parse(SOURCE.read_text())
    klass = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == 'PatrolBotBridge')
    methods = [item for item in klass.body if isinstance(item, ast.FunctionDef)
               and item.name in ('_poll_tuning_readback', '_current_tuning_readback')]
    namespace = {'time': NS(monotonic=lambda: now[0])}
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])), str(SOURCE), 'exec'), namespace)
    index = {name: {'accepts_edits': True, 'source': {'node': 'teleop_velocity_smoother',
             'key': 'max_velocity', 'index': axis}}
             for name, axis in (('max_linear_m_s', 0), ('max_angular_rad_s', 2))}
    store = NS(index=index, snapshot_state=lambda: {'active_revision': 1},
               load_revision=lambda revision: {'meta': {'generated_digest': 'sha256:' + 'a' * 64}})
    node = NS(_tuning_store=store, _tuning_readback_lock=threading.Lock(),
              _epoch=NS(tuning_hold=False), _session_generation=7)
    def inspect(revision, digest):
        assert revision == 1 and digest == 'sha256:' + 'a' * 64
        now[0] += delay
        if fault == 'generation':
            node._session_generation += 1
        elif fault == 'maintenance':
            node._epoch.tuning_hold = True
        return {}, {'teleop_velocity_smoother': {'max_velocity': [.3, 0., .7]}}
    node._tuning_backend = NS(inspect=inspect)
    for name in ('_poll_tuning_readback', '_current_tuning_readback'):
        setattr(node, name, MethodType(namespace[name], node))
    return node


@pytest.mark.parametrize('delay,fresh', [(0., True), (2.99, True), (3., True), (3.01, False)])
def test_readback_age_begins_before_native_observation_not_after_its_last_reply(delay, fresh):
    node = bridge(delay=delay)
    node._poll_tuning_readback()
    assert node._tuning_readback_at == 100.
    current = node._current_tuning_readback()
    assert (current is not None) is fresh
    if current is not None:
        assert current['revision'] == 1
        assert current['effective_limits'] == {'max_linear_m_s': .3, 'max_angular_rad_s': .7}


@pytest.mark.parametrize('fault', ['generation', 'maintenance'])
def test_a_runtime_boundary_during_observation_discards_the_entire_readback(fault):
    node = bridge(delay=.1, fault=fault)
    node._poll_tuning_readback()
    assert node._current_tuning_readback() is None
