"""Exercise the shipped smoother-manager hold methods without importing ROS."""

import ast
import json
import os
from pathlib import Path
from types import SimpleNamespace
import uuid
from patrolbot_bridge.tuning_lease import MaintenanceLease, serve


SOURCE = Path(__file__).resolve().parents[2] / 'patrolbot-launch/lifecycle_mgr.py'


def manager():
    tree = ast.parse(SOURCE.read_text())
    klass = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    methods = [node for node in klass.body if isinstance(node, ast.FunctionDef)
               and node.name in ('_durable_tuning_hold', '_set_maintenance', '_set_maintenance_v2', 'ensure_active')]
    namespace = {'Path': Path, 'os': os, 'json': json, 'uuid': uuid, 'serve': serve}
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])),
                 str(SOURCE), 'exec'), namespace)
    obj = SimpleNamespace(_maintenance=False, _legacy_maintenance=False,
                          _transition_busy=False, _tuning_lease=MaintenanceLease())
    obj._durable_tuning_hold = lambda: namespace['_durable_tuning_hold'](obj)
    obj._set_maintenance = lambda request, response: namespace['_set_maintenance'](
        obj, request, response)
    obj.ensure_active = lambda: namespace['ensure_active'](obj)
    obj._set_maintenance_v2 = lambda request, response: namespace['_set_maintenance_v2'](obj, request, response)
    return obj


def test_unknown_job_blocks_automatic_smoother_reactivation(tmp_path, monkeypatch):
    monkeypatch.setenv('PATROLBOT_STATE_DIR', str(tmp_path))
    base = tmp_path / 'config'
    (base / 'jobs').mkdir(parents=True)
    job_id = '11111111-1111-4111-8111-111111111111'
    (base / 'manifest.json').write_text(json.dumps({'latest_job': job_id}))
    job_file = base / 'jobs' / (job_id + '.json')
    job_file.write_text(json.dumps({'state': 'unknown'}))
    node = manager()
    assert node._durable_tuning_hold()
    # No get_state/change method exists: an attempted auto-transition would fail.
    node.ensure_active()


def test_v2_manager_ownership_cannot_be_cleared_by_legacy_bool_or_foreign_job():
    node = manager()
    owner = {'job_id': '11111111-1111-4111-8111-111111111111',
             'digest': 'sha256:' + 'a' * 64, 'plan_hash': 'sha256:' + 'b' * 64}
    assert node._set_maintenance_v2(SimpleNamespace(operation='acquire', **owner), SimpleNamespace()).accepted
    node._set_maintenance(SimpleNamespace(data=False), SimpleNamespace())
    assert node._maintenance
    foreign = dict(owner, job_id='22222222-2222-4222-8222-222222222222')
    assert not node._set_maintenance_v2(SimpleNamespace(operation='release', **foreign), SimpleNamespace()).accepted
    assert node._maintenance
    assert node._set_maintenance_v2(SimpleNamespace(operation='release', **owner), SimpleNamespace()).accepted
    assert not node._maintenance


def test_missing_or_corrupt_initialized_config_prevents_autoactive(tmp_path, monkeypatch):
    monkeypatch.setenv('PATROLBOT_STATE_DIR', str(tmp_path))
    (tmp_path / 'tuning-initialized.json').write_text('{}')
    node = manager()
    assert node._durable_tuning_hold()
    (tmp_path / 'config').mkdir()
    (tmp_path / 'config/manifest.json').write_text('{bad-json')
    assert node._durable_tuning_hold()


def test_maintenance_does_not_acknowledge_an_inflight_lifecycle_change():
    node = manager()
    node._transition_busy = True
    result = node._set_maintenance(SimpleNamespace(data=True), SimpleNamespace())
    assert not result.success and node._maintenance
    node._transition_busy = False
    result = node._set_maintenance(SimpleNamespace(data=True), SimpleNamespace())
    assert result.success
    node.ensure_active()
