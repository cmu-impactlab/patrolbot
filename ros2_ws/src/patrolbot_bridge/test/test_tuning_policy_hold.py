"""Policy refresh failures recover independently of durable job holds."""

import ast
import json
from pathlib import Path
import threading
from types import MethodType, SimpleNamespace as NS

from patrolbot_bridge.tuning_policy import limits_from_store
from patrolbot_bridge.tuning_store import RevisionStore


ROOT = Path(__file__).resolve().parents[4]
SOURCE = ROOT / 'ros2_ws/src/patrolbot_bridge/patrolbot_bridge/epoch_runtime.py'
REGISTRY = json.loads((ROOT / 'contracts/robot-tuning.registry.json').read_text())
ADMIN = {'id': 1, 'username': 'fixture', 'role': 'administrator'}


def owner():
    tree = ast.parse(SOURCE.read_text())
    klass = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    names = ('set_tuning_hold', 'set_tuning_policy_hold', '_set_tuning_hold_source')
    methods = [node for node in klass.body if isinstance(node, ast.FunctionDef)
               and node.name in names]
    namespace = {}
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])),
                 str(SOURCE), 'exec'), namespace)
    node = NS(get_logger=lambda: NS(warning=lambda _: None))
    epoch = NS(lock=threading.RLock(), node=node, tuning_hold=False,
               _tuning_hold_sources={}, _pending_pose=None, _accepted_seed_stamp=None,
               _navigation_ready=True)
    for name in names:
        setattr(epoch, name, MethodType(namespace[name], epoch))
    return epoch


def test_restoring_policy_never_clears_a_durable_job_hold():
    epoch = owner()
    epoch.set_tuning_policy_hold(True, 'policy read failed')
    assert epoch.tuning_hold
    epoch.set_tuning_hold(True, 'durable job unknown')
    epoch.set_tuning_policy_hold(False)
    assert epoch.tuning_hold
    epoch.set_tuning_hold(False, 'durable job unknown')
    assert not epoch.tuning_hold
    epoch.set_tuning_policy_hold(True, 'temporary read failure')
    epoch.set_tuning_policy_hold(False)
    assert not epoch.tuning_hold
    assert epoch._navigation_ready


def test_promote_and_policy_reads_share_one_revision_lock(tmp_path):
    store = RevisionStore(tmp_path / 'config', REGISTRY)
    store.initialize(actor=ADMIN, reason='unit fixture')
    store.save(request_id='11111111-1111-4111-8111-111111111111', base_revision=1,
               changes=[{'id': 'max_linear_m_s', 'value': .15}], actor=ADMIN)
    errors = []

    def promote():
        try:
            for revision in (2, 1, 2, 1):
                store.promote(revision=revision, scopes=['bringup', 'navigation'], actor=ADMIN)
        except Exception as exc:
            errors.append(exc)

    thread = threading.Thread(target=promote)
    thread.start()
    while thread.is_alive():
        try:
            _, limits = limits_from_store(store)
            assert limits['max_linear_m_s'] in (.15, .30)
        except Exception as exc:
            errors.append(exc)
    thread.join()
    assert errors == []
