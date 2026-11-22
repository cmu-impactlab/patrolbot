"""Exercise actual native backend admission/startup methods without importing ROS."""
import ast
import asyncio
import threading
from pathlib import Path
from types import SimpleNamespace as NS
from uuid import uuid4
import pytest
from patrolbot_bridge.runtime_contract import number, uuid
from patrolbot_bridge.epoch_recovery import EpochRecovery
from patrolbot_bridge.tuning_activation import Evidence, evaluate_evidence
from patrolbot_bridge.tuning_policy import PolicyCache


def backend_class():
    path = Path(__file__).resolve().parents[1] / 'patrolbot_bridge/runtime_backend.py'
    tree = ast.parse(path.read_text())
    body = [node for node in tree.body if isinstance(node, (ast.ClassDef, ast.Assign))]
    namespace = dict(threading=threading, Future=asyncio.Future, uuid4=uuid4, ReentrantCallbackGroup=object, RuntimeManual=NS(Request=NS),
                     SetBool=NS(Request=NS), number=number, uuid=uuid, evaluate_evidence=evaluate_evidence, EpochRecovery=EpochRecovery)
    exec(compile(ast.Module(body=body, type_ignores=[]), str(path), 'exec'), namespace)
    return namespace['RuntimeBackend']


def node():
    epoch = NS(lock=threading.RLock(), tuning_hold=False, _pending_pose=None,
               recovery=NS(stage=EpochRecovery.WAIT_SEED), _navigation_ready=False)
    facts = Evidence(**{key: False if key.endswith('_held') else True for key in Evidence.__dataclass_fields__})
    return NS(_epoch=epoch, _session_generation=2, _tuning_policy=PolicyCache(None),
        _tuning_evidence=lambda: facts, _control_lock=threading.RLock(), _pending_controls={},
        _manual_undock=NS(busy=False, unknown=False), _undock_turn=NS(busy=False, unknown=False),
        _undock_goal_reserved=False, create_client=lambda *a, **k: NS(service_is_ready=lambda:True),
        create_timer=lambda *a, **k: NS(cancel=lambda:None))


@pytest.mark.asyncio
async def test_core_restart_discovers_surviving_reservation_without_reapplying_or_releasing():
    n = node()
    backend = backend_class()(n, enabled=True)
    assert n._epoch.runtime_hold
    request_id = str(uuid4())
    calls = []
    async def consumer(name, phase, read_id):
        calls.append((name, phase))
        return dict(reservation=request_id if name == 'f710' else '')
    backend.consumer = consumer
    await backend.initialize()
    assert calls == [('web', 'read'), ('f710', 'read')]
    assert backend.reservation == request_id
    assert n._epoch.runtime_hold
    with pytest.raises(ValueError): backend.reserve(str(uuid4()), dict(role='operator'))


@pytest.mark.asyncio
async def test_missing_consumer_keeps_startup_admission_unknown_and_fresh_defaults_release_only_core():
    n = node()
    backend = backend_class()(n, enabled=True)
    async def missing(*args): raise TimeoutError('Offline missing consumer')
    backend.consumer = missing
    await backend.initialize()
    assert backend.initializing and n._epoch.runtime_hold
    async def defaults(*args): return dict(reservation='')
    backend.consumer = defaults
    await backend.initialize()
    assert not backend.initializing and not n._epoch.runtime_hold
    assert not n._epoch.runtime_block_drive
    request_id = str(uuid4())
    backend.reserve(request_id, dict(role='operator'))
    assert backend.reservation == request_id and n._epoch.runtime_hold
    assert not n._epoch.runtime_block_drive
    backend.begin_commit(backend.identity())
    assert n._epoch.runtime_block_drive
    backend.release(request_id)
    assert not backend.reservation and not n._epoch.runtime_hold


@pytest.mark.parametrize('owner', ['motion', 'tuning', 'control', 'manual_undock', 'undock_unknown', 'pending_pose', 'lifecycle_pending'])
def test_actual_core_reservation_checks_all_busy_or_unknown_sources(owner):
    n = node()
    backend = backend_class()(n, enabled=True)
    n._epoch.runtime_hold = False
    if owner == 'motion':
        n._tuning_evidence = lambda: Evidence()
    if owner == 'tuning': n._epoch.tuning_hold = True
    if owner == 'control': n._pending_controls['x'] = NS()
    if owner == 'manual_undock': n._manual_undock.busy = True
    if owner == 'undock_unknown': n._undock_turn.unknown = True
    if owner == 'pending_pose': n._epoch._pending_pose = object()
    if owner == 'lifecycle_pending': n._epoch.recovery.stage = EpochRecovery.READY
    with pytest.raises(ValueError): backend.reserve(str(uuid4()), dict(role='operator'))
    assert not backend.reservation


def test_epoch_reservation_does_not_start_lifecycle_or_erase_accepted_localization():
    path = Path(__file__).resolve().parents[1] / 'patrolbot_bridge/epoch_runtime.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'EpochRuntime')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'tick')
    namespace = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
    probes = []
    epoch = NS(lock=threading.RLock(), tuning_hold=False, runtime_hold=True, undock_hold=False,
        _navigation_ready=True, _accepted_seed_stamp=21,
        node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=1))),
        recovery=NS(observe_clock=lambda stamp:None, token=object()),
        _poll_tf_readiness=lambda:probes.append('tf'), poll_navigation_idle=lambda:probes.append('navigation'),
        tick_tuning_map_gate=lambda:pytest.fail('runtime reservation must serialize new lifecycle/map effects'))
    namespace['tick'](epoch)
    assert epoch._navigation_ready and epoch._accepted_seed_stamp == 21
    assert probes == ['tf', 'navigation']


@pytest.mark.asyncio
@pytest.mark.parametrize('timeout', [False, True])
async def test_native_call_has_bounded_completion_and_retires_late_future(timeout):
    n = node()
    backend = backend_class()(n, enabled=True)
    loop = asyncio.get_running_loop()
    future = loop.create_future()
    removed = []
    client = NS(service_is_ready=lambda:True,call_async=lambda request:future,
                remove_pending_request=removed.append)
    n.create_timer = lambda seconds, callback, **kwargs: loop.call_later(.001, callback)
    n.destroy_timer = lambda timer: timer.cancel()
    if timeout:
        with pytest.raises(TimeoutError): await backend.call(client, NS())
        assert removed == [future] and future.cancelled()
    else:
        expected = NS(accepted=True)
        loop.call_soon(future.set_result, expected)
        assert await backend.call(client, NS()) is expected
        assert not removed

@pytest.mark.parametrize('key', [None, b'offline-key'])
def test_pristine_default_does_not_require_optional_consumers(key):
    n = node()
    backend = backend_class()(n, key)
    assert not getattr(n._epoch, 'runtime_hold', False)
    assert not getattr(n._epoch, 'runtime_block_drive', False)
    assert not backend.initializing


@pytest.mark.asyncio
async def test_enabled_epoch_loss_stays_held_after_service_returns_until_explicit_recovery():
    n = node()
    backend = backend_class()(n, enabled=True)
    async def healthy(*args): return dict(reservation='')
    backend.consumer = healthy
    await backend.initialize()
    assert not n._epoch.runtime_hold
    async def lost(*args): raise TimeoutError('Consumer lost after enabled epoch')
    backend.consumer = lost
    await backend.initialize()
    assert n._epoch.runtime_hold and n._epoch.runtime_block_drive
    backend.consumer = healthy
    await backend.initialize()
    assert n._epoch.runtime_hold and n._epoch.runtime_block_drive

@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.asyncio
async def test_feature_mode_is_static_and_missing_mutation_key_is_not_policy_discovery(enabled):
    n = node()
    backend = backend_class()(n, None, enabled=enabled)
    calls = []
    async def current(*args):
        calls.append(args)
        return dict(reservation='', available=False)
    backend.consumer = current
    await backend.initialize()
    assert not n._epoch.runtime_hold and not n._epoch.runtime_block_drive
    assert bool(calls) == enabled
    backend.key = None
    assert backend.enabled == enabled


def test_disabled_mode_with_missing_optional_web_service_has_no_global_hold_or_mutations():
    n = node()
    n.create_client = lambda *a, **kw: NS(service_is_ready=lambda:False)
    backend = backend_class()(n, enabled=False)
    assert not n._epoch.runtime_hold and not n._epoch.runtime_block_drive
    with pytest.raises(ValueError, match='statically disabled'):
        backend.reserve(str(uuid4()), dict(role='operator'))

@pytest.mark.asyncio
async def test_monitor_cannot_recreate_released_hold_from_a_retired_transaction_read():
    n = node()
    backend = backend_class()(n, enabled=True)
    async def defaults(*args): return dict(reservation='')
    backend.consumer = defaults
    await backend.initialize()
    old_id = str(uuid4())
    async def straddle(name, phase, read_id):
        if name == 'web':
            backend.reserve(old_id, dict(role='operator'))
            backend.release(old_id)
        return dict(reservation=old_id)
    backend.consumer = straddle
    await backend.initialize()
    assert not backend.reservation
    assert not n._epoch.runtime_hold and not n._epoch.runtime_block_drive
