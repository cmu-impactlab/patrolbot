"""Local initialization requires explicit review and cannot race a live core."""

import json
from pathlib import Path

from patrolbot_bridge import tuning_initialize as init
from patrolbot_bridge.tuning_store import RevisionStore, StoreUnavailable
import pytest


REGISTRY = json.loads((Path(__file__).resolve().parents[4]
                       / 'contracts/robot-tuning.registry.json').read_text())


def _initialize(tmp_path, **kwargs):
    return init.initialize_clean(
        tmp_path / 'config', REGISTRY, expected=init.reviewed_candidate(REGISTRY),
        reason='unit fixture clean install', confirm_clean_install=True,
        _effective_uid=0, **kwargs)


def test_explicit_review_initializes_yaml_once_without_runtime_activation(tmp_path):
    result = _initialize(tmp_path)
    assert result['desired_revision'] == 1
    assert init.marker_path(tmp_path / 'config').is_file()
    reopened = RevisionStore(tmp_path / 'config', REGISTRY)
    assert reopened.read_startup()['revision'] == 1
    assert reopened.desired_overrides()['amcl.alpha1'] == .05
    with pytest.raises(StoreUnavailable):
        _initialize(tmp_path)


def test_core_runtime_guard_blocks_initialization(tmp_path):
    with init.RuntimeGuard(tmp_path / 'config'):
        with pytest.raises(BlockingIOError):
            _initialize(tmp_path)
    assert not (tmp_path / 'config').exists()
    assert not init.marker_path(tmp_path / 'config').exists()


def test_exclusive_initialization_guard_blocks_core_startup(tmp_path):
    with init.RuntimeGuard(tmp_path / 'config', exclusive=True):
        with pytest.raises(BlockingIOError):
            init.RuntimeGuard(tmp_path / 'config')


def test_interrupted_or_deleted_initialized_state_never_looks_clean(tmp_path, monkeypatch):
    def fail(*args, **kwargs):
        raise OSError('interrupted persistence')

    monkeypatch.setattr(RevisionStore, '_initialize', fail)
    with pytest.raises(OSError):
        _initialize(tmp_path)
    assert init.marker_path(tmp_path / 'config').exists()
    with pytest.raises(StoreUnavailable, match='previous_initialization'):
        _initialize(tmp_path)


def test_unreviewed_source_or_non_root_never_initializes(tmp_path):
    candidate = init.reviewed_candidate(REGISTRY)
    with pytest.raises(StoreUnavailable, match='local_administrator'):
        init.initialize_clean(tmp_path / 'config', REGISTRY, expected=candidate,
                              reason='fixture', confirm_clean_install=True, _effective_uid=1000)
    with pytest.raises(StoreUnavailable, match='reviewed_candidate_changed'):
        init.initialize_clean(tmp_path / 'config', REGISTRY, expected={},
                              reason='fixture', confirm_clean_install=True, _effective_uid=0)
    with pytest.raises(StoreUnavailable, match='confirmation'):
        init.initialize_clean(tmp_path / 'config', REGISTRY, expected=candidate,
                              reason='fixture', _effective_uid=0)
    assert not (tmp_path / 'config').exists()
