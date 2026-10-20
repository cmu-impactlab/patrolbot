"""Fixed consumer launch guards exclude initialization without starting ROS."""

import os

from patrolbot_bridge.tuning_initialize import RuntimeGuard
from patrolbot_bridge.tuning_run import prepare
import pytest


def test_enabled_consumer_retains_shared_guard_for_its_entire_launch(tmp_path, monkeypatch):
    monkeypatch.setenv('PATROLBOT_TUNING_DIR', str(tmp_path / 'config'))
    monkeypatch.setenv('PATROLBOT_TUNING_ENABLED', '1')
    with RuntimeGuard(tmp_path / 'config'):
        guard, command = prepare('navigation')
    try:
        assert command == ['ros2', 'launch', 'patrolbot_navigation', 'bringup.launch.py']
        assert os.get_inheritable(guard.fd)
        with pytest.raises(BlockingIOError):
            RuntimeGuard(tmp_path / 'config', exclusive=True)
    finally:
        guard.close()
    with RuntimeGuard(tmp_path / 'config', exclusive=True):
        pass


def test_enabled_consumer_never_ignores_missing_state_guard(tmp_path, monkeypatch):
    monkeypatch.setenv('PATROLBOT_TUNING_DIR', str(tmp_path / 'config'))
    monkeypatch.setenv('PATROLBOT_TUNING_ENABLED', '1')
    with pytest.raises(FileNotFoundError):
        prepare('navigation')
    assert list(tmp_path.iterdir()) == []


def test_legacy_role_with_no_tuning_state_remains_unchanged(tmp_path, monkeypatch):
    monkeypatch.setenv('PATROLBOT_TUNING_DIR', str(tmp_path / 'config'))
    monkeypatch.setenv('PATROLBOT_TUNING_ENABLED', '0')
    guard, command = prepare('navigation')
    assert guard is None
    assert command == ['ros2', 'launch', 'patrolbot_navigation', 'bringup.launch.py']
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('role', ['bridge; reboot', '../../outside', '', None])
def test_no_arbitrary_role_or_shell_command_is_accepted(role):
    with pytest.raises(ValueError):
        prepare(role)
