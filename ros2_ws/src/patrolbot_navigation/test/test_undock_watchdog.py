"""The unlocalized watchdog allowance belongs only to a fresh owned turn."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import sys

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'patrolbot_safety_watchdog.py'
sys.path.insert(0, str(SCRIPT.parent))
spec = importlib.util.spec_from_file_location('undock_watchdog', SCRIPT)
watchdog = importlib.util.module_from_spec(spec)
spec.loader.exec_module(watchdog)


def base():
    return NS(undock_turn_active=True, undock_turn_goal_id='abcdef0123456789' * 2,
              link_connected=True, hardware_state_valid=True, odom_epoch_valid=True,
              motors_enabled=True, dock_state_valid=True, dock_state='CLEAR_CONFIRMED',
              undock_active=False)


def test_scoped_turn_does_not_require_map_localization():
    message = base()
    message.localization_recovery_required = True
    assert watchdog.undock_turn_owned(message)


@pytest.mark.parametrize('field,value', [
    ('undock_turn_active', False), ('undock_turn_goal_id', ''),
    ('undock_turn_goal_id', 'not-a-goal'), ('link_connected', False),
    ('hardware_state_valid', False), ('odom_epoch_valid', False),
    ('motors_enabled', False), ('dock_state_valid', False),
    ('dock_state', 'DEPARTING'), ('undock_active', True)])
def test_no_generic_recovery_or_hardware_bypass(field, value):
    message = base()
    setattr(message, field, value)
    assert not watchdog.undock_turn_owned(message)
