"""Keybind resolution tests.

The dashboard lets operators rebind the on-screen camera controls. Resolution
happens here, as pure logic, so the rules are testable without a browser.

The important properties: there is always a working default, a bad override
never silently disables a control, and two actions can never share a key.
"""

import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from patrolbot_gimbal import keybinds as kb  # noqa: E402


# ---------------------------------------------------------------- defaults --

def test_defaults_cover_every_action():
    binds = kb.resolve({})
    assert set(binds) == set(kb.ACTIONS)


def test_defaults_are_self_consistent():
    """The shipped defaults must not themselves contain a conflict."""
    assert kb.find_conflicts(kb.DEFAULTS) == {}


def test_default_bindings_are_the_expected_keys():
    binds = kb.resolve({})
    assert binds['pan_left'] == 'ArrowLeft'
    assert binds['pan_right'] == 'ArrowRight'
    assert binds['tilt_up'] == 'ArrowUp'
    assert binds['tilt_down'] == 'ArrowDown'
    assert binds['stop'] == 'Space'


# --------------------------------------------------------------- overrides --

def test_an_override_replaces_only_that_action():
    binds = kb.resolve({'pan_left': 'KeyA'})
    assert binds['pan_left'] == 'KeyA'
    assert binds['pan_right'] == kb.DEFAULTS['pan_right']


def test_overriding_every_action_is_allowed():
    custom = {a: f'Key{chr(65 + i)}' for i, a in enumerate(kb.ACTIONS)}
    assert kb.resolve(custom) == custom


def test_unknown_action_in_overrides_is_rejected():
    """A typo must not silently do nothing."""
    with pytest.raises(kb.KeybindError):
        kb.resolve({'pan_lft': 'KeyA'})


def test_empty_key_is_rejected():
    with pytest.raises(kb.KeybindError):
        kb.resolve({'pan_left': ''})


def test_non_string_key_is_rejected():
    with pytest.raises(kb.KeybindError):
        kb.resolve({'pan_left': 37})


# --------------------------------------------------------------- conflicts --

def test_override_colliding_with_another_default_is_rejected():
    """Binding pan_left to the tilt_up default would break tilt_up."""
    with pytest.raises(kb.KeybindError) as exc:
        kb.resolve({'pan_left': kb.DEFAULTS['tilt_up']})
    assert 'tilt_up' in str(exc.value)


def test_two_overrides_colliding_with_each_other_is_rejected():
    with pytest.raises(kb.KeybindError):
        kb.resolve({'pan_left': 'KeyQ', 'pan_right': 'KeyQ'})


def test_rebinding_an_action_to_its_own_key_is_not_a_conflict():
    binds = kb.resolve({'pan_left': kb.DEFAULTS['pan_left']})
    assert binds['pan_left'] == kb.DEFAULTS['pan_left']


def test_swapping_two_bindings_is_allowed():
    """A straight swap is legal: neither key ends up bound twice."""
    binds = kb.resolve({
        'pan_left': kb.DEFAULTS['pan_right'],
        'pan_right': kb.DEFAULTS['pan_left'],
    })
    assert binds['pan_left'] == kb.DEFAULTS['pan_right']
    assert binds['pan_right'] == kb.DEFAULTS['pan_left']


def test_find_conflicts_reports_the_offending_actions():
    conflicts = kb.find_conflicts({'a': 'KeyQ', 'b': 'KeyQ', 'c': 'KeyW'})
    assert conflicts == {'KeyQ': ['a', 'b']}


# ------------------------------------------------------------------ resets --

def test_reset_returns_a_copy_not_the_shared_defaults():
    """A caller mutating the result must not corrupt DEFAULTS for everyone."""
    first = kb.resolve({})
    first['pan_left'] = 'MUTATED'
    assert kb.DEFAULTS['pan_left'] != 'MUTATED'
    assert kb.resolve({})['pan_left'] == kb.DEFAULTS['pan_left']
