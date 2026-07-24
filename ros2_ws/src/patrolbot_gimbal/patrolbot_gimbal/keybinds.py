"""Keybind resolution for the dashboard camera controls.

Pure logic so the rules are testable without a browser. The dashboard sends
the operator's overrides, this merges them onto the defaults and rejects
anything that would leave a control unreachable or ambiguous.

Key names are DOM KeyboardEvent.code values ('ArrowLeft', 'KeyA', 'Space'),
which are layout-independent — a Dvorak user rebinding to 'KeyA' gets the same
physical key a QWERTY user does.
"""

from typing import Dict, List, Mapping


class KeybindError(ValueError):
    """An override was malformed, unknown, or conflicting."""


# Every rebindable action. The widget renders controls in this order.
ACTIONS = (
    'pan_left',
    'pan_right',
    'tilt_up',
    'tilt_down',
    'roll_ccw',
    'roll_cw',
    'home',
    'stop',
)

# Shipped defaults. Arrows for pan/tilt because that matches the on-screen
# arrow pad; comma/period for roll because they sit under the fingers without
# colliding with anything; Space to stop because it is the largest key and
# stopping should be the easiest thing to do by accident-proof reflex.
DEFAULTS: Dict[str, str] = {
    'pan_left': 'ArrowLeft',
    'pan_right': 'ArrowRight',
    'tilt_up': 'ArrowUp',
    'tilt_down': 'ArrowDown',
    'roll_ccw': 'Comma',
    'roll_cw': 'Period',
    'home': 'Home',
    'stop': 'Space',
}


def find_conflicts(binds: Mapping[str, str]) -> Dict[str, List[str]]:
    """Return {key: [actions]} for every key bound more than once."""
    by_key: Dict[str, List[str]] = {}
    for action, key in binds.items():
        by_key.setdefault(key, []).append(action)
    return {key: sorted(actions)
            for key, actions in by_key.items() if len(actions) > 1}


def _validate_overrides(overrides: Mapping[str, str]) -> None:
    for action, key in overrides.items():
        if action not in DEFAULTS:
            raise KeybindError(
                f'unknown action {action!r}; expected one of '
                f'{", ".join(ACTIONS)}')
        if not isinstance(key, str):
            raise KeybindError(
                f'key for {action!r} must be a string, got '
                f'{type(key).__name__}')
        if not key.strip():
            raise KeybindError(
                f'key for {action!r} is empty; use the default or pick a key')


def resolve(overrides: Mapping[str, str]) -> Dict[str, str]:
    """Merge overrides onto the defaults and validate the result.

    Returns a fresh dict, so a caller mutating it cannot corrupt DEFAULTS for
    everyone else in the process.

    Raises KeybindError on an unknown action, a malformed key, or any
    resulting conflict. Rejecting is deliberate: silently dropping a bad
    override would leave a control that looks bound but does nothing.
    """
    _validate_overrides(overrides)

    binds = dict(DEFAULTS)
    binds.update(overrides)

    conflicts = find_conflicts(binds)
    if conflicts:
        detail = '; '.join(
            f'{key} is bound to {" and ".join(actions)}'
            for key, actions in sorted(conflicts.items()))
        raise KeybindError(f'conflicting keybinds: {detail}')

    return binds


def describe(binds: Mapping[str, str]) -> List[Dict[str, str]]:
    """Render bindings in display order, for the settings panel."""
    return [{'action': action, 'key': binds[action]}
            for action in ACTIONS if action in binds]
