"""
Bridge-owned turn hold: only E-stop or fault evidence stops the spin.

Authorized 2026-10-16 with the bridge-owned turn: while spinning, the
full hardware suite narrows to E-stop plus contact/fault evidence.
Per-bumper signals are unresolved on this platform (masks default to 0),
so contact evidence is the firmware fault/stall flags; degraded warnings
veto as well until #9 commissions mapped stall bits.
"""

import ast
import math
from pathlib import Path
import struct
from types import SimpleNamespace as NS


def _method():
    source = Path(__file__).parents[1] / 'patrolbot_bridge' / 'undock_turn.py'
    tree = ast.parse(source.read_text())
    runtime = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'UndockTurnRuntime'
    )
    method = next(
        node
        for node in runtime.body
        if isinstance(node, ast.FunctionDef) and node.name == 'spin_hold_problem'
    )
    namespace: dict = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    return namespace['spin_hold_problem']


def _runtime(estop=False, faults=0):
    return NS(node=NS(_estop_pressed=estop, _fault_flags=faults))


def test_hold_clear_when_healthy():
    assert _method()(_runtime()) == ''


def test_estop_asserted_holds_the_turn():
    assert _method()(_runtime(estop=True)) != ''


def test_any_fault_flags_hold_the_turn():
    for faults in (1, 1 << 7, 1 << 8, 255):
        assert _method()(_runtime(faults=faults)) != ''


def test_estop_takes_precedence_for_the_message():
    assert 'E-stop' in _method()(_runtime(estop=True, faults=3))


def _target_checker():
    source = Path(__file__).parents[1] / 'patrolbot_bridge' / 'undock_turn.py'
    tree = ast.parse(source.read_text())
    pieces = [
        node
        for node in tree.body
        if (
            isinstance(node, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == 'PI_FLOAT32' for t in node.targets)
        )
        or (isinstance(node, ast.FunctionDef) and node.name == 'acceptable_turn_target')
    ]
    namespace = {'math': math, 'struct': struct}
    exec(compile(ast.Module(body=pieces, type_ignores=[]), str(source), 'exec'), namespace)
    return namespace['acceptable_turn_target'], namespace['PI_FLOAT32']


def test_float32_rounded_default_pi_is_accepted():
    # The manager's default math.pi travels through a float32 field and
    # arrives marginally above the double constant; admission must honor
    # the default configuration instead of refusing it on the wire.
    check, pi32 = _target_checker()
    assert pi32 > math.pi
    assert check(math.pi)
    assert check(pi32)
    assert not check(pi32 + 1e-6)
    assert not check(0.0)
    assert not check(math.nan)
