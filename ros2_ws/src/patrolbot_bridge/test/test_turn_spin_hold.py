"""Bridge-owned turn hold: only E-stop or fault evidence stops the spin.

Authorized 2026-10-16 with the bridge-owned turn: while spinning, the
full hardware suite narrows to E-stop plus contact/fault evidence.
Per-bumper signals are unresolved on this platform (masks default to 0),
so contact evidence is the firmware fault/stall flags; degraded warnings
veto as well until #9 commissions mapped stall bits.
"""
import ast
from pathlib import Path
from types import SimpleNamespace as NS


def _method():
    source = Path(__file__).parents[1] / 'patrolbot_bridge' / 'undock_turn.py'
    tree = ast.parse(source.read_text())
    runtime = next(node for node in tree.body
                   if isinstance(node, ast.ClassDef)
                   and node.name == 'UndockTurnRuntime')
    method = next(node for node in runtime.body
                  if isinstance(node, ast.FunctionDef)
                  and node.name == 'spin_hold_problem')
    namespace: dict = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source),
                 'exec'), namespace)
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
