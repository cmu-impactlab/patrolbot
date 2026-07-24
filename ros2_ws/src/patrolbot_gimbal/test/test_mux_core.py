"""Arbitration between the joystick and the dashboard.

Precedence is a stated property, not an accident of message arrival order:
the joystick always outranks the web. An operator physically holding the
controller must never be overridden by a browser somewhere else.

Pure logic with an injected clock.
"""

import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from patrolbot_gimbal.mux_core import MuxCore, SOURCE_JOY, SOURCE_WEB  # noqa: E402


class FakeClock:
    def __init__(self):
        self.t = 500.0

    def __call__(self):
        return self.t

    def advance(self, seconds):
        self.t += seconds


@pytest.fixture
def mux():
    clock = FakeClock()
    return MuxCore(clock=clock, input_timeout_s=0.5), clock


def _cmd(pan=0.0, tilt=0.0, roll=0.0, valid=True):
    return {'pan': pan, 'tilt': tilt, 'roll': roll, 'valid': valid}


# ------------------------------------------------------------- precedence --

def test_nothing_received_yields_no_output(mux):
    m, _ = mux
    assert m.select() is None


def test_web_alone_is_forwarded(mux):
    m, _ = mux
    m.submit(SOURCE_WEB, _cmd(pan=10.0))
    chosen = m.select()
    assert chosen.source == SOURCE_WEB
    assert chosen.command['pan'] == 10.0


def test_joy_alone_is_forwarded(mux):
    m, _ = mux
    m.submit(SOURCE_JOY, _cmd(pan=20.0))
    assert m.select().source == SOURCE_JOY


def test_joy_outranks_web_when_both_are_valid(mux):
    m, _ = mux
    m.submit(SOURCE_WEB, _cmd(pan=10.0))
    m.submit(SOURCE_JOY, _cmd(pan=20.0))
    chosen = m.select()
    assert chosen.source == SOURCE_JOY
    assert chosen.command['pan'] == 20.0


def test_joy_outranks_web_regardless_of_arrival_order(mux):
    """Precedence must not depend on which message happened to land last."""
    m, _ = mux
    m.submit(SOURCE_JOY, _cmd(pan=20.0))
    m.submit(SOURCE_WEB, _cmd(pan=10.0))
    assert m.select().source == SOURCE_JOY


# ------------------------------------------------------- invalid handling --

def test_invalid_joy_yields_to_valid_web(mux):
    """Deadman released: the browser may drive the camera again."""
    m, _ = mux
    m.submit(SOURCE_WEB, _cmd(pan=10.0))
    m.submit(SOURCE_JOY, _cmd(valid=False))
    chosen = m.select()
    assert chosen.source == SOURCE_WEB
    assert chosen.command['pan'] == 10.0


def test_both_invalid_produces_an_explicit_hold(mux):
    """A hold is published, not silence — the driver must hear the difference."""
    m, _ = mux
    m.submit(SOURCE_WEB, _cmd(valid=False))
    m.submit(SOURCE_JOY, _cmd(valid=False))
    chosen = m.select()
    assert chosen is not None
    assert not chosen.command['valid']


def test_holding_joy_still_wins_over_valid_web(mux):
    """An operator holding the deadman with a centred stick means 'hold'."""
    m, _ = mux
    m.submit(SOURCE_WEB, _cmd(pan=10.0))
    m.submit(SOURCE_JOY, _cmd(pan=0.0, valid=True))
    chosen = m.select()
    assert chosen.source == SOURCE_JOY
    assert chosen.command['pan'] == 0.0


# --------------------------------------------------------------- staleness --

def test_a_stale_input_is_ignored(mux):
    m, clock = mux
    m.submit(SOURCE_JOY, _cmd(pan=20.0))
    clock.advance(0.6)
    m.submit(SOURCE_WEB, _cmd(pan=10.0))
    assert m.select().source == SOURCE_WEB


def test_all_inputs_stale_produces_a_hold(mux):
    """A publisher that dies must not leave its last command winning."""
    m, clock = mux
    m.submit(SOURCE_JOY, _cmd(pan=20.0))
    m.submit(SOURCE_WEB, _cmd(pan=10.0))
    clock.advance(0.6)
    chosen = m.select()
    assert chosen is not None
    assert not chosen.command['valid']


def test_input_exactly_at_timeout_is_stale(mux):
    m, clock = mux
    m.submit(SOURCE_JOY, _cmd(pan=20.0))
    clock.advance(0.5)
    chosen = m.select()
    assert not chosen.command['valid']


def test_refreshed_input_is_not_stale(mux):
    m, clock = mux
    m.submit(SOURCE_JOY, _cmd(pan=20.0))
    clock.advance(0.4)
    m.submit(SOURCE_JOY, _cmd(pan=20.0))
    clock.advance(0.4)
    assert m.select().source == SOURCE_JOY


# ------------------------------------------------------------------ safety --

def test_unknown_source_is_rejected(mux):
    """Only declared inputs may ever reach the driver."""
    m, _ = mux
    with pytest.raises(ValueError):
        m.submit('something_else', _cmd(pan=99.0))


def test_active_source_is_reported_for_the_ui(mux):
    m, _ = mux
    assert m.active_source() is None
    m.submit(SOURCE_WEB, _cmd(pan=10.0))
    m.select()
    assert m.active_source() == SOURCE_WEB
    m.submit(SOURCE_JOY, _cmd(pan=20.0))
    m.select()
    assert m.active_source() == SOURCE_JOY
