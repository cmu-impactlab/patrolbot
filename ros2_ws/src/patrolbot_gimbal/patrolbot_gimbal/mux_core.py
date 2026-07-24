"""Arbitration between gimbal command sources.

Kept separate from the node, and from twist_mux, on purpose. The robot's
velocity chain has its own arbiter that is safety-reviewed and commissioned;
this one decides only where the camera looks and shares no code, no topics and
no configuration with it.

Precedence is fixed and explicit: the joystick outranks the dashboard. Someone
physically holding the controller is present at the robot, and must not be
overridden by a browser elsewhere.
"""

import time
from dataclasses import dataclass
from typing import Dict, Optional

SOURCE_JOY = 'joy'
SOURCE_WEB = 'web'

# Highest priority first. Adding a source means adding it here; the mux
# rejects anything not declared, so an unreviewed publisher cannot slip in.
PRIORITY = (SOURCE_JOY, SOURCE_WEB)


@dataclass
class Selection:
    """The winning command and where it came from."""

    source: str
    command: dict


@dataclass
class _Entry:
    command: dict
    received_at: float


class MuxCore:
    """Selects one command from several sources, newest-valid-highest-priority."""

    def __init__(self, clock=time.monotonic, input_timeout_s: float = 0.5):
        self._clock = clock
        self._timeout = float(input_timeout_s)
        self._entries: Dict[str, _Entry] = {}
        self._active: Optional[str] = None

    def submit(self, source: str, command: dict) -> None:
        """Record the latest command from one source.

        Raises ValueError for an undeclared source rather than accepting it:
        an unknown publisher reaching the gimbal driver is exactly the kind of
        thing this package exists to prevent.
        """
        if source not in PRIORITY:
            raise ValueError(
                f'unknown gimbal command source {source!r}; '
                f'expected one of {", ".join(PRIORITY)}')
        self._entries[source] = _Entry(dict(command), self._clock())

    def _fresh(self, entry: _Entry) -> bool:
        return (self._clock() - entry.received_at) < self._timeout

    def select(self) -> Optional[Selection]:
        """Choose the command to forward.

        Returns None only when nothing has ever been received. Once any source
        has spoken, an explicit hold is returned instead of silence, so the
        driver can distinguish "hold still" from "the publisher vanished".
        """
        if not self._entries:
            self._active = None
            return None

        for source in PRIORITY:
            entry = self._entries.get(source)
            if entry is None:
                continue
            if not self._fresh(entry):
                continue
            if not entry.command.get('valid', False):
                continue
            self._active = source
            return Selection(source, dict(entry.command))

        # Nothing fresh and valid: hold, and say so explicitly.
        self._active = None
        return Selection(
            source='none',
            command={'pan': 0.0, 'tilt': 0.0, 'roll': 0.0, 'valid': False},
        )

    def active_source(self) -> Optional[str]:
        """Which source won the last selection, for display purposes only."""
        return self._active
