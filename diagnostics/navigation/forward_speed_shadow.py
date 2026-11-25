#!/usr/bin/env python3
"""Replay normalized observations locally; report guesses separately from live limits."""

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys

from autonomous_stop_budget import unique_fields
from shared_forward_speed import DriveMode, ForwardSpeedStudy, Intent, Sample


def _fields(record, keys):
    if not isinstance(record, dict) or record.keys() != set(keys.split()):
        raise ValueError(f'expected exactly: {keys}')


def _time(value):
    if type(value) not in (int, float) or not 0 <= value <= 1e12:
        raise ValueError('timestamp must be finite and nonnegative')
    return value


def _epoch(value):
    if type(value) is not int or not 1 <= value <= (1 << 53) - 1:
        raise ValueError('epoch must be a positive safe integer')
    return value


def _number(value):
    if type(value) not in (int, float) or abs(value) > 1e12 or not math.isfinite(value):
        raise ValueError('expected a finite number')
    return value


class ShadowTrace:
    """Accept an offline normalized trace, not a live command or a trusted source."""

    def __init__(self, **study_options):
        self._study = ForwardSpeedStudy(**study_options)
        self._epoch = 0
        self._lease = None
        self._mode = None
        self._last_at = 0.0
        self._owner_at = None

    def _sample(self, record, intent=False):
        if record is None:
            return None
        _fields(record, 'value received_at epoch' if intent else 'value received_at')
        value, at = _number(record['value']), _time(record['received_at'])
        if intent:
            epoch = _epoch(record['epoch'])
            lease = self._lease if (self._epoch == epoch and self._owner_at is not None
                                    and at >= self._owner_at) else None
            return Intent(value, at, lease)
        return Sample(value, at)

    def consume(self, event):
        if not isinstance(event, dict):
            raise ValueError('expected an event object')
        kind = event.get('event')
        at = _time(event.get('at'))
        if at < self._last_at:
            raise ValueError('events must be in monotonic order')

        if kind == 'owner':
            _fields(event, 'event at epoch mode')
            epoch = _epoch(event['epoch'])
            if epoch <= self._epoch:
                raise ValueError('owner epochs must strictly increase across handoffs and resets')
            mode = DriveMode(event['mode'])
            self._lease = self._study.take_control(mode)
            self._mode, self._epoch = mode.value, epoch
            self._owner_at = at
            result = None
        elif kind == 'reset':
            _fields(event, 'event at')
            self._study.reset()
            self._lease = self._mode = None
            self._owner_at = None
            result = None
        elif kind == 'selection':
            _fields(event, 'event at epoch selected_m_s')
            selected = _number(event['selected_m_s'])
            accepted = _epoch(event['epoch']) == self._epoch and self._study.select(self._lease, selected)
            result = {'at': at, 'selection_accepted_in_study': accepted, 'shadow_only': True}
        elif kind == 'frame':
            _fields(event, 'event at command measured clearance route_cap turn_cap robot_cap')
            measured = self._sample(event['measured'])
            decision = self._study.evaluate(
                self._lease, now=at, command=self._sample(event['command'], intent=True),
                measured=measured, clearance=self._sample(event['clearance']),
                **{key: None if event[key] is None else _number(event[key])
                   for key in ('route_cap', 'turn_cap', 'robot_cap')},
            )
            result = {
                'at': at, 'last_owner_mode': self._mode,
                'measured_sample': asdict(measured) if measured is not None else None,
                'would_apply': asdict(decision),
                'shadow_only': True,
            }
        else:
            raise ValueError(f'unknown event: {kind!r}')
        self._last_at = at
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path, help='local normalized JSON-lines capture')
    parser.add_argument('--short-zone-m', type=float, required=True, help='scenario input, not a setting')
    parser.add_argument('--max-age-s', type=float, required=True, help='scenario input, not a setting')
    args = parser.parse_args(argv)
    line_number = 0
    try:
        shadow = ShadowTrace(short_zone_m=args.short_zone_m, max_age_s=args.max_age_s)
        with args.trace.open(encoding='utf-8') as trace:
            for line_number, line in enumerate(trace, 1):
                if not line.strip():
                    continue
                report = shadow.consume(json.loads(line, object_pairs_hook=unique_fields))
                if report is not None:
                    print(json.dumps(report, allow_nan=False, separators=(',', ':')))
    except (OSError, ValueError, TypeError) as error:
        print(f'Shadow input rejected at line {line_number}: {error}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
