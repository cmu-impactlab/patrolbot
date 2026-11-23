#!/usr/bin/env python3
"""Evaluate one straight-line stopping-distance budget from offline measurements."""

import argparse
import json
import math
from pathlib import Path
import sys


POSITIVE = (
    'speed_m_s',
    'reaction_upper_s',
    'deceleration_lower_m_s2',
    'measured_stop_upper_m',
    'uncertainty_m',
    'clearance_margin_m',
    'detection_clearance_lower_m',
)


def evaluate(measurements):
    """Return only a distance comparison, never commissioning approval."""
    if not isinstance(measurements, dict) or set(measurements) != set(POSITIVE):
        raise ValueError('expected exactly these measurements: ' + ', '.join(POSITIVE))
    values = {}
    for name in POSITIVE:
        value = measurements[name]
        try:
            number = float(value) if type(value) in (int, float) else math.nan
        except OverflowError:
            number = math.inf
        if not math.isfinite(number) or number <= 0.0:
            raise ValueError(f'{name} must be a finite positive number in SI units')
        values[name] = number

    speed = values['speed_m_s']
    reaction = speed * values['reaction_upper_s']
    braking = (speed / values['deceleration_lower_m_s2']) * speed / 2
    model = reaction + braking
    required = max(model, values['measured_stop_upper_m']) + (
        values['uncertainty_m'] + values['clearance_margin_m'])
    available = values['detection_clearance_lower_m']
    if not all(math.isfinite(v) for v in (reaction, braking, model, required)):
        raise ValueError('stopping-distance calculation overflowed')
    return {
        'speed_m_s': speed,
        'reaction_travel_m': reaction,
        'model_braking_distance_m': braking,
        'model_stop_distance_m': model,
        'required_clearance_m': required,
        'detection_clearance_lower_m': available,
        'remaining_clearance_m': available - required,
        'distance_budget_met': available > required,
        'commissioning_approval': False,
    }


def unique_fields(pairs):
    fields = {}
    for key, value in pairs:
        if key in fields:
            raise ValueError(f'duplicate measurement: {key}')
        fields[key] = value
    return fields


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('measurements', type=Path, help='local JSON measurement file')
    args = parser.parse_args(argv)
    try:
        measured = json.loads(args.measurements.read_text(encoding='utf-8'),
                              object_pairs_hook=unique_fields)
        comparison = evaluate(measured)
    except (OSError, ValueError) as error:
        print(f'Cannot evaluate distance budget: {error}', file=sys.stderr)
        return 2
    print(json.dumps(comparison, indent=2, allow_nan=False))
    return 0 if comparison['distance_budget_met'] else 1


if __name__ == '__main__':
    sys.exit(main())
