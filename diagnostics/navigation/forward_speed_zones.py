#!/usr/bin/env python3
"""Review PatrolBot forward speed bands offline; never emit a Nav2 configuration."""

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys

import yaml

import autonomous_stop_budget as stop_budget


STUDY_MAX_M_S = 1.8  # Command under study, not a robot limit or recommendation.
BASELINE_ORDER = ('rotation_or_stopped', 'forward', 'reverse')
EVIDENCE_FIELDS = ('evidence_ref', 'observed_speed_upper_m_s', 'stop_budget')


@dataclass(frozen=True)
class Band:
    name: str
    linear_min: float
    linear_max: float
    theta_min: float
    theta_max: float
    front_edge_x_m: float | None = None


def select_band(bands, command_x, command_theta, previous=None):
    """Model Jazzy's non-holonomic selector: (active band, matched this command)."""
    for band in bands:
        if (band.linear_min <= command_x <= band.linear_max
                and band.theta_min <= command_theta <= band.theta_max):
            return band, True
    return previous, False


def number(value, field):
    try:
        result = float(value) if type(value) in (int, float) else math.nan
    except OverflowError:
        result = math.inf
    if not math.isfinite(result):
        raise ValueError(f'{field} must be a finite number')
    return result


def fields(mapping, expected, label):
    if not isinstance(mapping, dict) or set(mapping) != set(expected):
        raise ValueError(f'{label} requires exactly: {", ".join(expected)}')


def rectangle_front_edge(points):
    """Read the +x edge of a perimeter-ordered axis-aligned rectangle."""
    if not isinstance(points, str):
        raise ValueError('baseline points must be a Nav2 point string')
    points = json.loads(points)
    if (not isinstance(points, list) or len(points) != 4
            or any(not isinstance(pair, list) or len(pair) != 2 for pair in points)):
        raise ValueError('baseline points must form an axis-aligned rectangle')
    vertices = [(number(x, 'points x'), number(y, 'points y')) for x, y in points]
    xs, ys = (set(axis) for axis in zip(*vertices))
    if (len(xs) != 2 or len(ys) != 2 or len(set(vertices)) != 4
            or any((x1 == x2) == (y1 == y2)
                   for (x1, y1), (x2, y2) in zip(vertices, vertices[1:] + vertices[:1]))):
        raise ValueError('baseline points must form an axis-aligned rectangle')
    if not (min(xs) < 0 < max(xs) and min(ys) < 0 < max(ys)):
        raise ValueError('baseline rectangle must span the base_link origin')
    return max(xs)


def baseline_bands(config):
    """Read the current PatrolBot front layout, not arbitrary ROS parameter overlays."""
    try:
        params = config['collision_monitor']['ros__parameters']
        polygon = params['front_velocity_stop']
        if (params['base_frame_id'] != 'base_link' or not isinstance(params['polygons'], list)
                or 'front_velocity_stop' not in params['polygons']
                or polygon['type'] != 'velocity_polygon' or polygon['action_type'] != 'stop'
                or polygon['holonomic'] is not False or polygon.get('enabled', True) is not True
                or polygon['velocity_polygons'] != list(BASELINE_ORDER)):
            raise ValueError('expected the ordered PatrolBot base_link front velocity stop polygon')
        bands = []
        for name in BASELINE_ORDER:
            data = polygon[name]
            values = [number(data[key], f'{name}.{key}')
                      for key in ('linear_min', 'linear_max', 'theta_min', 'theta_max')]
            edge = None if name == 'reverse' else rectangle_front_edge(data['points'])
            bands.append(Band(name, *values, edge))
    except (KeyError, TypeError) as error:
        raise ValueError('invalid PatrolBot front velocity stop schema') from error
    stopped, forward, reverse = bands
    if not (reverse.linear_min < reverse.linear_max == stopped.linear_min < 0
            < stopped.linear_max == forward.linear_min < forward.linear_max
            and all(band.theta_min < 0 < band.theta_max for band in bands)):
        raise ValueError('expected contiguous reverse, stopped, and forward baseline ranges')
    return stopped, forward, reverse


def compare(band, evidence, leading_edge_x_m):
    fields(evidence, EVIDENCE_FIELDS, band.name)
    ref = evidence['evidence_ref']
    if not isinstance(ref, str) or not ref.strip():
        raise ValueError(f'{band.name}.evidence_ref must name the local test evidence')
    observed = number(evidence['observed_speed_upper_m_s'],
                      f'{band.name}.observed_speed_upper_m_s')
    if observed <= 0:
        raise ValueError(f'{band.name}.observed_speed_upper_m_s must be positive')
    try:
        budget = stop_budget.evaluate(evidence['stop_budget'])
    except ValueError as error:
        raise ValueError(f'{band.name}.stop_budget: {error}') from error
    geometry = band.front_edge_x_m - leading_edge_x_m
    observed_speed_covered = budget['speed_m_s'] >= observed
    command_speed_covered = budget['speed_m_s'] >= band.linear_max
    geometry_met = geometry > budget['required_clearance_m']
    return {
        'evidence_ref': ref,
        'observed_speed_upper_m_s': observed,
        'budget_covers_observed_speed': observed_speed_covered,
        'budget_covers_command_speed': command_speed_covered,
        'studied_front_edge_x_m': band.front_edge_x_m,
        'straight_geometry_clearance_m': geometry,
        'straight_geometry_met': geometry_met,
        'distance_budget': budget,
        'straight_line_check_met': (observed_speed_covered and command_speed_covered
                                    and geometry_met and budget['distance_budget_met']),
    }


def review(config, evidence):
    """Compare only straight, nonnegative command selections and supplied bounds."""
    stopped, forward, reverse = baseline_bands(config)
    fields(evidence, ('leading_edge_x_m', 'rotation_or_stopped', 'forward', 'extensions'), 'input')
    leading_edge = number(evidence['leading_edge_x_m'], 'leading_edge_x_m')
    if not 0 < leading_edge:
        raise ValueError('leading_edge_x_m must be positive')
    proposals = evidence['extensions']
    if not isinstance(proposals, list) or not proposals:
        raise ValueError('extensions must be a nonempty list of proposed higher command bands')

    extensions = []
    evidence_by_band = {band.name: evidence[band.name] for band in (stopped, forward)}
    last = forward
    for i, proposed in enumerate(proposals, 1):
        name = f'study_forward_{i}'
        fields(proposed, (*EVIDENCE_FIELDS, 'command_max_m_s', 'front_edge_x_m'), name)
        upper = number(proposed['command_max_m_s'], f'{name}.command_max_m_s')
        edge = number(proposed['front_edge_x_m'], f'{name}.front_edge_x_m')
        if not last.linear_max < upper <= STUDY_MAX_M_S:
            raise ValueError(f'{name}: command maxima must increase up to {STUDY_MAX_M_S} m/s')
        if edge < last.front_edge_x_m:
            raise ValueError(f'{name}: front edges must not decrease with higher command bands')
        last = Band(name, last.linear_max, upper, 0.0, 0.0, edge)
        extensions.append(last)
        evidence_by_band[name] = {key: proposed[key] for key in EVIDENCE_FIELDS}

    # Retain the exact stopped/forward boundary behavior. At all higher shared
    # endpoints the longest adjacent band wins. No broad angular or speed fallback.
    ordered = [stopped, *reversed(extensions), forward, reverse]
    reports = {}
    for band in (stopped, forward, *extensions):
        lower = max(0.0, band.linear_min)
        reports[band.name] = {
            'straight_command_interval_m_s': [lower, band.linear_max],
            'band_at_lower_endpoint': select_band(ordered, lower, 0.0)[0].name,
            'band_at_upper_endpoint': select_band(ordered, band.linear_max, 0.0)[0].name,
            **compare(band, evidence_by_band[band.name], leading_edge),
        }
    return {
        'scope': 'offline straight-line study only; no swept-field or runtime validation',
        'commissioning_approval': False,
        'swept_field_validated': False,
        'straight_line_checks_met': all(row['straight_line_check_met'] for row in reports.values()),
        'study_selector_order': [band.name for band in ordered],
        'bands': reports,
    }


class UniqueYamlLoader(yaml.SafeLoader):
    """Reject ambiguous duplicate keys rather than silently taking the last value."""

    def construct_mapping(self, node, deep=False):
        return stop_budget.unique_fields(self.construct_pairs(node, deep=deep))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline', type=Path, help='local PatrolBot Nav2 baseline YAML (read only)')
    parser.add_argument('evidence', type=Path, help='local JSON with tested bounds and study inputs')
    args = parser.parse_args(argv)
    try:
        config = yaml.load(args.baseline.read_text(encoding='utf-8'), Loader=UniqueYamlLoader)
        evidence = json.loads(args.evidence.read_text(encoding='utf-8'),
                              object_pairs_hook=stop_budget.unique_fields)
        result = review(config, evidence)
    except (OSError, ValueError, TypeError, yaml.YAMLError) as error:
        print(f'Cannot review forward speed zones: {error}', file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['straight_line_checks_met'] else 1


if __name__ == '__main__':
    sys.exit(main())
