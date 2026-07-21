#!/usr/bin/env python3
"""Validate that physical PatrolBot calibration cannot be half-commissioned."""

import argparse
import math
from pathlib import Path
import sys

import yaml


TRANSFORM_FIELDS = ('x_m', 'y_m', 'z_m', 'yaw_rad', 'pitch_rad', 'roll_rad')


def _finite_number(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def validate_calibration(data, require_commissioned=False):
    errors = []
    if data.get('schema_version') != 1:
        errors.append('schema_version must be 1')

    commissioned = data.get('commissioned') is True
    if require_commissioned and not commissioned:
        errors.append('physical calibration is not commissioned')
    if not commissioned:
        # The safe repository default is intentionally incomplete. Reject only
        # accidental partial activation, not empty measurement placeholders.
        if data.get('low_obstacle_sensor', {}).get('transform', {}).get(
                'measured') is True:
            errors.append('measured transform cannot be active before commissioning')
        if data.get('footprint', {}).get('measured') is True:
            errors.append('measured footprint cannot be active before commissioning')
        return errors

    sensor = data.get('low_obstacle_sensor', {})
    if sensor.get('physically_present') is not True:
        errors.append('low-obstacle sensor presence must be physically confirmed')
    if not str(sensor.get('stable_serial_device', '')).startswith('/dev/'):
        errors.append('stable_serial_device must be a /dev udev alias')
    if not str(sensor.get('frame_id', '')).strip():
        errors.append('low-obstacle frame_id is required')
    transform = sensor.get('transform', {})
    if transform.get('measured') is not True:
        errors.append('low-obstacle transform must be measured')
    for field in TRANSFORM_FIELDS:
        if not _finite_number(transform.get(field)):
            errors.append(f'low-obstacle transform {field} must be finite')

    footprint = data.get('footprint', {})
    points = footprint.get('points_m', [])
    if footprint.get('measured') is not True:
        errors.append('physical footprint must be measured')
    if not isinstance(points, list) or len(points) < 3:
        errors.append('measured footprint needs at least three points')
    elif any(
            not isinstance(point, list) or len(point) != 2 or
            not all(_finite_number(value) for value in point)
            for point in points):
        errors.append('every footprint point must contain two finite numbers')

    lidar = data.get('main_lidar', {})
    if lidar.get('height_measured') is not True or not _finite_number(
            lidar.get('z_m')):
        errors.append('main lidar z must be physically measured')

    near = data.get('near_field_safety_scan', {})
    for field in (
            'self_occlusion_tested', 'stop_distance_tested',
            'approved_for_collision_monitor'):
        if near.get(field) is not True:
            errors.append(f'near-field gate {field} must pass')
    return errors


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('path', type=Path)
    parser.add_argument('--require-commissioned', action='store_true')
    args = parser.parse_args(argv)
    data = yaml.safe_load(args.path.read_text(encoding='utf-8'))
    errors = validate_calibration(data, args.require_commissioned)
    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1
    print('physical calibration schema is safely uncommissioned' if not data[
        'commissioned'] else 'physical calibration is complete')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
