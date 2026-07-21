"""Tests for uncommissioned sensor and geometry scaffolding."""

from pathlib import Path
import sys

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / 'scripts'))
from validate_physical_calibration import validate_calibration  # noqa: E402


def calibration():
    return yaml.safe_load((
        PACKAGE_ROOT / 'config' / 'physical_calibration.yaml'
    ).read_text(encoding='utf-8'))


def test_repository_calibration_is_valid_but_uncommissioned():
    data = calibration()
    assert data['commissioned'] is False
    assert validate_calibration(data) == []
    assert 'physical calibration is not commissioned' in validate_calibration(
        data, require_commissioned=True)


def test_partial_measurement_cannot_be_marked_safe():
    data = calibration()
    data['commissioned'] = True
    errors = validate_calibration(data, require_commissioned=True)
    assert 'low-obstacle sensor presence must be physically confirmed' in errors
    assert 'physical footprint must be measured' in errors
    assert 'main lidar z must be physically measured' in errors
    assert 'near-field gate self_occlusion_tested must pass' in errors


def test_candidate_launch_is_not_in_production_bringup():
    bringup = (PACKAGE_ROOT / 'launch' / 'bringup.launch.py').read_text(
        encoding='utf-8')
    candidate = (
        PACKAGE_ROOT / 'launch' /
        'low_obstacle_sensor_candidate.launch.py').read_text(encoding='utf-8')
    assert 'low_obstacle_sensor_candidate' not in bringup
    assert "default_value='false'" in candidate
    assert "remappings=[('scan', '/scan_low')]" in candidate


def test_active_geometry_is_not_modified_by_candidate_scaffolding():
    params = yaml.safe_load((
        PACKAGE_ROOT / 'config' / 'nav2_params.yaml'
    ).read_text(encoding='utf-8'))
    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_ = params['global_costmap']['global_costmap']['ros__parameters']
    assert local['footprint'] == global_['footprint']
    assert '0.255' in local['footprint']
    bringup = (PACKAGE_ROOT / 'launch' / 'bringup.launch.py').read_text(
        encoding='utf-8')
    assert "arguments=['0.037', '0', '0.2'" in bringup
