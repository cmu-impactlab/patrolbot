"""Offline only. Oversized synthetic geometry is never PatrolBot evidence."""

import json
import math
from pathlib import Path

import pytest
import yaml

import forward_speed_zones as study


@pytest.fixture
def baseline():
    def band(lower, upper):
        return dict(linear_min=lower, linear_max=upper, theta_min=-1.0, theta_max=1.0,
                    points='[[16, 8], [16, -8], [-8, -8], [-8, 8]]')

    polygon = dict(type='velocity_polygon', action_type='stop', holonomic=False,
                   velocity_polygons=['rotation_or_stopped', 'forward', 'reverse'],
                   rotation_or_stopped=band(-0.125, 0.125), forward=band(0.125, 0.25),
                   reverse=band(-0.25, -0.125))
    return {'collision_monitor': {'ros__parameters': dict(
        base_frame_id='base_link', polygons=['front_velocity_stop'], front_velocity_stop=polygon)}}


@pytest.fixture
def evidence():
    def measured():
        return dict(evidence_ref='SYNTHETIC UNIT TEST ONLY', observed_speed_upper_m_s=2.0,
                    stop_budget=dict(speed_m_s=2.0, reaction_upper_s=1.0,
                                     deceleration_lower_m_s2=2.0, measured_stop_upper_m=4.0,
                                     uncertainty_m=1.0, clearance_margin_m=1.0,
                                     detection_clearance_lower_m=128.0))

    return dict(leading_edge_x_m=8.0, rotation_or_stopped=measured(), forward=measured(),
                extensions=[
                    dict(**measured(), command_max_m_s=0.5, front_edge_x_m=32.0),
                    dict(**measured(), command_max_m_s=1.8, front_edge_x_m=64.0),
                ])


def test_boundary_order_of_study_is_faster_first_except_retained_stopped(baseline, evidence):
    report = study.review(baseline, evidence)
    assert report['study_selector_order'] == [
        'rotation_or_stopped', 'study_forward_2', 'study_forward_1', 'forward', 'reverse']
    bands = report['bands']
    assert bands['rotation_or_stopped']['band_at_upper_endpoint'] == 'rotation_or_stopped'
    assert bands['forward']['band_at_lower_endpoint'] == 'rotation_or_stopped'
    assert bands['forward']['band_at_upper_endpoint'] == 'study_forward_1'
    assert bands['study_forward_1']['band_at_upper_endpoint'] == 'study_forward_2'
    assert bands['study_forward_2']['band_at_lower_endpoint'] == 'study_forward_2'
    assert bands['study_forward_2']['band_at_upper_endpoint'] == 'study_forward_2'
    assert report['straight_line_checks_met'] is True
    assert report['commissioning_approval'] is report['swept_field_validated'] is False


def test_selector_overlap_adjacent_floats_and_angular_limits():
    long = study.Band('synthetic_long', 0.5, 1.8, -1.0, 1.0)
    short = study.Band('synthetic_short', 0.25, 0.5, -1.0, 1.0)
    ordered = [long, short]
    assert study.select_band(ordered, 0.5, -1.0) == (long, True)
    assert study.select_band(ordered, 0.5, 1.0) == (long, True)
    assert study.select_band(ordered[::-1], 0.5, 0.0) == (short, True)
    assert study.select_band(ordered, math.nextafter(0.5, 0.0), 0.0) == (short, True)
    assert study.select_band(ordered, math.nextafter(0.5, math.inf), 0.0) == (long, True)
    assert study.select_band(ordered, math.nextafter(1.8, math.inf), 0.0, short) == (short, False)
    assert study.select_band(ordered, 0.5, math.nextafter(1.0, math.inf)) == (None, False)


def test_checked_in_baseline_can_be_read_without_modification():
    path = (Path(__file__).resolve().parents[2]
            / 'ros2_ws/src/patrolbot_navigation/config/nav2_params.yaml')
    bands = study.baseline_bands(yaml.load(path.read_text(), Loader=study.UniqueYamlLoader))
    stopped, forward, reverse = bands
    assert study.select_band(bands, stopped.linear_max, 0.0) == (stopped, True)
    assert study.select_band(bands, stopped.linear_min, 0.0) == (stopped, True)
    assert study.select_band(bands, reverse.linear_min, 0.0) == (reverse, True)
    # A higher command is not covered by the baseline; stale selection is not a fallback.
    assert study.select_band(bands, math.nextafter(forward.linear_max, math.inf), 0.0,
                             forward) == (forward, False)


def test_geometry_and_selection_reject_a_different_baseline_shape_or_order(baseline):
    polygon = baseline['collision_monitor']['ros__parameters']['front_velocity_stop']
    polygon['forward']['points'] = '[[16, 8], [-8, -8], [-8, 8], [16, -8]]'
    with pytest.raises(ValueError, match='axis-aligned rectangle'):
        study.baseline_bands(baseline)
    polygon['velocity_polygons'].reverse()
    with pytest.raises(ValueError, match='ordered PatrolBot'):
        study.baseline_bands(baseline)


@pytest.mark.parametrize('value', [0.25, 0.5, math.nextafter(1.8, math.inf), math.inf])
def test_proposed_command_maxima_must_strictly_increase(baseline, evidence, value):
    evidence['extensions'][-1]['command_max_m_s'] = value
    with pytest.raises(ValueError, match='command_max_m_s|command maxima'):
        study.review(baseline, evidence)


def test_front_edges_cannot_shrink(baseline, evidence):
    evidence['extensions'][-1]['front_edge_x_m'] = 31.0
    with pytest.raises(ValueError, match='front edges must not decrease'):
        study.review(baseline, evidence)


def test_budget_at_observed_underspeed_does_not_clear_a_faster_command_band(baseline, evidence):
    extension = evidence['extensions'][-1]
    extension['observed_speed_upper_m_s'] = 1.0
    extension['stop_budget']['speed_m_s'] = 1.0
    report = study.review(baseline, evidence)
    row = report['bands']['study_forward_2']
    assert row['budget_covers_observed_speed'] is True
    assert row['budget_covers_command_speed'] is report['straight_line_checks_met'] is False


@pytest.mark.parametrize('name', ['rotation_or_stopped', 'forward'])
def test_lag_requires_budget_at_actual_speed_in_retained_zones(baseline, evidence, name):
    evidence[name]['observed_speed_upper_m_s'] = 8.0
    row = study.review(baseline, evidence)['bands'][name]
    assert row['straight_geometry_met'] is True  # Small-speed geometry alone gives false comfort.
    assert row['budget_covers_observed_speed'] is row['straight_line_check_met'] is False

    # Even with independent stopping evidence at the actual speed, switching to
    # the shorter field before braking finishes leaves insufficient geometry.
    evidence[name]['stop_budget'].update(speed_m_s=8.0, measured_stop_upper_m=24.0)
    report = study.review(baseline, evidence)
    row = report['bands'][name]
    assert row['budget_covers_observed_speed'] is row['distance_budget']['distance_budget_met'] is True
    assert row['straight_geometry_met'] is report['straight_line_checks_met'] is False


def test_downshift_matches_short_band_even_while_ground_speed_is_high(baseline):
    stopped, forward, reverse = study.baseline_bands(baseline)
    extension = study.Band('synthetic_extension', forward.linear_max, 1.8, 0.0, 0.0, 64.0)
    ordered = [stopped, extension, forward, reverse]
    active, _ = study.select_band(ordered, 1.8, 0.0)
    assert study.select_band(ordered, 0.20, 0.0, active) == (forward, True)
    assert study.select_band(ordered, 0.0, 0.0, active) == (stopped, True)
    assert study.select_band(ordered, 1.8, 0.001, active) == (active, False)


def test_missing_or_invalid_evidence_fails_closed(baseline, evidence):
    with pytest.raises(ValueError, match='rotation_or_stopped'):
        study.review(baseline, {key: val for key, val in evidence.items()
                                if key != 'rotation_or_stopped'})
    evidence['forward']['stop_budget'].pop('reaction_upper_s')
    with pytest.raises(ValueError, match='forward.stop_budget'):
        study.review(baseline, evidence)
    evidence['forward'].pop('observed_speed_upper_m_s')
    with pytest.raises(ValueError, match='observed_speed_upper_m_s'):
        study.review(baseline, evidence)


@pytest.mark.parametrize('limit', ['detection', 'geometry'])
def test_exact_clearance_equality_is_insufficient(baseline, evidence, limit):
    if limit == 'detection':
        evidence['forward']['stop_budget']['detection_clearance_lower_m'] = 6.0
    else:
        evidence['leading_edge_x_m'] = 10.0  # Synthetic front edge 16 minus required 6.
    row = study.review(baseline, evidence)['bands']['forward']
    assert row['distance_budget']['distance_budget_met'] is (limit != 'detection')
    assert row['straight_geometry_met'] is (limit != 'geometry')
    assert row['straight_line_check_met'] is False


def test_cli_reports_met_unmet_and_rejects_duplicate_data(tmp_path, capsys, baseline, evidence):
    params = tmp_path / 'not-a-runtime-config.yaml'
    recorded = tmp_path / 'synthetic-not-field-data.json'
    params.write_text(yaml.safe_dump(baseline))
    recorded.write_text(json.dumps(evidence))
    args = [str(params), str(recorded)]
    assert study.main(args) == 0
    assert json.loads(capsys.readouterr().out)['straight_line_checks_met'] is True
    evidence['forward']['observed_speed_upper_m_s'] = 3.0
    recorded.write_text(json.dumps(evidence))
    assert study.main(args) == 1
    assert json.loads(capsys.readouterr().out)['straight_line_checks_met'] is False
    recorded.write_text('{"leading_edge_x_m": 8, "leading_edge_x_m": 10}')
    assert study.main(args) == 2
    assert 'duplicate' in capsys.readouterr().err
    params.write_text('collision_monitor: {}\ncollision_monitor: {}\n')
    assert study.main(args) == 2
    assert 'duplicate' in capsys.readouterr().err
