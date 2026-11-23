"""Recorded/synthetic data only: no graph, service, command, or device access."""

import importlib.util
import json
from pathlib import Path

import pytest


SPEC = importlib.util.spec_from_file_location(
    'autonomous_stop_budget', Path(__file__).with_name('autonomous_stop_budget.py'))
budget = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(budget)


@pytest.fixture
def measurements():
    # Synthetic example, deliberately not evidence for the deployed robot.
    return dict(speed_m_s=2.0, reaction_upper_s=0.5, deceleration_lower_m_s2=1.0,
                measured_stop_upper_m=2.5, uncertainty_m=0.25, clearance_margin_m=0.5,
                detection_clearance_lower_m=4.0)


def test_budget_uses_worse_of_model_and_measured_stop(measurements):
    result = budget.evaluate(measurements)
    assert result['reaction_travel_m'] == 1.0
    assert result['model_braking_distance_m'] == 2.0
    assert result['required_clearance_m'] == 3.75
    assert result['distance_budget_met'] is True
    assert result['commissioning_approval'] is False
    measurements['measured_stop_upper_m'] = 4.0
    result = budget.evaluate(measurements)
    assert result['required_clearance_m'] == 4.75
    assert result['distance_budget_met'] is False


@pytest.mark.parametrize('available,met', [(3.74, False), (3.75, False), (3.76, True)])
def test_clearance_must_strictly_exceed_required(measurements, available, met):
    measurements['detection_clearance_lower_m'] = available
    assert budget.evaluate(measurements)['distance_budget_met'] is met


@pytest.mark.parametrize('invalid', [None, '0.5', True, 0, -0.1, float('nan'), float('inf')])
def test_unknown_or_invalid_evidence_cannot_pass(measurements, invalid):
    measurements['reaction_upper_s'] = invalid
    with pytest.raises(ValueError, match='reaction_upper_s'):
        budget.evaluate(measurements)


def test_input_shape_and_arithmetic_overflow_fail_closed(measurements):
    with pytest.raises(ValueError, match='exactly'):
        budget.evaluate({**measurements, 'commissioning_approval': True})
    missing = measurements.copy()
    missing.pop('measured_stop_upper_m')
    with pytest.raises(ValueError, match='exactly'):
        budget.evaluate(missing)
    with pytest.raises(ValueError, match='overflow'):
        budget.evaluate({**measurements, 'deceleration_lower_m_s2': 1e-320})


def test_cli_exits_for_met_unmet_and_duplicate_evidence(tmp_path, capsys, measurements):
    path = tmp_path / 'measurements.json'
    path.write_text(json.dumps(measurements))
    assert budget.main([str(path)]) == 0
    assert json.loads(capsys.readouterr().out)['commissioning_approval'] is False
    path.write_text(json.dumps({**measurements, 'detection_clearance_lower_m': 3.5}))
    assert budget.main([str(path)]) == 1
    assert json.loads(capsys.readouterr().out)['distance_budget_met'] is False
    path.write_text('{"speed_m_s": 1, "speed_m_s": 2}')
    assert budget.main([str(path)]) == 2
    assert 'duplicate measurement' in capsys.readouterr().err
