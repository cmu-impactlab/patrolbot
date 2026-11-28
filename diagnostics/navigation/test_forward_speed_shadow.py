import json

import pytest

from forward_speed_shadow import ShadowTrace, main


def owner(at, epoch, mode='navigation'):
    return dict(event='owner', at=at, epoch=epoch, mode=mode)


def frame(at, epoch, *, target=1.8, speed=0.0, distance=3.2, measured_at=None, robot_cap=1.8):
    return dict(event='frame', at=at,
                command=dict(value=target, received_at=at, epoch=epoch),
                measured=dict(value=speed, received_at=at if measured_at is None else measured_at),
                clearance=dict(value=distance, received_at=at),
                route_cap=1.8, turn_cap=1.8, robot_cap=robot_cap)


@pytest.fixture
def shadow():
    return ShadowTrace(short_zone_m=0.4, max_age_s=0.25)


def test_trace_separates_effective_intent_and_measured_motion(shadow):
    shadow.consume(owner(1.0, 1))
    shadow.consume(dict(event='selection', at=1.0, epoch=1, selected_m_s=1.8))
    report = shadow.consume(frame(1.1, 1, target=0.0, speed=1.4, robot_cap=0.30))
    assert report['measured_sample']['value'] == 1.4
    assert report['shadow_only'] is True
    assert report['would_apply'] == {
        'requested_m_s': 1.8, 'bounded_intent_m_s': 0.0,
        'required_zone_m': 2.8, 'reason': 'within_study_caps',
    }


def test_slowing_and_handoff_retains_age_and_original_owner(shadow):
    shadow.consume(owner(1.0, 1))
    shadow.consume(owner(1.1, 2, 'controller'))
    result = shadow.consume(frame(1.2, 1, speed=1.0))
    assert result['would_apply']['reason'] == 'no_fresh_forward_intent'
    assert result['would_apply']['required_zone_m'] == 2.8
    assert shadow.consume(frame(1.3, 2))['would_apply']['reason'] == 'no_current_owner'
    shadow.consume(owner(1.4, 3, 'dashboard'))
    result = shadow.consume(frame(1.6, 3, measured_at=1.2))
    assert result['measured_sample']['received_at'] == 1.2
    assert result['would_apply']['reason'] == 'unknown_measured_speed'


def test_reset_does_not_admit_reused_epochs_or_restore_selection(shadow):
    shadow.consume(owner(1, 1))
    shadow.consume(dict(event='selection', at=1, epoch=1, selected_m_s=1.8))
    shadow.consume(dict(event='reset', at=2))
    with pytest.raises(ValueError, match='epochs'):
        shadow.consume(owner(2, 1))
    shadow.consume(owner(2, 2))
    result = shadow.consume(frame(2, 2))
    assert result['would_apply']['requested_m_s'] == 0.22


def test_command_received_before_ownership_is_not_fresh_for_that_owner(shadow):
    shadow.consume(owner(2, 1))
    event = frame(2.1, 1)
    event['command']['received_at'] = 1.99
    assert shadow.consume(event)['would_apply']['reason'] == 'no_fresh_forward_intent'


def test_cli_replays_reports_and_rejects_a_decreasing_time(tmp_path, capsys):
    capture = tmp_path / 'synthetic.jsonl'
    records = (owner(1, 1), frame(2, 1), frame(1.5, 1))
    capture.write_text(''.join(json.dumps(record) + '\n' for record in records))
    assert main([str(capture), '--short-zone-m', '0.4', '--max-age-s', '0.25']) == 2
    result = capsys.readouterr()
    assert json.loads(result.out)['would_apply']['bounded_intent_m_s'] == 0.22
    assert 'line 3' in result.err and 'monotonic' in result.err


@pytest.mark.parametrize('raw', [
    '{"event":"reset","at":0,"at":1}',
    '{"event":"reset","at":NaN}',
    '{"event":"owner","at":1,"epoch":1,"mode":"undock"}',
])
def test_cli_rejects_ambiguous_invalid_or_out_of_scope_input(tmp_path, capsys, raw):
    capture = tmp_path / 'synthetic.jsonl'
    capture.write_text(raw + '\n')
    assert main([str(capture), '--short-zone-m', '0.4', '--max-age-s', '0.25']) == 2
    result = capsys.readouterr()
    assert not result.out
    assert 'line 1' in result.err
