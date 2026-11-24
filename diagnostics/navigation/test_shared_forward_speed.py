"""Synthetic frames only: nothing here is evidence of PatrolBot stopping."""

import math

import pytest

from shared_forward_speed import DriveMode, ForwardSpeedStudy, Sample


def frame(study, lease, *, now=10.0, command=1.8, measured=0.0, clearance=3.0,
          route_cap=1.8, turn_cap=1.8, robot_cap=1.8):
    def sample(value):
        return Sample(value, now) if type(value) in (int, float) else value

    return study.evaluate(
        lease, now=now, command=sample(command), measured=sample(measured),
        clearance=sample(clearance), route_cap=route_cap, turn_cap=turn_cap,
        robot_cap=robot_cap,
    )


@pytest.fixture
def study():
    return ForwardSpeedStudy(short_zone_m=0.4, max_age_s=0.25)


@pytest.mark.parametrize('mode', list(DriveMode))
def test_change_during_motion_does_not_bypass_any_lower_bound(study, mode):
    lease = study.take_control(mode)
    assert frame(study, lease).bounded_intent_m_s == (0.22 if mode is DriveMode.NAVIGATION else 0.30)
    assert study.select(lease, 1.8)
    assert frame(study, lease, command=0.0).bounded_intent_m_s == 0.0
    assert frame(study, lease).bounded_intent_m_s == 1.8
    assert frame(study, lease, route_cap=0.5).bounded_intent_m_s == 0.5
    assert frame(study, lease, route_cap=0.5, turn_cap=0.2).bounded_intent_m_s == 0.2
    assert frame(study, lease, robot_cap=0.3).bounded_intent_m_s == 0.3
    assert study.select(lease, 0.6)
    assert frame(study, lease, route_cap=0.9).bounded_intent_m_s == 0.6
    assert frame(study, lease, route_cap=0.0).bounded_intent_m_s == 0.0


@pytest.mark.parametrize('mode', list(DriveMode))
def test_fast_straight_slow_turn_and_stop_track_measured_speed(study, mode):
    lease = study.take_control(mode)
    assert study.select(lease, 1.8)
    fast = frame(study, lease, measured=1.8)
    assert (fast.bounded_intent_m_s, fast.required_zone_m) == (1.8, 2.8)

    turn = frame(study, lease, now=10.1, measured=1.6, turn_cap=0.2)
    assert (turn.bounded_intent_m_s, turn.required_zone_m) == (0.2, 2.8)
    assert study.select(lease, 0.15)
    reduced = frame(study, lease, now=10.2, measured=1.2, turn_cap=0.2)
    assert (reduced.bounded_intent_m_s, reduced.required_zone_m) == (0.15, 2.8)
    stopped_command = frame(study, lease, now=10.3, command=0.0, measured=0.6)
    assert (stopped_command.bounded_intent_m_s, stopped_command.required_zone_m) == (0.0, 2.8)
    settled = frame(study, lease, now=10.4, command=0.0, measured=0.0)
    assert (settled.bounded_intent_m_s, settled.required_zone_m) == (0.0, 0.4)


def test_short_zone_only_when_already_slow_and_long_zone_is_not_clear(study):
    lease = study.take_control(DriveMode.CONTROLLER)
    assert study.select(lease, 1.8)
    limited = frame(study, lease, clearance=2.8)
    assert (limited.bounded_intent_m_s, limited.required_zone_m) == (0.22, 0.4)
    stopped = frame(study, lease, measured=1.0, clearance=2.8)
    assert (stopped.bounded_intent_m_s, stopped.required_zone_m, stopped.reason) == (0.0, 2.8, 'blocked')
    assert frame(study, lease, measured=0.0).reason == 'no_current_owner'
    assert frame(study, study.take_control(DriveMode.CONTROLLER)).bounded_intent_m_s == 1.8


def test_handoffs_keep_profiles_separate_and_connection_reset_clears_them(study):
    nav = study.take_control(DriveMode.NAVIGATION)
    assert study.select(nav, 1.8)
    dashboard = study.take_control(DriveMode.DASHBOARD)
    assert not study.select(nav, 1.8)
    dropped = frame(study, nav, measured=1.5)
    assert (dropped.reason, dropped.bounded_intent_m_s, dropped.required_zone_m) == ('no_current_owner', 0.0, 2.8)
    assert frame(study, dashboard, measured=1.5).bounded_intent_m_s == 0.30
    assert study.select(dashboard, 0.1)
    controller = study.take_control(DriveMode.CONTROLLER)
    assert frame(study, controller).bounded_intent_m_s == 0.30
    new_nav = study.take_control(DriveMode.NAVIGATION)
    assert frame(study, new_nav).bounded_intent_m_s == 1.8
    study.reset()
    assert frame(study, new_nav, measured=0.8).required_zone_m == 2.8
    assert frame(study, study.take_control(DriveMode.NAVIGATION)).bounded_intent_m_s == 0.22
    assert frame(study, study.take_control(DriveMode.DASHBOARD)).bounded_intent_m_s == 0.30


@pytest.mark.parametrize(('changes', 'reason'), [
    ({'measured': Sample(0.0, 9.75)}, 'unknown_measured_speed'),
    ({'measured': Sample(0.0, 10.1)}, 'unknown_measured_speed'),
    ({'measured': math.nan}, 'unknown_measured_speed'),
    ({'measured': 1.81}, 'unknown_measured_speed'),
    ({'clearance': Sample(5.0, 9.75)}, 'unknown_clearance'),
    ({'clearance': math.inf}, 'unknown_clearance'),
    ({'command': Sample(1.8, 9.75)}, 'no_fresh_forward_intent'),
    ({'command': -0.1}, 'no_fresh_forward_intent'),
    ({'robot_cap': None}, 'unknown_cap'),
    ({'route_cap': 2.0}, 'unknown_cap'),
])
def test_unverified_inputs_stop_and_cannot_automatically_resume(study, changes, reason):
    lease = study.take_control(DriveMode.NAVIGATION)
    assert study.select(lease, 1.8)
    samples = {'measured': 1.8, **changes}
    held = frame(study, lease, **samples)
    assert (held.bounded_intent_m_s, held.required_zone_m, held.reason) == (0.0, 2.8, reason)
    assert frame(study, lease).reason == 'no_current_owner'


@pytest.mark.parametrize('auto_resume', [False, True])
def test_nav_obstacle_clearance_can_resume_only_under_explicit_policy(auto_resume):
    study = ForwardSpeedStudy(short_zone_m=0.4, max_age_s=0.25, nav_resume_permitted=auto_resume)
    nav = study.take_control(DriveMode.NAVIGATION)
    assert frame(study, nav, clearance=0.1).reason == 'blocked'
    assert frame(study, nav).bounded_intent_m_s == (0.22 if auto_resume else 0.0)


@pytest.mark.parametrize('requested', [True, math.nan, math.inf, -0.1, math.nextafter(1.8, math.inf)])
def test_invalid_operator_selection_leaves_previous_cap_unchanged(study, requested):
    lease = study.take_control(DriveMode.NAVIGATION)
    assert not study.select(lease, requested)
    assert frame(study, lease).bounded_intent_m_s == 0.22
