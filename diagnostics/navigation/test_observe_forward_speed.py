from observe_forward_speed import ForwardSpeedObservation


def report(observe, now, **overrides):
    sources = dict(base_source=b'bridge', single_odom_source=True, single_command_source=True)
    return observe.report(now, **{**sources, **overrides})


def test_downstream_command_is_never_admitted_as_owned_and_zero_does_not_shorten_zone():
    observe = ForwardSpeedObservation(short_zone_m=0.4, max_age_s=0.25)
    observe.base_state(b'bridge', 1, True, 0.0, 1.0)
    observe.odometry(1.4, 1.01)
    observe.downstream_command(0.0, 1.02)
    snapshot = report(observe, 1.1)
    assert snapshot['shadow_only']
    assert snapshot['last_owner_mode'] is None
    assert snapshot['effective_cap_m_s'] is None
    assert snapshot['measured_sample'] == {'value': 1.4, 'received_at': 1.01}
    assert snapshot['downstream_command_sample'] == {'value': 0.0, 'received_at': 1.02}
    assert snapshot['reason'] == 'no_current_owner'
    assert snapshot['candidate_zone_m'] == 2.8


def test_reconnect_clears_old_data_but_reports_new_observations():
    observe = ForwardSpeedObservation(short_zone_m=0.4, max_age_s=0.25)
    observe.base_state(b'bridge', 1, True, 0.0, 1.0)
    observe.odometry(0.0, 1.01)
    observe.downstream_command(0.22, 1.01)
    observe.base_state(b'bridge', 2, True, 0.0, 1.03)
    snapshot = report(observe, 1.05)
    assert snapshot['observed_hardware_generation'] == 2
    assert snapshot['downstream_command_sample'] is snapshot['measured_sample'] is None
    assert snapshot['reason'] == 'unknown_measured_speed'
    observe.odometry(0.0, 1.06)
    assert report(observe, 1.07)['measured_sample']['value'] == 0.0
    observe.base_state(b'new-bridge', 2, True, 0.0, 1.08)
    assert report(observe, 1.09, base_source=b'new-bridge')['measured_sample'] is None


def test_stale_or_multiple_sources_are_not_reused():
    observe = ForwardSpeedObservation(short_zone_m=0.4, max_age_s=0.25)
    observe.base_state(b'bridge', 1, True, 0.0, 1.0)
    observe.odometry(0.0, 1.01)
    assert report(observe, 1.02, single_odom_source=False)['measured_sample'] is None
    assert report(observe, 1.03)['measured_sample'] is None
    observe.odometry(0.0, 1.04)
    observe.downstream_command(0.1, 1.04)
    observe.base_state(b'bridge', 1, True, 0.0, 1.2)
    snapshot = report(observe, 1.3)
    assert snapshot['observed_hardware_generation'] == 1
    assert snapshot['measured_sample'] is snapshot['downstream_command_sample'] is None
    assert snapshot['candidate_zone_m'] == 2.8
    observe.odometry(0.0, 1.31)
    observe.base_state(b'bridge', 1, True, 0.30, 1.32)
    snapshot = report(observe, 1.33)
    assert snapshot['observed_hardware_generation'] is snapshot['measured_sample'] is None
