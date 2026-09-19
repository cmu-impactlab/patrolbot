from status_report import Check, build_report


def _check(scope: str, name: str, status: str, evidence: str = "fixture") -> Check:
    return Check(scope, name, status, evidence)


def _report(*checks: Check) -> tuple[dict, int]:
    return build_report(list(checks))


def test_connected_stack_is_ready():
    report, exit_code = _report(
        _check("container", "navigation", "pass"),
        _check("pi", "lifecycle.controller_server", "pass"),
        _check("sensor", "topic.odom.fresh", "pass"),
        _check("sensor", "tf.odom-base_link", "pass"),
    )
    assert (report["overall"], report["reason"], exit_code) == (
        "ready", None, 0)


def test_sbc_down_preserves_pi_liveness_and_reports_expected_degradation():
    report, exit_code = _report(
        _check("container", "navigation", "pass"),
        _check("pi", "lifecycle.controller_server", "pass"),
        _check("sensor", "topic.odom.fresh", "degraded", "SBC unavailable"),
        _check("sensor", "tf.odom-base_link", "skipped", "requires odometry"),
    )
    assert (report["overall"], report["reason"], exit_code) == (
        "degraded", "sbc-unavailable", 2)
    assert report["checks"][1]["name"] == "lifecycle.controller_server"


def test_partial_lifecycle_is_a_pi_readiness_failure():
    report, exit_code = _report(
        _check("container", "navigation", "pass"),
        _check("pi", "lifecycle.controller_server", "fail", "inactive"),
        _check("sensor", "topic.odom.fresh", "degraded"),
    )
    assert (report["overall"], report["reason"], exit_code) == (
        "degraded", "pi-readiness", 2)


def test_duplicate_sensor_publisher_is_not_hidden_by_sbc_state():
    report, exit_code = _report(
        _check("container", "navigation", "pass"),
        _check("pi", "lifecycle.controller_server", "pass"),
        _check("sensor", "topic.odom.publishers", "fail", "2 expected 1"),
    )
    assert (report["overall"], report["reason"], exit_code) == (
        "degraded", "sensor-readiness", 2)


def test_stale_dynamic_tf_with_connected_sensors_is_a_sensor_failure():
    report, exit_code = _report(
        _check("container", "navigation", "pass"),
        _check("sensor", "topic.odom.fresh", "pass"),
        _check("sensor", "tf.odom-base_link", "fail", "unavailable"),
    )
    assert (report["overall"], report["reason"], exit_code) == (
        "degraded", "sensor-readiness", 2)
