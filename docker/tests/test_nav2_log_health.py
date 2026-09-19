import importlib.util
import json
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "nav2-log-health.py"
SPEC = importlib.util.spec_from_file_location("nav2_log_health", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def group(summary, logger, topic):
    return next(item for item in summary["observed_log_window"]["groups"]
                if item["logger"] == logger and item["topic"] == topic)


def test_current_prefixed_logs_use_latest_cumulative_count():
    summary = MODULE.summarize([
        "[container] [nav2_costmap_2d]: The scan observation buffer has not "
        "been updated for 1.00 seconds, and it should be updated every 0.50 "
        "seconds (stale checks: 1).",
        "[container] [nav2_costmap_2d]: The scan observation buffer has not "
        "been updated for 31.00 seconds, and it should be updated every 0.50 "
        "seconds (stale checks: 156).",
        "[container] [nav2_costmap_2d]: The scan observation buffer is current "
        "again after 160 stale checks.",
    ])
    item = group(summary, "nav2_costmap_2d", "scan")
    assert item["warning_log_count"] == 2
    assert item["latest_stale_checks"] == 160
    assert item["recovery_count"] == 1
    assert summary["observed_log_window"]["observation"] == "stale_observed"


def test_legacy_warning_has_unknown_counter():
    summary = MODULE.summarize([
        "The laser observation buffer has not been updated for 2.00 seconds, "
        "and it should be updated every 1.00 seconds.",
    ])
    item = group(summary, "unknown", "laser")
    assert item["warning_log_count"] == 1
    assert item["latest_stale_checks"] is None


def test_empty_window_is_zero_observed_and_unknown():
    window = MODULE.summarize([])["observed_log_window"]
    assert window["warning_log_count"] == 0
    assert window["recovery_count"] == 0
    assert window["groups"] == []
    assert window["observation"] == "unknown"
    assert "healthy" not in json.dumps(window)


def test_same_topic_from_distinct_loggers_stays_separate():
    summary = MODULE.summarize([
        "[costmap_a]: The scan observation buffer has not been updated for "
        "1.00 seconds, and it should be updated every 0.50 seconds "
        "(stale checks: 1).",
        "[costmap_b]: The scan observation buffer has not been updated for "
        "1.00 seconds, and it should be updated every 0.50 seconds "
        "(stale checks: 1).",
    ])
    assert group(summary, "costmap_a", "scan")["warning_log_count"] == 1
    assert group(summary, "costmap_b", "scan")["warning_log_count"] == 1


def test_counter_decrease_is_explicit_reset_without_session_inference():
    summary = MODULE.summarize([
        "[costmap]: The scan observation buffer has not been updated for 1.00 "
        "seconds, and it should be updated every 0.50 seconds (stale checks: 9).",
        "[costmap]: The scan observation buffer has not been updated for 1.00 "
        "seconds, and it should be updated every 0.50 seconds (stale checks: 2).",
    ])
    item = group(summary, "costmap", "scan")
    assert item["latest_stale_checks"] == 2
    assert item["counter_reset_observed"] is True
    assert item["counter_reset_count"] == 1
    assert "session" not in item
