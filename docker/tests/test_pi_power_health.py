"""Tests for docker/pi-power-health.py. Pure filesystem reads, no hardware.

Every case drives the module against a temporary directory tree, so the suite
runs on any machine — including CI, where no firmware node exists. That absence
is itself one of the cases under test: the script must report "unknown" rather
than "clear".
"""

import importlib.util
import json
from pathlib import Path
import sys

import pytest


def _load_module():
    """Import the script by path; its name is not a Python identifier."""
    script = Path(__file__).resolve().parents[1] / "pi-power-health.py"
    spec = importlib.util.spec_from_file_location("pi_power_health", script)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


pph = _load_module()


# --- the firmware word -------------------------------------------------------

@pytest.mark.parametrize(
    "text,expected",
    [
        ("0x0", 0),
        ("0x50005", 0x50005),
        ("50005", 0x50005),          # kernels that omit the 0x prefix
        ("  0x20002  \n", 0x20002),  # surrounding whitespace
    ],
)
def test_read_throttle_word_parses_hex_with_or_without_prefix(
    tmp_path, text, expected
):
    node = tmp_path / "get_throttled"
    node.write_text(text, encoding="utf-8")
    assert pph.read_throttle_word(node) == expected


@pytest.mark.parametrize("text", ["", "   \n", "not-hex", "-1"])
def test_read_throttle_word_returns_none_for_unusable_content(tmp_path, text):
    node = tmp_path / "get_throttled"
    node.write_text(text, encoding="utf-8")
    assert pph.read_throttle_word(node) is None


def test_read_throttle_word_returns_none_when_node_absent(tmp_path):
    assert pph.read_throttle_word(tmp_path / "nope") is None


def test_read_throttle_word_returns_none_when_node_is_a_directory(tmp_path):
    node = tmp_path / "get_throttled"
    node.mkdir()
    assert pph.read_throttle_word(node) is None


# --- decoding ----------------------------------------------------------------

def test_decode_zero_word_clears_every_flag():
    flags = pph.decode_throttle_word(0)
    assert flags and all(value is False for value in flags.values())


def test_decode_unreadable_word_is_unknown_not_clear():
    """Distinction the issue turns on: absence must never read as all-clear."""
    flags = pph.decode_throttle_word(None)
    assert flags and all(value is None for value in flags.values())


def test_decode_separates_now_from_since_boot():
    # 0x50005: under-voltage now + currently throttled, and both since boot.
    flags = pph.decode_throttle_word(0x50005)
    assert flags["undervoltage_now"] is True
    assert flags["throttled_now"] is True
    assert flags["arm_frequency_capped_now"] is False
    assert flags["undervoltage_since_boot"] is True
    assert flags["throttled_since_boot"] is True
    assert flags["arm_frequency_capped_since_boot"] is False


def test_decode_history_without_current_condition():
    """0x50000 is the state after the supply recovers: sticky bits only."""
    flags = pph.decode_throttle_word(0x50000)
    assert flags["undervoltage_now"] is False
    assert flags["throttled_now"] is False
    assert flags["undervoltage_since_boot"] is True
    assert flags["throttled_since_boot"] is True


def test_decode_soft_temp_limit_bits():
    flags = pph.decode_throttle_word(0x80008)
    assert flags["soft_temp_limit_now"] is True
    assert flags["soft_temp_limit_since_boot"] is True


# --- the hwmon second witness ------------------------------------------------

def _make_hwmon(root: Path, name: str, alarm: str | None) -> Path:
    entry = root / "hwmon0"
    entry.mkdir(parents=True)
    (entry / "name").write_text(name + "\n", encoding="utf-8")
    if alarm is not None:
        (entry / "in0_lcrit_alarm").write_text(alarm + "\n", encoding="utf-8")
    return entry


@pytest.mark.parametrize("value,expected", [("0", False), ("1", True)])
def test_undervoltage_alarm_read_from_rpi_volt(tmp_path, value, expected):
    _make_hwmon(tmp_path, "rpi_volt", value)
    assert pph.read_undervoltage_alarm(tmp_path) is expected


def test_undervoltage_alarm_ignores_other_hwmon_devices(tmp_path):
    _make_hwmon(tmp_path, "cpu_thermal", "1")
    assert pph.read_undervoltage_alarm(tmp_path) is None


def test_undervoltage_alarm_none_when_hwmon_root_absent(tmp_path):
    assert pph.read_undervoltage_alarm(tmp_path / "missing") is None


def test_undervoltage_alarm_none_when_alarm_file_missing(tmp_path):
    _make_hwmon(tmp_path, "rpi_volt", None)
    assert pph.read_undervoltage_alarm(tmp_path) is None


# --- cpu frequency corroboration --------------------------------------------

def test_cpu_frequency_reports_ratio(tmp_path):
    (tmp_path / "scaling_cur_freq").write_text("600000\n", encoding="utf-8")
    (tmp_path / "cpuinfo_max_freq").write_text("2400000\n", encoding="utf-8")
    assert pph.read_cpu_frequency(tmp_path) == {
        "current_khz": 600000,
        "max_khz": 2400000,
        "ratio_of_max": 0.25,
    }


def test_cpu_frequency_absent_is_none_not_zero(tmp_path):
    assert pph.read_cpu_frequency(tmp_path) == {
        "current_khz": None,
        "max_khz": None,
        "ratio_of_max": None,
    }


def test_cpu_frequency_ratio_none_when_max_is_zero(tmp_path):
    (tmp_path / "scaling_cur_freq").write_text("600000\n", encoding="utf-8")
    (tmp_path / "cpuinfo_max_freq").write_text("0\n", encoding="utf-8")
    assert pph.read_cpu_frequency(tmp_path)["ratio_of_max"] is None


# --- report and exit status --------------------------------------------------

def _patch_sources(
    monkeypatch,
    word_text,
    tmp_path,
    hwmon_root=None,
    cpufreq_root=None,
):
    node = tmp_path / "get_throttled"
    if word_text is not None:
        node.write_text(word_text, encoding="utf-8")
    monkeypatch.setattr(
        pph, "THROTTLE_NODE", node
    )
    monkeypatch.setattr(
        pph, "HWMON_ROOT", hwmon_root or (tmp_path / "no-hwmon")
    )
    monkeypatch.setattr(
        pph, "CPUFREQ_ROOT", cpufreq_root or (tmp_path / "no-cpufreq")
    )


def test_collect_marks_unreadable_source(monkeypatch, tmp_path):
    _patch_sources(monkeypatch, None, tmp_path=tmp_path)
    report = pph.collect()
    assert report["readable"] is False
    assert report["raw"] is None
    assert report["asserted"] == []


def test_collect_lists_asserted_bits(monkeypatch, tmp_path):
    _patch_sources(monkeypatch, "0x20002", tmp_path=tmp_path)
    report = pph.collect()
    assert report["readable"] is True
    assert report["raw"] == "0x20002"
    assert set(report["asserted"]) == {
        "arm_frequency_capped_now",
        "arm_frequency_capped_since_boot",
    }


def test_main_exit_zero_when_clear(monkeypatch, tmp_path, capsys):
    _patch_sources(monkeypatch, "0x0", tmp_path=tmp_path)
    assert pph.main([]) == 0
    assert "OVERALL=clear" in capsys.readouterr().out


def test_main_exit_one_on_finding(monkeypatch, tmp_path, capsys):
    _patch_sources(monkeypatch, "0x50005", tmp_path=tmp_path)
    assert pph.main([]) == 1
    out = capsys.readouterr().out
    assert "OVERALL=finding" in out
    assert "undervoltage_now" in out


def test_main_exit_two_when_unreadable_and_says_so(
    monkeypatch, tmp_path, capsys
):
    _patch_sources(monkeypatch, None, tmp_path=tmp_path)
    assert pph.main([]) == 2
    out = capsys.readouterr().out
    assert "OVERALL=unknown" in out
    assert "not an all-clear" in out


def test_main_json_output_is_parseable(monkeypatch, tmp_path, capsys):
    _patch_sources(monkeypatch, "0x50000", tmp_path=tmp_path)
    assert pph.main(["--json"]) == 1
    payload = json.loads(capsys.readouterr().out)
    assert payload["raw"] == "0x50000"
    assert payload["flags"]["undervoltage_since_boot"] is True
    assert payload["flags"]["undervoltage_now"] is False


def test_report_mentions_hwmon_unknown_when_driver_absent(
    monkeypatch, tmp_path, capsys
):
    _patch_sources(monkeypatch, "0x0", tmp_path=tmp_path)
    pph.main([])
    assert "unknown (no rpi_volt hwmon)" in capsys.readouterr().out


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
