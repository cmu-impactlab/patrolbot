#!/usr/bin/env python3
"""Read-only Pi power/throttle report that needs neither `vcgencmd` nor root.

Issue #59 asks for current and historical throttle/undervoltage state to be
readable by the operator "without granting broad device access". `vcgencmd`
cannot supply that here: it lives in `raspi-utils`, talks to the VideoCore
mailbox through `/dev/vcio`, and on this Pi it is not available to the `ubuntu`
user — which is why the 2026-08-29 scan could report two retained undervoltage
events but no current throttle state.

The firmware exposes the same 32-bit word the mailbox call returns through
sysfs, world-readable:

    /sys/devices/platform/soc/soc:firmware/get_throttled

and the Pi's voltage monitor exposes a separate in-alarm flag through hwmon:

    /sys/class/hwmon/hwmon*/in0_lcrit_alarm   (name == "rpi_volt")

Neither needs a device node, a group membership, or a subprocess. Reading them
is exactly as read-only as the rest of this directory's status tooling.

Usage
-----
    ./pi-power-health.py            # human-readable report
    ./pi-power-health.py --json     # one JSON object, for logging or a probe

Exit status
-----------
    0  no throttle or undervoltage bit is set, now or since boot
    1  at least one bit is set (this is a finding, not a crash)
    2  the firmware word could not be read at all (non-Pi host, or the node is
       absent) — the report says so rather than implying an all-clear

Deliberately never reports "clear" from an unreadable source. An unreadable
node yields ``None``/``unknown``; a false all-clear on a power question is
worse than no answer, because it closes the investigation #59 wants kept open.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


THROTTLE_NODE = Path("/sys/devices/platform/soc/soc:firmware/get_throttled")
HWMON_ROOT = Path("/sys/class/hwmon")
RPI_VOLT_NAME = "rpi_volt"
CPUFREQ_ROOT = Path("/sys/devices/system/cpu/cpu0/cpufreq")

# Bit layout of the firmware's get_throttled word, as documented by Raspberry
# Pi. The low nibble is "right now"; the 0x1xxxx nibble is "has happened since
# boot" and is sticky until a power cycle.
NOW_BITS = (
    (0x1, "undervoltage_now", "under-voltage detected"),
    (0x2, "arm_frequency_capped_now", "ARM frequency capped"),
    (0x4, "throttled_now", "currently throttled"),
    (0x8, "soft_temp_limit_now", "soft temperature limit active"),
)
SINCE_BOOT_BITS = (
    (
        0x10000,
        "undervoltage_since_boot",
        "under-voltage has occurred",
    ),
    (
        0x20000,
        "arm_frequency_capped_since_boot",
        "ARM frequency capping has occurred",
    ),
    (
        0x40000,
        "throttled_since_boot",
        "throttling has occurred",
    ),
    (
        0x80000,
        "soft_temp_limit_since_boot",
        "soft temperature limit has occurred",
    ),
)


def read_throttle_word(node: Path = THROTTLE_NODE) -> int | None:
    """Return the firmware throttle word, or None when it cannot be read.

    The node holds hex, with or without a leading 0x depending on the kernel,
    so both are accepted. Any read or parse failure is an absence of data and
    never a zero.
    """
    try:
        raw = node.read_text(encoding="utf-8").strip()
    except OSError:
        return None
    if not raw:
        return None
    try:
        word = int(raw, 16)
    except ValueError:
        return None
    return word if word >= 0 else None


def decode_throttle_word(word: int | None) -> dict[str, bool | None]:
    """Decode every documented bit.

    An unreadable word leaves each flag None.
    """
    flags: dict[str, bool | None] = {}
    for mask, key, _label in NOW_BITS + SINCE_BOOT_BITS:
        flags[key] = None if word is None else bool(word & mask)
    return flags


def read_undervoltage_alarm(hwmon_root: Path = HWMON_ROOT) -> bool | None:
    """Return the rpi_volt hwmon low-critical alarm, or None when absent.

    This is a second, independent witness to the same physical condition: the
    firmware word can be read while the hwmon driver is missing, and vice
    versa, so a disagreement between them is itself worth seeing.
    """
    try:
        candidates = sorted(hwmon_root.iterdir())
    except OSError:
        return None
    for entry in candidates:
        try:
            name_text = (entry / "name").read_text(encoding="utf-8").strip()
            if name_text != RPI_VOLT_NAME:
                continue
            alarm_text = (
                (entry / "in0_lcrit_alarm").read_text(encoding="utf-8").strip()
            )
            return bool(int(alarm_text))
        except (OSError, ValueError):
            continue
    return None


def read_cpu_frequency(
    cpufreq_root: Path = CPUFREQ_ROOT,
) -> dict[str, int | float | None]:
    """Return current/max CPU kHz and the ratio, as portable corroboration.

    Not Pi-specific and not a substitute for the firmware word: a capped clock
    is consistent with throttling but also with an idle governor. It is here so
    that a host where the firmware node is missing still reports something
    measurable rather than nothing at all.
    """
    def read_khz(name: str) -> int | None:
        try:
            text = (cpufreq_root / name).read_text(encoding="utf-8").strip()
            return int(text)
        except (OSError, ValueError):
            return None

    current = read_khz("scaling_cur_freq")
    maximum = read_khz("cpuinfo_max_freq")
    ratio: float | None = None
    if current is not None and maximum:
        ratio = round(current / maximum, 3)
    return {"current_khz": current, "max_khz": maximum, "ratio_of_max": ratio}


def collect() -> dict[str, object]:
    """Gather the whole report. Performs reads only.

    The module-level path constants are read here rather than relied on as
    default arguments, so overriding one (a test, or a host that mounts sysfs
    elsewhere) actually takes effect: a default is bound at definition time and
    would silently keep pointing at the original node.
    """
    word = read_throttle_word(THROTTLE_NODE)
    flags = decode_throttle_word(word)
    alarm = read_undervoltage_alarm(HWMON_ROOT)
    asserted = [key for key, value in flags.items() if value]
    return {
        "source": str(THROTTLE_NODE),
        "readable": word is not None,
        "raw": None if word is None else f"0x{word:x}",
        "flags": flags,
        "asserted": asserted,
        "rpi_volt_lcrit_alarm": alarm,
        "cpu_frequency": read_cpu_frequency(CPUFREQ_ROOT),
    }


def format_report(report: dict[str, object]) -> str:
    """Render the report for an operator reading a terminal."""
    lines = ["Pi power and throttle state (read-only, no vcgencmd, no root)"]
    if not report["readable"]:
        lines.append(
            f"  firmware word        unreadable at {report['source']}"
        )
        lines.append("  status               UNKNOWN — not an all-clear")
    else:
        lines.append(f"  firmware word        {report['raw']}")
        flags = report["flags"]
        assert isinstance(flags, dict)
        for _mask, key, label in NOW_BITS:
            status = "YES" if flags[key] else "no "
            lines.append(f"  now                  {status}  {label}")
        for _mask, key, label in SINCE_BOOT_BITS:
            status = "YES" if flags[key] else "no "
            lines.append(f"  since boot           {status}  {label}")

    alarm = report["rpi_volt_lcrit_alarm"]
    if alarm is None:
        alarm_text = "unknown (no rpi_volt hwmon)"
    else:
        alarm_text = "YES" if alarm else "no"
    lines.append(f"  rpi_volt lcrit alarm {alarm_text}")

    freq = report["cpu_frequency"]
    assert isinstance(freq, dict)
    if freq["current_khz"] is None or freq["max_khz"] is None:
        lines.append("  cpu clock            unknown")
    else:
        cur = freq["current_khz"]
        mx = freq["max_khz"]
        ratio = freq["ratio_of_max"]
        lines.append(
            f"  cpu clock            {cur} / {mx} kHz"
            f" ({ratio} of max)"
        )

    asserted = report["asserted"]
    assert isinstance(asserted, list)
    if not report["readable"]:
        lines.append("\nOVERALL=unknown reason=firmware-node-unreadable")
    elif asserted:
        lines.append("\nOVERALL=finding bits=" + ",".join(asserted))
    else:
        lines.append("\nOVERALL=clear")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only Pi power/throttle report; no vcgencmd, no root."
        ),
    )
    parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
        help="emit one JSON object instead of the human-readable report",
    )
    args = parser.parse_args(argv)

    report = collect()
    if args.as_json:
        print(json.dumps(report, sort_keys=True))
    else:
        print(format_report(report))

    if not report["readable"]:
        return 2
    asserted = report["asserted"]
    assert isinstance(asserted, list)
    return 1 if asserted else 0


if __name__ == "__main__":
    raise SystemExit(main())
