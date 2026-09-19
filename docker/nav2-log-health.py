#!/usr/bin/env python3
"""Summarize supplied Nav2 stale-observation log lines as bounded JSON."""

import json
import re
import sys


STALE_RE = re.compile(
    r"The (?P<topic>.+?) observation buffer has not been updated for "
    r"(?P<age>[0-9]+(?:\.[0-9]+)?) seconds, and it should be updated every "
    r"(?P<rate>[0-9]+(?:\.[0-9]+)?) seconds"
    r"(?: \(stale checks: (?P<count>[0-9]+)\))?\.")
RECOVERY_RE = re.compile(
    r"The (?P<topic>.+?) observation buffer is current again after "
    r"(?P<count>[0-9]+) stale checks\.")


def _message(line):
    """Accept raw log lines and JSON log records without spawning anything."""
    try:
        value = json.loads(line)
    except json.JSONDecodeError:
        return line
    if isinstance(value, dict):
        for key in ("message", "msg", "text", "log"):
            if isinstance(value.get(key), str):
                return value[key]
    return line


def _logger(line, message_start):
    prefix = line[:message_start]
    matches = re.findall(r"\[([^\]]+)\]", prefix)
    return matches[-1] if matches else "unknown"


def summarize(lines):
    groups = {}
    for raw in lines:
        line = raw.rstrip("\n")
        message = _message(line)
        stale = STALE_RE.search(message)
        recovery = RECOVERY_RE.search(message)
        if stale is None and recovery is None:
            continue
        match = stale or recovery
        topic = match.group("topic")
        start = message.find(match.group(0))
        logger = _logger(message, start)
        key = f"{logger}|{topic}"
        group = groups.setdefault(key, {
            "logger": logger,
            "topic": topic,
            "warning_log_count": 0,
            "latest_stale_checks": None,
            "recovery_count": 0,
            "counter_reset_observed": False,
            "counter_reset_count": 0,
        })
        if stale is not None:
            group["warning_log_count"] += 1
            count = stale.group("count")
            if count is not None:
                count = int(count)
                previous = group["latest_stale_checks"]
                if previous is not None and count < previous:
                    group["counter_reset_observed"] = True
                    group["counter_reset_count"] += 1
                group["latest_stale_checks"] = count
        else:
            group["recovery_count"] += 1
            count = int(recovery.group("count"))
            previous = group["latest_stale_checks"]
            if previous is not None and count < previous:
                group["counter_reset_observed"] = True
                group["counter_reset_count"] += 1
            group["latest_stale_checks"] = count

    warning_count = sum(item["warning_log_count"] for item in groups.values())
    recovery_count = sum(item["recovery_count"] for item in groups.values())
    observation = "unknown"
    if warning_count:
        observation = "stale_observed"
    elif recovery_count:
        observation = "recovery_observed"
    return {
        "observed_log_window": {
            "observation": observation,
            "warning_log_count": warning_count,
            "recovery_count": recovery_count,
            "groups": list(groups.values()),
        }
    }


def main():
    json.dump(summarize(sys.stdin), sys.stdout, sort_keys=True)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
