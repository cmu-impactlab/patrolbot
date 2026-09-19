#!/usr/bin/env python3
"""Classify and serialize bounded PatrolBot readiness checks.

Each command-line argument is scope|name|status|evidence. The collector in
status.sh owns the read-only probes; this module owns the deterministic overall
state so connected and degraded scenarios can be tested without ROS.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import sys


SCOPES = frozenset({"container", "pi", "sensor"})
STATUSES = frozenset({"pass", "fail", "degraded", "skipped"})


@dataclass(frozen=True)
class Check:
    scope: str
    name: str
    status: str
    evidence: str


def parse_check(specification: str) -> Check:
    parts = specification.split("|", 3)
    if len(parts) != 4:
        raise ValueError(f"invalid check specification: {specification!r}")
    check = Check(*parts)
    if check.scope not in SCOPES:
        raise ValueError(f"invalid check scope: {check.scope!r}")
    if check.status not in STATUSES:
        raise ValueError(f"invalid check status: {check.status!r}")
    if not check.name:
        raise ValueError("check name must not be empty")
    return check


def classify(checks: list[Check]) -> tuple[str, str | None, int]:
    if not checks:
        return "unhealthy", "no-checks", 1
    if any(check.scope == "container" and check.status == "fail"
           for check in checks):
        return "unhealthy", "container-liveness", 1
    if any(check.scope == "pi" and check.status == "fail"
           for check in checks):
        return "degraded", "pi-readiness", 2
    if any(check.scope == "sensor" and check.status == "fail"
           for check in checks):
        return "degraded", "sensor-readiness", 2
    if any(check.status in {"degraded", "skipped"} for check in checks):
        return "degraded", "sbc-unavailable", 2
    return "ready", None, 0


def build_report(checks: list[Check]) -> tuple[dict, int]:
    overall, reason, exit_code = classify(checks)
    report = {
        "overall": overall,
        "reason": reason,
        "checks": [asdict(check) for check in checks],
    }
    return report, exit_code


def main(arguments: list[str]) -> int:
    try:
        checks = [parse_check(argument) for argument in arguments]
    except ValueError as exc:
        print(f"status report input error: {exc}", file=sys.stderr)
        return 1
    report, exit_code = build_report(checks)
    print("REPORT_JSON=" + json.dumps(report, separators=(",", ":")))
    suffix = f" reason={report['reason']}" if report["reason"] else ""
    print(f"OVERALL={report['overall']}{suffix}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
