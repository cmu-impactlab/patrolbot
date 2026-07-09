#!/usr/bin/env python3
"""Docker liveness probe that does not create ROS/DDS participants."""

from __future__ import annotations

import os
from pathlib import Path
import sys


EXPECTED_PROCESSES = {
    "bringup": (
        "twist_mux/twist_mux",
        "nav2_velocity_smoother/velocity_smoother",
        "lifecycle_mgr.py",
    ),
    "bridge": (
        "patrolbot_bridge/bridge_node",
    ),
    "navigation": (
        "component_container_isolated",
        "joy/joy_node",
        "patrolbot_joy_teleop.py",
        "patrolbot_safety_watchdog.py",
        "static_transform_publisher",
    ),
}


def process_commands() -> list[str]:
    commands: list[str] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode()
        except (FileNotFoundError, PermissionError, ProcessLookupError, UnicodeDecodeError):
            continue
        if command:
            commands.append(command)
    return commands


def main() -> int:
    role = os.environ.get("PATROLBOT_ROLE", "")
    expected = EXPECTED_PROCESSES.get(role)
    if expected is None:
        print(f"unknown PATROLBOT_ROLE: {role!r}", file=sys.stderr)
        return 2

    commands = process_commands()
    missing = [needle for needle in expected if not any(needle in cmd for cmd in commands)]
    if missing:
        print(f"{role} missing processes: {', '.join(missing)}", file=sys.stderr)
        return 1

    print(f"{role} processes healthy")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
