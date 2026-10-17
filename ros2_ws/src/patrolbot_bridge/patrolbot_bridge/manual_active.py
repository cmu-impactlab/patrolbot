"""
Deadman evidence for manual drive while localization is unavailable.

Two human channels publish deadman evidence — the joystick teleop on
/patrolbot/manual_active and the dashboard on-screen teleop on
/patrolbot/web_teleop_active (relayed by the Pi web bridge) — at their tick
rate while a human holds the deadman or the command is still coasting to a
stop. A fresh report from either admits nonzero DRIVE through the bridge
even when localization recovery, Nav2 readiness, AMCL pose freshness or map
state would hold autonomy. Autonomous sources never assert it, so autonomy
stays blocked by commands_allowed exactly as before.

Kept free of ROS imports so the timeout and bypass predicate can be
unit-tested on a development host without a ROS installation.
"""

import math

# Matches the mux joy timeout and the teleop tick: several missed reports
# mean the human is gone or the teleop died, and the bypass must close. The
# same window governs both evidence topics (F710 and dashboard); the web
# bridge's own 300 ms zero watchdog closes its channel even earlier.
MANUAL_ACTIVE_TIMEOUT = 0.5


def manual_drive_bypass(
    manual_age_s,
    *,
    undock_hold,
    lidar_ready,
    guard_ready,
    raw_odom_ready,
    map_blocked,
    mutation_blocked,
):
    """
    Return True when fresh deadman evidence may bypass the localization hold.

    Only the localization gates are waived: recovery stage, Nav2 readiness,
    AMCL pose freshness and map state (loaded, failed, absent) never refuse
    manual DRIVE on their own. Everything physical still blocks: undock
    ownership, unavailable LiDAR, odometry-origin readiness, stale or
    non-finite raw odometry, and any in-flight or unknown map/lifecycle
    mutation owner (map_blocked and mutation_blocked are computed by the
    caller from the epoch runtime). Anything non-finite or stale fails closed.
    """
    try:
        fresh = math.isfinite(manual_age_s) and 0.0 <= manual_age_s <= MANUAL_ACTIVE_TIMEOUT
    except (TypeError, ValueError):
        return False
    if not fresh:
        return False
    if undock_hold:
        return False
    if not lidar_ready:
        return False
    if not guard_ready:
        return False
    if not raw_odom_ready:
        return False
    if map_blocked:
        return False
    if mutation_blocked:
        return False
    return True
