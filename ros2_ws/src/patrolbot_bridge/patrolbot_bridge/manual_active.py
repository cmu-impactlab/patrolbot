"""
Deadman evidence for manual drive during localization recovery.

The joystick teleop publishes /patrolbot/manual_active at its tick rate while
a human holds the deadman (or the command is still coasting to a stop). The
bridge lets nonzero DRIVE through while localization recovery is pending only
when this evidence is fresh. Autonomous sources never assert it, so autonomy
stays blocked by commands_allowed exactly as before.

Kept free of ROS imports so the timeout and bypass predicate can be
unit-tested on a development host without a ROS installation.
"""

# Matches the mux joy timeout and the teleop tick: several missed reports
# mean the human is gone or the teleop died, and the bypass must close.
MANUAL_ACTIVE_TIMEOUT = 0.5


def manual_drive_bypass(
    manual_age_s, *, undock_hold, lidar_ready, map_hold, guard_ready, recovery_required
):
    """
    Return True when a fresh deadman may bypass the recovery DRIVE block.

    Only the localization-recovery condition is bypassed, and only while
    recovery is actually pending. Undock ownership, LiDAR readiness, map
    holds and odometry-origin readiness still block unconditionally, exactly
    as in commands_allowed. Anything unfinite or stale fails closed.
    """
    try:
        fresh = manual_age_s is not None and 0.0 <= manual_age_s <= MANUAL_ACTIVE_TIMEOUT
    except TypeError:
        return False
    if not fresh:
        return False
    if undock_hold:
        return False
    if not lidar_ready:
        return False
    if map_hold:
        return False
    if not guard_ready:
        return False
    return bool(recovery_required)
