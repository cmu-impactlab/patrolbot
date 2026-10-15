#!/usr/bin/env python3
"""Unit tests for the deadman-evidence manual DRIVE bypass (manual_active)."""
import unittest

from patrolbot_bridge.manual_active import (
    MANUAL_ACTIVE_TIMEOUT,
    manual_drive_bypass,
)


def base_kwargs(**overrides):
    kwargs = dict(
        undock_hold=False,
        lidar_ready=True,
        map_hold=False,
        guard_ready=True,
        recovery_required=True,
    )
    kwargs.update(overrides)
    return kwargs


class ManualDriveBypassTest(unittest.TestCase):
    def test_timeout_value_matches_mux_joy_timeout(self):
        self.assertEqual(MANUAL_ACTIVE_TIMEOUT, 0.5)

    def test_fresh_deadman_bypasses_recovery_block_only(self):
        self.assertTrue(manual_drive_bypass(0.0, **base_kwargs()))
        self.assertTrue(manual_drive_bypass(0.49, **base_kwargs()))

    def test_stale_or_missing_evidence_fails_closed(self):
        self.assertFalse(manual_drive_bypass(0.51, **base_kwargs()))
        self.assertFalse(manual_drive_bypass(60.0, **base_kwargs()))
        self.assertFalse(manual_drive_bypass(None, **base_kwargs()))
        self.assertFalse(manual_drive_bypass(float("nan"), **base_kwargs()))
        self.assertFalse(manual_drive_bypass(-0.1, **base_kwargs()))

    def test_no_bypass_when_recovery_not_pending(self):
        # Normal path (commands_allowed) governs; autonomy stays blocked here.
        self.assertFalse(manual_drive_bypass(0.0, **base_kwargs(recovery_required=False)))

    def test_hard_blocks_are_never_bypassed(self):
        self.assertFalse(manual_drive_bypass(0.0, **base_kwargs(undock_hold=True)))
        self.assertFalse(manual_drive_bypass(0.0, **base_kwargs(lidar_ready=False)))
        self.assertFalse(manual_drive_bypass(0.0, **base_kwargs(map_hold=True)))
        self.assertFalse(manual_drive_bypass(0.0, **base_kwargs(guard_ready=False)))


if __name__ == "__main__":
    unittest.main()
