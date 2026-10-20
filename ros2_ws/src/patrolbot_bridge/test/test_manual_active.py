#!/usr/bin/env python3
"""Unit tests for the deadman-evidence manual DRIVE bypass (manual_active)."""

import unittest

from patrolbot_bridge.manual_active import (
    MANUAL_ACTIVE_TIMEOUT,
    manual_drive_bypass,
)


def base_kwargs(**overrides):
    kwargs = {
        'undock_hold': False,
        'lidar_ready': True,
        'guard_ready': True,
        'raw_odom_ready': True,
        'map_blocked': False,
        'mutation_blocked': False,
        'tuning_hold': False,
    }
    kwargs.update(overrides)
    return kwargs


class ManualDriveBypassTest(unittest.TestCase):
    def test_timeout_value_matches_mux_joy_timeout(self):
        self.assertEqual(MANUAL_ACTIVE_TIMEOUT, 0.5)

    def test_fresh_evidence_admits_with_all_hard_gates_clear(self):
        self.assertTrue(manual_drive_bypass(0.0, **base_kwargs()))
        self.assertTrue(manual_drive_bypass(0.49, **base_kwargs()))

    def test_recovery_state_is_not_a_gate(self):
        # Localization recovery pending (the old recovery_required=True case),
        # recovery satisfied, Nav2 ready or not, AMCL pose fresh or stale and
        # loaded/failed/absent map state are all deliberately absent from the
        # predicate: they must never refuse manual DRIVE. Only the hard gates
        # below and a fresh report matter.
        self.assertTrue(manual_drive_bypass(0.1, **base_kwargs()))

    def test_settled_or_absent_map_state_does_not_block(self):
        # The caller flags only in-flight/unknown map states; a settled map
        # (ready or failed) and an absent map arrive as map_blocked=False.
        self.assertTrue(manual_drive_bypass(0.0, **base_kwargs(map_blocked=False)))

    def test_stale_or_missing_evidence_fails_closed(self):
        self.assertFalse(manual_drive_bypass(0.51, **base_kwargs()))
        self.assertFalse(manual_drive_bypass(60.0, **base_kwargs()))
        self.assertFalse(manual_drive_bypass(None, **base_kwargs()))
        self.assertFalse(manual_drive_bypass(float('nan'), **base_kwargs()))
        self.assertFalse(manual_drive_bypass(float('inf'), **base_kwargs()))
        self.assertFalse(manual_drive_bypass(-0.1, **base_kwargs()))
        self.assertFalse(manual_drive_bypass('fresh', **base_kwargs()))

    def test_stale_or_invalid_raw_odometry_fails_closed(self):
        self.assertFalse(manual_drive_bypass(0.0, **base_kwargs(raw_odom_ready=False)))

    def test_hard_blocks_are_never_bypassed(self):
        self.assertFalse(manual_drive_bypass(0.0, **base_kwargs(undock_hold=True)))
        self.assertFalse(manual_drive_bypass(0.0, **base_kwargs(lidar_ready=False)))
        self.assertFalse(manual_drive_bypass(0.0, **base_kwargs(guard_ready=False)))
        self.assertFalse(manual_drive_bypass(0.0, **base_kwargs(map_blocked=True)))
        self.assertFalse(manual_drive_bypass(0.0, **base_kwargs(mutation_blocked=True)))

    def test_tuning_activation_hold_is_never_bypassed(self):
        # A pending/unknown tuning job holds manual DRIVE even with fresh
        # deadman evidence and every other gate clear.
        self.assertFalse(manual_drive_bypass(0.0, **base_kwargs(tuning_hold=True)))


if __name__ == '__main__':
    unittest.main()
