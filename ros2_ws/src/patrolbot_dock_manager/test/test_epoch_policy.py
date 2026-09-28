"""Epoch recovery must enable an operator seed without enabling motion."""
import unittest

from patrolbot_dock_manager.epoch_policy import epoch_hardware_ready


class EpochPolicyTests(unittest.TestCase):
    def test_missing_origin_and_legacy_payload_fail_closed(self):
        self.assertFalse(epoch_hardware_ready())
        for origin in (False, None):
            for recovery in (True, False):
                self.assertFalse(epoch_hardware_ready(
                    origin_valid=origin, recovery_required=recovery,
                    recovery_stage='wait-operator-seed',
                    for_pose_initialization=True))

    def test_seed_wait_allows_only_initializer(self):
        for stage in ('wait-operator-seed', 'wait-amcl-pose'):
            self.assertTrue(epoch_hardware_ready(
                origin_valid=True, recovery_stage=stage,
                for_pose_initialization=True))
            self.assertFalse(epoch_hardware_ready(
                origin_valid=True, recovery_stage=stage))

    def test_reset_and_failed_stages_cannot_seed(self):
        for stage in ('', 'wait-origin', 'wait-amcl-reset',
                      'wait-first-odometry', 'wait-amcl-tf', 'failed:activate',
                      'pause-navigation', 'resume-navigation', 'ready'):
            self.assertFalse(epoch_hardware_ready(
                origin_valid=True, recovery_stage=stage,
                for_pose_initialization=True), stage)

    def test_completed_recovery_allows_both_consumers(self):
        for initializer in (False, True):
            self.assertTrue(epoch_hardware_ready(
                origin_valid=True, recovery_required=False,
                recovery_stage='ready', for_pose_initialization=initializer))


if __name__ == '__main__':
    unittest.main()
