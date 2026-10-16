"""Recovery must prove lifecycle, seed and result ordering for one origin."""

import unittest

from patrolbot_bridge.epoch_recovery import EpochRecovery

A = '123e4567-e89b-12d3-a456-426614174000'
B = 'aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee'


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.state = EpochRecovery()
        self.token = self.state.observe_origin(A)

    def test_only_reset_seed_and_new_pose_release_recovery(self):
        s = self.state
        self.assertTrue(s.required)
        self.assertFalse(s.operator_seed(self.token, 10, 10))
        self.assertFalse(s.amcl_pose(self.token, 11, 11))
        self.assertTrue(s.reset_complete(self.token, 20))
        self.assertFalse(s.amcl_pose(self.token, 21, 21))
        self.assertTrue(s.operator_seed(self.token, 22, 22))
        self.assertFalse(s.amcl_pose(self.token, 19, 23))
        self.assertFalse(s.amcl_pose(self.token, 22, 23))
        self.assertTrue(s.amcl_pose(self.token, 23, 23))
        self.assertFalse(s.required)

    def test_link_reconnect_same_origin_does_not_request_another_reset(self):
        self.state.reset_complete(self.token, 10)
        self.assertEqual(self.state.observe_origin(A), self.token)
        self.assertEqual(self.state.stage, EpochRecovery.WAIT_SEED)

    def test_origin_change_rejects_inflight_ack_and_pose(self):
        self.state.reset_complete(self.token, 10)
        self.state.operator_seed(self.token, 11, 11)
        new = self.state.observe_origin(B)
        self.assertNotEqual(new, self.token)
        self.assertFalse(self.state.amcl_pose(self.token, 12, 12))
        self.assertFalse(self.state.reset_complete(self.token, 13))
        self.assertTrue(self.state.required)
        self.assertTrue(self.state.reset_complete(new, 14))

    def test_bridge_restart_does_not_accept_prior_ack_or_surviving_amcl(self):
        restarted = EpochRecovery()
        new = restarted.observe_origin(A)
        self.assertNotEqual(new.bridge_instance, self.token.bridge_instance)
        self.assertFalse(restarted.reset_complete(self.token, 10))
        self.assertFalse(restarted.amcl_pose(new, 11, 11))
        self.assertTrue(restarted.required)

    def test_clock_regression_revokes_ready_and_old_token(self):
        s = self.state
        s.reset_complete(self.token, 10)
        s.operator_seed(self.token, 11, 11)
        s.amcl_pose(self.token, 12, 12)
        self.assertFalse(s.required)
        self.assertFalse(s.observe_clock(9))
        self.assertTrue(s.required)
        self.assertNotEqual(s.token, self.token)
        self.assertFalse(s.reset_complete(self.token, 10))

    def test_future_and_queued_seeds_never_release(self):
        s = self.state
        s.reset_complete(self.token, 10)
        for stamp in (9, 10, 12, True, 11.0):
            self.assertFalse(s.operator_seed(self.token, stamp, 11))
        self.assertTrue(s.operator_seed(self.token, 12, 12))
        self.assertFalse(s.operator_seed(self.token, 11, 13))
        self.assertFalse(s.amcl_pose(self.token, 20, 13))
        self.assertTrue(s.required)

    def test_new_seed_revokes_ready_until_a_new_result(self):
        s = self.state
        s.reset_complete(self.token, 10)
        s.operator_seed(self.token, 11, 11)
        s.amcl_pose(self.token, 12, 12)
        self.assertTrue(s.operator_seed(self.token, 13, 13))
        self.assertTrue(s.required)
        self.assertFalse(s.amcl_pose(self.token, 12, 14))


if __name__ == '__main__':
    unittest.main()
