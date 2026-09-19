"""Origin/transport interleaving contracts; no ROS or hardware required."""
import unittest

from patrolbot_bridge.odom_epoch import OdomEpochGuard, parse_frame_epoch

A = '123e4567-e89b-12d3-a456-426614174000'
B = 'aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee'


def frame(epoch=A):
    return 'ODOM:0,0,0,0,0|LASER:1,1|ODOM_EPOCH:' + epoch


class EpochTests(unittest.TestCase):
    def test_link_reconnect_preserves_origin_and_never_requests_reset(self):
        guard = OdomEpochGuard()
        first = guard.begin_session()
        self.assertFalse(guard.announce(first, A).reset_required)
        guard.disconnect(first)
        second = guard.begin_session()
        decision = guard.announce(second, A)
        self.assertTrue(decision.accepted)
        self.assertFalse(decision.reset_required)
        self.assertEqual(guard.generation, 1)
        self.assertFalse(guard.check_frame(first, frame()).accepted)
        self.assertTrue(guard.check_frame(second, frame()).accepted)

    def test_origin_change_requests_exactly_one_reset(self):
        guard = OdomEpochGuard()
        first = guard.begin_session()
        guard.announce(first, A)
        second = guard.begin_session()
        self.assertTrue(guard.announce(second, B).reset_required)
        self.assertFalse(guard.announce(second, B).reset_required)
        self.assertEqual(guard.generation, 2)
        self.assertFalse(guard.check_frame(first, frame()).accepted)
        self.assertTrue(guard.check_frame(second, frame(B)).accepted)
        third = guard.begin_session()
        self.assertFalse(guard.announce(third, A).accepted)

    def test_no_frame_without_current_banner(self):
        guard = OdomEpochGuard()
        self.assertFalse(guard.announce(0, A).accepted)
        session = guard.begin_session()
        self.assertFalse(guard.check_frame(session, frame()).accepted)
        self.assertFalse(guard.announce(session, None).accepted)
        self.assertFalse(guard.announce(session, A).accepted)

    def test_bad_frame_latches_transport_invalid_without_changing_origin(self):
        for bad in [frame(B), 'ODOM:0,0,0,0,0|LASER:1',
                    frame() + '|ODOM_EPOCH:' + A, frame('9' * 5000)]:
            with self.subTest(bad=bad[:80]):
                guard = OdomEpochGuard()
                session = guard.begin_session()
                guard.announce(session, A)
                self.assertFalse(guard.check_frame(session, bad).accepted)
                self.assertFalse(guard.check_frame(session, frame()).accepted)
                self.assertFalse(guard.announce(session, A).accepted)
                self.assertEqual(guard.epoch, A)
                self.assertEqual(guard.generation, 1)

    def test_old_disconnect_does_not_invalidate_new_transport(self):
        guard = OdomEpochGuard()
        old = guard.begin_session()
        guard.announce(old, A)
        current = guard.begin_session()
        guard.announce(current, A)
        guard.disconnect(old)
        self.assertTrue(guard.check_frame(current, frame()).accepted)

    def test_mid_transport_banner_change_is_not_authority_to_reset(self):
        guard = OdomEpochGuard()
        session = guard.begin_session()
        guard.announce(session, A)
        decision = guard.announce(session, B)
        self.assertFalse(decision.accepted)
        self.assertFalse(decision.reset_required)
        self.assertFalse(guard.check_frame(session, frame()).accepted)

    def test_restarted_guard_has_no_established_origin(self):
        guard = OdomEpochGuard()
        session = guard.begin_session()
        self.assertFalse(guard.ready)
        self.assertIsNone(guard.epoch)
        self.assertFalse(guard.check_frame(session, frame()).accepted)

    def test_frame_parser_requires_exact_canonical_uuid(self):
        self.assertEqual(parse_frame_epoch(frame()), A)
        for value in ['', 'invalid', A.upper(), A + '\n']:
            with self.assertRaises(ValueError):
                parse_frame_epoch(frame(value))


if __name__ == '__main__':
    unittest.main()
