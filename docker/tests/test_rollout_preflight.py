"""ROS-free rollout preflight freshness tests."""
import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'rollout_preflight.py'
spec = importlib.util.spec_from_file_location('rollout_preflight', SCRIPT)
preflight = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preflight)


def base(received):
    return {
        'received': received,
        'link_connected': True,
        'hardware_state_valid': True,
        'odom_epoch_valid': True,
        'telemetry_age': 0.1,
        'motors_enabled': False,
        'undock_active': False,
        'dock_state': 'DOCKED_CONFIRMED',
    }


def odom(received):
    return {
        'received': received,
        'finite': True,
        'age': 0.1,
        'linear': 0.0,
        'angular': 0.0,
    }


def capture(base_times, odom_times, now):
    return preflight.evaluate([base(t) for t in base_times],
                              [odom(t) for t in odom_times], now)


class RolloutPreflightTests(unittest.TestCase):
    def test_end_of_capture_outage_rejects_early_safe_samples(self):
        result = capture([0.0, 1.0, 2.0], [2.0, 2.1, 2.2], now=5.0)

        self.assertEqual(result['base_samples'], 3)
        self.assertEqual(result['odom_samples'], 3)
        self.assertFalse(result['base_safe'])
        self.assertFalse(result['odom_safe'])
        self.assertFalse(result['safe'])

    def test_either_stream_stopping_at_freshness_boundary_rejects(self):
        self.assertFalse(capture([4.0, 4.1, 4.5], [4.7, 4.8, 4.9], 5.0)['safe'])
        self.assertFalse(capture([4.7, 4.8, 4.9], [4.0, 4.1, 4.5], 5.0)['safe'])

    def test_recent_samples_at_capture_end_are_accepted(self):
        result = capture([3.6, 4.1, 4.8], [4.7, 4.85, 4.95], now=5.0)

        self.assertTrue(result['base_safe'])
        self.assertTrue(result['odom_safe'])
        self.assertTrue(result['safe'])

    def test_missing_or_insufficient_stream_data_fails_closed(self):
        self.assertFalse(capture([], [4.7, 4.85, 4.95], now=5.0)['safe'])
        self.assertFalse(capture([4.0, 4.5, 4.9], [], now=5.0)['safe'])
        self.assertFalse(capture([4.7, 4.9], [4.7, 4.85, 4.95], now=5.0)['safe'])

    def test_future_receipt_timestamp_is_rejected(self):
        result = capture([4.0, 4.5, 5.1], [4.7, 4.85, 5.1], now=5.0)

        self.assertFalse(result['base_safe'])
        self.assertFalse(result['odom_safe'])
        self.assertFalse(result['safe'])


if __name__ == '__main__':
    unittest.main()
