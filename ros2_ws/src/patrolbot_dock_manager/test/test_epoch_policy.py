"""Epoch recovery must enable an operator seed without enabling motion."""
import unittest
from types import SimpleNamespace

import pytest

from patrolbot_dock_manager.epoch_policy import epoch_hardware_ready, lidar_ready


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

    def test_undock_bypass_needs_origin_but_ignores_recovery_labels(self):
        # Authorized 2026-10-16: undock admission is E-stop plus charging
        # contact. Map/AMCL recovery labels must not gate the undock path,
        # but without a valid origin nothing is permitted.
        self.assertFalse(epoch_hardware_ready(for_undock=True))
        self.assertFalse(epoch_hardware_ready(
            origin_valid=False, for_undock=True))
        for stage in ('', 'wait-origin', 'wait-amcl-reset',
                      'wait-first-odometry', 'wait-amcl-tf',
                      'failed:activate', 'pause-navigation',
                      'resume-navigation', 'ready', 'wait-operator-seed',
                      'wait-amcl-pose'):
            self.assertTrue(epoch_hardware_ready(
                origin_valid=True, recovery_stage=stage,
                for_undock=True), stage)


if __name__ == '__main__':
    unittest.main()


def lidar_base(**changes):
    values = dict(lidar_state='READY', lidar_scan_age=0.05,
                  telemetry_age=0.05, lidar_power_supported=False)
    values.update(changes)
    return SimpleNamespace(**values)


def test_fresh_scans_allow_pose_recovery_without_commissioned_power_control():
    assert lidar_ready(lidar_base(), receipt_age=0.1, stamp_age=0.1)
    assert epoch_hardware_ready(origin_valid=True,
        recovery_stage='wait-operator-seed', for_pose_initialization=True)
    assert not epoch_hardware_ready(origin_valid=True,
        recovery_stage='wait-operator-seed')


@pytest.mark.parametrize('state', ['', 'UNKNOWN', 'OFF', 'STARTING', 'FAULT'])
def test_nonready_lidar_never_admits_dock_pose_or_motion(state):
    assert not lidar_ready(lidar_base(lidar_state=state), receipt_age=0., stamp_age=0.)


@pytest.mark.parametrize('field,value', [
    ('lidar_scan_age', -1.), ('lidar_scan_age', .51),
    ('lidar_scan_age', float('nan')), ('lidar_scan_age', float('inf')),
    ('telemetry_age', -1.), ('telemetry_age', .51),
    ('telemetry_age', float('nan')), ('telemetry_age', float('inf')),
])
def test_invalid_or_stale_lidar_measurements_fail_closed(field, value):
    assert not lidar_ready(lidar_base(**{field: value}), receipt_age=0., stamp_age=0.)


@pytest.mark.parametrize('receipt_age,stamp_age', [
    (.51, 0.), (0., .51), (-.1, 0.), (0., -.1),
    (float('nan'), 0.), (0., float('inf')), (.46, .1), (.1, .46),
])
def test_lidar_age_includes_dds_and_receipt_age(receipt_age, stamp_age):
    assert not lidar_ready(lidar_base(), receipt_age=receipt_age, stamp_age=stamp_age)


def test_missing_lidar_fields_cannot_restore_legacy_readiness():
    assert not lidar_ready(None, receipt_age=0., stamp_age=0.)
    assert not lidar_ready(SimpleNamespace(), receipt_age=0., stamp_age=0.)
