"""Driver decision logic: watchdog, limit enforcement, and state tracking.

This is the layer that decides what actually reaches the gimbal, so it is kept
free of both ROS and serial I/O and driven by an injected clock. Every rule
here is a safety rule.

The central one: a client is not trusted. The dashboard could send anything,
so limits are enforced here, on the robot, not only in the sender.
"""

import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from patrolbot_gimbal import gimbal_policy as gp        # noqa: E402
from patrolbot_gimbal import sbgc_protocol as p         # noqa: E402
from patrolbot_gimbal.driver_core import DriverCore     # noqa: E402


# CMD_CONTROL payload offsets. Three mode bytes, then speed/angle interleaved
# per axis in ROLL, PITCH, YAW order:
#
#   [0:3]   mode bytes
#   [3:5]   ROLL  speed     [5:7]    ROLL  angle
#   [7:9]   PITCH speed     [9:11]   PITCH angle
#   [11:13] YAW   speed     [13:15]  YAW   angle
#
# pan is YAW, tilt is PITCH.
PAN_SPEED = slice(11, 13)
PAN_ANGLE = slice(13, 15)
TILT_SPEED = slice(7, 9)
TILT_ANGLE = slice(9, 11)


def _i16(payload, where):
    return int.from_bytes(payload[where], 'little', signed=True)


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def advance(self, seconds):
        self.t += seconds


@pytest.fixture
def core():
    clock = FakeClock()
    return DriverCore(cfg=gp.default_config(), clock=clock,
                      command_timeout_s=0.5), clock


# ----------------------------------------------------------------- watchdog --

def test_no_command_yet_means_hold(core):
    driver, _ = core
    assert driver.tick() == driver.hold_payload()
    assert driver.command_timed_out


def test_a_fresh_rate_command_is_forwarded(core):
    driver, _ = core
    driver.submit_rate(pan=30.0, tilt=0.0, roll=0.0)
    payload = driver.tick()
    assert not driver.command_timed_out
    # pan maps to YAW; 30 deg/s -> 246 units
    assert _i16(payload, PAN_SPEED) == 246


def test_command_expires_after_the_timeout(core):
    """The fail-closed guarantee: a silent publisher stops the gimbal."""
    driver, clock = core
    driver.submit_rate(pan=30.0, tilt=0.0, roll=0.0)
    driver.tick()

    clock.advance(0.51)
    assert driver.tick() == driver.hold_payload()
    assert driver.command_timed_out


def test_command_does_not_expire_early(core):
    driver, clock = core
    driver.submit_rate(pan=30.0, tilt=0.0, roll=0.0)
    clock.advance(0.49)
    driver.tick()
    assert not driver.command_timed_out


def test_refreshing_the_command_resets_the_watchdog(core):
    driver, clock = core
    driver.submit_rate(pan=30.0, tilt=0.0, roll=0.0)
    clock.advance(0.4)
    driver.submit_rate(pan=30.0, tilt=0.0, roll=0.0)
    clock.advance(0.4)
    driver.tick()
    assert not driver.command_timed_out


def test_an_invalid_command_holds_immediately(core):
    """Deadman released: hold now, do not wait for the timeout."""
    driver, _ = core
    driver.submit_rate(pan=30.0, tilt=0.0, roll=0.0)
    driver.submit_invalid()
    assert driver.tick() == driver.hold_payload()


# ------------------------------------------------------- limit enforcement --

def test_rate_is_gated_at_the_upper_limit(core):
    driver, clock = core
    driver.set_position(pan=driver.cfg.pan_max_deg, tilt=0.0, roll=0.0)
    driver.submit_rate(pan=30.0, tilt=0.0, roll=0.0)
    payload = driver.tick()
    assert _i16(payload, PAN_SPEED) == 0


def test_rate_away_from_the_limit_is_still_allowed(core):
    driver, _ = core
    driver.set_position(pan=driver.cfg.pan_max_deg, tilt=0.0, roll=0.0)
    driver.submit_rate(pan=-30.0, tilt=0.0, roll=0.0)
    payload = driver.tick()
    assert _i16(payload, PAN_SPEED) == -246


def test_angle_command_is_clamped_server_side(core):
    """A client asking for 400 deg gets the limit, not 400 deg."""
    driver, _ = core
    driver.submit_angle(pan=400.0, tilt=0.0, roll=0.0)
    payload = driver.tick()
    assert _i16(payload, PAN_ANGLE) == p.deg_to_units(driver.cfg.pan_max_deg)


def test_angle_command_is_clamped_at_the_lower_limit(core):
    driver, _ = core
    driver.submit_angle(pan=0.0, tilt=-400.0, roll=0.0)
    payload = driver.tick()
    assert _i16(payload, TILT_ANGLE) == p.deg_to_units(driver.cfg.tilt_min_deg)


def test_absurd_rate_is_clamped_not_forwarded(core):
    """A malformed or malicious client must not command full speed."""
    driver, _ = core
    driver.submit_rate(pan=1e6, tilt=0.0, roll=0.0)
    payload = driver.tick()
    assert _i16(payload, PAN_SPEED) == p.degs_to_units(driver.cfg.max_pan_deg_s)


def test_nan_and_inf_are_rejected_as_invalid(core):
    """Non-finite values would encode as garbage; treat them as a hold."""
    driver, _ = core
    driver.submit_rate(pan=float('nan'), tilt=0.0, roll=0.0)
    assert driver.tick() == driver.hold_payload()

    driver.submit_rate(pan=float('inf'), tilt=0.0, roll=0.0)
    assert driver.tick() == driver.hold_payload()


# --------------------------------------------------------- state tracking --

def test_board_angles_update_the_reported_state(core):
    driver, _ = core
    payload = bytearray(18)
    payload[0:2] = p.deg_to_units(10.0).to_bytes(2, 'little', signed=True)
    payload[6:8] = p.deg_to_units(-20.0).to_bytes(2, 'little', signed=True)
    payload[12:14] = p.deg_to_units(30.0).to_bytes(2, 'little', signed=True)

    driver.on_frame(p.CMD_GET_ANGLES, bytes(payload))
    state = driver.state()
    assert state.imu_roll == pytest.approx(10.0, abs=0.05)
    assert state.imu_tilt == pytest.approx(-20.0, abs=0.05)
    assert state.imu_pan == pytest.approx(30.0, abs=0.05)


def test_a_malformed_angle_frame_is_ignored_not_fatal(core):
    driver, _ = core
    driver.on_frame(p.CMD_GET_ANGLES, bytes(17))
    assert driver.state().fault != ''


def test_board_error_frame_is_surfaced_as_a_fault(core):
    driver, _ = core
    driver.on_frame(p.CMD_ERROR, b'\x01')
    assert 'error' in driver.state().fault.lower()


def test_state_reports_the_configured_limits(core):
    driver, _ = core
    state = driver.state()
    assert state.pan_min == driver.cfg.pan_min_deg
    assert state.tilt_max == driver.cfg.tilt_max_deg


def test_position_estimate_integrates_rate_between_board_updates(core):
    """Between IMU frames the driver still needs a position to gate against."""
    driver, clock = core
    driver.set_position(pan=0.0, tilt=0.0, roll=0.0)
    driver.submit_rate(pan=10.0, tilt=0.0, roll=0.0)
    driver.tick()
    clock.advance(1.0)
    driver.submit_rate(pan=10.0, tilt=0.0, roll=0.0)
    driver.tick()
    assert driver.state().target_pan == pytest.approx(10.0, abs=0.5)


def test_board_angles_override_the_integrated_estimate(core):
    """Ground truth wins; the estimate is only a stand-in."""
    driver, _ = core
    driver.set_position(pan=50.0, tilt=0.0, roll=0.0)
    payload = bytearray(18)
    payload[12:14] = p.deg_to_units(5.0).to_bytes(2, 'little', signed=True)
    driver.on_frame(p.CMD_GET_ANGLES, bytes(payload))
    assert driver.state().imu_pan == pytest.approx(5.0, abs=0.05)
