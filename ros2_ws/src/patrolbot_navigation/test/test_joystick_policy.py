"""Unit tests for joystick behavior that does not require a ROS graph."""

import sys
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from joystick_policy import (  # noqa: E402
    LidarChord,
    apply_deadzone,
    axis_value,
    button_pressed,
    drive_intent,
    ramp_toward,
    rising_edge,
)


def test_axis_value_is_bounded_and_missing_axes_are_zero():
    assert axis_value([2.0, -2.0], 0) == 1.0
    assert axis_value([2.0, -2.0], 1) == -1.0
    assert axis_value([0.25], 4) == 0.0


def test_deadzone_suppresses_center_noise_without_rescaling():
    assert apply_deadzone(0.119, 0.12) == 0.0
    assert apply_deadzone(-0.119, 0.12) == 0.0
    assert apply_deadzone(0.12, 0.12) == 0.12


def test_button_and_rising_edge_are_fail_closed():
    assert button_pressed([0, 1], 1)
    assert not button_pressed([0, 1], 4)
    assert rising_edge(True, False)
    assert not rising_edge(True, True)
    assert not rising_edge(False, True)


def test_ramp_toward_never_overshoots():
    assert ramp_toward(0.0, 0.35, 0.1) == 0.1
    assert ramp_toward(0.3, 0.35, 0.1) == 0.35
    assert ramp_toward(0.0, -0.35, 0.1) == -0.1
    assert ramp_toward(-0.3, -0.35, 0.1) == -0.35
    assert ramp_toward(0.2, 0.0, 0.3) == 0.0


def test_shared_f710_drive_intent_requires_rb_and_nonzero_direction():
    buttons = [0] * 6
    axes = [0.0] * 8
    axes[1] = 0.5
    assert drive_intent(axes, buttons, 0.12) == (False, 0.0, 0.0)
    buttons[5] = 1
    assert drive_intent(axes, buttons, 0.12) == (True, 0.5, 0.0)


def shoulder_input(lb=False, rb=False, lt=False, rt=False):
    buttons = [0, 0, 0, 0, int(lb), int(rb)]
    axes = [0.0, 0.0, -1.0 if lt else 1.0, 0.0, 0.0,
            -1.0 if rt else 1.0, 0.0, 0.0]
    return axes, buttons


def test_lidar_chord_needs_release_before_first_toggle_and_between_toggles():
    chord = LidarChord()
    pressed = shoulder_input(True, True, True, True)
    assert not chord.update(*pressed, now=1.0, stamp=1.0)
    assert chord.block_drive
    assert not chord.update(*shoulder_input(), now=1.1, stamp=1.1)
    assert not chord.block_drive
    assert chord.update(*pressed, now=1.2, stamp=1.2)
    assert not chord.update(*pressed, now=1.3, stamp=1.3)
    assert not chord.update(*shoulder_input(rb=True), now=1.4, stamp=1.4)
    assert chord.block_drive
    assert not chord.update(*pressed, now=1.5, stamp=1.5)
    assert not chord.update(*shoulder_input(), now=1.6, stamp=1.6)
    assert chord.update(*pressed, now=1.7, stamp=1.7)


def test_lidar_chord_leaves_rb_only_drive_and_reserves_partial_chord_until_release():
    chord = LidarChord()
    chord.update(*shoulder_input(rb=True), now=1.0, stamp=1.0)
    assert not chord.block_drive
    chord.update(*shoulder_input(), now=1.1, stamp=1.1)
    axes, buttons = shoulder_input(rb=True)
    axes[2] = 0.0
    chord.update(axes, buttons, now=1.2, stamp=1.2)
    assert chord.block_drive
    chord.update(*shoulder_input(rb=True), now=1.3, stamp=1.3)
    assert chord.block_drive
    chord.update(*shoulder_input(), now=1.4, stamp=1.4)
    assert not chord.block_drive


def test_lidar_chord_neutral_trigger_initialization_cannot_arm():
    chord = LidarChord()
    assert not chord.update([0.0] * 8, [0] * 6, now=1.0, stamp=1.0)
    assert not chord.update(*shoulder_input(True, True, True, True),
                            now=1.1, stamp=1.1)


def test_lidar_chord_rejects_stale_future_gap_and_malformed_input():
    invalid = [([], [], 1.1, 1.1),
               (*shoulder_input(True, True, True, True), 2.0, 2.0),
               (*shoulder_input(True, True, True, True), 1.1, 0.0),
               (*shoulder_input(True, True, True, True), 1.1, 1.2),
               (*shoulder_input(True, True, True, True), float('nan'), 1.1)]
    for value in [float('nan'), float('inf'), -2.0, 2.0, 'bad', None]:
        axes, buttons = shoulder_input(True, True, True, True)
        axes[2] = value
        invalid.append((axes, buttons, 1.1, 1.1))
    for value in [2, -1, 'bad', None]:
        axes, buttons = shoulder_input(True, True, True, True)
        buttons[4] = value
        invalid.append((axes, buttons, 1.1, 1.1))
    for axes, buttons, now, stamp in invalid:
        chord = LidarChord()
        chord.update(*shoulder_input(), now=1.0, stamp=1.0)
        assert not chord.update(axes, buttons, now=now, stamp=stamp)
        assert chord.block_drive
        assert not chord.update(*shoulder_input(True, True, True, True),
                                now=2.1, stamp=2.1)


def test_lidar_chord_reset_forces_new_release_baseline():
    chord = LidarChord()
    chord.update(*shoulder_input(), now=1.0, stamp=1.0)
    chord.reset()
    assert not chord.update(*shoulder_input(True, True, True, True),
                            now=1.1, stamp=1.1)


def test_abandoned_chord_stays_drive_blocked_across_timeout_and_invalid_reports():
    for abandon in ('gap', 'invalid', 'reset'):
        chord = LidarChord()
        chord.update(*shoulder_input(), now=1.0, stamp=1.0)
        chord.update(*shoulder_input(lb=True, rb=True), now=1.1, stamp=1.1)
        if abandon == 'invalid':
            chord.update([], [], now=1.2, stamp=1.2)
        elif abandon == 'reset':
            chord.reset()
        assert not chord.update(*shoulder_input(rb=True), now=2.0, stamp=2.0)
        assert chord.block_drive
        chord.update(*shoulder_input(), now=2.1, stamp=2.1)
        assert not chord.block_drive


def test_reordered_pressed_report_cannot_toggle_after_a_newer_release():
    chord = LidarChord()
    chord.update(*shoulder_input(), now=1.1, stamp=1.1)
    assert not chord.update(*shoulder_input(True, True, True, True),
                            now=1.2, stamp=1.0)
    assert chord.block_drive
