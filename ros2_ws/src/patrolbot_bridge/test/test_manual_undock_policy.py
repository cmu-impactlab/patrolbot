"""No ROS, socket, robot or runtime dependency for Start input acceptance."""

from patrolbot_bridge.manual_undock_policy import StartGesture
from patrolbot_bridge.undock_protocol import format_manual_undock_request, parse_hello
import pytest


def sample(gesture, t, start=False, busy=False, rb=False, linear=0., stamp=None):
    buttons = [0] * 11
    buttons[7], buttons[5] = int(start), int(rb)
    return gesture.update([0., linear, 1., 0., 0., 1., 0., 0.], buttons,
                          now=t, stamp=t if stamp is None else stamp, busy=busy)


def test_release_then_full_second_then_no_repeat_and_fresh_cancel():
    g = StartGesture()
    assert sample(g, 1.) is None
    for i in range(20):
        assert sample(g, 1.1 + i * .05, start=True) is None
    assert sample(g, 2.11, start=True) == 'start'
    assert sample(g, 2.16, start=True, busy=True) is None
    assert sample(g, 2.21, busy=True) is None
    assert sample(g, 2.26, start=True, busy=True) == 'cancel'
    assert sample(g, 2.31, start=True, busy=True) is None


@pytest.mark.parametrize('cause', [
    'startup-held', 'outage', 'rb', 'stick', 'old', 'future', 'duplicate'])
def test_invalid_or_unreleased_input_cannot_launch(cause):
    g = StartGesture()
    if cause != 'startup-held':
        sample(g, 1.)
    if cause == 'outage':
        sample(g, 1.1, start=True)
        sample(g, 2., start=True)
    elif cause == 'rb':
        sample(g, 1.1, start=True, rb=True)
    elif cause == 'stick':
        sample(g, 1.1, start=True, linear=.5)
    elif cause == 'old':
        sample(g, 1.1, start=True, stamp=.1)
    elif cause == 'future':
        sample(g, 1.1, start=True, stamp=2.)
    elif cause == 'duplicate':
        sample(g, 1.1, start=True)
        sample(g, 1.15, start=True, stamp=1.1)
    for i in range(30):
        assert sample(g, 3. + i * .05, start=True) is None


def test_disconnect_does_not_cancel_and_held_reconnect_is_not_a_new_press():
    g = StartGesture()
    sample(g, 1., busy=True)
    assert sample(g, 3., start=True, busy=True) is None
    sample(g, 3.1, busy=True)
    assert sample(g, 3.2, start=True, busy=True) == 'cancel'


def test_wire_variant_is_fixed_and_explicitly_capability_gated():
    identity = '11111111-2222-3333-4444-555555555555'
    assert format_manual_undock_request(identity) == (
        f'MANUAL_UNDOCK:{identity}:0.5300:0.1000:45.000:0\n').encode()
    assert 'MANUAL_UNDOCK_V1' not in parse_hello('HELLO:PROTOCOL=2:FEATURES=UNDOCK').features
    with pytest.raises(ValueError):
        format_manual_undock_request('bad\nDRIVE:1:0')
