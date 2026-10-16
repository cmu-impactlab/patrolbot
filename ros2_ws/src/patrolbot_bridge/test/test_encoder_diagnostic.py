import math

from patrolbot_bridge.encoder_diagnostic import EncoderDiagnostic, EncoderSample
import pytest


def sample(at, left=0, right=0, x=0.0, yaw=0.0, origin='boot-a'):
    return EncoderSample(at, origin, left, right, x, 0.0, yaw)


def test_counter_wrap_and_fused_heading_wrap_are_differences_not_jumps():
    diagnostic = EncoderDiagnostic(counter_modulus=65536, max_samples=3)
    assert diagnostic.observe(sample(1.0, 65534, 2, yaw=math.pi - 0.01)) is None
    delta = diagnostic.observe(sample(1.1, 3, 65534, x=0.02, yaw=-math.pi + 0.01))
    assert delta.left_counts == 5
    assert delta.right_counts == -4
    assert delta.sip_distance_m == pytest.approx(0.02)
    assert delta.sip_yaw_rad == pytest.approx(0.02)
    assert delta.elapsed_s == pytest.approx(0.1)


@pytest.mark.parametrize('bad', [sample(1.0), sample(0.9), sample(1.1, origin='boot-b')])
def test_wrong_origin_or_nonadvancing_time_never_reanchors(bad):
    diagnostic = EncoderDiagnostic(counter_modulus=65536, max_samples=3)
    diagnostic.observe(sample(1.0))
    with pytest.raises(ValueError):
        diagnostic.observe(bad)
    assert diagnostic.observe(sample(1.1, left=4)).left_counts == 4


def test_ambiguous_half_wrap_nonfinite_and_out_of_range_rejected():
    diagnostic = EncoderDiagnostic(counter_modulus=65536, max_samples=3)
    diagnostic.observe(sample(1.0))
    for bad in [sample(1.1, left=32768), sample(math.nan), sample(1.1, left=65536)]:
        with pytest.raises(ValueError):
            diagnostic.observe(bad)
    assert diagnostic.observe(sample(1.1, left=1)).left_counts == 1


def test_diagnostic_stops_at_fixed_budget():
    diagnostic = EncoderDiagnostic(counter_modulus=65536, max_samples=2)
    diagnostic.observe(sample(1.0))
    diagnostic.observe(sample(1.1))
    with pytest.raises(StopIteration):
        diagnostic.observe(sample(1.2))
