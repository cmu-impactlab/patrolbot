"""Raw hardware evidence does not depend on AMCL or permit identity guessing."""

from patrolbot_bridge.manual_telemetry import ManualTelemetry
import pytest

EPOCH = '11111111-2222-3333-4444-555555555555'


def frame(odom='0,0,0,0,0', scan='1,1,', age='0', epoch=EPOCH):
    return f'ODOM:{odom}|LASER:{scan}|LASER_AGE_MS:{age}|ODOM_EPOCH:{epoch}'


def test_valid_raw_evidence_has_bounded_age_and_exact_identity():
    raw = ManualTelemetry()
    raw.observe(frame(age='100'), 10., now=10.1, session=2, epoch=EPOCH)
    assert raw.ready(now=10.2, session=2, epoch=EPOCH)
    assert not raw.ready(now=10.2, session=3, epoch=EPOCH)
    assert not raw.ready(now=10.2, session=2, epoch='different')
    assert not raw.ready(now=10.6, session=2, epoch=EPOCH)
    assert not raw.ready(now=9., session=2, epoch=EPOCH)


@pytest.mark.parametrize('wire', [
    frame(odom='nan,0,0,0,0'), frame(odom='0,0,0,.31,0'),
    frame(odom='0,0,0,0,1.3'), frame(scan='nan,1,'), frame(scan='1,'),
    frame(age='501'), frame(age='-1'), frame(epoch='bad'),
    'ODOM:0,0,0,0,0|LASER:1,1,|ODOM_EPOCH:' + EPOCH])
def test_bad_observation_immediately_invalidates_previous_evidence(wire):
    raw = ManualTelemetry()
    raw.observe(frame(), 10., now=10., session=1, epoch=EPOCH)
    assert raw.ready(now=10., session=1, epoch=EPOCH)
    raw.observe(wire, 10.05, now=10.05, session=1, epoch=EPOCH)
    assert not raw.ready(now=10.05, session=1, epoch=EPOCH)


def test_continuity_survives_link_reconnect_but_explicit_new_origin_resets_it():
    raw = ManualTelemetry()
    raw.observe(frame(), 10., now=10., session=1, epoch=EPOCH)
    raw.observe(frame(odom='2,0,0,0,0'), 10.1, now=10.1, session=2, epoch=EPOCH)
    assert not raw.ready(now=10.1, session=2, epoch=EPOCH)
    new = 'aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee'
    raw.observe(frame(odom='2,0,0,0,0', epoch=new), 10.2, now=10.2, session=3, epoch=new)
    assert raw.ready(now=10.2, session=3, epoch=new)
