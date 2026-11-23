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


def test_manual_raw_evidence_keeps_original_age_and_requires_new_sip_after_reconnect():
    raw = ManualTelemetry()

    def observe(received, stamp, age, session=1):
        raw.observe(f'{frame()}|ODOM_SOURCE_MS:{stamp},{age}', received, now=received,
                    session=session, epoch=EPOCH, source_timing=True)

    observe(10., 1000, 10)
    assert raw.received == pytest.approx(9.99)
    observe(10.1, 1000, 110)
    assert raw.ready(now=10.1, session=1, epoch=EPOCH)
    assert raw.received == pytest.approx(9.99)
    assert raw.scan_at == pytest.approx(10.1)
    observe(10.2, 1000, 210, session=2)
    assert not raw.ready(now=10.2, session=2, epoch=EPOCH)
    observe(10.3, 1300, 10, session=2)
    assert raw.ready(now=10.3, session=2, epoch=EPOCH)
    assert raw.received == pytest.approx(10.29)
