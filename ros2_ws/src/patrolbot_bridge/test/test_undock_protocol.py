"""
Offline tests for the SBC-owned UNDOCK wire contract.

These run without ROS, a socket or a robot. The frames below are byte-for-byte
what hardware/patrolbot_hw_server/protocol.cpp emits; tests/protocol_test.cpp
asserts the same strings from the C++ side, so the two ends cannot drift apart
without one of the two suites failing.
"""

from patrolbot_bridge.undock_protocol import (format_dock_reset, format_undock_cancel,
                                              format_undock_request, parse_hello,
                                              parse_undock_event, parse_undock_result,
                                              SENSOR_BLOCKED)
import pytest

REQUEST_ID = '123e4567-e89b-12d3-a456-426614174000'
EVENT = f'UNDOCK_EVENT:{REQUEST_ID}:4:BACKING_OUT:0.1234:0.0056:0.0102:3:1:' f'1.5000:2:DEPARTING'
RESULT = (
    f'UNDOCK_RESULT:{REQUEST_ID}:OK:0:undocked:6:HANDOFF_READY:0.5300:0.0010:'
    f'0.0020:3:1:0:99.0000:1:CLEAR_CONFIRMED:1'
)


def test_undock_request_is_one_parameterised_line():
    payload = format_undock_request(REQUEST_ID, 0.53, 0.10, 30.0, False)
    assert payload == (f'UNDOCK:{REQUEST_ID}:0.5300:0.1000:30.000:0\n').encode('ascii')
    assert b'DRIVE' not in payload


def test_validation_mode_is_explicit_on_the_wire():
    payload = format_undock_request(REQUEST_ID, 0.10, 0.05, 20.0, True)
    assert payload.endswith(b':1\n')


@pytest.mark.parametrize(
    'clearance,speed,timeout',
    [
        (0.0, 0.10, 30.0),
        (0.53, 0.0, 30.0),
        (0.53, 0.10, 0.0),
        (-0.1, 0.10, 30.0),
        (float('nan'), 0.10, 30.0),
        (float('inf'), 0.10, 30.0),
    ],
)
def test_nonsense_goals_never_reach_the_wire(clearance, speed, timeout):
    with pytest.raises(ValueError):
        format_undock_request(REQUEST_ID, clearance, speed, timeout, False)


def test_cancel_names_the_target_operation():
    # The payload is the id of the operation being cancelled, not a new request
    # id: the SBC dispatches it outside its request tracker for exactly that
    # reason.
    assert format_undock_cancel(REQUEST_ID) == (f'UNDOCK_CANCEL:{REQUEST_ID}\n').encode('ascii')
    with pytest.raises(ValueError):
        format_undock_cancel('not-a-uuid')


def test_dock_reset_is_a_bare_request_id():
    assert format_dock_reset(REQUEST_ID) == (f'DOCK_RESET:{REQUEST_ID}\n').encode('ascii')


def test_event_round_trip():
    event = parse_undock_event(EVENT)
    assert event.request_id == REQUEST_ID
    assert event.phase == 4
    assert event.phase_name == 'BACKING_OUT'
    assert event.distance_traveled == pytest.approx(0.1234)
    assert event.lateral_error == pytest.approx(0.0056)
    assert event.heading_error == pytest.approx(0.0102)
    assert event.charge_state == 3
    assert event.motors_enabled
    assert event.minimum_rear_range == pytest.approx(1.5)
    assert event.release_attempts == 2
    assert event.dock_state == 'DEPARTING'


def test_successful_result_round_trip():
    result = parse_undock_result(RESULT)
    assert result.success
    assert result.code == 0
    assert result.message == 'undocked'
    assert result.phase_name == 'HANDOFF_READY'
    assert result.distance_traveled == pytest.approx(0.53)
    # The defining contract of a successful handoff: stopped, motors live,
    # proven clear.
    assert result.motors_enabled
    assert result.physically_clear
    assert result.dock_state == 'CLEAR_CONFIRMED'


def test_failed_result_reports_disabled_motors_and_a_readable_message():
    line = (
        f'UNDOCK_RESULT:{REQUEST_ID}:ERROR:11:rear_path_blocked:7:STOPPING:'
        f'0.1200:0.0000:0.0000:0:0:0:0.1000:3:FAULT_STOPPED:0'
    )
    result = parse_undock_result(line)
    assert not result.success
    assert result.code == SENSOR_BLOCKED
    assert result.message == 'rear path blocked'
    assert not result.motors_enabled
    assert not result.physically_clear
    assert result.dock_state == 'FAULT_STOPPED'


@pytest.mark.parametrize(
    'line',
    [
        EVENT + ':extra',
        EVENT.replace(':BACKING_OUT:', ':FLYING:'),
        EVENT.replace(':DEPARTING', ':SOMEWHERE'),
        EVENT.replace(f':{REQUEST_ID}:', ':not-a-uuid:'),
        EVENT.replace(':0.1234:', ':nan:'),
        EVENT.replace(':3:1:', ':3:2:'),
    ],
)
def test_malformed_events_fail_closed(line):
    with pytest.raises(ValueError):
        parse_undock_event(line)


@pytest.mark.parametrize(
    'line',
    [
        RESULT + ':extra',
        RESULT.replace(':OK:', ':MAYBE:'),
        RESULT.replace(':HANDOFF_READY:', ':PARKED:'),
        RESULT.replace(':CLEAR_CONFIRMED:', ':CLEARISH:'),
        RESULT.replace(':0:undocked:', ':not-int:undocked:'),
    ],
)
def test_malformed_results_fail_closed(line):
    with pytest.raises(ValueError):
        parse_undock_result(line)


def test_hello_advertises_undock_support():
    hello = parse_hello('HELLO:PROTOCOL=2:FEATURES=UNDOCK')
    assert hello.protocol == 2
    assert hello.supports_undock()


def test_server_without_the_feature_is_refused_not_trusted():
    hello = parse_hello('HELLO:PROTOCOL=1:FEATURES=')
    assert not hello.supports_undock()


def test_other_lines_are_not_misclassified():
    telemetry = 'AUX:SONAR=|BATT=24.0,80,0,25|FLAGS=0,0,0,0,0'
    assert parse_undock_event(telemetry) is None
    assert parse_undock_result(telemetry) is None
    assert parse_hello(telemetry) is None
    assert parse_undock_event(RESULT) is None
    assert parse_undock_result(EVENT) is None


def test_epoch_identity_in_hello_is_explicit_and_optional():
    epoch = '123e4567-e89b-12d3-a456-426614174000'
    assert parse_hello('HELLO:PROTOCOL=2:FEATURES=UNDOCK').odom_epoch is None
    hello = parse_hello('HELLO:PROTOCOL=2:FEATURES=UNDOCK:ODOM_EPOCH=' + epoch)
    assert hello.odom_epoch == epoch


@pytest.mark.parametrize(
    'value',
    [
        '',
        '123',
        'none',
        '123E4567-e89b-12d3-a456-426614174000',
        '123e4567-e89b-12d3-a456-426614174000:ODOM_EPOCH=' '123e4567-e89b-12d3-a456-426614174000',
    ],
)
def test_malformed_or_duplicate_epoch_is_rejected(value):
    with pytest.raises(ValueError):
        parse_hello('HELLO:PROTOCOL=2:FEATURES=UNDOCK:ODOM_EPOCH=' + value)
