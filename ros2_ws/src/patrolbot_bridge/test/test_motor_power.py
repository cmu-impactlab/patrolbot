"""Motor power is independent of movement admission and preserves disable priority."""
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from patrolbot_bridge.bridge_node import PatrolBotBridge
from patrolbot_interfaces.srv import MotorDisable, MotorEnable

ID = "123e4567-e89b-12d3-a456-426614174000"

def bridge():
    b = PatrolBotBridge.__new__(PatrolBotBridge)
    for name in ("_state_lock", "_pose_lock", "_control_lock", "sock_lock", "_hardware_operation_lock"):
        setattr(b, name, threading.Lock())
    b._motor_disable_generation = 0
    b._pending_controls = {}
    b._session_generation = 1; b._link_connected = True
    b._charge_state = 0; b._motors_enabled = True; b._estop_pressed = True
    b._fault_flags = 4; b._undock_active = True; b._dock_state = "DOCKED_CONFIRMED"
    b._charge_state_at = b._flags_at = b._dock_state_at = b._pose_at = 0.
    b._vx = .2; b._vth = .1; b.CONTROL_TIMEOUT = .01
    b._epoch = NS(lock=threading.RLock(), commands_allowed=False, maps=NS(hold=True))
    b._lidar_ready = Mock(return_value=False)
    b.get_logger = Mock(return_value=Mock()); b.sock = Mock(); b.sent = []
    def send(payload):
        b.sent.append(payload)
        verb, request_id = payload.decode().strip().split(":")
        b._handle_control_reply(f"ACK:{verb}:{request_id}:ACCEPTED")
        motors, estop = (1, 0) if verb == "MOTOR_ENABLE" else (0, 1)
        b._handle_control_reply(f"RESULT:{verb}:{request_id}:OK:0:confirmed:0:{motors}:{estop}")
    b.sock.sendall.side_effect = send
    return b


def test_disable_ignores_motion_faults_estop_and_recovery():
    b = bridge()
    result = b._motor_disable_callback(MotorDisable.Request(request_id=ID), MotorDisable.Response())
    assert result.success and not result.motors_enabled
    assert b.sent == [f"MOTOR_DISABLE:{ID}\n".encode()]


@pytest.mark.parametrize("connected,request_id", [(False, ID), (True, "bad")])
def test_disable_still_requires_valid_identity_and_hardware_link(connected, request_id):
    b = bridge(); b._link_connected = connected
    result = b._motor_disable_callback(MotorDisable.Request(request_id=request_id), MotorDisable.Response())
    assert not result.success and not b.sent


@pytest.mark.parametrize("dock_state", ["DOCKED_CONFIRMED", "UNKNOWN", "DEPARTING", "FAULT_STOPPED"])
def test_enable_ignores_motion_dock_staleness_faults_lidar_and_recovery(dock_state):
    b = bridge()
    b._estop_pressed = False; b._motors_enabled = False; b._dock_state = dock_state
    result = b._motor_enable_callback(MotorEnable.Request(request_id=ID), MotorEnable.Response())
    assert result.success and result.motors_enabled
    assert b.sent == [f"MOTOR_ENABLE:{ID}\n".encode()]


def test_enable_preserves_estop():
    b = bridge()
    result = b._motor_enable_callback(MotorEnable.Request(request_id=ID), MotorEnable.Response())
    assert not result.success and "E-stop" in result.message and not b.sent


def test_disable_supersedes_an_enable_before_it_reaches_the_wire(monkeypatch):
    from patrolbot_bridge import bridge_node
    b = bridge(); b._estop_pressed = False
    original = bridge_node.format_request
    def format_request(command, request_id):
        if command == "MOTOR_ENABLE":
            disabled = b._motor_disable_callback(MotorDisable.Request(request_id=ID), MotorDisable.Response())
            assert disabled.success
        return original(command, request_id)
    monkeypatch.setattr(bridge_node, "format_request", format_request)
    result = b._motor_enable_callback(MotorEnable.Request(request_id=ID), MotorEnable.Response())
    assert not result.success and result.message == "canceled by motor disable"
    assert b.sent == [f"MOTOR_DISABLE:{ID}\n".encode()]
