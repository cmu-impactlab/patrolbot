"""Motor requests use base velocity and preserve a disable during recovery."""
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
    b.get_logger = Mock(return_value=Mock()); b.sock = Mock(); b.sent = []
    def send(payload):
        b.sent.append(payload)
        verb, request_id = payload.decode().strip().split(":")
        b._handle_control_reply(f"ACK:{verb}:{request_id}:ACCEPTED")
        b._handle_control_reply(f"RESULT:{verb}:{request_id}:OK:0:confirmed:0:0:1")
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


def test_enable_refuses_real_motion_even_with_motors_disabled():
    b = bridge(); now = time.monotonic()
    b._charge_state_at = b._flags_at = b._dock_state_at = b._pose_at = now
    b._epoch.maps.hold = False
    b._motors_enabled = False; b._estop_pressed = False; b._fault_flags = 0; b._undock_active = False
    b._dock_state = "CLEAR_CONFIRMED"
    result = b._motor_enable_callback(MotorEnable.Request(request_id=ID), MotorEnable.Response())
    assert not result.success and "stationary" in result.message and not b.sent
