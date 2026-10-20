#!/usr/bin/env python3
"""
Exercise the real cmd_vel gating against a recording fake SBC socket.

`cmd_vel_callback`, `_manual_active_cb`, `_web_teleop_active_cb` and
`_manual_drive_allowed` are compiled straight from the bridge source, so
this pins the exact DRIVE bytes
written while commands_allowed is False. The deployed generation once
admitted a manual command and then dropped it behind an extra unconditional
return; the exact bytes below are the regression evidence for that defect.
Every hard blocker that must still refuse nonzero DRIVE is covered here too.
"""

import ast
import math
from pathlib import Path
import threading
import time
from types import MethodType, SimpleNamespace as NS
from unittest.mock import Mock

from patrolbot_bridge.manual_active import manual_drive_bypass
from patrolbot_bridge.tuning_policy import clamp_drive
import pytest

PACKAGE = Path(__file__).parents[1] / 'patrolbot_bridge'
BRIDGE_SOURCE = PACKAGE / 'bridge_node.py'
UNDOCK_SOURCE = PACKAGE / 'undock_turn.py'


def _class_methods(path, class_name, names):
    tree = ast.parse(path.read_text())
    klass = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return [
        node
        for node in klass.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]


def _module_function(path, name):
    tree = ast.parse(path.read_text())
    return next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def _namespace():
    methods = _class_methods(
        BRIDGE_SOURCE,
        'PatrolBotBridge',
        {'_manual_active_cb', '_web_teleop_active_cb', '_manual_drive_allowed',
         'cmd_vel_callback'},
    )
    assert {node.name for node in methods} == {
        '_manual_active_cb',
        '_web_teleop_active_cb',
        '_manual_drive_allowed',
        'cmd_vel_callback',
    }
    namespace = {
        'time': time,
        'math': math,
        'manual_drive_bypass': manual_drive_bypass,
        'clamp_drive': clamp_drive,
    }
    exec(
        compile(
            ast.Module(
                body=methods + [_module_function(UNDOCK_SOURCE, 'pure_angular')],
                type_ignores=[],
            ),
            str(BRIDGE_SOURCE),
            'exec',
        ),
        namespace,
    )
    return namespace


_BRIDGE = _namespace()


class RecordingSocket:
    def __init__(self):
        self.writes = []

    def sendall(self, payload):
        self.writes.append(payload.decode('utf-8'))


def make_bridge(
    *,
    commands_allowed=True,
    manual_age=0.0,
    web_manual_age=None,
    undock_busy=False,
    drive_allowed=False,
    undock_active=False,
    undock_hold=False,
    lidar_ready=True,
    guard_ready=True,
    raw_odom_age=0.0,
    pose_received=True,
    map_error='',
    maps_present=True,
    maps_hold=False,
    maps_stage='ready',
    map_goal=False,
    map_future=False,
    future=None,
    failed=False,
    tuning_hold=False,
):
    """Stand-in node carrying exactly the fields the real callbacks read."""
    now = time.monotonic()
    epoch = NS(
        lock=threading.RLock(),
        commands_allowed=commands_allowed,
        undock_hold=undock_hold,
        tuning_hold=tuning_hold,
        guard=NS(ready=guard_ready),
        maps=NS(hold=maps_hold, stage=maps_stage) if maps_present else None,
        _map_error=map_error,
        _map_goal=object() if map_goal else None,
        _map_future=object() if map_future else None,
        _future=NS() if future else None,
        _future_kind=future,
        _failed=failed,
    )
    node = NS(
        _state_lock=threading.Lock(),
        _pose_lock=threading.Lock(),
        _undock_active=undock_active,
        _undock_turn=NS(busy=undock_busy, drive_allowed=drive_allowed),
        _epoch=epoch,
        _lidar_ready=Mock(return_value=lidar_ready),
        _manual_active_at=0.0 if manual_age is None else now - manual_age,
        _web_teleop_active_at=(
            0.0 if web_manual_age is None else now - web_manual_age),
        _pose_received=pose_received,
        _pose_at=now - raw_odom_age,
        _odom_tf_timeout=0.5,
        sock=RecordingSocket(),
        sock_lock=threading.Lock(),
    )
    node._manual_active_cb = MethodType(_BRIDGE['_manual_active_cb'], node)
    node._web_teleop_active_cb = MethodType(_BRIDGE['_web_teleop_active_cb'], node)
    node._manual_drive_allowed = MethodType(_BRIDGE['_manual_drive_allowed'], node)
    node.cmd_vel_callback = MethodType(_BRIDGE['cmd_vel_callback'], node)
    return node


def drive(bridge, linear=0.0, angular=0.0):
    bridge.cmd_vel_callback(
        NS(linear=NS(x=linear, y=0.0, z=0.0), angular=NS(x=0.0, y=0.0, z=angular))
    )
    return bridge.sock.writes


def test_admitted_manual_drive_is_written_while_commands_allowed_is_false():
    bridge = make_bridge(commands_allowed=False, manual_age=0.1)
    assert drive(bridge, linear=0.05) == ['DRIVE:0.05:0.0\n']


def test_admitted_manual_turn_is_written_while_commands_allowed_is_false():
    bridge = make_bridge(commands_allowed=False, manual_age=0.1)
    assert drive(bridge, angular=0.3) == ['DRIVE:0.0:0.3\n']


def test_commands_allowed_path_forwards_without_manual_evidence():
    bridge = make_bridge(commands_allowed=True, manual_age=None)
    assert drive(bridge, linear=0.05) == ['DRIVE:0.05:0.0\n']


@pytest.mark.parametrize('manual_age', [0.51, 60.0, None])
def test_stale_or_missing_deadman_drops_nonzero_drive_while_held(manual_age):
    bridge = make_bridge(commands_allowed=False, manual_age=manual_age)
    assert drive(bridge, linear=0.05) == []


def test_release_stop_is_forwarded_while_localization_is_held():
    bridge = make_bridge(commands_allowed=False, manual_age=None)
    assert drive(bridge) == ['DRIVE:0.0:0.0\n']


def test_undock_busy_blocks_manual_drive():
    bridge = make_bridge(
        commands_allowed=False, manual_age=0.0, undock_busy=True, drive_allowed=False
    )
    assert drive(bridge, linear=0.05) == []
    assert drive(bridge, angular=0.3) == []


def test_undock_turn_admits_only_drive_allowed_pure_angular():
    angular = make_bridge(
        commands_allowed=False, manual_age=None, undock_busy=True, drive_allowed=True
    )
    assert drive(angular, angular=0.3) == ['DRIVE:0.0:0.3\n']
    linear = make_bridge(
        commands_allowed=False, manual_age=None, undock_busy=True, drive_allowed=True
    )
    assert drive(linear, linear=0.05) == []


def test_settled_failed_map_still_admits_manual_drive():
    bridge = make_bridge(
        commands_allowed=False, manual_age=0.0, maps_hold=True, maps_stage='failed'
    )
    assert drive(bridge, linear=0.05) == ['DRIVE:0.05:0.0\n']


def test_read_only_readiness_future_does_not_block_manual_drive():
    bridge = make_bridge(commands_allowed=False, manual_age=0.0, future='read-only')
    assert drive(bridge, linear=0.05) == ['DRIVE:0.05:0.0\n']


@pytest.mark.parametrize(
    'overrides',
    [
        {'lidar_ready': False},
        {'guard_ready': False},
        {'raw_odom_age': 1.0},
        {'pose_received': False},
        {'undock_hold': True},
        {'map_error': 'map catalog unavailable'},
        {'maps_present': False},
        {'maps_hold': True, 'maps_stage': 'hold'},
        {'maps_hold': True, 'maps_stage': 'wait-localization'},
        {'map_goal': True},
        {'map_future': True},
        {'future': 'mutation'},
        {'failed': True},
        {'tuning_hold': True},
    ],
)
def test_in_flight_or_unknown_owners_still_drop_manual_drive(overrides):
    bridge = make_bridge(commands_allowed=False, manual_age=0.0, **overrides)
    assert drive(bridge, linear=0.05) == []


def test_deadman_callback_refreshes_only_on_true():
    bridge = make_bridge(commands_allowed=False, manual_age=1.0)
    assert bridge._manual_drive_allowed() is False
    bridge._manual_active_cb(NS(data=True))
    assert bridge._manual_drive_allowed() is True
    mark = bridge._manual_active_at
    bridge._manual_active_cb(NS(data=False))
    assert bridge._manual_active_at == mark


def test_dashboard_evidence_alone_admits_drive_while_localization_held():
    bridge = make_bridge(commands_allowed=False, manual_age=None,
                         web_manual_age=0.1)
    assert drive(bridge, linear=0.05) == ['DRIVE:0.05:0.0\n']


def test_active_policy_clamps_ordinary_drive_but_never_the_undock_turn():
    from patrolbot_bridge.tuning_policy import PolicyCache

    def probe():
        return 'gen-1', {'max_linear_m_s': 0.1, 'max_angular_rad_s': 0.2}

    bridge = make_bridge(commands_allowed=True, manual_age=None)
    bridge._tuning_policy = PolicyCache(probe)
    bridge._tuning_policy.refresh()
    # Manual/autonomy ordinary DRIVE is clamped down to the active policy.
    assert drive(bridge, linear=0.25) == ['DRIVE:0.1:0.0\n']
    bridge.sock.writes.clear()
    assert drive(bridge, angular=0.5) == ['DRIVE:0.0:0.2\n']
    # The undock turn keeps its exclusive profile, untouched by the policy.
    undock = make_bridge(commands_allowed=False, manual_age=None,
                         undock_busy=True, drive_allowed=True)
    undock._tuning_policy = bridge._tuning_policy
    assert drive(undock, angular=0.5) == ['DRIVE:0.0:0.5\n']


def test_unknown_policy_falls_back_to_the_hard_envelope_never_higher():
    from patrolbot_bridge.tuning_policy import PolicyCache

    def broken():
        raise RuntimeError('store unavailable')

    bridge = make_bridge(commands_allowed=True, manual_age=None)
    bridge._tuning_policy = PolicyCache(broken)
    bridge._tuning_policy.refresh()
    # Falls back to 0.30/0.70: still clamped, never raised.
    assert drive(bridge, linear=0.4) == ['DRIVE:0.3:0.0\n']
    bridge.sock.writes.clear()
    assert drive(bridge, angular=0.9) == ['DRIVE:0.0:0.7\n']


def test_f710_evidence_alone_admits_drive_while_localization_held():
    bridge = make_bridge(commands_allowed=False, manual_age=0.1,
                         web_manual_age=None)
    assert drive(bridge, linear=0.05) == ['DRIVE:0.05:0.0\n']


def test_freshest_evidence_of_either_channel_counts():
    # A stale channel ages out; the fresh one keeps the bypass open.
    stale_f710 = make_bridge(commands_allowed=False, manual_age=2.0,
                             web_manual_age=0.1)
    assert drive(stale_f710, linear=0.05) == ['DRIVE:0.05:0.0\n']
    stale_web = make_bridge(commands_allowed=False, manual_age=0.1,
                            web_manual_age=2.0)
    assert drive(stale_web, linear=0.05) == ['DRIVE:0.05:0.0\n']


def test_both_channels_stale_drop_drive_while_localization_held():
    bridge = make_bridge(commands_allowed=False, manual_age=1.0,
                         web_manual_age=1.0)
    assert drive(bridge, linear=0.05) == []


def test_web_teleop_callback_refreshes_only_on_true():
    bridge = make_bridge(commands_allowed=False, manual_age=None,
                         web_manual_age=1.0)
    assert bridge._manual_drive_allowed() is False
    bridge._web_teleop_active_cb(NS(data=True))
    assert bridge._manual_drive_allowed() is True
    mark = bridge._web_teleop_active_at
    bridge._web_teleop_active_cb(NS(data=False))
    assert bridge._web_teleop_active_at == mark
