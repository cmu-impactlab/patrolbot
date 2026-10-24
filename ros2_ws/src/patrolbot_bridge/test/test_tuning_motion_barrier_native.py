"""Real DDS timestamps at the core DRIVE boundary, with a recording fake SBC.

No bridge construction, device, network peer or robot command is involved. The
actual bridge callback and epoch hold/release implementation run on stand-ins;
messages and MessageInfo come from the real middleware in an isolated domain.
"""

import os
import threading
import time
from types import MethodType, SimpleNamespace as NS

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from patrolbot_bridge.bridge_node import PatrolBotBridge
from patrolbot_bridge.epoch_runtime import EpochRuntime

pytestmark = pytest.mark.skipif(
    os.environ.get('PATROLBOT_RUN_INTEGRATION') != '1'
    or int(os.environ.get('ROS_DOMAIN_ID', '0')) < 90,
    reason='requires isolated integration domain >=90')


def test_real_middleware_never_replays_a_command_queued_before_release():
    rclpy.init()
    receiver = Node('tuning_motion_barrier_receiver')
    sender = Node('tuning_motion_barrier_sender')
    executor = SingleThreadedExecutor()
    executor.add_node(receiver)
    executor.add_node(sender)
    writes, received = [], []
    epoch = NS(lock=threading.RLock(), tuning_hold=False, _tuning_hold_sources={},
               commands_allowed=True, undock_hold=False, guard=NS(ready=True),
               maps=NS(hold=False), _map_error='', _failed=False,
               _map_goal=None, _map_future=None, _future=None, _future_kind=None)
    core = NS(_epoch=epoch, _state_lock=threading.Lock(), sock_lock=threading.Lock(),
              _pose_lock=threading.Lock(), _pose_received=True, _pose_at=time.monotonic(),
              _odom_tf_timeout=.5, _manual_active_at=0., _web_teleop_active_at=0.,
              _undock_active=False, _undock_turn=NS(busy=False),
              _lidar_ready=lambda: True, get_clock=receiver.get_clock,
              get_logger=receiver.get_logger, sock=NS(sendall=writes.append))
    epoch.node = core
    core._manual_drive_allowed = MethodType(PatrolBotBridge._manual_drive_allowed, core)
    def callback(message, info):
        received.append(info['source_timestamp'] if isinstance(info, dict) else info.source_timestamp)
        PatrolBotBridge.cmd_vel_callback(core, message, info)
    receiver.create_subscription(Twist, '/tuning_fixture/cmd_vel', callback, 10)
    publisher = sender.create_publisher(Twist, '/tuning_fixture/cmd_vel', 10)
    def spin_until(predicate):
        deadline = time.monotonic() + 3.
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert predicate()
    def send():
        message = Twist()
        message.linear.x = .05
        publisher.publish(message)
    try:
        spin_until(lambda: publisher.get_subscription_count() == 1)
        EpochRuntime._set_tuning_hold_source(epoch, 'job', True, 'isolated fixture')
        send()
        spin_until(lambda: len(received) == 1)
        assert writes == []

        # Do not run the receiver executor until release. This is a genuine DDS
        # queued sample, not a fabricated old MessageInfo or timestamped Twist.
        send()
        EpochRuntime._set_tuning_hold_source(epoch, 'job', False)
        barrier = core._tuning_motion_admission_after_ns
        assert barrier > received[0]
        spin_until(lambda: len(received) == 2)
        assert received[-1] <= barrier and writes == []
        assert core._manual_active_at == core._web_teleop_active_at == 0.

        send()
        spin_until(lambda: len(received) == 3)
        assert received[-1] > barrier
        assert writes == [b'DRIVE:0.05:0.0\n']

        # New middleware evidence does not bypass normal raw-odom/manual gates.
        epoch.commands_allowed = False
        core._pose_received = False
        core._manual_active_at = time.monotonic()
        send()
        spin_until(lambda: len(received) == 4)
        assert len(writes) == 1
    finally:
        executor.shutdown()
        sender.destroy_node()
        receiver.destroy_node()
        rclpy.shutdown()
