#!/usr/bin/env python3
"""Bounded, read-only Nav2 command-path capture.

Subscribes to the complete velocity chain plus the safety/localization streams
needed to explain a short movement followed by a stop. It never publishes,
calls a service, or sends an action goal. Output is newline-delimited JSON so
events from different topics can be correlated by monotonic timestamp.
"""

import argparse
import json
import time
from collections import defaultdict
from functools import partial

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from patrolbot_interfaces.msg import BaseState
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_action_status_default,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan, PointCloud2


TWIST_TOPICS = (
    '/input/safety_controller',
    '/input/joy',
    '/input/navi',
    '/cmd_vel_out',
    '/cmd_vel',
)


class CommandPathCapture(Node):
    def __init__(self):
        super().__init__('patrolbot_nav_command_capture')
        self.started = time.monotonic()
        self.counts = defaultdict(int)
        self.last_seen = {}
        self.last_rate_emit = self.started
        self.last_odom_emit = 0.0

        for topic in TWIST_TOPICS:
            self.create_subscription(
                Twist, topic, partial(self._twist, topic), 10)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(
            LaserScan, '/scan', partial(self._freshness, '/scan'),
            qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/sonar', partial(self._freshness, '/sonar'),
            qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            partial(self._freshness, '/amcl_pose'), 10)
        self.create_subscription(
            BaseState, '/patrolbot/base_state', self._base_state, 10)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self._nav_status, qos_profile_action_status_default)

    def _elapsed(self):
        return round(time.monotonic() - self.started, 6)

    def _emit(self, event, **fields):
        print(json.dumps(
            {'elapsed_s': self._elapsed(), 'event': event, **fields},
            separators=(',', ':'), sort_keys=True), flush=True)

    def _mark(self, topic):
        self.counts[topic] += 1
        self.last_seen[topic] = time.monotonic()

    def _twist(self, topic, message):
        self._mark(topic)
        self._emit(
            'twist', topic=topic,
            linear_x=round(float(message.linear.x), 6),
            angular_z=round(float(message.angular.z), 6))

    def _odom(self, message):
        self._mark('/odom')
        now = time.monotonic()
        # Five samples per second is enough to prove stop distance without
        # flooding the capture at the bridge's 20-25 Hz telemetry rate.
        if now - self.last_odom_emit >= 0.2:
            self.last_odom_emit = now
            self._emit(
                'odom',
                linear_x=round(float(message.twist.twist.linear.x), 6),
                angular_z=round(float(message.twist.twist.angular.z), 6),
                x=round(float(message.pose.pose.position.x), 6),
                y=round(float(message.pose.pose.position.y), 6))

    def _freshness(self, topic, _message):
        self._mark(topic)

    def _base_state(self, message):
        self._mark('/patrolbot/base_state')
        self._emit(
            'base_state',
            session_generation=int(message.session_generation),
            link_connected=bool(message.link_connected),
            telemetry_age=round(float(message.telemetry_age), 6),
            hardware_state_valid=bool(message.hardware_state_valid),
            charge_state=int(message.charge_state),
            motors_enabled=bool(message.motors_enabled),
            estop_pressed=bool(message.estop_pressed),
            fault_flags=int(message.fault_flags))

    def _nav_status(self, message):
        self._mark('/navigate_to_pose/_action/status')
        statuses = []
        for entry in message.status_list:
            statuses.append({
                'goal_id': bytes(entry.goal_info.goal_id.uuid).hex(),
                'status': int(entry.status),
            })
        self._emit('nav_status', statuses=statuses)

    def emit_rates(self):
        now = time.monotonic()
        elapsed = now - self.last_rate_emit
        if elapsed < 1.0:
            return
        rates = {}
        ages = {}
        for topic in (
                '/scan', '/sonar', '/odom', '/amcl_pose',
                '/patrolbot/base_state'):
            rates[topic] = round(self.counts[topic] / elapsed, 3)
            last = self.last_seen.get(topic)
            ages[topic] = None if last is None else round(now - last, 3)
            self.counts[topic] = 0
        self.last_rate_emit = now
        self._emit('freshness', rates_hz=rates, ages_s=ages)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--duration', type=float, default=20.0,
        help='Capture duration in seconds (1-120).')
    arguments = parser.parse_args()
    if not 1.0 <= arguments.duration <= 120.0:
        parser.error('--duration must be between 1 and 120 seconds')
    return arguments


def main(args=None):
    arguments = parse_args()
    rclpy.init(args=args)
    node = CommandPathCapture()
    deadline = time.monotonic() + arguments.duration
    node._emit('capture_start', duration_s=arguments.duration)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.emit_rates()
    finally:
        node.emit_rates()
        node._emit('capture_end')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
