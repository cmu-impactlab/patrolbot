#!/usr/bin/env python3
"""Opt-in Pi telemetry observer. No command publisher or actuator interface."""

import argparse
import json
import math
import time

from forward_speed_shadow import ShadowTrace


class ForwardSpeedObservation:
    """Keep telemetry, unattributed output and offline inference separate."""

    def __init__(self, *, short_zone_m, max_age_s):
        self._shadow = ShadowTrace(short_zone_m=short_zone_m, max_age_s=max_age_s)
        self._max_age_s = max_age_s
        self._session = None
        self._base_at = None
        self._odom = None
        self._output = None

    def base_state(self, source, generation, connected, telemetry_age, now):
        session = (source, generation) if (
            source and generation > 0 and connected and 0 <= telemetry_age < self._max_age_s
        ) else None
        if session != self._session or not self._fresh(self._base_at, now):
            self._odom = self._output = None
        self._session = session
        self._base_at = now if session is not None else None

    @staticmethod
    def _sample(value, now):
        return dict(value=value, received_at=now) if math.isfinite(value) else None

    def _fresh(self, received_at, now):
        return received_at is not None and 0 <= now - received_at < self._max_age_s

    def odometry(self, value, now):
        self._odom = self._sample(value, now)

    def downstream_command(self, value, now):
        self._output = self._sample(value, now)

    def report(self, now, *, base_source, single_odom_source, single_command_source):
        session = self._session
        if session and (session[0] != base_source or not self._fresh(self._base_at, now)):
            session = None
        if session is None:
            self._session = self._base_at = None
            self._odom = self._output = None
        measured = (self._odom if session and single_odom_source and self._odom
                    and self._fresh(self._odom['received_at'], now) else None)
        output = (self._output if session and single_command_source and self._output
                  and self._fresh(self._output['received_at'], now) else None)
        if not single_odom_source:
            self._odom = None
        if not single_command_source:
            self._output = None
        report = self._shadow.consume(dict(
            event='frame', at=now, command=None, measured=measured, clearance=None,
            route_cap=None, turn_cap=None, robot_cap=None,
        ))
        result = report.pop('would_apply')
        report.update(
            observed_hardware_generation=session[1] if session else None,
            downstream_command_sample=output,
            effective_cap_m_s=None,
            candidate_zone_m=result['required_zone_m'],
            reason=result['reason'],
        )
        return report


def _base_source(node):
    # Older Jazzy rclpy does not include publisher_gid in callback metadata.
    # This is a graph observation, not authentication or message attribution.
    publishers = node.get_publishers_info_by_topic('/patrolbot/base_state')
    return tuple(publishers[0].endpoint_gid) if len(publishers) == 1 else None


def attach(node, observe):
    """Attach best-effort subscriptions; sample age starts at observer receipt."""
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from patrolbot_interfaces.msg import BaseState
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    def base(message):
        observe.base_state(
            _base_source(node), message.session_generation,
            message.link_connected, message.telemetry_age, time.monotonic(),
        )

    def odom(message):
        observe.odometry(
            message.twist.twist.linear.x if message.child_frame_id == 'base_link' else math.nan,
            time.monotonic(),
        )

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                     reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(BaseState, '/patrolbot/base_state', base, qos)
    node.create_subscription(Odometry, '/odom', odom, qos)
    node.create_subscription(
        Twist, '/cmd_vel',
        lambda message: observe.downstream_command(message.linear.x, time.monotonic()), qos,
    )


def snapshot(node, observe, now):
    return observe.report(
        now,
        base_source=_base_source(node),
        single_odom_source=node.count_publishers('/odom') == 1,
        single_command_source=node.count_publishers('/cmd_vel') == 1,
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=20.0, help='seconds (1–120)')
    parser.add_argument('--short-zone-m', type=float, required=True, help='study input, not configuration')
    parser.add_argument('--max-age-s', type=float, required=True, help='study input, not configuration')
    args = parser.parse_args(argv)
    if not 1 <= args.duration <= 120:
        parser.error('--duration must be between 1 and 120 seconds')
    try:
        observe = ForwardSpeedObservation(short_zone_m=args.short_zone_m, max_age_s=args.max_age_s)
    except ValueError as error:
        parser.error(str(error))

    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node('patrolbot_forward_speed_observer', enable_rosout=False,
                                start_parameter_services=False, use_global_arguments=False)
        attach(node, observe)
        started = time.monotonic()
        next_report = started
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now >= next_report:
                report = snapshot(node, observe, now)
                print(json.dumps(report, allow_nan=False, separators=(',', ':')), flush=True)
                next_report = now + 0.2
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
