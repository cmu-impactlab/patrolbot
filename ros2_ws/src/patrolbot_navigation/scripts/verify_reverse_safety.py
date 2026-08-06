#!/usr/bin/env python3
"""Read-only runtime verification of PatrolBot's reverse safety chain.

Covers the two findings that only mean anything together:

* **PB-AUDIT-003** — the custom recovery tree must actually be the one the
  running ``bt_navigator`` loaded, so ``BackUp`` stays reachable regardless of
  how a navigation failure was classified.
* **PB-AUDIT-004** — the rear sonar must actually be consumed by the collision
  monitor and the dock manager, and the points it publishes must be
  geometrically credible against the transducers measured on this robot.

Both findings were originally *configuration that disagreed with runtime*, so
every value here is read back from the live node, never from the repository
file. The behaviour-tree XML is read from the path the running node reports.

**This tool never publishes, never calls a service, and never sends an action
goal.** It cannot move the robot. The ``recovery`` phase is passive too: it only
listens to /behavior_tree_log while an operator induces a navigation failure.

Usage inside the navigation container::

    ros2 run patrolbot_navigation verify_reverse_safety.py            # readonly
    ros2 run patrolbot_navigation verify_reverse_safety.py --phase recovery \
        --duration 120

Exit status is 0 when every check in the selected phases passed.
"""

import argparse
import json
import math
import struct
import sys
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from functools import partial
from nav2_msgs.msg import BehaviorTreeLog
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import (
    qos_profile_action_status_default,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2, PointField

sys.path.insert(0, str(Path(__file__).resolve().parent))
from reverse_safety_rules import (  # noqa: E402
    Check,
    all_ok,
    check_behavior_tree_log,
    check_collision_monitor_runtime,
    check_navigator_variant,
    check_recovery_tree,
    check_runtime_tree_selection,
    check_sonar_capture,
    check_sonar_cloud,
    check_sonar_stream,
    check_stale_guard,
    load_sonar_geometry,
    subscribers_are_settled,
)


BT_NAVIGATOR = '/bt_navigator'
COLLISION_MONITOR = '/collision_monitor'

EXPECTED_TREES = {
    'default_nav_to_pose_bt_xml': 'navigate_recovery_always_try.xml',
    'default_nav_through_poses_bt_xml':
        'navigate_through_poses_recovery_always_try.xml',
}

COLLISION_MONITOR_PARAMETERS = (
    'observation_sources',
    'base_shift_correction',
    'source_timeout',
    'sonar.source_timeout',
    'sonar.enabled',
    'sonar.type',
    'sonar.topic',
    'sonar.min_height',
    'sonar.max_height',
    'sonar.min_range',
    'sonar.use_global_height',
    'scan.source_timeout',
    'rear_velocity_stop.sources_names',
    'rear_velocity_stop.min_points',
    'rear_velocity_stop.max_points',
    'rear_velocity_stop.action_type',
)

SONAR_TYPE = 'sensor_msgs/msg/PointCloud2'

# The two consumers PB-AUDIT-004 exists to guarantee: the collision monitor
# stops reverse motion on a rear echo, the dock manager refuses to hand off
# when the sonar is stale or the rear is not clear.
REQUIRED_SONAR_SUBSCRIBERS = frozenset({
    'collision_monitor', 'patrolbot_dock_manager'})

# Action status streams. Both recovery trees use identical node names, so this
# is the only thing that says which navigator a recovery belonged to.
NAVIGATOR_STATUS_TOPICS = {
    'to_pose': '/navigate_to_pose/_action/status',
    'through_poses': '/navigate_through_poses/_action/status',
}

# Declared by Nav2 but not served while the deprecated `min_points` is what the
# configuration sets. Absent means "Nav2 derived it from min_points", not
# "misconfigured"; present it is still checked.
OPTIONAL_COLLISION_MONITOR_PARAMETERS = frozenset({
    'rear_velocity_stop.max_points',
})

# /sonar is a ~5 Hz stream. Below this the collision monitor's 0.5 s
# source_timeout is being ridden, and reverse is one missed frame from a
# fail-closed stop.
MINIMUM_SONAR_RATE_HZ = 4.0

# Used to measure the capture window when the live per-source timeout is
# unreadable or disabled. It never substitutes for the verdict:
# sonar.stale_guard_is_armed reports that separately.
DEFAULT_SOURCE_TIMEOUT_S = 0.5

GEOMETRY_CANDIDATES = (
    Path(__file__).resolve().parents[1] / 'config' / 'rear_sonar_geometry.yaml',
    Path('/opt/patrolbot_ws/install/patrolbot_navigation/share/'
         'patrolbot_navigation/config/rear_sonar_geometry.yaml'),
)


def find_geometry_file(explicit=None):
    candidates = (Path(explicit),) if explicit else GEOMETRY_CANDIDATES
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        'rear_sonar_geometry.yaml not found in {}'.format(
            [str(path) for path in candidates]))


def decode_cloud(message):
    """Unpack a PointCloud2 into plain (x, y, z) tuples.

    Deliberately strict, and identical in spirit to the dock manager's own
    decoder: a malformed cloud must raise rather than silently produce fewer
    points, because "no points" is what "rear is clear" looks like.
    """
    fields = {field.name: field for field in message.fields}
    for name in ('x', 'y', 'z'):
        if name not in fields:
            raise ValueError('sonar cloud missing field {!r}'.format(name))
        if fields[name].datatype != PointField.FLOAT32:
            raise ValueError('sonar {} must be FLOAT32'.format(name))
    endian = '>' if message.is_bigendian else '<'
    points = []
    for row in range(message.height):
        row_start = row * message.row_step
        for column in range(message.width):
            start = row_start + column * message.point_step
            values = tuple(
                struct.unpack_from(
                    endian + 'f', message.data, start + fields[name].offset)[0]
                for name in ('x', 'y', 'z'))
            if not all(math.isfinite(value) for value in values):
                raise ValueError('non-finite sonar point')
            points.append(values)
    return points


class ReverseSafetyVerifier(Node):
    def __init__(self):
        super().__init__('patrolbot_reverse_safety_verifier')
        self.sonar_messages = []
        self.behavior_events = []
        self.active_variants = set()
        self.create_subscription(
            PointCloud2, '/sonar', self._sonar, qos_profile_sensor_data)
        self.create_subscription(
            BehaviorTreeLog, '/behavior_tree_log', self._behavior_tree, 10)
        for variant, topic in NAVIGATOR_STATUS_TOPICS.items():
            self.create_subscription(
                GoalStatusArray, topic, partial(self._nav_status, variant),
                qos_profile_action_status_default)

    def _sonar(self, message):
        self.sonar_messages.append((time.monotonic(), message))

    def _behavior_tree(self, message):
        for event in message.event_log:
            self.behavior_events.append((event.node_name, event.current_status))

    def _nav_status(self, variant, message):
        for status in message.status_list:
            if status.status in (
                    GoalStatus.STATUS_ACCEPTED,
                    GoalStatus.STATUS_EXECUTING,
                    GoalStatus.STATUS_CANCELING):
                self.active_variants.add(variant)

    # Arrival timestamps are evidence here, so the executor must not be the
    # thing that delays them. At a 0.1 s idle timeout this loop produced a
    # single 0.513 s "gap" against a stream whose real p99 is 0.336 s and whose
    # maximum over 677 consecutive samples was 0.357 s.
    SPIN_POLL_S = 0.02

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=self.SPIN_POLL_S)

    def read_parameters(self, node_name, names, timeout_s=10.0):
        """Read live parameters. Read-only by construction: the client is only
        ever asked to get, never to set.

        Deliberately one request per name. ``GetParameters`` answers a batch
        containing any name the node cannot produce with an EMPTY value list,
        so a single unreadable name silently blanks every other value in the
        same request -- which reads exactly like nine real misconfigurations.
        `rear_velocity_stop.max_points` is such a name on this robot: Nav2
        lists it but does not serve it. Names that cannot be read are returned
        separately instead of being mapped to None.
        """
        client = AsyncParameterClient(self, node_name)
        if not client.wait_for_services(timeout_sec=timeout_s):
            raise RuntimeError(
                'parameter services for {} never appeared'.format(node_name))
        values = {}
        unreadable = []
        for name in names:
            future = client.get_parameters([name])
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
            if not future.done():
                raise RuntimeError(
                    'parameter read of {} from {} timed out'.format(
                        name, node_name))
            answered = future.result().values
            if len(answered) != 1:
                unreadable.append(name)
                continue
            values[name] = parameter_value_to_python(answered[0])
        return values, unreadable

    def sonar_subscriber_names(self, attempts=6, settle_s=1.0):
        """Who is subscribed to /sonar, with the right message type.

        Retried, because the graph cache lags for nodes composed into
        nav2_container and one cold query can miss `collision_monitor`
        entirely. Introspection errors are returned, never swallowed: a node
        that could not be queried is unknown, not absent, and reporting it as
        absent would be a false alarm exactly as bad as a false pass.
        """
        names = set()
        errors = {}
        for attempt in range(attempts):
            names = set()
            errors = {}
            for node_name, namespace in self.get_node_names_and_namespaces():
                full = (namespace.rstrip('/') + '/' + node_name).lstrip('/')
                try:
                    subscriptions = self.get_subscriber_names_and_types_by_node(
                        node_name, namespace)
                except Exception as error:  # noqa: BLE001 - reported, not hidden
                    errors[full] = str(error)
                    continue
                for topic, types in subscriptions:
                    if topic == '/sonar' and SONAR_TYPE in types:
                        names.add(full)
            if subscribers_are_settled(
                    names, errors, REQUIRED_SONAR_SUBSCRIBERS):
                break
            if attempt + 1 < attempts:
                self.spin_for(settle_s)
        return names, errors


def phase_wiring(node, args):
    checks = []

    bt_params, bt_unreadable = node.read_parameters(
        BT_NAVIGATOR, tuple(EXPECTED_TREES))
    checks.append(Check(
        'bt_navigator.parameters_readable',
        not bt_unreadable,
        'unreadable={}'.format(bt_unreadable)))
    checks.extend(check_runtime_tree_selection(bt_params, EXPECTED_TREES))

    # Read the XML the running node actually points at. Checking the repository
    # copy instead is exactly the mistake PB-AUDIT-003 was: the file on disk was
    # right and the loaded tree was not.
    for parameter, expected in EXPECTED_TREES.items():
        path = bt_params.get(parameter)
        if not path:
            checks.append(Check(
                'tree.readable[{}]'.format(expected), False,
                '{} is unset'.format(parameter)))
            continue
        try:
            xml_text = Path(path).read_text(encoding='utf-8')
        except OSError as error:
            checks.append(Check(
                'tree.readable[{}]'.format(expected), False, str(error)))
            continue
        checks.append(Check(
            'tree.readable[{}]'.format(expected), True, path))
        checks.extend(check_recovery_tree(xml_text, expected))

    monitor_params, monitor_unreadable = node.read_parameters(
        COLLISION_MONITOR, COLLISION_MONITOR_PARAMETERS)
    # Nav2 declares max_points but does not serve it while the deprecated
    # min_points is the one in the config, so it is expected to be unreadable
    # here. Anything else unreadable is a real finding.
    unexpected = [
        name for name in monitor_unreadable
        if name not in OPTIONAL_COLLISION_MONITOR_PARAMETERS
    ]
    checks.append(Check(
        'collision_monitor.parameters_readable',
        not unexpected,
        'unreadable={} tolerated={}'.format(monitor_unreadable, sorted(
            set(monitor_unreadable) & OPTIONAL_COLLISION_MONITOR_PARAMETERS))))
    subscribers, introspection_errors = node.sonar_subscriber_names()
    checks.append(Check(
        'graph.introspection_complete',
        not introspection_errors,
        'nodes_that_could_not_be_queried={}'.format(introspection_errors)))
    checks.extend(check_collision_monitor_runtime(
        monitor_params, subscribers))
    return checks


def phase_sonar(node, args):
    geometry_path = find_geometry_file(args.geometry)
    geometry = load_sonar_geometry(
        yaml.safe_load(geometry_path.read_text(encoding='utf-8')))

    # The height band and timeout come from the running monitor, because those
    # are the values that decide whether a published point is ever seen.
    filters, unreadable = node.read_parameters(
        COLLISION_MONITOR,
        ('sonar.min_height', 'sonar.max_height', 'sonar.source_timeout'))
    checks = [
        Check('sonar.geometry_source', True, str(geometry_path)),
        Check(
            'sonar.filter_parameters_readable',
            not unreadable,
            'unreadable={}'.format(unreadable)),
    ]
    height_band = None
    if filters.get('sonar.min_height') is not None \
            and filters.get('sonar.max_height') is not None:
        height_band = (
            filters['sonar.min_height'], filters['sonar.max_height'])

    # The PER-SOURCE timeout, never the global one, and never a falsy-value
    # fallback. See check_stale_guard for why both matter.
    stale_guard, source_timeout = check_stale_guard(
        filters.get('sonar.source_timeout'), DEFAULT_SOURCE_TIMEOUT_S)
    checks.append(stale_guard)

    started = time.monotonic()
    node.sonar_messages.clear()
    node.spin_for(args.duration)
    finished = time.monotonic()

    captured = list(node.sonar_messages)
    checks.extend(check_sonar_stream(
        [message.header.frame_id for _at, message in captured],
        [at for at, _message in captured],
        started, finished, MINIMUM_SONAR_RATE_HZ, source_timeout))

    total_points = 0
    matched_points = 0
    worst_offset = 0.0
    for _at, message in captured:
        try:
            points = decode_cloud(message)
        except (ValueError, struct.error) as error:
            checks.append(Check('sonar.cloud_decodes', False, str(error)))
            continue
        cloud_checks, verdicts = check_sonar_cloud(
            message.header.frame_id, points, geometry, height_band)
        total_points += len(verdicts)
        matched_points += sum(1 for verdict in verdicts if verdict.matched)
        worst_offset = max(
            [worst_offset]
            + [verdict.off_m for verdict in verdicts if verdict.matched])
        checks.extend(check for check in cloud_checks if not check.ok)

    checks.extend(check_sonar_capture(
        len(captured), total_points, matched_points, worst_offset, geometry))
    return checks


def phase_recovery(node, args):
    """Passive observation of an operator-induced navigation failure.

    Nothing here commands anything. Start this first, then have the operator
    send the goal that is expected to fail. PB-AUDIT-003's acceptance covers
    both navigator variants, so run it once per variant with --variant.
    """
    node.behavior_events.clear()
    node.active_variants.clear()
    node.spin_for(args.duration)
    checks = [Check(
        'behavior_tree.events_observed',
        bool(node.behavior_events),
        'events={}'.format(len(node.behavior_events)))]
    if node.behavior_events:
        checks.extend(check_behavior_tree_log(node.behavior_events))
    checks.extend(check_navigator_variant(args.variant, node.active_variants))
    return checks


PHASES = {
    'wiring': phase_wiring,
    'sonar': phase_sonar,
    'recovery': phase_recovery,
}

READ_ONLY_PHASES = ('wiring', 'sonar')


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        '--phase', action='append', choices=sorted(PHASES),
        help='Phase to run; repeatable. Default: wiring and sonar, which '
             'need no motion and no operator.')
    parser.add_argument(
        '--duration', type=float, default=10.0,
        help='Seconds to listen in the sonar and recovery phases.')
    parser.add_argument('--geometry', help='Override rear_sonar_geometry.yaml.')
    parser.add_argument(
        '--variant', choices=sorted(NAVIGATOR_STATUS_TOPICS),
        default='to_pose',
        help='Which navigator the recovery phase is expected to observe. Both '
             'trees use identical node names, so the run must say which one it '
             'is; the action status stream corroborates it.')
    parser.add_argument('--json', action='store_true', help='Emit JSON.')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    phases = args.phase or list(READ_ONLY_PHASES)

    rclpy.init()
    node = ReverseSafetyVerifier()
    results = {}
    try:
        # Let discovery settle before asking who subscribes to /sonar.
        node.spin_for(2.0)
        for phase in phases:
            results[phase] = PHASES[phase](node, args)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    every_check = [check for checks in results.values() for check in checks]
    if args.json:
        print(json.dumps({
            'phases': {
                name: [check.as_dict() for check in checks]
                for name, checks in results.items()
            },
            'ok': all_ok(every_check),
        }, indent=2))
    else:
        for phase, checks in results.items():
            print('[{}]'.format(phase))
            for check in checks:
                print('  {:4} {:52} {}'.format(
                    'PASS' if check.ok else 'FAIL', check.name, check.detail))
        print('\nOVERALL={}'.format(
            'pass' if all_ok(every_check) else 'FAIL'))
    return 0 if all_ok(every_check) else 1


if __name__ == '__main__':
    sys.exit(main())
