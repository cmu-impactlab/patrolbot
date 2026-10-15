"""Offline tests for the reverse safety chain rules (PB-AUDIT-003 / -004).

No ROS, no robot. These pin the decision logic that
`verify_reverse_safety.py` applies to live data, including the failure modes
that must not be allowed to pass quietly: a stock recovery tree, a rear polygon
that needs more points than the robot has transducers, and sonar points that do
not come from the transducers that were actually measured.
"""

import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / 'scripts'))

from reverse_safety_rules import (  # noqa: E402
    RECOVERY_ORDER,
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
    classify_sonar_point,
    load_sonar_geometry,
    recovery_activation_sequence,
    subscribers_are_settled,
)

GEOMETRY_FILE = PACKAGE_ROOT / 'config' / 'rear_sonar_geometry.yaml'
TREE_DIR = PACKAGE_ROOT / 'config' / 'behavior_trees'

# Decoded from a live /sonar cloud on 2026-08-06 with the robot OFF the dock
# (dock_state CLEAR_CONFIRMED, charge_state 0): one point per live transducer
# (11, 12, 13). Over the 45 clouds captured in that window every point sat
# within 1.6e-06 m of its transducer's ray.
LIVE_SONAR_SAMPLE = (
    (-1.9214199781417847, -0.3384239971637726, 0.0),
    (-0.6076830029487610, 0.1067769974470138, 0.0),
    (-2.0428600311279297, 1.1794999837875366, 0.0),
)


def geometry():
    return load_sonar_geometry(
        yaml.safe_load(GEOMETRY_FILE.read_text(encoding='utf-8')))


def point_on_ray(unit, distance, lateral=0.0):
    """A point `distance` along a transducer's ray, offset sideways."""
    return (
        unit.x_m + distance * unit.dx - lateral * unit.dy,
        unit.y_m + distance * unit.dy + lateral * unit.dx,
        0.0,
    )


def live_collision_monitor_params(**overrides):
    """The values read back from the running node on 2026-08-06."""
    params = {
        'observation_sources': ['scan', 'sonar'],
        'base_shift_correction': False,
        'source_timeout': 0.5,
        'sonar.source_timeout': 0.5,
        'sonar.enabled': True,
        'scan.source_timeout': 0.5,
        'sonar.type': 'pointcloud',
        'sonar.topic': '/sonar',
        'sonar.min_height': -0.1,
        'sonar.max_height': 0.5,
        'sonar.min_range': 0.0,
        'sonar.use_global_height': False,
        'rear_velocity_stop.sources_names': ['sonar'],
        'rear_velocity_stop.min_points': 1,
        'rear_velocity_stop.max_points': 0,
        'rear_velocity_stop.action_type': 'stop',
    }
    params.update(overrides)
    return params


LIVE_SONAR_SUBSCRIBERS = {'collision_monitor', 'patrolbot_dock_manager'}


def failed(checks):
    return sorted(check.name for check in checks if not check.ok)


class SonarGeometryTest(unittest.TestCase):
    """PB-AUDIT-004: published points must come from the measured mounts."""

    def setUp(self):
        self.geometry = geometry()

    def test_geometry_matches_the_commissioned_live_units(self):
        self.assertEqual(self.geometry.indices, [11, 12, 13])
        self.assertEqual(self.geometry.frame_id, 'base_link')
        self.assertEqual(self.geometry.max_range_m, 4.335)

    def test_dead_units_are_not_listed(self):
        # 0-10, 14 and 15 never updated over a 300 s capture. Listing a dead
        # unit makes all_required_sensors_fresh permanently false on the SBC,
        # which refuses every undock.
        for index in list(range(0, 11)) + [14, 15]:
            self.assertNotIn(index, self.geometry.indices)

    def test_point_on_a_measured_ray_is_attributed_to_that_unit(self):
        for unit in self.geometry.units:
            verdict = classify_sonar_point(
                point_on_ray(unit, 1.5), self.geometry)
            self.assertEqual(verdict.reason, 'matched')
            self.assertEqual(verdict.index, unit.index)
            self.assertAlmostEqual(verdict.along_m, 1.5, places=6)
            self.assertLess(verdict.off_m, 1e-9)

    def test_point_off_every_ray_is_rejected(self):
        unit = self.geometry.units[0]
        verdict = classify_sonar_point(
            point_on_ray(unit, 1.5, lateral=0.25), self.geometry)
        self.assertEqual(verdict.reason, 'no_matching_ray')
        self.assertFalse(verdict.matched)

    def test_point_behind_a_transducer_is_rejected(self):
        unit = self.geometry.units[0]
        verdict = classify_sonar_point(
            point_on_ray(unit, -1.0), self.geometry)
        self.assertEqual(verdict.reason, 'behind_transducer')

    def test_point_beyond_the_range_cutoff_is_rejected(self):
        unit = self.geometry.units[0]
        verdict = classify_sonar_point(
            point_on_ray(unit, self.geometry.max_range_m + 0.5), self.geometry)
        self.assertEqual(verdict.reason, 'beyond_range_cutoff')

    def test_adjacent_units_are_not_confusable_at_working_range(self):
        # 11 and 12 sit 92 mm apart and point 20 degrees away from each other.
        # A point from one must never fall within tolerance of the other, or
        # the whole attribution is decoration.
        by_index = {unit.index: unit for unit in self.geometry.units}
        for index, other_index in ((11, 12), (12, 11), (12, 13), (13, 12)):
            point = point_on_ray(by_index[index], 1.0)
            _along, off = by_index[other_index].project(point[0], point[1])
            self.assertGreater(off, 10 * self.geometry.ray_tolerance_m)

    def test_live_sample_is_fully_attributed(self):
        checks, verdicts = check_sonar_cloud(
            'base_link', LIVE_SONAR_SAMPLE, self.geometry)
        self.assertTrue(all_ok(checks), failed(checks))
        self.assertEqual(sorted(v.index for v in verdicts), [11, 12, 13])
        for verdict in verdicts:
            self.assertEqual(verdict.reason, 'matched')
            self.assertLess(verdict.off_m, 1e-4)

    def test_forward_point_fails_rear_only(self):
        checks, _verdicts = check_sonar_cloud(
            'base_link', [(1.2, 0.0, 0.0)], self.geometry)
        self.assertIn('sonar.is_rear_only', failed(checks))

    def test_wrong_frame_fails(self):
        checks, _verdicts = check_sonar_cloud(
            'laser_frame', LIVE_SONAR_SAMPLE, self.geometry)
        self.assertIn('sonar.frame_id', failed(checks))

    def test_empty_cloud_is_well_formed(self):
        # An empty cloud is "no echo closer than the cutoff", which is normal
        # and must not be reported as a geometry fault. Staleness is judged by
        # arrival age, never by point count.
        checks, verdicts = check_sonar_cloud('base_link', [], self.geometry)
        self.assertTrue(all_ok(checks), failed(checks))
        self.assertEqual(verdicts, [])

    def test_a_window_of_only_empty_clouds_proves_nothing(self):
        # 0 attributed of 0 published is not a passing geometry validation: it
        # is also exactly what a dead sonar array looks like, because every
        # unit's 5000 mm no-echo value is filtered out before publication.
        checks = check_sonar_capture(
            clouds=42, total_points=0, matched_points=0,
            worst_offset_m=0.0, geometry=self.geometry)
        self.assertIn('sonar.echoes_observed', failed(checks))

    def test_a_window_with_echoes_is_judged_on_attribution(self):
        checks = check_sonar_capture(
            clouds=42, total_points=126, matched_points=126,
            worst_offset_m=2e-06, geometry=self.geometry)
        self.assertTrue(all_ok(checks), failed(checks))
        checks = check_sonar_capture(
            clouds=42, total_points=126, matched_points=125,
            worst_offset_m=2e-06, geometry=self.geometry)
        self.assertIn('sonar.points_all_attributed', failed(checks))

    def test_point_outside_the_monitors_height_band_fails(self):
        # Geometrically perfect and completely invisible: the collision monitor
        # discards it before any polygon is evaluated.
        unit = self.geometry.units[0]
        x, y, _z = point_on_ray(unit, 1.5)
        checks, _verdicts = check_sonar_cloud(
            'base_link', [(x, y, 0.0)], self.geometry, height_band=(0.1, 0.5))
        self.assertIn('sonar.points_survive_the_height_filter', failed(checks))
        checks, _verdicts = check_sonar_cloud(
            'base_link', [(x, y, 0.0)], self.geometry, height_band=(-0.1, 0.5))
        self.assertTrue(all_ok(checks), failed(checks))

    def test_point_exactly_at_the_cutoff_is_rejected(self):
        # The SBC drops `range >= 4335.0`, so the cutoff itself is unreachable.
        unit = self.geometry.units[0]
        verdict = classify_sonar_point(
            point_on_ray(unit, self.geometry.max_range_m), self.geometry)
        self.assertEqual(verdict.reason, 'beyond_range_cutoff')

    @staticmethod
    def even_arrivals(count, period=0.22, start=0.05):
        return [start + index * period for index in range(count)]

    def stream(self, arrivals, window=10.0, frames=None, timeout=0.5):
        return check_sonar_stream(
            frames if frames is not None else ['base_link'] * len(arrivals),
            arrivals, 0.0, window, 4.0, timeout)

    def test_stream_rate_below_the_floor_fails(self):
        checks = self.stream(self.even_arrivals(10, 1.0))
        self.assertIn('sonar.rate', failed(checks))
        checks = self.stream(self.even_arrivals(45))
        self.assertTrue(all_ok(checks), failed(checks))

    def test_single_long_gap_fails_even_when_the_average_rate_is_fine(self):
        # The case an average hides: the stream met 4 Hz overall but went
        # silent for longer than the monitor's source_timeout.
        arrivals = self.even_arrivals(40) + [0.05 + 40 * 0.22 + 0.9]
        arrivals += [arrivals[-1] + 0.05 * index for index in range(1, 30)]
        checks = self.stream(arrivals, window=arrivals[-1] + 0.05)
        self.assertIn('sonar.no_gap_reaches_source_timeout', failed(checks))
        self.assertNotIn('sonar.rate', failed(checks))

    def test_silence_at_the_end_of_the_window_fails(self):
        # 40 healthy arrivals then nothing for the rest of the capture: every
        # internal gap is fine and the average rate is fine, but the source was
        # stale when the window closed.
        arrivals = self.even_arrivals(40)
        checks = self.stream(arrivals, window=arrivals[-1] + 1.42)
        self.assertIn('sonar.no_gap_reaches_source_timeout', failed(checks))

    def test_silence_at_the_start_of_the_window_fails(self):
        arrivals = self.even_arrivals(45, start=1.2)
        checks = self.stream(arrivals, window=arrivals[-1] + 0.05)
        self.assertIn('sonar.no_gap_reaches_source_timeout', failed(checks))

    def test_a_completely_silent_window_fails(self):
        checks = self.stream([], window=10.0)
        self.assertIn('sonar.no_gap_reaches_source_timeout', failed(checks))
        self.assertIn('sonar.rate', failed(checks))

    def test_stream_frame_flapping_fails(self):
        arrivals = self.even_arrivals(45)
        checks = self.stream(
            arrivals, window=arrivals[-1] + 0.05,
            frames=['base_link'] * 44 + ['odom'])
        self.assertIn('sonar.frame_is_stable', failed(checks))


class CollisionMonitorRuntimeTest(unittest.TestCase):
    """PB-AUDIT-004: the monitor must consume sonar and fail closed."""

    def test_live_configuration_passes(self):
        checks = check_collision_monitor_runtime(
            live_collision_monitor_params(), LIVE_SONAR_SUBSCRIBERS)
        self.assertTrue(all_ok(checks), failed(checks))

    def test_sonar_dropped_from_observation_sources_fails(self):
        checks = check_collision_monitor_runtime(
            live_collision_monitor_params(observation_sources=['scan']),
            LIVE_SONAR_SUBSCRIBERS)
        self.assertIn(
            'collision_monitor.observation_sources', failed(checks))

    def test_rear_polygon_needing_more_points_than_transducers_fails(self):
        # The named landmine: matching the scan polygon's threshold would need
        # eight simultaneous echoes from three transducers, i.e. never.
        checks = check_collision_monitor_runtime(
            live_collision_monitor_params(
                **{'rear_velocity_stop.min_points': 8,
                   'rear_velocity_stop.max_points': 7}),
            LIVE_SONAR_SUBSCRIBERS)
        self.assertIn(
            'collision_monitor.rear_single_point_stops', failed(checks))
        self.assertIn(
            'collision_monitor.rear_max_points_agrees', failed(checks))

    def test_zero_source_timeout_fails(self):
        # Zero means "never time out" in Nav2, which turns a dead sonar into
        # permanent permission to reverse.
        checks = check_collision_monitor_runtime(
            live_collision_monitor_params(**{'sonar.source_timeout': 0.0}),
            LIVE_SONAR_SUBSCRIBERS)
        self.assertIn(
            'collision_monitor.sonar.source_timeout', failed(checks))

    def test_disabled_sonar_source_fails(self):
        checks = check_collision_monitor_runtime(
            live_collision_monitor_params(**{'sonar.enabled': False}),
            LIVE_SONAR_SUBSCRIBERS)
        self.assertIn(
            'collision_monitor.sonar_source_enabled', failed(checks))

    def test_base_shift_correction_true_fails(self):
        # True SIGABRTs the whole composed nav2_container after a reconnect.
        checks = check_collision_monitor_runtime(
            live_collision_monitor_params(base_shift_correction=True),
            LIVE_SONAR_SUBSCRIBERS)
        self.assertIn(
            'collision_monitor.base_shift_correction_stays_false',
            failed(checks))

    def test_rear_polygon_fed_by_the_laser_fails(self):
        checks = check_collision_monitor_runtime(
            live_collision_monitor_params(
                **{'rear_velocity_stop.sources_names': ['scan']}),
            LIVE_SONAR_SUBSCRIBERS)
        self.assertIn(
            'collision_monitor.rear_polygon_is_sonar_fed', failed(checks))

    def test_unconsumed_sonar_fails(self):
        # The original finding: /sonar published with zero subscribers.
        checks = check_collision_monitor_runtime(
            live_collision_monitor_params(), set())
        self.assertIn('sonar.is_consumed', failed(checks))

    def test_dock_manager_alone_is_not_enough(self):
        checks = check_collision_monitor_runtime(
            live_collision_monitor_params(), {'patrolbot_dock_manager'})
        self.assertIn('sonar.is_consumed', failed(checks))

    def test_height_band_that_discards_the_published_cloud_fails(self):
        # The whole rear stop silently stops working: every published point has
        # z == 0.0, so a band starting above it filters all of them out.
        checks = check_collision_monitor_runtime(
            live_collision_monitor_params(**{'sonar.min_height': 0.1}),
            LIVE_SONAR_SUBSCRIBERS)
        self.assertIn(
            'collision_monitor.sonar_height_band_admits_the_cloud',
            failed(checks))

    def test_nonzero_min_range_fails(self):
        # Deleting near-field returns is the PB-AUDIT-018 failure mode. On the
        # rear source it removes exactly the echoes that matter while reversing.
        checks = check_collision_monitor_runtime(
            live_collision_monitor_params(**{'sonar.min_range': 0.25}),
            LIVE_SONAR_SUBSCRIBERS)
        self.assertIn(
            'collision_monitor.sonar_keeps_near_field', failed(checks))

    def test_wrong_source_topic_or_type_fails(self):
        checks = check_collision_monitor_runtime(
            live_collision_monitor_params(**{'sonar.topic': '/scan'}),
            LIVE_SONAR_SUBSCRIBERS)
        self.assertIn('collision_monitor.sonar_source_topic', failed(checks))
        checks = check_collision_monitor_runtime(
            live_collision_monitor_params(**{'sonar.type': 'scan'}),
            LIVE_SONAR_SUBSCRIBERS)
        self.assertIn('collision_monitor.sonar_source_type', failed(checks))

    def test_use_global_height_true_fails(self):
        # Nav2 would then demand a `height` field the bridge never publishes,
        # invalidating the source while every other wiring check looks healthy.
        checks = check_collision_monitor_runtime(
            live_collision_monitor_params(
                **{'sonar.use_global_height': True}),
            LIVE_SONAR_SUBSCRIBERS)
        self.assertIn(
            'collision_monitor.sonar_uses_the_published_fields',
            failed(checks))

    def test_undeclared_parameter_fails_rather_than_skips(self):
        params = live_collision_monitor_params()
        params['rear_velocity_stop.min_points'] = None
        checks = check_collision_monitor_runtime(
            params, LIVE_SONAR_SUBSCRIBERS)
        self.assertIn(
            'collision_monitor.rear_single_point_stops', failed(checks))


class RecoveryTreeTest(unittest.TestCase):
    """PB-AUDIT-003: BackUp must stay reachable, and reachable first."""

    def deployed_trees(self):
        return sorted(TREE_DIR.glob('*.xml'))

    def test_both_deployed_trees_pass(self):
        trees = self.deployed_trees()
        self.assertEqual(len(trees), 2, [path.name for path in trees])
        for path in trees:
            checks = check_recovery_tree(
                path.read_text(encoding='utf-8'), path.name)
            self.assertTrue(all_ok(checks), (path.name, failed(checks)))

    def test_recovery_order_is_backup_clearing_spin_wait(self):
        self.assertEqual(
            RECOVERY_ORDER, ('BackUp', 'ClearingActions', 'Spin', 'Wait'))

    def test_spin_before_backup_fails(self):
        path = TREE_DIR / 'navigate_recovery_always_try.xml'
        root = ET.fromstring(path.read_text(encoding='utf-8'))
        round_robin = root.find('.//RoundRobin')
        backup = round_robin.find('BackUp')
        spin = round_robin.find('Spin')
        round_robin.remove(backup)
        round_robin.insert(list(round_robin).index(spin) + 1, backup)
        checks = check_recovery_tree(
            ET.tostring(root, encoding='unicode'), 'mutated')
        self.assertIn('tree.recovery_order', failed(checks))

    def test_restoring_the_stock_error_code_gate_fails(self):
        path = TREE_DIR / 'navigate_recovery_always_try.xml'
        text = path.read_text(encoding='utf-8')
        stock = text.replace(
            '<AlwaysSuccess/>',
            '<Fallback>'
            '<WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>'
            '<WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>'
            '</Fallback>')
        checks = check_recovery_tree(stock, 'stock-gate')
        self.assertIn('tree.recovery_gate_is_unconditional', failed(checks))

    def test_backup_distance_and_speed_are_pinned(self):
        path = TREE_DIR / 'navigate_recovery_always_try.xml'
        text = path.read_text(encoding='utf-8')
        checks = check_recovery_tree(
            text.replace('backup_speed="0.05"', 'backup_speed="0.20"'),
            'fast-backup')
        self.assertIn('tree.backup_speed', failed(checks))
        checks = check_recovery_tree(
            text.replace('backup_dist="0.10"', 'backup_dist="0.50"'),
            'long-backup')
        self.assertIn('tree.backup_distance', failed(checks))

    def test_reduced_retries_fail(self):
        path = TREE_DIR / 'navigate_recovery_always_try.xml'
        text = path.read_text(encoding='utf-8')
        checks = check_recovery_tree(
            text.replace('number_of_retries="9"', 'number_of_retries="6"'),
            'stock-retries')
        self.assertIn('tree.recovery_retries', failed(checks))

    def test_tree_without_a_round_robin_fails_loudly(self):
        # No RecoveryNode at all: rejected at the anchor, before anything is
        # looked for inside it.
        checks = check_recovery_tree(
            '<root BTCPP_format="4" main_tree_to_execute="MainTree">'
            '<BehaviorTree ID="MainTree">'
            '<AlwaysSuccess/></BehaviorTree></root>', 'empty')
        self.assertIn(
            'tree.executed_root_is_the_recovery_node', failed(checks))

    def test_the_right_root_with_no_round_robin_fails_on_the_round_robin(self):
        checks = check_recovery_tree(
            '<root BTCPP_format="4" main_tree_to_execute="MainTree">'
            '<BehaviorTree ID="MainTree">'
            '<RecoveryNode number_of_retries="9" name="NavigateRecovery">'
            '<PipelineSequence name="NavigateWithReplanning"/>'
            '<Sequence><AlwaysSuccess/>'
            '<ReactiveFallback name="RecoveryFallback"><GoalUpdated/>'
            '</ReactiveFallback></Sequence>'
            '</RecoveryNode></BehaviorTree></root>', 'no-round-robin')
        self.assertNotIn(
            'tree.executed_root_is_the_recovery_node', failed(checks))
        self.assertIn('tree.round_robin_present', failed(checks))

    def test_a_decoy_round_robin_outside_the_recovery_fallback_fails(self):
        # A correct-looking round-robin somewhere harmless must not satisfy the
        # check while the branch that actually runs after a navigation failure
        # has none. This is what anchoring to RecoveryFallback buys.
        decoy = (
            '<root BTCPP_format="4" main_tree_to_execute="MainTree">'
            '<BehaviorTree ID="MainTree">'
            '<RecoveryNode number_of_retries="9" name="NavigateRecovery">'
            '<PipelineSequence name="NavigateWithReplanning">'
            '<RoundRobin name="RecoveryActions">'
            '<Sequence name="ClearingActions"/>'
            '<BackUp backup_dist="0.10" backup_speed="0.05"/>'
            '<Spin spin_dist="1.57"/><Wait wait_duration="3.0"/>'
            '</RoundRobin>'
            '</PipelineSequence>'
            '<Sequence><AlwaysSuccess/>'
            '<ReactiveFallback name="RecoveryFallback"><GoalUpdated/>'
            '</ReactiveFallback></Sequence>'
            '</RecoveryNode></BehaviorTree></root>')
        checks = check_recovery_tree(decoy, 'decoy')
        self.assertIn('tree.round_robin_present', failed(checks))

    def test_a_duplicate_named_decoy_fallback_does_not_satisfy_the_check(self):
        # Harder version of the same trick: the decoy carries the SAME name as
        # the real recovery fallback and sits earlier in document order, while
        # the branch that actually runs after a navigation failure has had its
        # round-robin removed. Anchoring by name alone passes this tree.
        real = (TREE_DIR / 'navigate_recovery_always_try.xml').read_text(
            encoding='utf-8')
        root = ET.fromstring(real)
        outer = root.find('.//RecoveryNode[@name="NavigateRecovery"]')
        recovery_branch = list(outer)[1]
        genuine_fallback = recovery_branch.find(
            'ReactiveFallback[@name="RecoveryFallback"]')

        # Move the intact fallback out to a harmless place ahead of the real
        # branch, and leave the real branch with an empty one.
        recovery_branch.remove(genuine_fallback)
        decoy_host = outer.find('PipelineSequence')
        decoy_host.insert(0, genuine_fallback)
        recovery_branch.append(
            ET.Element('ReactiveFallback', {'name': 'RecoveryFallback'}))

        checks = check_recovery_tree(
            ET.tostring(root, encoding='unicode'), 'duplicate-name-decoy')
        self.assertIn('tree.round_robin_present', failed(checks))

    def test_a_duplicate_outer_recovery_node_does_not_satisfy_the_check(self):
        # Hardest version: an INTACT copy of the whole NavigateRecovery node is
        # planted in a branch that never ticks, ahead of the real one, and the
        # real one is broken. Anchoring with `.//` finds the decoy first and
        # passes every check. Only resolving main_tree_to_execute -> the single
        # entry node catches this.
        real = (TREE_DIR / 'navigate_recovery_always_try.xml').read_text(
            encoding='utf-8')
        root = ET.fromstring(real)
        tree = root.find('BehaviorTree[@ID="MainTree"]')
        genuine = list(tree)[0]

        decoy = ET.fromstring(ET.tostring(genuine, encoding='unicode'))
        broken = ET.fromstring(ET.tostring(genuine, encoding='unicode'))
        broken_branch = list(broken)[1]
        broken_branch.remove(
            broken_branch.find('ReactiveFallback[@name="RecoveryFallback"]'))

        tree.remove(genuine)
        wrapper = ET.SubElement(tree, 'Fallback')
        wrapper.append(decoy)
        wrapper.append(broken)

        checks = check_recovery_tree(
            ET.tostring(root, encoding='unicode'), 'duplicate-outer-decoy')
        self.assertIn(
            'tree.executed_root_is_the_recovery_node', failed(checks))

    def test_deployed_trees_have_exactly_one_of_each_anchor(self):
        for path in self.deployed_trees():
            root = ET.parse(path).getroot()
            for name in ('NavigateRecovery', 'RecoveryFallback'):
                matches = [
                    element for element in root.iter()
                    if element.attrib.get('name') == name
                ]
                self.assertEqual(len(matches), 1, (path.name, name))


class StaleGuardTest(unittest.TestCase):
    """The per-source staleness timeout, and the fallback that must never
    become the verdict."""

    def test_armed_timeout_passes_and_is_used_for_measurement(self):
        check, timeout = check_stale_guard(0.5, 0.5)
        self.assertTrue(check.ok)
        self.assertEqual(timeout, 0.5)

    def test_zero_timeout_fails_and_never_borrows_the_default(self):
        # 0.0 means "never time out" in Nav2: a dead sonar becomes permanent
        # permission to reverse. The fallback keeps the window measurable but
        # must not make this pass.
        check, timeout = check_stale_guard(0.0, 0.5)
        self.assertFalse(check.ok)
        self.assertEqual(timeout, 0.5)

    def test_unreadable_timeout_fails(self):
        check, timeout = check_stale_guard(None, 0.5)
        self.assertFalse(check.ok)
        self.assertEqual(timeout, 0.5)

    def test_a_shorter_live_timeout_is_what_gaps_are_judged_against(self):
        _check, timeout = check_stale_guard(0.25, 0.5)
        self.assertEqual(timeout, 0.25)


class SubscriberSettlingTest(unittest.TestCase):
    """A node that could not be queried is unknown, not absent."""

    REQUIRED = frozenset({'collision_monitor', 'patrolbot_dock_manager'})

    def test_all_present_and_no_errors_settles(self):
        self.assertTrue(subscribers_are_settled(
            self.REQUIRED, {}, self.REQUIRED))

    def test_present_but_with_an_introspection_error_keeps_retrying(self):
        self.assertFalse(subscribers_are_settled(
            self.REQUIRED, {'bt_navigator': 'node vanished'}, self.REQUIRED))

    def test_missing_subscriber_keeps_retrying(self):
        self.assertFalse(subscribers_are_settled(
            {'collision_monitor'}, {}, self.REQUIRED))

    def test_extra_subscribers_do_not_prevent_settling(self):
        self.assertTrue(subscribers_are_settled(
            set(self.REQUIRED) | {'patrolbot_reverse_safety_verifier'},
            {}, self.REQUIRED))


class RuntimeTreeSelectionTest(unittest.TestCase):
    """The original PB-AUDIT-003 defect: runtime pointed at the stock tree."""

    EXPECTED = {
        'default_nav_to_pose_bt_xml': 'navigate_recovery_always_try.xml',
        'default_nav_through_poses_bt_xml':
            'navigate_through_poses_recovery_always_try.xml',
    }

    def test_live_paths_pass(self):
        params = {
            'default_nav_to_pose_bt_xml':
                '/opt/patrolbot_ws/install/patrolbot_navigation/share/'
                'patrolbot_navigation/config/behavior_trees/'
                'navigate_recovery_always_try.xml',
            'default_nav_through_poses_bt_xml':
                '/opt/patrolbot_ws/install/patrolbot_navigation/share/'
                'patrolbot_navigation/config/behavior_trees/'
                'navigate_through_poses_recovery_always_try.xml',
        }
        checks = check_runtime_tree_selection(params, self.EXPECTED)
        self.assertTrue(all_ok(checks), failed(checks))

    def test_stock_tree_fails(self):
        params = {
            'default_nav_to_pose_bt_xml':
                '/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/'
                'navigate_to_pose_w_replanning_and_recovery.xml',
            'default_nav_through_poses_bt_xml':
                '/opt/patrolbot_ws/install/patrolbot_navigation/share/'
                'patrolbot_navigation/config/behavior_trees/'
                'navigate_through_poses_recovery_always_try.xml',
        }
        checks = check_runtime_tree_selection(params, self.EXPECTED)
        self.assertIn(
            'bt_navigator.default_nav_to_pose_bt_xml', failed(checks))

    def test_unset_parameter_fails(self):
        checks = check_runtime_tree_selection(
            {'default_nav_to_pose_bt_xml': None,
             'default_nav_through_poses_bt_xml': None},
            self.EXPECTED)
        self.assertEqual(len(failed(checks)), 2)


class BehaviorTreeLogTest(unittest.TestCase):
    """The supervised half of PB-AUDIT-003, asserted rather than eyeballed."""

    @staticmethod
    def round_robin_events():
        events = []
        for name in RECOVERY_ORDER:
            events.append((name, 'RUNNING'))
            events.append((name, 'FAILURE' if name == 'BackUp' else 'SUCCESS'))
        return events

    def test_full_round_robin_passes(self):
        checks = check_behavior_tree_log(self.round_robin_events())
        self.assertTrue(all_ok(checks), failed(checks))

    def test_noise_from_other_nodes_is_ignored(self):
        events = [('ComputePathToPose', 'FAILURE'), ('FollowPath', 'IDLE')]
        events.extend(self.round_robin_events())
        events.append(('NavigateRecovery', 'FAILURE'))
        checks = check_behavior_tree_log(events)
        self.assertTrue(all_ok(checks), failed(checks))

    def test_backup_never_running_fails(self):
        events = [
            event for event in self.round_robin_events()
            if event[0] != 'BackUp'
        ]
        checks = check_behavior_tree_log(events)
        self.assertIn(
            'behavior_tree.all_recovery_actions_ran', failed(checks))

    def test_spin_before_backup_fails(self):
        events = [
            ('ClearingActions', 'SUCCESS'),
            ('Spin', 'SUCCESS'),
            ('BackUp', 'SUCCESS'),
            ('Wait', 'SUCCESS'),
        ]
        checks = check_behavior_tree_log(events)
        self.assertIn(
            'behavior_tree.contiguous_round_robin_observed', failed(checks))

    def test_idle_transitions_do_not_count_as_running(self):
        events = [(name, 'IDLE') for name in RECOVERY_ORDER]
        checks = check_behavior_tree_log(events)
        self.assertIn(
            'behavior_tree.all_recovery_actions_ran', failed(checks))

    def test_repeated_ticks_of_one_node_collapse_to_one_activation(self):
        events = []
        for name in RECOVERY_ORDER:
            events.extend([(name, 'RUNNING')] * 5)
            events.append((name, 'SUCCESS'))
        self.assertEqual(
            recovery_activation_sequence(events), list(RECOVERY_ORDER))
        checks = check_behavior_tree_log(events)
        self.assertTrue(all_ok(checks), failed(checks))

    def test_four_names_scattered_across_separate_retries_do_not_pass(self):
        # The false pass that first-occurrence ordering allowed: each recovery
        # action appears, in the right relative order, but never as one
        # progression through the round-robin.
        events = [
            ('ClearingActions', 'SUCCESS'),
            ('ClearingActions', 'SUCCESS'),
            ('BackUp', 'FAILURE'),
            ('ClearingActions', 'SUCCESS'),
            ('Spin', 'SUCCESS'),
            ('ClearingActions', 'SUCCESS'),
            ('Wait', 'SUCCESS'),
        ]
        checks = check_behavior_tree_log(events)
        self.assertIn(
            'behavior_tree.contiguous_round_robin_observed', failed(checks))
        # All four still ran, so the weaker check alone would have passed.
        self.assertNotIn(
            'behavior_tree.all_recovery_actions_ran', failed(checks))

    def test_progression_inside_a_longer_window_passes(self):
        events = [('ClearingActions', 'SUCCESS'), ('Spin', 'SUCCESS')]
        events.extend(self.round_robin_events())
        events.append(('Wait', 'SUCCESS'))
        checks = check_behavior_tree_log(events)
        self.assertTrue(all_ok(checks), failed(checks))


class NavigatorVariantTest(unittest.TestCase):
    """Both trees use identical node names; only the action stream separates
    them, and PB-AUDIT-003's acceptance covers both variants."""

    def test_expected_variant_active_passes(self):
        checks = check_navigator_variant('to_pose', {'to_pose'})
        self.assertTrue(all_ok(checks), failed(checks))

    def test_no_navigator_active_fails(self):
        checks = check_navigator_variant('to_pose', set())
        self.assertIn('behavior_tree.navigator_variant', failed(checks))

    def test_the_other_variant_running_fails(self):
        checks = check_navigator_variant('through_poses', {'to_pose'})
        self.assertIn('behavior_tree.navigator_variant', failed(checks))

    def test_ambiguous_window_with_both_active_fails(self):
        checks = check_navigator_variant(
            'to_pose', {'to_pose', 'through_poses'})
        self.assertIn('behavior_tree.navigator_variant', failed(checks))


if __name__ == '__main__':
    unittest.main()
