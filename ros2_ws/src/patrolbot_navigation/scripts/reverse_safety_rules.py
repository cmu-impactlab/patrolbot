"""Pure decision logic for PatrolBot's reverse safety chain.

Reverse is the one direction where the laser cannot help: the LMS-200 faces
forward, so a backwards move is guarded by the rear sonar (PB-AUDIT-004) and is
only ever commanded through Nav2's ``BackUp`` recovery behaviour
(PB-AUDIT-003). These two findings are deliberately checked together, because
each one is only as good as the other: an always-reachable ``BackUp`` that runs
sensor-blind is worse than no ``BackUp`` at all.

Everything here is a pure function over already-collected data. No ROS imports,
no I/O, no clock. `verify_reverse_safety.py` gathers the live inputs and calls
these; `test_reverse_safety_rules.py` calls them with fixtures.
"""

import math
import xml.etree.ElementTree as ET


# The RoundRobin children of both recovery trees, in the order the trees are
# written. BackUp runs first so a blocked robot attempts a short sonar-guarded
# reverse before another stationary clearing/replanning interval or rotation.
RECOVERY_ORDER = ('BackUp', 'ClearingActions', 'Spin', 'Wait')

# BT.CPP reports these while a node owns the tick, or as its terminal verdict.
ACTIVE_STATUSES = ('RUNNING', 'SUCCESS', 'FAILURE')

# Stock nav2 gates the recovery round-robin behind an error-code classification.
# This robot's false dead ends (CAD-artifact phantom obstacles, AMCL drift) are
# not always classified as recoverable, so the gate is replaced by AlwaysSuccess
# and these two nodes must not reappear in front of the round-robin.
STOCK_RECOVERY_GATES = (
    'WouldAControllerRecoveryHelp',
    'WouldAPlannerRecoveryHelp',
)

# The bridge builds every sonar point at exactly this height, so the collision
# monitor's height band has to admit it.
PUBLISHED_SONAR_Z = 0.0


class Check:
    """One named verdict, with the observation that produced it."""

    def __init__(self, name, ok, detail):
        self.name = name
        self.ok = bool(ok)
        self.detail = detail

    def __repr__(self):
        return 'Check({!r}, {}, {!r})'.format(self.name, self.ok, self.detail)

    def as_dict(self):
        return {'check': self.name, 'ok': self.ok, 'detail': self.detail}


def all_ok(checks):
    return all(check.ok for check in checks)


# ---------------------------------------------------------------------------
# PB-AUDIT-004 — rear sonar geometry
# ---------------------------------------------------------------------------


class SonarUnit:
    """One live transducer, as measured on the robot."""

    def __init__(self, index, x_m, y_m, theta_deg):
        self.index = int(index)
        self.x_m = float(x_m)
        self.y_m = float(y_m)
        self.theta_deg = float(theta_deg)
        radians = math.radians(self.theta_deg)
        self.dx = math.cos(radians)
        self.dy = math.sin(radians)

    def project(self, x, y):
        """Split a point into (distance along the ray, distance off it)."""
        vx = x - self.x_m
        vy = y - self.y_m
        along = vx * self.dx + vy * self.dy
        off = abs(vx * self.dy - vy * self.dx)
        return along, off


class SonarGeometry:
    def __init__(self, frame_id, max_range_m, ray_tolerance_m, units):
        self.frame_id = frame_id
        self.max_range_m = float(max_range_m)
        self.ray_tolerance_m = float(ray_tolerance_m)
        self.units = list(units)

    @property
    def indices(self):
        return [unit.index for unit in self.units]


def load_sonar_geometry(document):
    """Build the geometry from the parsed rear_sonar_geometry.yaml mapping."""
    section = document['rear_sonar_geometry']
    units = [
        SonarUnit(
            entry['index'], entry['x_m'], entry['y_m'], entry['theta_deg'])
        for entry in section['units']
    ]
    if not units:
        raise ValueError('rear_sonar_geometry lists no units')
    return SonarGeometry(
        frame_id=section['frame_id'],
        max_range_m=section['max_range_m'],
        ray_tolerance_m=section['ray_tolerance_m'],
        units=units,
    )


class PointVerdict:
    def __init__(self, x, y, z, index, along_m, off_m, reason):
        self.x = x
        self.y = y
        self.z = z
        self.index = index
        self.along_m = along_m
        self.off_m = off_m
        self.reason = reason

    @property
    def matched(self):
        return self.reason == 'matched'

    def as_dict(self):
        return {
            'x': round(self.x, 4),
            'y': round(self.y, 4),
            'z': round(self.z, 4),
            'sonar_index': self.index,
            'range_m': None if self.along_m is None else round(self.along_m, 4),
            'ray_offset_m': None if self.off_m is None else round(self.off_m, 6),
            'reason': self.reason,
        }


def classify_sonar_point(point, geometry):
    """Attribute one published point to the transducer that could have made it.

    The SBC emits ``origin + range * direction`` for every echo closer than the
    cutoff, so a credible point lies on exactly one measured ray, in front of
    that transducer, within the cutoff. A point that matches nothing is either a
    unit this robot is not supposed to have live, a mapping error, or a units
    bug — all of which must be visible rather than silently trusted while
    reversing.
    """
    x, y, z = point[0], point[1], point[2]
    best = None
    for unit in geometry.units:
        along, off = unit.project(x, y)
        if best is None or off < best[2]:
            best = (unit.index, along, off)

    index, along, off = best
    if off > geometry.ray_tolerance_m:
        reason = 'no_matching_ray'
    elif along <= 0.0:
        # On the ray but behind the transducer: geometrically impossible for an
        # echo, and a sign the mount angle no longer matches the hardware.
        reason = 'behind_transducer'
    elif along >= geometry.max_range_m:
        # The SBC drops `range >= 4335.0` before emitting, so a point AT the
        # cutoff cannot have come from it.
        reason = 'beyond_range_cutoff'
    else:
        reason = 'matched'
    return PointVerdict(x, y, z, index, along, off, reason)


def check_sonar_cloud(frame_id, points, geometry, height_band=None):
    """Validate one published /sonar cloud against the measured mounts.

    ``height_band`` is the collision monitor's live ``(min_height, max_height)``
    for the sonar source. A point outside it is discarded by the monitor before
    any polygon sees it, so a geometrically perfect point at the wrong height is
    still an invisible obstacle.
    """
    checks = []
    checks.append(Check(
        'sonar.frame_id',
        frame_id == geometry.frame_id,
        'frame_id={} expected={}'.format(frame_id, geometry.frame_id)))

    verdicts = [classify_sonar_point(point, geometry) for point in points]
    unmatched = [verdict for verdict in verdicts if not verdict.matched]
    checks.append(Check(
        'sonar.points_lie_on_measured_rays',
        not unmatched,
        'points={} unmatched={} {}'.format(
            len(verdicts), len(unmatched),
            [verdict.as_dict() for verdict in unmatched])))

    matched = [verdict for verdict in verdicts if verdict.matched]
    worst_offset = max((verdict.off_m for verdict in matched), default=0.0)
    checks.append(Check(
        'sonar.ray_residual_within_tolerance',
        worst_offset <= geometry.ray_tolerance_m,
        'worst_offset_m={:.6f} tolerance_m={}'.format(
            worst_offset, geometry.ray_tolerance_m)))

    # Sonar is rear-only. A forward point would mean a front transducer came
    # alive, and nothing in the collision monitor expects sonar in front: the
    # rear polygon's forward zone is a deliberately unreachable envelope.
    forward = [verdict for verdict in verdicts if verdict.x > 0.0]
    checks.append(Check(
        'sonar.is_rear_only',
        not forward,
        'forward_points={}'.format([verdict.as_dict() for verdict in forward])))

    if height_band is not None:
        low, high = height_band
        outside = [
            verdict for verdict in verdicts
            if not low <= verdict.z <= high
        ]
        checks.append(Check(
            'sonar.points_survive_the_height_filter',
            not outside,
            'height_band=({}, {}) discarded={}'.format(
                low, high, [verdict.as_dict() for verdict in outside])))

    return checks, verdicts


def check_stale_guard(sonar_source_timeout, fallback_s):
    """Judge the sonar source's own staleness timeout, and pick the value the
    gap measurement should be judged against.

    Returns ``(check, timeout_for_measurement)``.

    Two traps, both of which have to stay closed:

    * The **global** ``source_timeout`` is not the effective one -- Nav2 lets
      each source override it, so reading the global value can certify a guard
      that the sonar source does not use.
    * A configured ``0.0`` disables stale detection entirely. Falling back to a
      healthy-looking default on a falsy value would report a working guard
      while the live one is switched off, so the fallback is used only to keep
      measuring the window, never to decide the verdict.
    """
    armed = isinstance(sonar_source_timeout, float) and sonar_source_timeout > 0.0
    check = Check(
        'sonar.stale_guard_is_armed',
        armed,
        'sonar.source_timeout={} (0.0 disables stale detection)'.format(
            sonar_source_timeout))
    return check, (sonar_source_timeout if armed else fallback_s)


def subscribers_are_settled(names, errors, required):
    """Whether graph introspection can stop retrying.

    Both halves matter. Stopping as soon as the required subscribers appear
    leaves any transient introspection error in the report, failing a healthy
    robot; stopping only on a clean sweep would spin forever on a node that
    never answers. This is the "answer is complete" predicate; the retry budget
    bounds the rest.
    """
    return set(required) <= set(names) and not errors


def check_sonar_capture(clouds, total_points, matched_points, worst_offset_m,
                        geometry):
    """Judge a whole capture window, not just the clouds inside it.

    An empty cloud is legitimate -- it means "no echo closer than the cutoff".
    A window in which NO cloud carried a point is not evidence that the geometry
    is right; it is indistinguishable from every transducer returning the
    filtered 5000 mm no-echo value, which is what a dead sonar array looks like.
    Geometry can only be validated against real echoes, so this fails and asks
    for an object behind the robot.
    """
    checks = [Check(
        'sonar.echoes_observed',
        total_points > 0,
        'clouds={} points={} (place an object behind the robot if 0: an '
        'all-empty window cannot validate geometry)'.format(
            clouds, total_points))]
    if total_points:
        checks.append(Check(
            'sonar.points_all_attributed',
            total_points == matched_points,
            'clouds={} points={} matched={} worst_ray_offset_m={:.6f} '
            'live_indices={}'.format(
                clouds, total_points, matched_points, worst_offset_m,
                geometry.indices)))
    return checks


def check_sonar_stream(frame_ids, arrival_times, window_start, window_end,
                       minimum_rate_hz, source_timeout_s):
    """Rate, gaps and frame consistency across a captured window of /sonar.

    ``window_start``/``window_end`` bound the capture. They are what makes the
    silence at the EDGES of the window visible: forty arrivals at 5 Hz followed
    by 1.4 s of nothing satisfies both an average rate and every internal gap,
    while the source was actually stale when the capture ended.
    """
    checks = []
    message_count = len(arrival_times)
    elapsed_s = window_end - window_start
    rate = message_count / elapsed_s if elapsed_s > 0 else 0.0
    checks.append(Check(
        'sonar.rate',
        rate >= minimum_rate_hz,
        'messages={} elapsed_s={:.2f} rate_hz={:.2f} minimum_hz={}'.format(
            message_count, elapsed_s, rate, minimum_rate_hz)))

    # An average rate hides the case that actually matters: one long silence.
    # Any silence at or beyond the monitor's source_timeout means the sonar
    # source went stale and reverse was blocked -- fail-closed, but not
    # something to discover during a reverse. Both edges of the window count.
    if arrival_times:
        boundaries = (
            [arrival_times[0] - window_start]
            + [
                later - earlier
                for earlier, later in zip(arrival_times, arrival_times[1:])
            ]
            + [window_end - arrival_times[-1]]
        )
    else:
        boundaries = [elapsed_s]
    worst_gap = max(boundaries)
    checks.append(Check(
        'sonar.no_gap_reaches_source_timeout',
        worst_gap < source_timeout_s,
        'worst_gap_s={:.3f} (leading={:.3f} trailing={:.3f}) '
        'source_timeout_s={} samples={}'.format(
            worst_gap,
            boundaries[0] if arrival_times else elapsed_s,
            boundaries[-1] if arrival_times else elapsed_s,
            source_timeout_s, message_count)))

    distinct = sorted(set(frame_ids))
    checks.append(Check(
        'sonar.frame_is_stable',
        len(distinct) <= 1,
        'frames={}'.format(distinct)))
    return checks


# ---------------------------------------------------------------------------
# PB-AUDIT-004 — the collision monitor must actually consume the sonar
# ---------------------------------------------------------------------------


def check_collision_monitor_runtime(params, sonar_subscribers):
    """Assert the LIVE collision monitor parameters, not the repo file.

    PB-AUDIT-003's original finding was a runtime parameter that disagreed with
    the reviewed configuration, so every value here is read back from the
    running node.
    """
    checks = []

    sources = list(params.get('observation_sources') or [])
    checks.append(Check(
        'collision_monitor.observation_sources',
        'sonar' in sources and 'scan' in sources,
        'observation_sources={}'.format(sources)))

    rear_sources = list(params.get('rear_velocity_stop.sources_names') or [])
    checks.append(Check(
        'collision_monitor.rear_polygon_is_sonar_fed',
        rear_sources == ['sonar'],
        'rear_velocity_stop.sources_names={}'.format(rear_sources)))

    # Four sparse transducers (three of them live) produce at most three points.
    # A single credible echo must be able to stop a reverse, so this may never
    # be raised towards the scan polygon's threshold.
    min_points = params.get('rear_velocity_stop.min_points')
    checks.append(Check(
        'collision_monitor.rear_single_point_stops',
        min_points == 1,
        'rear_velocity_stop.min_points={}'.format(min_points)))

    # Jazzy's replacement for min_points. It is the number of points that may be
    # inside the polygon WITHOUT acting, so a one-point stop is max_points == 0.
    max_points = params.get('rear_velocity_stop.max_points')
    if max_points is not None:
        checks.append(Check(
            'collision_monitor.rear_max_points_agrees',
            max_points == 0,
            'rear_velocity_stop.max_points={}'.format(max_points)))

    action = params.get('rear_velocity_stop.action_type')
    checks.append(Check(
        'collision_monitor.rear_action_is_stop',
        action == 'stop',
        'rear_velocity_stop.action_type={}'.format(action)))

    # Fail closed on missing data. /sonar is ~5 Hz, so 0.5 s tolerates one
    # missed period and stops after roughly two or three. Never zero: in Nav2 a
    # zero source_timeout means "never time out".
    for name in ('source_timeout', 'sonar.source_timeout', 'scan.source_timeout'):
        value = params.get(name)
        checks.append(Check(
            'collision_monitor.{}'.format(name),
            value == 0.5,
            '{}={}'.format(name, value)))

    enabled = params.get('sonar.enabled')
    checks.append(Check(
        'collision_monitor.sonar_source_enabled',
        enabled is True,
        'sonar.enabled={}'.format(enabled)))

    # The source must be the rear sonar cloud and nothing else. A source that
    # is enabled, fresh and correctly polygon-wired but reading the wrong topic
    # or parsed as the wrong type is indistinguishable from a working one until
    # something is actually behind the robot.
    source_type = params.get('sonar.type')
    checks.append(Check(
        'collision_monitor.sonar_source_type',
        source_type == 'pointcloud',
        'sonar.type={}'.format(source_type)))
    source_topic = params.get('sonar.topic')
    checks.append(Check(
        'collision_monitor.sonar_source_topic',
        source_topic == '/sonar',
        'sonar.topic={}'.format(source_topic)))

    # The filters that can throw every point away without any other check
    # noticing. The bridge publishes z exactly 0.0, so the height band must
    # admit 0.0; a min_range above 0 deletes the near-field echoes that matter
    # most while reversing (the PB-AUDIT-018 failure mode, on the rear).
    min_height = params.get('sonar.min_height')
    max_height = params.get('sonar.max_height')
    admits_published_height = (
        isinstance(min_height, float) and isinstance(max_height, float)
        and min_height <= PUBLISHED_SONAR_Z <= max_height)
    checks.append(Check(
        'collision_monitor.sonar_height_band_admits_the_cloud',
        admits_published_height,
        'sonar.min_height={} sonar.max_height={} published_z={}'.format(
            min_height, max_height, PUBLISHED_SONAR_Z)))

    min_range = params.get('sonar.min_range')
    checks.append(Check(
        'collision_monitor.sonar_keeps_near_field',
        min_range == 0.0,
        'sonar.min_range={}'.format(min_range)))

    # With use_global_height true Nav2 requires a `height` field on the cloud.
    # The bridge publishes x/y/z only, so the source would be rejected outright
    # -- fail-closed, but it would mean reverse has no rear source at all while
    # every other check still reports a healthy wiring.
    use_global_height = params.get('sonar.use_global_height')
    checks.append(Check(
        'collision_monitor.sonar_uses_the_published_fields',
        use_global_height is False,
        'sonar.use_global_height={} (published fields are x/y/z)'.format(
            use_global_height)))

    # MUST stay False. With True the monitor looks up laser_frame->odom at each
    # scan's exact stamp; after a bridge/SBC reconnect the TF gap makes that
    # throw BackwardExtrapolationException, which is uncaught and SIGABRTs the
    # whole composed nav2_container.
    shift = params.get('base_shift_correction')
    checks.append(Check(
        'collision_monitor.base_shift_correction_stays_false',
        shift is False,
        'base_shift_correction={}'.format(shift)))

    subscribers = sorted(sonar_subscribers)
    checks.append(Check(
        'sonar.is_consumed',
        'collision_monitor' in subscribers
        and 'patrolbot_dock_manager' in subscribers,
        'subscribers={}'.format(subscribers)))

    return checks


# ---------------------------------------------------------------------------
# PB-AUDIT-003 — the recovery tree that keeps BackUp reachable
# ---------------------------------------------------------------------------


def check_recovery_tree(xml_text, source):
    """Structural assertions on one recovery behaviour tree."""
    checks = []
    root = ET.fromstring(xml_text)

    # Anchored to the tree that actually EXECUTES, by structure, never by
    # searching for a name. Any `.//` search matches the first element carrying
    # the name, so a duplicate placed in a branch that is never ticked -- even
    # a duplicate `NavigateRecovery` -- satisfies the check while the real
    # branch is broken.
    #
    # The chain is exact and every step is a direct child of the previous one:
    #   root[main_tree_to_execute] -> BehaviorTree[ID] -> RecoveryNode
    #   [NavigateRecovery] -> (second child) -> ReactiveFallback
    #   [RecoveryFallback] -> RoundRobin
    main_tree_id = root.attrib.get('main_tree_to_execute')
    main_tree = (
        root.find('BehaviorTree[@ID="{}"]'.format(main_tree_id))
        if main_tree_id else None)
    entry = (
        list(main_tree)[0]
        if main_tree is not None and len(list(main_tree)) == 1 else None)
    outer = (
        entry
        if entry is not None
        and entry.tag == 'RecoveryNode'
        and entry.attrib.get('name') == 'NavigateRecovery'
        else None)
    if outer is None:
        checks.append(Check(
            'tree.executed_root_is_the_recovery_node', False,
            '{}: main_tree_to_execute={!r} entry={}'.format(
                source, main_tree_id,
                None if entry is None else entry.tag)))
        return checks
    checks.append(Check(
        'tree.executed_root_is_the_recovery_node', True,
        '{}: {} -> RecoveryNode[NavigateRecovery]'.format(
            source, main_tree_id)))

    # A second node wearing either anchoring name is never legitimate here and
    # would make the structural anchor ambiguous to a future reader.
    for name in ('NavigateRecovery', 'RecoveryFallback'):
        duplicates = [
            element for element in root.iter()
            if element.attrib.get('name') == name
        ]
        checks.append(Check(
            'tree.unique_anchor[{}]'.format(name),
            len(duplicates) == 1,
            '{}: {} nodes named {}'.format(source, len(duplicates), name)))

    recovery_branch = (
        list(outer)[1] if len(list(outer)) > 1 else None)
    recovery_fallback = (
        recovery_branch.find('ReactiveFallback[@name="RecoveryFallback"]')
        if recovery_branch is not None else None)
    round_robin = (
        recovery_fallback.find('RoundRobin')
        if recovery_fallback is not None else None)
    if round_robin is None:
        checks.append(Check(
            'tree.round_robin_present', False,
            '{}: no RoundRobin on the NavigateRecovery failure branch '
            '(branch={} recovery_fallback={})'.format(
                source, recovery_branch is not None,
                recovery_fallback is not None)))
        return checks

    observed = []
    for child in round_robin:
        observed.append(child.attrib.get('name') or child.tag)
    checks.append(Check(
        'tree.recovery_order',
        observed == list(RECOVERY_ORDER),
        '{}: round_robin={} expected={}'.format(
            source, observed, list(RECOVERY_ORDER))))

    # The whole point of the custom tree: no error-code classification stands
    # between a failed navigation and the round-robin. Judged on the same
    # structurally anchored branch.
    gate_text = ET.tostring(recovery_branch, encoding='unicode')
    stock_gates = [gate for gate in STOCK_RECOVERY_GATES if gate in gate_text]
    always_success = recovery_branch.find('AlwaysSuccess') is not None
    checks.append(Check(
        'tree.recovery_gate_is_unconditional',
        not stock_gates and always_success,
        '{}: stock_gates={} always_success={}'.format(
            source, stock_gates, always_success)))

    backup = round_robin.find('BackUp')
    if backup is None:
        checks.append(Check(
            'tree.backup_present', False,
            '{}: no BackUp in the round-robin'.format(source)))
        return checks

    # Pinned to the live-guarded values. BackUp consults the local costmap,
    # which is blind behind, so its guarding is the rear sonar in the collision
    # monitor -- the distance and speed here are what that was verified at.
    checks.append(Check(
        'tree.backup_distance',
        backup.attrib.get('backup_dist') == '0.10',
        '{}: backup_dist={}'.format(source, backup.attrib.get('backup_dist'))))
    checks.append(Check(
        'tree.backup_speed',
        backup.attrib.get('backup_speed') == '0.05',
        '{}: backup_speed={}'.format(source, backup.attrib.get('backup_speed'))))

    retries = outer.attrib.get('number_of_retries')
    checks.append(Check(
        'tree.recovery_retries',
        retries == '9',
        '{}: number_of_retries={} (>= two full round-robin cycles)'.format(
            source, retries)))

    return checks


def check_runtime_tree_selection(params, expected_basenames):
    """The bt_navigator must be RUNNING the custom trees, not the stock ones."""
    checks = []
    for parameter, expected in expected_basenames.items():
        value = params.get(parameter) or ''
        basename = value.rsplit('/', 1)[-1]
        checks.append(Check(
            'bt_navigator.{}'.format(parameter),
            basename == expected,
            '{}={} (basename={} expected={})'.format(
                parameter, value, basename, expected)))
    return checks


def recovery_activation_sequence(events, expected_order=RECOVERY_ORDER):
    """The recovery actions that took the tick, in order, without repeats.

    BT.CPP reports a node several times per activation (RUNNING then a terminal
    status, often across several ticks), so consecutive duplicates are collapsed
    to one entry. Names outside the round-robin are dropped.
    """
    wanted = set(expected_order)
    sequence = []
    for node_name, status in events:
        if node_name not in wanted or status not in ACTIVE_STATUSES:
            continue
        if not sequence or sequence[-1] != node_name:
            sequence.append(node_name)
    return sequence


def check_behavior_tree_log(events, expected_order=RECOVERY_ORDER):
    """Assert the recovery round-robin actually ran, in order.

    ``events`` is every ``(node_name, current_status)`` pair seen on
    /behavior_tree_log, in arrival order. This is the supervised half of
    PB-AUDIT-003: a planning failure is induced, and the round-robin must be
    observed rather than assumed.

    The evidence demanded is one **contiguous** progression through the
    round-robin. Taking the first occurrence of each name anywhere in the window
    is not enough: a long window spans retries and goals, and four first-sightings
    that happen to fall in the right order can be assembled from unrelated
    activations without the round-robin ever advancing as designed.
    """
    checks = []
    wanted = list(expected_order)
    sequence = recovery_activation_sequence(events, expected_order)

    observed = set(sequence)
    missing = [name for name in wanted if name not in observed]
    checks.append(Check(
        'behavior_tree.all_recovery_actions_ran',
        not missing,
        'sequence={} missing={}'.format(sequence, missing)))

    runs = [
        index for index in range(len(sequence) - len(wanted) + 1)
        if sequence[index:index + len(wanted)] == wanted
    ]
    checks.append(Check(
        'behavior_tree.contiguous_round_robin_observed',
        bool(runs),
        'expected={} sequence={} matches_at={}'.format(wanted, sequence, runs)))

    # The reason the order matters at all: reversing before rotating. Checked
    # inside one observed progression, not across the whole window.
    if runs:
        cycle = sequence[runs[0]:runs[0] + len(wanted)]
        checks.append(Check(
            'behavior_tree.backup_precedes_spin',
            cycle.index('BackUp') < cycle.index('Spin'),
            'cycle={}'.format(cycle)))

    return checks


def check_navigator_variant(expected_variant, active_variants):
    """Which navigator produced the observed recovery.

    Both recovery trees use identical node names, so /behavior_tree_log alone
    cannot say whether a run exercised the to-pose or the through-poses tree.
    The action status stream is what distinguishes them, and PB-AUDIT-003's
    acceptance covers BOTH variants -- so the run has to name which one it was
    and be corroborated.
    """
    active = sorted(active_variants)
    return [
        Check(
            'behavior_tree.navigator_variant',
            active == [expected_variant],
            'expected={} active_during_window={}'.format(
                expected_variant, active)),
    ]
