"""Fixed hot-update audit for Navigation2 1.3.12 and the local F710 consumer.

Source-backed behavior, not deployment evidence. Runtime capability collection
must independently confirm the pinned implementation and consumed-value readback.
Unknown keys/states are never assumed hot. Constructor/PF reset fields select one
whole-navigation-container redeploy, not a per-node lifecycle choreography.
"""

NATIVE_VERSION = 'navigation2-1.3.12'
NAVIGATION_SERVICE = 'navigation'
NAVIGATION_NODES = (
    'amcl', 'behavior_server', 'bt_navigator', 'collision_monitor', 'controller_server',
    'joy_node', 'laser_static_tf', 'lifecycle_manager_localization',
    'lifecycle_manager_navigation', 'map_server', 'nav2_container',
    'patrolbot_controller_feedback', 'patrolbot_dock_manager', 'patrolbot_joy_teleop',
    'patrolbot_safety_watchdog', 'planner_server', 'route_server', 'smoother_server',
    'velocity_smoother', 'waypoint_follower',
)

AMCL_HOT = frozenset(('alpha1', 'alpha2', 'alpha3', 'alpha4', 'update_min_d', 'update_min_a',
                      'max_beams', 'sigma_hit', 'z_hit', 'z_rand'))
AMCL_FILTER_RESET = frozenset(('min_particles', 'max_particles', 'pf_err', 'pf_z'))
RPP_HOT = frozenset('FollowPath.' + key for key in (
    'desired_linear_vel', 'lookahead_dist', 'min_lookahead_dist', 'max_lookahead_dist',
    'lookahead_time', 'rotate_to_heading_angular_vel', 'min_approach_linear_velocity',
    'regulated_linear_scaling_min_speed'))
RPP_RESTART = frozenset(('FollowPath.approach_velocity_scaling_dist',))
SMOOTHER_HOT = frozenset(('max_velocity', 'min_velocity'))
SPIN_RESTART = frozenset(('max_rotational_vel', 'min_rotational_vel'))
F710_HOT = frozenset(('max_linear', 'max_angular', 'max_linear_cap', 'max_angular_cap',
                      'min_linear', 'min_angular'))


def classify(node, keys, state):
    """Return hot / nav2_redeploy / unsupported plus an explanatory reason.

    Lifecycle states are 1 unconfigured, 2 inactive, 3 active. Missing or
    transitional state is unavailable evidence, not permission to repair it by
    restarting. F710's fixed post-set implementation reports 'running'.
    """
    keys = frozenset(keys)
    if not keys:
        return 'hot', 'No effective consumer value changed.'
    if node == 'patrolbot_joy_teleop':
        if not keys <= F710_HOT or state != 'running':
            return 'unsupported', 'F710 consumed-policy implementation/state is unavailable.'
        return 'hot', 'F710 atomic post-set callback refreshes caps/minima and preserves lower operator trims.'
    if state not in (1, 2, 3) or type(state) is not int:
        return 'unsupported', node + ': lifecycle state is missing or transitional.'
    if node == 'amcl':
        if not keys <= AMCL_HOT | AMCL_FILTER_RESET:
            return 'unsupported', 'Unreviewed AMCL parameter mutation.'
        if keys & AMCL_FILTER_RESET:
            return 'nav2_redeploy', 'AMCL particle-filter settings recreate filter state; restart Nav2 and require a new seed.'
        if state != 3:
            return 'nav2_redeploy', 'AMCL dynamic callback is installed only while active.'
        return 'hot', 'AMCL active callback consumes thresholds/model inputs without recreating its particle filter.'
    if node == 'controller_server':
        if not keys <= RPP_HOT | RPP_RESTART:
            return 'unsupported', 'Unreviewed RPP parameter mutation.'
        if keys & RPP_RESTART:
            return 'nav2_redeploy', 'RPP approach scaling distance is constructor-cached, absent from the dynamic callback.'
        if state == 1:
            return 'nav2_redeploy', 'The RPP parameter handler has not been constructed.'
        return 'hot', 'RPP configured parameter handler consumes this complete batch while active or inactive.'
    if node in ('teleop_velocity_smoother', 'velocity_smoother'):
        if not keys <= SMOOTHER_HOT:
            return 'unsupported', 'Unreviewed smoother parameter mutation.'
        if state == 3:
            return 'hot', 'Active smoother callback consumes complete max/min velocity arrays.'
        if node == 'velocity_smoother':
            return 'nav2_redeploy', 'Inactive Nav2 smoother has no dynamic callback; its cache requires reconstruction.'
        return 'unsupported', 'The final bringup smoother is not active; a Nav2-only redeploy cannot repair that separate service.'
    if node == 'behavior_server':
        if keys <= SPIN_RESTART:
            return 'nav2_redeploy', 'Spin rotational bounds are cached during plugin configuration.'
        return 'unsupported', 'Unreviewed behavior parameter mutation.'
    return 'unsupported', 'Unknown fixed tuning consumer.'
