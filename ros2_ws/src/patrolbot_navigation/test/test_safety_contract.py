"""Static safety-contract tests for PatrolBot navigation configuration."""

import math
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'patrolbot_navigation_launch.py'
PARAMS_FILE = PACKAGE_ROOT / 'config' / 'nav2_params.yaml'
BRIDGE_FILE = PACKAGE_ROOT.parent / 'patrolbot_bridge' / 'patrolbot_bridge' / 'bridge_node.py'
DOCK_MANAGER_FILE = (
    PACKAGE_ROOT.parent / 'patrolbot_dock_manager' /
    'patrolbot_dock_manager' / 'dock_manager.py'
)
DOCK_PLUGIN_FILE = (
    PACKAGE_ROOT.parent / 'patrolbot_docking_plugins' / 'src' /
    'patrolbot_charging_dock.cpp'
)
INTERFACES_ROOT = PACKAGE_ROOT.parent / 'patrolbot_interfaces'
LAUNCH_PACKAGE_ROOT = PACKAGE_ROOT.parent / 'patrolbot-launch'
REPOSITORY_ROOT = next(
    candidate for candidate in (
        PACKAGE_ROOT.parents[2], PACKAGE_ROOT.parents[1])
    if (candidate / 'docker' / 'Dockerfile').exists()
)
COMPOSE_FILE = REPOSITORY_ROOT / 'docker' / 'docker-compose.yml'
DOCKERFILE = REPOSITORY_ROOT / 'docker' / 'Dockerfile'
STATUS_FILE = REPOSITORY_ROOT / 'docker' / 'status.sh'
SOURCE_DIGEST_FILE = REPOSITORY_ROOT / 'docker' / 'source-tree-digest.sh'
SERVER_CANDIDATES = (
    PACKAGE_ROOT.parents[2] / 'hardware' / 'patrolbot_hw_server' /
    'patrolbot_server.cpp',
    PACKAGE_ROOT.parents[1] / 'hardware' / 'patrolbot_hw_server' /
    'patrolbot_server.cpp',
)
SERVER_FILE = next(path for path in SERVER_CANDIDATES if path.exists())
PROTOCOL_FILE = SERVER_FILE.with_name('protocol.cpp')
SERVER_SYSTEMD_ROOT = (
    REPOSITORY_ROOT / 'hardware' / 'patrolbot_hw_server' / 'systemd'
)
SERVER_UNIT_FILE = SERVER_SYSTEMD_ROOT / 'patrolbot-server.service'
SERVER_LOG_DROPIN_FILE = (
    SERVER_SYSTEMD_ROOT / 'patrolbot-server.service.d' / 'log.conf'
)
SERVER_JOURNALD_FILE = (
    SERVER_SYSTEMD_ROOT / 'journald.conf.d' / '90-patrolbot.conf'
)
SERVER_MAINTENANCE_ROOT = (
    REPOSITORY_ROOT / 'hardware' / 'patrolbot_hw_server' / 'maintenance'
)


class SafetyContractTest(unittest.TestCase):
    """Protect incident-verified settings and command-path containment."""

    @classmethod
    def setUpClass(cls):
        cls.launch_text = LAUNCH_FILE.read_text(encoding='utf-8')
        cls.params = yaml.safe_load(PARAMS_FILE.read_text(encoding='utf-8'))

    def test_docking_server_is_not_launched_or_lifecycle_managed(self):
        self.assertNotIn('docking_server', self.launch_text)
        self.assertNotIn('opennav_docking', self.launch_text)

    def test_navigation_producers_are_arbitrated_before_smoothing(self):
        controller = "('cmd_vel', 'input/nav_controller')"
        behavior = "('cmd_vel', 'input/nav_behavior')"
        smoother = "('cmd_vel', 'cmd_vel_nav')"
        # Each producer is remapped in composed and non-composed modes. Only the
        # upstream mux may publish the shared pre-smoother topic.
        self.assertEqual(self.launch_text.count(controller), 2)
        self.assertEqual(self.launch_text.count(behavior), 2)
        self.assertEqual(self.launch_text.count(smoother), 2)
        self.assertIn("name='nav_command_mux'", self.launch_text)
        self.assertIn("('cmd_vel_out', 'cmd_vel_nav')", self.launch_text)

        mux = self.params['nav_command_mux']['ros__parameters']
        self.assertIs(mux['use_stamped'], False)
        self.assertEqual(mux['topics']['controller'], {
            'topic': 'input/nav_controller', 'timeout': 0.25, 'priority': 5})
        self.assertEqual(mux['topics']['behavior'], {
            'topic': 'input/nav_behavior', 'timeout': 0.25, 'priority': 7})
        self.assertEqual(mux['topics']['docking'], {
            'topic': 'input/docking', 'timeout': 0.25, 'priority': 9})

    def test_reconnect_safe_collision_monitor_setting_is_pinned(self):
        collision = self.params['collision_monitor']['ros__parameters']
        self.assertIs(collision['base_shift_correction'], False)

    def test_undock_spin_matches_hardware_turn_cap(self):
        behavior = self.params['behavior_server']['ros__parameters']
        self.assertEqual(behavior['max_rotational_vel'], 0.7)

    def test_rpp_internal_collision_check_stays_disabled(self):
        follow_path = self.params['controller_server']['ros__parameters']['FollowPath']
        self.assertIs(follow_path['use_collision_detection'], False)

    def test_inflation_cost_scaling_stays_deployed_float(self):
        local = self.params['local_costmap']['local_costmap']['ros__parameters']
        global_ = self.params['global_costmap']['global_costmap']['ros__parameters']
        for config in (local, global_):
            value = config['inflation_layer']['cost_scaling_factor']
            self.assertIsInstance(value, float)
            self.assertEqual(value, 4.0)

    def test_close_range_scan_filter_remains_pinned(self):
        bridge_text = BRIDGE_FILE.read_text(encoding='utf-8')
        self.assertIn('SCAN_RANGE_MIN = 0.25', bridge_text)

    def test_near_field_candidate_is_disabled_and_not_a_safety_source(self):
        bridge_text = BRIDGE_FILE.read_text(encoding='utf-8')
        self.assertIn(
            "'publish_unfiltered_scan_candidate', False", bridge_text)
        collision = self.params['collision_monitor']['ros__parameters']
        self.assertNotIn(
            '/scan_unfiltered_candidate', yaml.safe_dump(collision))

    def test_battery_diagnostic_uses_ros_byte_constants(self):
        bridge_text = BRIDGE_FILE.read_text(encoding='utf-8')
        for level in (
                'DiagnosticStatus.OK',
                'DiagnosticStatus.WARN',
                'DiagnosticStatus.ERROR'):
            self.assertIn(level, bridge_text)

    def test_amcl_never_claims_map_origin_automatically(self):
        amcl = self.params['amcl']['ros__parameters']
        self.assertIs(amcl['set_initial_pose'], False)
        self.assertIs(amcl['always_reset_initial_pose'], False)

    def test_guarded_undock_uses_the_commissioned_main_charger_pose(self):
        manager = self.params['patrolbot_dock_manager']['ros__parameters']
        self.assertIs(manager['dock_pose_configured'], True)
        self.assertEqual(manager['dock_id'], 'main_charger')
        self.assertAlmostEqual(manager['dock_pose_x'], 99.28925458355519)
        self.assertAlmostEqual(manager['dock_pose_y'], 126.33778290362004)
        self.assertAlmostEqual(manager['dock_pose_yaw'], -1.578938606346)
        self.assertIs(manager['allow_front_bumper_at_dock'], True)
        self.assertEqual(manager['sensor_timeout'], 0.5)
        self.assertEqual(manager['operator_motor_window'], 10.0)
        self.assertEqual(manager['dock_departure_charge_grace'], 8.0)
        self.assertAlmostEqual(manager['undock_turn_yaw'], math.pi)
        self.assertEqual(manager['spin_timeout'], 20.0)
        self.assertEqual(manager['dock_pose_initialization_timeout'], 5.0)

    def test_measured_bumper_masks_are_loaded_by_compose(self):
        bridge = yaml.safe_load((
            PACKAGE_ROOT.parent / 'patrolbot_bridge' / 'config' /
            'bridge_params.yaml').read_text(encoding='utf-8'))[
                'patrolbot_bridge']['ros__parameters']
        self.assertIs(bridge['bumpers_mapped'], True)
        self.assertEqual(bridge['front_bumper_mask'], 0x7e00)
        self.assertEqual(bridge['rear_bumper_mask'], 0x007e)
        compose = yaml.safe_load((
            REPOSITORY_ROOT / 'docker' /
            'docker-compose.yml').read_text(encoding='utf-8'))
        bridge_command = compose['services']['bridge']['command']
        self.assertIn('--params-file', bridge_command)
        self.assertIn(
            '/opt/patrolbot_ws/src/patrolbot_bridge/config/bridge_params.yaml',
            bridge_command)

    def test_system_watchdog_enforces_power_and_hardware_state(self):
        watchdog = (
            PACKAGE_ROOT / 'scripts' /
            'patrolbot_safety_watchdog.py').read_text(encoding='utf-8')
        self.assertIn("'battery_stop_voltage', 23.0", watchdog)
        self.assertIn("'stop_on_faults', True", watchdog)
        self.assertIn("'stop_on_estop', True", watchdog)
        self.assertIn("'bumper_enforcement_enabled', False", watchdog)
        self.assertIn("'/patrolbot/base_state'", watchdog)
        self.assertIn("'/cmd_vel_out'", watchdog)
        config = self.params['patrolbot_safety_watchdog']['ros__parameters']
        self.assertEqual(config['battery_stop_voltage'], 23.0)
        self.assertEqual(config['battery_timeout'], 1.0)
        self.assertIs(config['stop_on_faults'], True)
        self.assertIs(config['stop_on_estop'], True)
        self.assertIs(config['bumper_enforcement_enabled'], False)

    def test_navigation_smoother_has_no_ignored_topic_parameter(self):
        smoother = self.params['velocity_smoother']['ros__parameters']
        self.assertNotIn('cmd_vel_topic', smoother)
        self.assertIs(smoother['enable_stamped_cmd_vel'], False)
        self.assertEqual(smoother['velocity_timeout'], 1.0)

    def test_teleop_smoother_uses_only_declared_jazzy_parameters(self):
        smoother = yaml.safe_load((
            LAUNCH_PACKAGE_ROOT / 'param' / 'defaults' /
            'smoother.yaml').read_text(encoding='utf-8'))[
                'teleop_velocity_smoother']['ros__parameters']
        obsolete = {
            'speed_lim_v', 'speed_lim_w', 'accel_lim_v', 'accel_lim_w',
            'frequency', 'decel_factor', 'robot_feedback'}
        self.assertFalse(obsolete & smoother.keys())
        self.assertEqual(smoother['max_velocity'], [0.35, 0.0, 0.7])
        self.assertEqual(smoother['min_velocity'], [-0.35, 0.0, -0.7])
        self.assertEqual(smoother['max_accel'], [1.0, 0.0, 1.5])
        self.assertEqual(smoother['max_decel'], [-1.4, 0.0, -2.0])
        self.assertEqual(smoother['velocity_timeout'], 0.5)
        self.assertIs(smoother['enable_stamped_cmd_vel'], False)

    def test_empty_mobile_base_component_container_is_removed(self):
        mobile_base = (
            LAUNCH_PACKAGE_ROOT / 'launch' /
            'mobile_base.xml').read_text(encoding='utf-8')
        self.assertNotIn('component_container', mobile_base)
        self.assertNotIn('mobile_base_nodelet_manager', mobile_base)
        self.assertIn('exec="twist_mux"', mobile_base)

    def test_dds_discovery_is_localhost_only_in_every_container(self):
        compose = yaml.safe_load(COMPOSE_FILE.read_text(encoding='utf-8'))
        self.assertEqual(
            compose['x-common']['environment'][
                'ROS_AUTOMATIC_DISCOVERY_RANGE'], 'LOCALHOST')
        for service in ('bringup', 'bridge', 'navigation'):
            self.assertEqual(
                compose['services'][service]['environment'][
                    'ROS_AUTOMATIC_DISCOVERY_RANGE'], 'LOCALHOST')
        status = STATUS_FILE.read_text(encoding='utf-8')
        self.assertIn('expected LOCALHOST in every container', status)

    def test_optional_dashboard_profile_does_not_block_core_compose(self):
        compose = yaml.safe_load(COMPOSE_FILE.read_text(encoding='utf-8'))
        web_bridge = compose['services']['web-bridge']
        self.assertEqual(
            web_bridge['environment']['WEB_BRIDGE_SERVER_URL'],
            '${WEB_BRIDGE_SERVER_URL:-}')
        self.assertEqual(
            web_bridge['environment']['WEB_BRIDGE_TOKEN'],
            '${WEB_BRIDGE_TOKEN:-}')
        command = web_bridge['command'][0]
        self.assertIn(
            '$${WEB_BRIDGE_SERVER_URL:?set WEB_BRIDGE_SERVER_URL', command)
        self.assertIn('$${WEB_BRIDGE_TOKEN:?set WEB_BRIDGE_TOKEN', command)
        self.assertIn('$$PYTHONPATH', command)

    def test_image_provenance_digest_is_build_and_runtime_enforced(self):
        dockerfile = DOCKERFILE.read_text(encoding='utf-8')
        compose = COMPOSE_FILE.read_text(encoding='utf-8')
        status = STATUS_FILE.read_text(encoding='utf-8')
        self.assertIn('ARG PATROLBOT_SOURCE_SHA256=unknown', dockerfile)
        self.assertIn('io.patrolbot.source-tree-sha256', dockerfile)
        self.assertIn('PATROLBOT_SOURCE_SHA256:', compose)
        self.assertIn('image must match deployed source', status)
        digest = subprocess.run(
            [str(SOURCE_DIGEST_FILE), str(REPOSITORY_ROOT)],
            check=True, capture_output=True, text=True).stdout.strip()
        self.assertEqual(len(digest), 64)
        self.assertTrue(all(character in '0123456789abcdef' for character in digest))

    def test_joystick_unavailability_is_a_readiness_failure(self):
        status = STATUS_FILE.read_text(encoding='utf-8')
        self.assertIn('/dev/input/js0', status)
        self.assertIn('joystick receiver absent', status)

    def test_joystick_takeover_cancels_navigation_and_fails_closed(self):
        teleop = (
            PACKAGE_ROOT / 'scripts' /
            'patrolbot_joy_teleop.py').read_text(encoding='utf-8')
        bringup = (
            PACKAGE_ROOT / 'launch' /
            'bringup.launch.py').read_text(encoding='utf-8')
        mux = yaml.safe_load((
            LAUNCH_PACKAGE_ROOT / 'param' / 'defaults' /
            'mux.yaml').read_text(encoding='utf-8'))[
                'cmd_vel_mux']['ros__parameters']

        self.assertIn(
            "'/navigate_to_pose/_action/cancel_goal'", teleop)
        self.assertIn('CancelGoal.Request()', teleop)
        self.assertIn('self.active or moving or self._takeover_pending', teleop)
        self.assertIn("'device_id': 0", bringup)
        self.assertIn("'autorepeat_rate': 20.0", bringup)
        self.assertIn("'sticky_buttons': False", bringup)
        self.assertEqual(mux['topics']['joy'], {
            'topic': 'input/joy', 'timeout': 0.5, 'priority': 8})

    def test_nav_goal_capture_is_bounded_and_read_only(self):
        capture = (
            PACKAGE_ROOT / 'scripts' /
            'capture_nav_command_path.py').read_text(encoding='utf-8')
        wrapper = (
            REPOSITORY_ROOT / 'docker' /
            'capture-nav-goal.sh').read_text(encoding='utf-8')
        for forbidden in (
                'create_publisher(', 'create_client(', 'ActionClient('):
            self.assertNotIn(forbidden, capture)
        self.assertIn("'--duration'", capture)
        self.assertIn('arguments.duration <= 120.0', capture)
        self.assertIn('timeout --signal=INT --kill-after=2', wrapper)
        self.assertIn("grep -q '[r]os2 topic hz'", wrapper)

    def test_operator_pose_initialization_and_readiness_are_typed(self):
        manager_text = DOCK_MANAGER_FILE.read_text(encoding='utf-8')
        service_text = (INTERFACES_ROOT / 'srv' /
                        'InitializeDockPose.srv').read_text(encoding='utf-8')
        readiness_text = (INTERFACES_ROOT / 'msg' /
                          'DockReadiness.msg').read_text(encoding='utf-8')
        self.assertIn("'/patrolbot/initialize_dock_pose'", manager_text)
        self.assertIn("'/initialpose'", manager_text)
        self.assertIn('operator_authorized', service_text)
        self.assertIn('ready_for_undock', readiness_text)
        self.assertIn('blocking_reason', readiness_text)

    def test_dock_manager_never_publishes_velocity_directly(self):
        manager_text = DOCK_MANAGER_FILE.read_text(encoding='utf-8')
        self.assertNotIn('create_publisher(Twist', manager_text)
        self.assertNotIn('create_publisher(\n            Twist', manager_text)
        self.assertNotIn("'/cmd_vel'", manager_text.split(
            'def _refresh_command_path', 1)[0])
        self.assertIn("ActionClient(\n            self, BackUp, '/backup'", manager_text)
        self.assertIn("'assisted_teleop', 'backup'", manager_text)
        # Nav2 BackUp takes positive target distance and produces negative Twist.
        self.assertIn('backup.target.x = float(distance)', manager_text)

    def test_runtime_command_path_contract_covers_every_stage(self):
        manager_text = DOCK_MANAGER_FILE.read_text(encoding='utf-8')
        for topic in (
                '/input/nav_controller', '/input/nav_behavior',
                '/input/docking', '/cmd_vel_nav', '/cmd_vel_smoothed',
                '/input/navi', '/cmd_vel_out', '/cmd_vel'):
            self.assertIn(f"'{topic}'", manager_text)

    def test_cancel_stops_backup_and_requires_zero_velocity_confirmation(self):
        manager_text = DOCK_MANAGER_FILE.read_text(encoding='utf-8')
        self.assertIn('nav_goal.cancel_goal_async()', manager_text)
        self.assertIn('_wait_for_odom_zero(1.5)', manager_text)
        self.assertIn('goal_handle.is_cancel_requested', manager_text)

    def test_disconnect_path_stops_base_and_cannot_replay_old_velocity(self):
        server_text = SERVER_FILE.read_text(encoding='utf-8')
        disconnect = server_text.split(
            'while (robot.isRunning() && client_connected)', 1)[1]
        disconnect = disconnect.split(
            'ArLog::log(ArLog::Normal,\n'
            '               "Pi disconnected;', 1)[0]
        self.assertIn('robot.setVel(0);', disconnect)
        self.assertIn('robot.setRotVel(0);', disconnect)
        self.assertIn('robot.stop();', disconnect)
        self.assertNotIn('lastCommand', server_text)
        self.assertNotIn('lastVelocity', server_text)

    def test_hardware_controls_cannot_enable_or_move_as_a_drive_side_effect(self):
        server_text = SERVER_FILE.read_text(encoding='utf-8')
        protocol_text = PROTOCOL_FILE.read_text(encoding='utf-8')
        self.assertEqual(server_text.count('robot->enableMotors();'), 1)
        self.assertIn('COMMAND_MOTOR_ENABLE', server_text)
        release_branch = server_text.split(
            'if (command.type == patrolbot::COMMAND_CHARGE_RELEASE)', 1)[1]
        release_branch = release_branch.split(
            '} else if (command.type == patrolbot::COMMAND_MOTOR_ENABLE)', 1)[0]
        self.assertIn('comInt(kUarcsChargeCommand, 0)', release_branch)
        self.assertIn('disableMotors()', release_branch)
        self.assertLess(
            release_branch.index('disableMotors()'),
            release_branch.index('comInt(kUarcsChargeCommand, 0)'))
        self.assertNotIn('setVel(', release_branch)
        self.assertNotIn('setRotVel(', release_branch)
        self.assertNotIn('enableMotors()', release_branch)
        self.assertNotIn('motors_must_be_disabled_for_release', server_text)
        self.assertIn('state.fault_flags != 0', server_text)
        self.assertIn('patrolbot::classifyDrive(', server_text)
        self.assertIn('fault_flags != 0', protocol_text)
        self.assertIn('DRIVE_ALLOWED_DOCK_DEPARTURE', protocol_text)

    def test_sbc_service_logging_is_compatible_with_systemd_229(self):
        unit_text = SERVER_UNIT_FILE.read_text(encoding='utf-8')
        dropin_text = SERVER_LOG_DROPIN_FILE.read_text(encoding='utf-8')
        self.assertIn(
            'ExecStart=/home/ros/patrolbot_hw_server/patrolbot_server '
            '-rh 127.0.0.1 -rrtp 7000', unit_text)
        self.assertIn('Restart=always', unit_text)
        self.assertIn('StandardOutput=journal', dropin_text)
        self.assertIn('StandardError=journal', dropin_text)
        self.assertNotIn('append:', dropin_text)

        journal_text = SERVER_JOURNALD_FILE.read_text(encoding='utf-8')
        self.assertIn('Storage=persistent', journal_text)
        self.assertIn('SystemMaxUse=128M', journal_text)
        self.assertIn('SystemKeepFree=2G', journal_text)
        self.assertIn('MaxRetentionSec=30day', journal_text)

    def test_sbc_root_maintenance_assets_fail_closed(self):
        chrony = (
            SERVER_MAINTENANCE_ROOT / 'chrony-makestep.directive'
        ).read_text(encoding='utf-8')
        sshd = (
            SERVER_MAINTENANCE_ROOT / 'sshd-hardening.txt'
        ).read_text(encoding='utf-8')
        guide = (
            SERVER_MAINTENANCE_ROOT / 'README.md'
        ).read_text(encoding='utf-8')

        self.assertEqual(chrony.count('makestep 1.0 -1'), 1)
        self.assertIn('PasswordAuthentication no', sshd)
        self.assertIn('ChallengeResponseAuthentication no', sshd)
        self.assertIn('PermitRootLogin no', sshd)
        self.assertIn('robot is stationary and motors are disabled', guide)
        self.assertIn('do not install it as a drop-in', guide)
        self.assertIn('does not close PB-AUDIT-022', guide)
        self.assertIn('not included as an executable change', guide)
        self.assertIn('Wait for the LCD to report `PC Power OFF`', guide)
        self.assertIn('An SSH disconnect by itself is not proof', guide)
        self.assertNotIn('chronyd -p', guide)

    def test_rear_sonar_is_a_fail_closed_collision_source(self):
        collision = self.params['collision_monitor']['ros__parameters']
        self.assertEqual(collision['observation_sources'], ['scan', 'sonar'])
        self.assertEqual(collision['source_timeout'], 0.5)
        self.assertEqual(collision['scan']['source_timeout'], 0.5)
        self.assertEqual(collision['sonar']['source_timeout'], 0.5)
        self.assertEqual(collision['sonar']['type'], 'pointcloud')
        self.assertEqual(collision['sonar']['topic'], '/sonar')
        self.assertEqual(collision['front_velocity_stop']['sources_names'], ['scan'])
        self.assertEqual(collision['front_velocity_stop']['min_points'], 8)
        self.assertEqual(collision['rear_velocity_stop']['sources_names'], ['sonar'])
        self.assertEqual(collision['rear_velocity_stop']['min_points'], 1)

    def test_dock_plugin_uses_typed_charge_state_not_battery_current(self):
        docking = self.params['docking_server']['ros__parameters']
        plugin = docking['charging_dock']
        self.assertEqual(
            plugin['plugin'],
            'patrolbot_docking_plugins::PatrolBotChargingDock')
        plugin_text = DOCK_PLUGIN_FILE.read_text(encoding='utf-8')
        self.assertIn('state_.charge_state', plugin_text)
        self.assertIn('"/battery"', plugin_text)
        self.assertNotIn('message->current', plugin_text)
        self.assertIn('"/patrolbot/charge_release"', plugin_text)

    @staticmethod
    def _select_subpolygon(polygon, linear, angular):
        """Mirror Jazzy's ordered first-match VelocityPolygon selection."""
        for name in polygon['velocity_polygons']:
            candidate = polygon[name]
            if (
                candidate['linear_min'] <= linear <= candidate['linear_max']
                and candidate['theta_min'] <= angular <= candidate['theta_max']
            ):
                return name
        return None

    def test_velocity_polygon_full_envelope_has_no_holes(self):
        collision = self.params['collision_monitor']['ros__parameters']
        for polygon_name in ('front_velocity_stop', 'rear_velocity_stop'):
            polygon = collision[polygon_name]
            # Exhaustive millimeter-per-second by centiradian-per-second sweep,
            # including combined arcs and the +/-0.01 boundary bands.
            for linear_mm in range(-260, 261):
                linear = linear_mm / 1000.0
                for angular_centirad in range(-100, 101):
                    angular = angular_centirad / 100.0
                    selected = self._select_subpolygon(polygon, linear, angular)
                    self.assertIsNotNone(
                        selected,
                        f'{polygon_name} hole at v={linear}, w={angular}',
                    )
                    if -0.01 <= linear <= 0.01:
                        expected = 'rotation_or_stopped'
                    elif linear > 0.01:
                        expected = 'forward'
                    else:
                        expected = 'reverse'
                    self.assertEqual(selected, expected)

    def test_recovery_trees_back_up_before_spin(self):
        tree_dir = PACKAGE_ROOT / 'config' / 'behavior_trees'
        for path in tree_dir.glob('*.xml'):
            root = ET.parse(path).getroot()
            round_robin = root.find('.//RoundRobin')
            self.assertIsNotNone(round_robin)
            actions = [child.tag for child in round_robin]
            self.assertLess(actions.index('BackUp'), actions.index('Spin'))
            backup = round_robin.find('BackUp')
            self.assertEqual(backup.attrib['backup_dist'], '0.10')
            self.assertEqual(backup.attrib['backup_speed'], '0.05')


if __name__ == '__main__':
    unittest.main()
