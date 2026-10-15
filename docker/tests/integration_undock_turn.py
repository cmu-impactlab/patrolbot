"""Installed bridge turn, real Nav2 Spin, and a raycast plant without AMCL.

Run one case per fresh network-none/private-IPC container. The fake SBC scripts
the hardware reverse; the subsequent turn must move through the real ROS chain.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


def loaded_nav2_libraries(proc=Path('/proc')):
    names = ('libcontroller_server_core.so', 'libbehavior_server_core.so',
             'libbt_navigator_core.so', 'libnav2_lifecycle_manager_core.so')
    found = {name: set() for name in names}
    for maps in proc.glob('[0-9]*/maps'):
        try:
            lines = maps.read_text().splitlines()
        except OSError:
            continue
        for line in lines:
            path = line.split()[-1]
            if Path(path).name in found:
                found[Path(path).name].add(path)
    return {name: sorted(paths) for name, paths in found.items()}


def validate_capture(capture):
    assert all(len(paths) == 1 and paths[0].startswith('/opt/patrolbot_nav2/install/')
               for paths in capture['native_libraries'].values()), 'unreviewed Nav2 library loaded'
    assert len(capture['native_libraries']) == 4, 'native Nav2 library evidence incomplete'
    assert capture['no_amcl_or_seed'], 'an estimator or operator seed appeared'
    assert capture['normal_goals_held'], 'ordinary navigation escaped the hold'
    assert capture['hardware_clear_handoff'], 'typed hardware clearance was not confirmed'
    assert capture['token_correlated'], 'turn did not consume the exact hardware goal/session'
    assert capture['replay_rejected'], 'spent or expired handoff was accepted again'
    assert capture['stopped'], 'external plant did not stop'
    assert capture['final_drive'] == [0., 0.], 'SBC did not receive a terminal zero'
    assert not capture['drive_allowed_at_end'], 'owned turn permission remained live'
    assert all(sample[1] == 0. and 0. <= sample[2] <= .700001
               for sample in capture['turn_drives']), 'turn contained linear or invalid angular drive'
    if capture['scenario'] in ('reach', 'manager'):
        assert capture['status'] == 4 and capture['result']['success']
        assert capture['result']['code'] == 0
        assert not capture['revocation_reason'], 'turn succeeded after a late safety revocation'
        assert capture['turn_drives'] and any(v[2] > 0. for v in capture['turn_drives'])
        assert abs(capture['plant_yaw_delta'] - math.pi) <= .12, 'plant did not turn 180 degrees'
        assert capture['plant_translation_m'] <= .01, 'the turn translated the plant'
        assert capture['active_scoped_state_seen'], 'fresh typed turn state never became active'
        assert capture['all_navigation_inactive'], 'borrowed lifecycle resources were not restored'
    elif capture['scenario'] == 'handoff-expiry':
        assert not capture['turn_drives'], 'an expired handoff moved the plant'
        assert 'expired' in capture['revocation_reason']
    elif capture['scenario'] == 'lifecycle-late':
        assert capture['status'] == 6 and not capture['result']['success']
        assert capture['unknown_mutation_held'] and capture['late_reply_received']
        assert not capture['turn_drives'], 'a timed-out lifecycle mutation authorized motion'
    else:
        assert capture['status'] in (5, 6) and not capture['result']['success']
        assert capture['fault_requested_while_live'] and capture['fault_recent_nonzero']
        assert capture['turn_drives'] and any(v[2] > 0. for v in capture['turn_drives'])
        assert capture['zero_after_fault_s'] <= .6, 'a fault did not promptly reach the SBC as zero'
        assert capture['all_navigation_inactive'], 'known borrowed lifecycle resources were not restored'
        if capture['scenario'] == 'reconnect':
            assert capture['session_changed']
            assert any(word in capture['revocation_reason']
                       for word in ('transport', 'session', 'disconnect'))
            assert all(v[1:] == [0., 0.] for v in capture['new_session_drives'])
        if capture['scenario'] == 'cancel':
            assert capture['status'] == 5 and capture['result']['code'] == 4
            assert capture['revocation_reason'] == 'operator canceled undock turn'
        if capture['scenario'] == 'foreign-publisher':
            assert capture['foreign_endpoint_count'] >= 2
            assert 'publisher' in capture['revocation_reason']
        if capture['scenario'] == 'source-expiry':
            assert capture['source_restored_after_revocation']
            assert 'source expired' in capture['revocation_reason']
            assert capture['no_nonzero_after_expiry'], 'expired source resumed or kept driving'
            assert capture['expiry_result_s'] <= 4., 'expiry relied on the 20s Spin timeout'


def main():
    assert os.environ.get('PATROLBOT_ISOLATED_TEST') == '1'
    assert {p.name for p in Path('/sys/class/net').iterdir()} == {'lo'}
    sys.path.insert(0, '/validation/docker/tests')
    sys.path.insert(0, '/validation/ros2_ws/src/patrolbot_bridge/test')
    from closed_loop_fixture import ClosedLoopFakeSbc
    from test_fake_sbc_integration import wait_until as fixture_wait
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from lifecycle_msgs.srv import ChangeState, GetState
    from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
    from patrolbot_bridge.bridge_node import PatrolBotBridge
    from patrolbot_bridge import undock_turn
    from patrolbot_interfaces.action import HardwareUndock, UndockTurn, Undock
    from patrolbot_interfaces.msg import BaseState
    from patrolbot_bridge.map_catalog import MapCatalog
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
    from rclpy.task import Future
    from rosidl_runtime_py.convert import message_to_ordereddict
    from unique_identifier_msgs.msg import UUID

    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=(
        'reach', 'cancel', 'handoff-expiry', 'lifecycle-late',
            'reconnect', 'foreign-publisher', 'source-expiry', 'manager'), default='reach')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    installed = Path(undock_turn.__file__).resolve()
    assert installed.is_relative_to('/opt/undock_test_ws/install'), installed
    started = time.monotonic()
    deadline = started + 120.
    capture = {'scenario': args.scenario, 'physical_acceptance': False,
               'hardware_reverse_scripted': True, 'hardware_reverse_motion_observed': False,
               'installed_core_file': str(installed),
               'installed_core_sha256': hashlib.sha256(installed.read_bytes()).hexdigest()}
    capture['nav2_prefixes'] = {name: get_package_prefix(name) for name in (
        'nav2_lifecycle_manager', 'nav2_controller', 'nav2_behaviors', 'nav2_bt_navigator')}
    assert all(Path(prefix).is_relative_to('/opt/patrolbot_nav2/install')
               for prefix in capture['nav2_prefixes'].values()), capture['nav2_prefixes']
    processes = []
    node = bridge = fake = executor = log = manager = None
    log_path = args.output.with_suffix('.log')
    status = result = None
    fault_at = None
    clients = []
    observers = {'/initialpose': [], '/amcl_pose': [], '/patrolbot/base_state': [], '/spin/_action/status': []}
    velocity = {topic: [] for topic in (
        '/input/nav_behavior', '/input/docking', '/cmd_vel_nav',
        '/cmd_vel_smoothed', '/input/navi', '/input/undock_turn',
        '/input/safety_controller', '/cmd_vel_out', '/cmd_vel')}
    late_reply = threading.Event()
    runtime_samples = []
    runtime = None

    def until(predicate, seconds=10.):
        remaining = deadline - time.monotonic()
        assert remaining > 0., '120-second case budget exhausted'
        return fixture_wait(predicate, timeout=min(seconds, remaining))

    def wait(future, seconds=10.):
        assert until(future.done, seconds), 'ROS future timed out'
        return future.result()

    def state(name):
        client = node.create_client(GetState, '/' + name + '/get_state')
        clients.append(client)
        assert client.wait_for_service(timeout_sec=2.), name
        return wait(client.call_async(GetState.Request()), 2.).current_state.id

    def navigation_inactive():
        return all(state(name) == 2 for name in (*undock_turn.NAVIGATION_NODES,
                                                undock_turn.LOCAL_NODES[0]))

    def publishers():
        return {topic: [dict(name=info.node_name, namespace=info.node_namespace,
                             type=info.topic_type)
                        for info in bridge.get_publishers_info_by_topic(topic)]
                for topic in undock_turn.PATH_PUBLISHERS}

    def stopped():
        samples = fake.snapshot()['samples']
        return len(samples) >= 5 and all(
            abs(s['velocity'][0]) < .001 and abs(s['velocity'][1]) < .001
            for s in samples[-5:])

    def goals_held():
        requests = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y = fake.world_pose[:2]
        pose.pose.orientation.w = 1.
        for kind, name in ((NavigateToPose, '/navigate_to_pose'),
                           (NavigateThroughPoses, '/navigate_through_poses')):
            client = ActionClient(node, kind, name)
            clients.append(client)
            assert client.wait_for_server(timeout_sec=2.), name
            goal = kind.Goal()
            if kind is NavigateToPose:
                goal.pose = pose
            else:
                goal.poses = [pose]
            handle = wait(client.send_goal_async(goal), 2.)
            requests.append(not handle.accepted)
        return all(requests) and not bridge._epoch.commands_allowed

    class TimedPlant(ClosedLoopFakeSbc):
        def __init__(self, *values, **options):
            self.drive_receipts = []
            super().__init__(*values, **options)

        def _reply(self, connection, line):
            super()._reply(connection, line)
            if line.startswith('DRIVE:'):
                with self._lock:
                    self.drive_receipts.append([
                        time.monotonic(), self.connections,
                        *map(float, line.split(':')[1:])])

    class DelayedReply:
        def __init__(self, client):
            self.client = client

        def call_async(self, request):
            actual = self.client.call_async(request)
            delayed = Future()

            def deliver():
                if not fixture_wait(actual.done, timeout=5.):
                    delayed.set_exception(RuntimeError('actual lifecycle request did not complete'))
                    return
                time.sleep(6.)
                delayed.set_result(actual.result())
                late_reply.set()

            threading.Thread(target=deliver, daemon=True).start()
            return delayed

    try:
        rclpy.init()
        node = Node('isolated_undock_turn')
        share = Path(get_package_share_directory('patrolbot_navigation'))
        catalog = MapCatalog(share / 'maps')
        entry = catalog.maps['cmuq-floor1']
        from geometry import MapRaster
        raster = MapRaster(entry['path'], 'cmuq-floor1', entry['revision'])
        margin = math.ceil(1.2 / raster.resolution)
        origin = None
        for gy in range(margin, raster.height - margin, 5):
            for gx in range(margin, raster.width - margin, 5):
                if all(raster.grid_cell(x, y) == 'free'
                       for y in range(gy-margin, gy+margin+1)
                       for x in range(gx-margin, gx+margin+1)):
                    lx, ly = (gx+.5)*raster.resolution, (gy+.5)*raster.resolution
                    yaw = raster.origin[2]
                    origin = (raster.origin[0]+math.cos(yaw)*lx-math.sin(yaw)*ly,
                              raster.origin[1]+math.sin(yaw)*lx+math.cos(yaw)*ly, yaw)
                    break
            if origin:
                break
        assert origin, 'canonical map lacks existing 2.4m test-scene clearance'
        fake = TimedPlant(entry['path'], 'cmuq-floor1', entry['revision'], origin, beams=181)
        with fake._lock:
            fake.charge_state = 2
            fake.dock_state = 'DOCKED_CONFIRMED'
            fake.motors_enabled = False
        bridge = PatrolBotBridge(parameter_overrides=[
            Parameter('server_ip', value='127.0.0.1'),
            Parameter('server_port', value=fake.port),
            Parameter('reconnect_delay', value=.1)])
        runtime = bridge._undock_turn
        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(node)
        executor.add_node(bridge)
        if args.scenario == 'manager':
            from patrolbot_dock_manager.dock_manager import DockManager
            manager = DockManager()
            executor.add_node(manager)
        def record_runtime():
            with bridge._epoch.lock:
                runtime_samples.append(dict(time=time.monotonic(), phase=runtime.phase,
                    hardware_id=(runtime.hardware_goal_id.hex() if runtime.hardware_goal_id else ''),
                    token_id=(runtime.token[0].hex() if runtime.token else ''),
                    spin_id=(runtime.spin_id.hex() if runtime.spin_id else ''),
                    drive_allowed=runtime.drive_allowed, reason=runtime.reason))
        node.create_timer(.02, record_runtime)
        threading.Thread(target=executor.spin, daemon=True).start()
        for topic, kind in (('/initialpose', PoseWithCovarianceStamped),
                            ('/amcl_pose', PoseWithCovarianceStamped),
                            ('/patrolbot/base_state', BaseState),
                            ('/spin/_action/status', GoalStatusArray)):
            qos = (qos_profile_action_status_default if 'status' in topic
                   else qos_profile_sensor_data if topic == '/amcl_pose' else 10)
            node.create_subscription(kind, topic,
                lambda message, topic=topic: observers[topic].append([
                    time.monotonic(), message_to_ordereddict(message)]), qos)
        for topic in velocity:
            node.create_subscription(Twist, topic,
                lambda message, topic=topic: velocity[topic].append([
                    time.monotonic(), message.linear.x, message.angular.z]), 10)
        log = log_path.open('w')
        commands = [
            ['ros2', 'run', 'rclcpp_components', 'component_container_isolated',
             '--ros-args', '-r', '__node:=nav2_container',
             '--params-file', str(share / 'config/nav2_params.yaml')],
            ['ros2', 'launch', 'patrolbot_navigation', 'patrolbot_navigation_launch.py',
             'use_composition:=True', 'container_name:=nav2_container',
             'params_file:=' + str(share / 'config/nav2_params.yaml')],
            ['ros2', 'launch', 'patrolbot-launch', 'bringup.xml'],
            ['ros2', 'run', 'patrolbot_navigation', 'patrolbot_safety_watchdog.py'],
            ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
             '0.037', '0', '0.2', '0', '0', '3.14159', 'base_link', 'laser_frame']]
        for command in commands:
            processes.append(subprocess.Popen(command, stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True))
        assert until(lambda: runtime.idle_at is not None
                     and not runtime.hardware_problem(charging=True, stationary=True), 75.), {
                         'phase': runtime.phase, 'problem': runtime.hardware_problem(charging=True),
                         'epoch': bridge._epoch.status_stage}
        capture['native_libraries'] = loaded_nav2_libraries()
        assert all(len(paths) == 1 and paths[0].startswith('/opt/patrolbot_nav2/install/')
                   for paths in capture['native_libraries'].values()), capture['native_libraries']
        assert navigation_inactive()
        assert goals_held()
        capture['publisher_contract'] = publishers()
        assert until(lambda: undock_turn.exclusive_publishers(bridge), 5.), publishers()
        capture['no_amcl_or_seed'] = not any(observers[t] for t in ('/initialpose', '/amcl_pose'))
        names = {name for name, _namespace in node.get_node_names_and_namespaces()}
        assert 'amcl' not in names and 'lifecycle_manager_localization' not in names
        if args.scenario == 'manager':
            assert until(lambda: manager._base_state is not None
                         and manager._snapshot(for_undock=True).hardware_state_valid, 3.)
            outer = ActionClient(node, Undock, '/patrolbot/undock')
            clients.append(outer)
            assert outer.wait_for_server(timeout_sec=2.)
            departure = Undock.Goal()
            departure.operator_authorized = True
            departure.dock_id = manager.get_parameter('dock_id').value
            departure.reverse_distance = manager.get_parameter('undock_clearance_distance').value
            departure.reverse_speed = manager.get_parameter('undock_reverse_speed').value
            capture['manager_request'] = message_to_ordereddict(departure)
            initial_plant = fake.snapshot()['world_pose']
            turn_at = time.monotonic()
            outer_handle = wait(outer.send_goal_async(departure), 2.)
            assert outer_handle.accepted, manager._snapshot(for_undock=True)
            terminal = outer_handle.get_result_async()
            assert until(lambda: runtime.spin_id is not None or terminal.done(), 15.)
            assert not terminal.done(), terminal.result()
            capture['owned_spin_goal_id'] = runtime.spin_id.hex()
            assert until(lambda: any(v[0] >= turn_at and v[3] > 0.
                         for v in fake.drive_receipts) or terminal.done(), 12.)
            assert not terminal.done(), terminal.result()
            capture['normal_goals_held_during_turn'] = goals_held()
            wrapped = wait(terminal, 35.)
            status, result = wrapped.status, message_to_ordereddict(wrapped.result)
            capture['hardware_clear_handoff'] = bool(wrapped.result.physically_clear
                and wrapped.result.final_dock_state == 'CLEAR_CONFIRMED'
                and wrapped.result.final_motors_enabled
                and abs(wrapped.result.distance_traveled-.53) < .001)
            request = UndockTurn.Goal()
            request.hardware_goal_id = manager._hardware_goal_id
            request.session_generation = bridge._session_generation
            capture['hardware_goal_id'] = bytes(request.hardware_goal_id.uuid).hex()
            capture['token_correlated'] = any(
                sample['token_id'] == capture['hardware_goal_id']
                and sample['hardware_id'] == capture['hardware_goal_id']
                for sample in runtime_samples)
            turn = ActionClient(node, UndockTurn, '/patrolbot/undock_turn')
            clients.append(turn)
            assert turn.wait_for_server(timeout_sec=2.)
        else:
            hardware = ActionClient(node, HardwareUndock, '/patrolbot/hardware_undock')
            clients.append(hardware)
            assert hardware.wait_for_server(timeout_sec=2.)
            reverse = HardwareUndock.Goal()
            reverse.clearance_distance = .53
            reverse.reverse_speed = .1
            reverse.maximum_duration = 45.
            handoff = wait(hardware.send_goal_async(reverse), 3.)
            assert handoff.accepted, {'reason': runtime.reason, 'problem': runtime.hardware_problem(charging=True)}
            wrapped = wait(handoff.get_result_async(), 15.)
            capture['hardware_result'] = message_to_ordereddict(wrapped.result)
            capture['hardware_goal_id'] = bytes(handoff.goal_id.uuid).hex()
            capture['hardware_clear_handoff'] = bool(
                wrapped.status == 4 and wrapped.result.success and wrapped.result.code == 0
                and wrapped.result.physically_clear and wrapped.result.motors_enabled
                and wrapped.result.final_dock_state == 'CLEAR_CONFIRMED')
            assert capture['hardware_clear_handoff'], capture['hardware_result']
            assert until(lambda: not runtime.hardware_problem(turn=True, stationary=True), 2.)
            turn = ActionClient(node, UndockTurn, '/patrolbot/undock_turn')
            clients.append(turn)
            assert turn.wait_for_server(timeout_sec=2.)
            request = UndockTurn.Goal()
            request.hardware_goal_id = UUID(uuid=list(handoff.goal_id.uuid))
            request.session_generation = wrapped.result.session_generation
            capture['token_correlated'] = bool(runtime.token
                and runtime.token[0] == bytes(request.hardware_goal_id.uuid)
                and runtime.token[1] == request.session_generation)
            assert capture['token_correlated']
            turn_at = time.monotonic()
            initial_plant = fake.snapshot()['world_pose']
            if args.scenario == 'handoff-expiry':
                assert until(lambda: bool(runtime.reason), 10.)
            else:
                if args.scenario == 'lifecycle-late':
                    key = undock_turn.LOCAL_NODES[0]
                    runtime.changes[key] = DelayedReply(runtime.changes[key])
                handle = wait(turn.send_goal_async(request), 2.)
                assert handle.accepted, runtime.reason
                terminal = handle.get_result_async()
                if args.scenario != 'lifecycle-late':
                    assert until(lambda: any(v[0] >= turn_at and v[3] > 0.
                                 for v in fake.drive_receipts), 12.), runtime.reason
                    capture['owned_spin_goal_id'] = runtime.spin_id.hex()
                    capture['normal_goals_held_during_turn'] = goals_held()
                if args.scenario in ('cancel', 'reconnect', 'foreign-publisher', 'source-expiry'):
                    fault_at = time.monotonic()
                    capture['fault_requested_at'] = fault_at
                    capture['fault_requested_while_live'] = runtime.drive_allowed
                    capture['fault_recent_nonzero'] = any(
                        0. <= fault_at - v[0] <= .25 and v[3] > 0.
                        for v in fake.drive_receipts)
                    assert capture['fault_requested_while_live'], runtime.reason
                    assert capture['fault_recent_nonzero'], 'no fresh actual drive before fault'
                    if args.scenario == 'cancel':
                        wait(handle.cancel_goal_async(), 2.)
                    elif args.scenario == 'reconnect':
                        capture['initial_session'] = bridge._session_generation
                        fake.drop_connection()
                    elif args.scenario == 'foreign-publisher':
                        foreign = Node('velocity_smoother')
                        executor.add_node(foreign)
                        foreign_pub = foreign.create_publisher(Twist, '/cmd_vel_smoothed', 10)
                        assert until(lambda: len(node.get_publishers_info_by_topic(
                            '/cmd_vel_smoothed')) == 2, 2.)
                        capture['foreign_endpoint_count'] = len(
                            node.get_publishers_info_by_topic('/cmd_vel_smoothed'))
                    else:
                        capture['source_expiry_at'] = runtime.behavior_at + .25
                        subscriptions = [sub for sub in bridge.subscriptions
                                         if sub.topic_name == '/input/nav_behavior']
                        assert len(subscriptions) == 1
                        assert bridge.destroy_subscription(subscriptions[0])
                        assert until(lambda: bool(runtime.reason)
                                     and not runtime.drive_allowed, .75)
                        restored_subscription = bridge.create_subscription(
                            Twist, '/input/nav_behavior', runtime.behavior, 10,
                            callback_group=bridge._callbacks)
                        capture['source_restored_after_revocation'] = True
                wrapped = wait(terminal, 30.)
                status, result = wrapped.status, message_to_ordereddict(wrapped.result)
                if args.scenario == 'source-expiry':
                    capture['expiry_result_s'] = time.monotonic() - capture['source_expiry_at']
                    capture['no_nonzero_after_expiry'] = all(
                        v[2:] == [0., 0.] for v in fake.drive_receipts
                        if v[0] >= capture['source_expiry_at'])
                if args.scenario == 'lifecycle-late':
                    assert until(late_reply.is_set, 4.)
                    capture['unknown_mutation_held'] = bool(runtime.unknown
                        and bridge._epoch.undock_hold and not runtime.drive_allowed
                        and not bridge._epoch.commands_allowed)
                    capture['late_reply_received'] = late_reply.is_set()
                if args.scenario == 'reconnect':
                    assert until(lambda: bridge._session_generation > capture['initial_session'], 3.)
                    capture['session_changed'] = True
                    capture['new_session_drives'] = [v[:1] + v[2:] for v in fake.drive_receipts
                        if v[1] > capture['initial_session']]
        assert until(stopped, 2.)
        assert until(lambda: fake.drive_receipts and fake.drive_receipts[-1][2:] == [0., 0.], 2.)
        retry = wait(turn.send_goal_async(request), 2.)
        capture['replay_rejected'] = not retry.accepted
        capture['normal_goals_held'] = (goals_held()
            and capture.get('normal_goals_held_during_turn', True))
        capture['all_navigation_inactive'] = (navigation_inactive()
            if args.scenario != 'lifecycle-late' else False)
        capture['drive_allowed_at_end'] = runtime.drive_allowed
        capture['revocation_reason'] = runtime.reason
        capture['stopped'] = stopped()
        capture['final_drive'] = fake.drive_receipts[-1][2:]
        capture['turn_drives'] = [v[:1] + v[2:] for v in fake.drive_receipts
            if v[0] >= turn_at and (v[2] != 0. or v[3] != 0.)]
        plant = fake.snapshot()['world_pose']
        capture['plant_yaw_delta'] = math.remainder(plant[2] - initial_plant[2], 2.*math.pi)
        if capture['plant_yaw_delta'] < 0.:
            capture['plant_yaw_delta'] += 2.*math.pi
        capture['plant_translation_m'] = math.hypot(plant[0]-initial_plant[0], plant[1]-initial_plant[1])
        capture['active_scoped_state_seen'] = any(
            data['undock_turn_active']
            and data['undock_turn_goal_id'] == capture.get('owned_spin_goal_id')
            and data['session_generation'] == request.session_generation
            and data['odom_epoch'] == fake.odom_epoch
            and data['localization_recovery_required'] and data['odom_epoch_valid']
            and data['hardware_state_valid'] and data['motors_enabled']
            and data['dock_state_valid'] and data['dock_state'] == 'CLEAR_CONFIRMED'
            for _stamp, data in observers['/patrolbot/base_state'])
        if fault_at is not None:
            zeros = [v for v in fake.drive_receipts if v[0] >= fault_at and v[2:] == [0., 0.]]
            assert zeros, 'no actual zero after fault'
            capture['zero_after_fault_s'] = zeros[0][0] - fault_at
        capture['status'] = status
        capture['result'] = result
        capture['no_amcl_or_seed'] = not any(observers[t] for t in ('/initialpose', '/amcl_pose'))
        validate_capture(capture)
        capture['passed'] = True
    except Exception as error:
        capture['passed'] = False
        capture['failure'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        original_error = sys.exc_info()[0] is not None
        errors = []

        def clean(name, operation):
            try:
                operation()
            except Exception as error:
                errors.append(name + ': ' + str(error))

        if runtime:
            capture['final_runtime'] = {'phase': runtime.phase, 'reason': runtime.reason,
                'unknown': runtime.unknown, 'drive_allowed': runtime.drive_allowed}
            clean('close test lease', lambda: runtime.revoke('isolated test ended'))
        if fake:
            capture['plant'] = fake.snapshot()
            capture['drive_receipts'] = list(fake.drive_receipts)
        capture['observers'] = observers
        capture['velocity_stages'] = velocity
        capture['runtime_samples'] = runtime_samples
        if node and executor:
            def deactivate_late_fixture():
                if args.scenario == 'lifecycle-late' and late_reply.is_set():
                    # The deliberately delayed response is now known complete.
                    # Only test teardown deactivates it; core permission remains held.
                    name = undock_turn.LOCAL_NODES[0]
                    if state(name) == 3:
                        client = node.create_client(ChangeState, '/' + name + '/change_state')
                        request = ChangeState.Request()
                        request.transition.id = 4
                        assert client.wait_for_service(timeout_sec=2.)
                        assert wait(client.call_async(request), 5.).success
            clean('late lifecycle fixture shutdown', deactivate_late_fixture)
            if args.scenario == 'foreign-publisher' and 'foreign' in locals():
                clean('foreign test publisher shutdown', foreign.destroy_node)
            clean('executor shutdown', lambda: executor.shutdown(timeout_sec=5.))
        if manager:
            clean('manager shutdown', manager.destroy_node)
        if bridge:
            clean('bridge shutdown', bridge.destroy_node)
        if fake:
            clean('plant shutdown', fake.close)
        if node:
            clean('observer shutdown', node.destroy_node)
        if rclpy.ok():
            clean('ROS shutdown', rclpy.try_shutdown)
        for process in reversed(processes):
            if process.poll() is None:
                def stop(process=process):
                    if process.args[1] == 'run':
                        os.killpg(process.pid, signal.SIGINT)
                    else:
                        process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=15.)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                        raise AssertionError('launch required forced termination')
                clean('launched process shutdown', stop)
        capture['process_exit_codes'] = [process.returncode for process in processes]
        if any(code != 0 for code in capture['process_exit_codes']):
            errors.append('launched process exited nonzero: ' + str(capture['process_exit_codes']))
        if log:
            log.close()
        if log_path.exists():
            capture['teardown_errors'] = [line for line in log_path.read_text().splitlines()
                if 'process has died' in line]
            errors.extend(capture['teardown_errors'])
        capture['total_elapsed_s'] = time.monotonic() - started
        if capture['total_elapsed_s'] > 120.:
            errors.append('120-second budget exceeded including teardown')
        if errors:
            capture['cleanup_errors'] = errors
            capture['passed'] = False
        try:
            args.output.write_text(json.dumps(capture, indent=2) + '\n')
        except OSError as error:
            errors.append('evidence write: ' + str(error))
            capture['passed'] = False
            capture['cleanup_errors'] = errors
            print(json.dumps(capture), flush=True)
        print(json.dumps({key: capture.get(key) for key in (
            'scenario', 'passed', 'failure', 'status', 'result', 'final_runtime',
            'plant_yaw_delta', 'zero_after_fault_s', 'total_elapsed_s', 'cleanup_errors')}), flush=True)
        if errors and not original_error:
            raise AssertionError('; '.join(errors))


if __name__ == '__main__':
    main()
