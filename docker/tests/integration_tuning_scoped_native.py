#!/usr/bin/env python3
"""Network-none native 1.3.12 input, manager and consumed-cache regressions.

No hardware, odometry, map, pose, motor, host network or production state.
Action probe goals use nonmoving fixture callbacks only. This is NOT complete
all-action/no-replay acceptance: manager-only evidence must still say false.
"""
import json
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

if os.environ.get("PATROLBOT_ISOLATED_TEST") != "1":
    raise SystemExit("Requires explicit isolated test environment")
if not os.environ.get("ROS_DOMAIN_ID") or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise SystemExit("Requires isolated ROS domain and localhost-only discovery")

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from lifecycle_msgs.srv import ChangeState, GetState
from patrolbot_interfaces.srv import TuningMaintenance, TuningTransition
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_srvs.srv import SetBool, Trigger


class Sibling(LifecycleNode):
    """Unplanned sibling under the real C++ manager, counted lifecycle callbacks."""
    def __init__(self):
        super().__init__("planner_server")
        self.transitions = []

    def on_configure(self, state):
        self.transitions.append("configure")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.transitions.append("activate")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.transitions.append("deactivate")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.transitions.append("cleanup")
        return TransitionCallbackReturn.SUCCESS


class FailedChild(LifecycleNode):
    def __init__(self):
        super().__init__('amcl')

    def on_configure(self, state):
        return TransitionCallbackReturn.FAILURE


class Fixture:
    def __init__(self):
        self.node = rclpy.create_node("scoped_native_test")
        self.sibling = Sibling()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.sibling)
        self.failed_child = FailedChild()
        self.executor.add_node(self.failed_child)
        self.children = []
        self.clients = {}
        # Retained clients exist before native processes start, matching the
        # production transport. Creating a brand-new reply reader immediately
        # before a mutation can race DDS endpoint discovery independently of
        # service-name readiness. Never resend a mutation to hide that race.
        for manager in ('scoped_manager', 'controller_scoped_manager', 'fault_scoped_manager', 'navigator_scoped_manager'):
            for kind, suffix in ((ManageLifecycleNodes, 'manage_nodes'), (Trigger, 'is_active'),
                                 (TuningMaintenance, 'tuning_maintenance'), (TuningTransition, 'tuning_transition')):
                name = '/' + manager + '/' + suffix
                self.clients[kind, name] = self.node.create_client(kind, name)
        for kind, suffix in ((ChangeState, 'change_state'), (GetState, 'get_state'),
                             (SetParametersAtomically, 'set_parameters_atomically'), (Trigger, 'tuning_cache')):
            name = '/hot_amcl_probe/' + suffix
            self.clients[kind, name] = self.node.create_client(kind, name)
        # Match the retained production client. Creating a new reply reader
        # immediately before the first acquire can lose its first DDS response.
        name = '/bt_navigator/tuning_maintenance'
        self.clients[TuningMaintenance, name] = self.node.create_client(TuningMaintenance, name)
        self.manager = "/scoped_manager"
        self.ready_managers = set()
        self.directory = tempfile.TemporaryDirectory(prefix="scoped-native-")

    def spawn(self, name, command):
        output = open(Path(self.directory.name) / (name + ".log"), "w+")
        process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        self.children.append((name, process, output))

    def call(self, service_type, name, request=None, timeout=15):
        key = (service_type, name)
        if key not in self.clients:
            self.clients[key] = self.node.create_client(service_type, name)
        client = self.clients[key]
        deadline = time.monotonic() + timeout
        while not client.wait_for_service(timeout_sec=0.05):
            self.executor.spin_once(timeout_sec=0.01)
            assert time.monotonic() < deadline, f"Service unavailable: {name}"
        future = client.call_async(request or service_type.Request())
        while not future.done():
            self.executor.spin_once(timeout_sec=0.02)
            assert time.monotonic() < deadline, f"Service timeout (never retried): {name}"
        return future.result()

    def manage(self, command):
        self.manager_ready()
        return self.call(ManageLifecycleNodes, self.manager + "/manage_nodes",
                         ManageLifecycleNodes.Request(command=command)).success

    def lease(self, operation, job="native-job-1", digest="a" * 64, plan="b" * 64):
        self.manager_ready()
        return self.call(TuningMaintenance, self.manager + "/tuning_maintenance",
                         TuningMaintenance.Request(operation=operation, job_id=job,
                                                    digest=digest, plan_hash=plan))

    def manager_ready(self):
        if self.manager not in self.ready_managers:
            # Read-only startup barrier before any mutating manager request;
            # service discovery alone predates its callback-thread readiness.
            # Discard/re-probe only READS during discovery. No transition or
            # maintenance request is ever retried after an ambiguous response.
            self.reply_ready(Trigger, self.manager + '/is_active', Trigger.Request())
            self.ready_managers.add(self.manager)

    def reply_ready(self, kind, name, request):
        # Fixed read-only discovery probes only. Mutations use call(), which
        # neither retries nor treats a missing reply as a definitive refusal.
        assert kind is Trigger or (kind is TuningMaintenance and request.operation == 'inspect')
        key = (kind, name)
        if key not in self.clients:
            self.clients[key] = self.node.create_client(kind, name)
        client = self.clients[key]
        deadline = time.monotonic() + 15.
        while True:
            assert time.monotonic() < deadline, f'Read-only startup unavailable: {name}'
            if not client.wait_for_service(timeout_sec=.05):
                self.executor.spin_once(timeout_sec=.01)
                continue
            future = client.call_async(request)
            probe_end = min(deadline, time.monotonic() + .5)
            while not future.done() and time.monotonic() < probe_end:
                self.executor.spin_once(timeout_sec=.01)
            if future.done() and future.result() is not None:
                return future.result()
            client.remove_pending_request(future)
            future.cancel()

    def transition(self, node, transition, state, job="native-job-1"):
        return self.call(TuningTransition, self.manager + "/tuning_transition",
                         TuningTransition.Request(job_id=job, digest="a" * 64,
                                                  plan_hash="b" * 64, node_name=node,
                                                  transition_id=transition, expected_state=state))

    def cache(self, name):
        result = self.call(Trigger, name)
        assert result.success, result.message
        return json.loads(result.message)["values"]

    def set(self, node, values):
        parameters = []
        for name, value in values.items():
            if isinstance(value, list):
                data = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                      double_array_value=value)
            elif type(value) is int:
                data = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
            else:
                data = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
            parameters.append(Parameter(name=name, value=data))
        response = self.call(SetParametersAtomically, f"/{node}/set_parameters_atomically",
                             SetParametersAtomically.Request(parameters=parameters))
        assert response.result.successful, response.result.reason

    def close(self):
        failures = []
        for name, process, output in reversed(self.children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                code = process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
                code = -signal.SIGKILL
            if code not in (0, -signal.SIGINT):
                failures.append((name, code))
            output.flush()
            output.seek(0)
            print(f"--- {name} ({code}) ---\n{output.read()}")
            output.close()
        self.executor.shutdown()
        self.node.destroy_node()
        self.sibling.destroy_node()
        self.failed_child.destroy_node()
        self.directory.cleanup()
        assert not failures, f"Unclean native children: {failures}"


def test_native(f):
    config = Path(f.directory.name) / "fixture.yaml"
    config.write_text("""
velocity_smoother:
  ros__parameters:
    feedback: OPEN_LOOP
    max_velocity: [0.3, 0.0, 0.7]
    min_velocity: [-0.3, 0.0, -0.7]
behavior_server:
  ros__parameters:
    behavior_plugins: [spin]
    spin.plugin: nav2_behaviors::Spin
    max_rotational_vel: 0.7
    min_rotational_vel: 0.4
scoped_manager:
  ros__parameters:
    node_names: [velocity_smoother, behavior_server, planner_server]
    autostart: false
    bond_timeout: 0.0
controller_server:
  ros__parameters:
    controller_plugins: [FollowPath]
    FollowPath.plugin: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
    FollowPath.desired_linear_vel: 0.25
    FollowPath.rotate_to_heading_angular_vel: 0.7
    FollowPath.regulated_linear_scaling_min_speed: 0.1
controller_scoped_manager:
  ros__parameters:
    node_names: [controller_server]
    autostart: false
    bond_timeout: 0.0
fault_scoped_manager:
  ros__parameters:
    node_names: [amcl]
    autostart: false
    bond_timeout: 0.0
navigator_scoped_manager:
  ros__parameters:
    node_names: [bt_navigator]
    autostart: false
    bond_timeout: 0.0
hot_amcl_probe:
  ros__parameters:
    set_initial_pose: false
    always_reset_initial_pose: true
    alpha1: 0.05
    alpha2: 0.05
    alpha3: 0.05
    alpha4: 0.05
    update_min_d: 0.1
    update_min_a: 0.1
    min_particles: 500
    max_particles: 2000
    max_beams: 60
    sigma_hit: 0.2
    z_hit: 0.5
    z_rand: 0.5
""")
    for package, executable in (("nav2_velocity_smoother", "velocity_smoother"),
                                ("nav2_behaviors", "behavior_server")):
        f.spawn(executable, ["ros2", "run", package, executable,
                             "--ros-args", "--params-file", str(config)])
    f.spawn("manager", ["ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager",
                        "--ros-args", "-r", "__node:=scoped_manager", "--params-file", str(config)])
    f.spawn("rpp", ["/opt/scoped-tests/probe-build/cache_probe"])
    # Service discovery precedes the manager's retained callback-thread startup.
    # Read-only readiness must complete before the first mutating request.
    f.manager_ready()
    assert f.manage(ManageLifecycleNodes.Request.CONFIGURE)
    assert f.sibling.transitions == ["configure"]
    acquired = f.lease("acquire")
    assert acquired.accepted, acquired.detail
    evidence = json.loads(acquired.evidence_json)
    assert evidence["held"] and not evidence["all_action_paths_excluded"]
    assert set(evidence["states"].values()) == {2}
    assert f.lease("acquire").accepted  # Same-owner acquire is idempotent.
    assert not f.lease("acquire", job="foreign").accepted
    assert not f.lease("inspect", digest="c" * 64).accepted
    assert not f.lease("release", plan="c" * 64).accepted
    assert not f.lease("release", job="foreign").accepted
    assert not f.lease("acquire", job='"invalid').accepted
    assert not f.manage(ManageLifecycleNodes.Request.RESUME)
    assert not f.manage(ManageLifecycleNodes.Request.CLEANUP)
    assert not f.transition("planner_server", 2, 2).accepted  # Managed, not allowlisted.
    assert not f.transition("amcl", 2, 2).accepted  # Allowlisted, not managed.
    assert not f.transition("velocity_smoother", 3, 2).accepted  # Never activate prior inactive.
    assert not f.transition("velocity_smoother", 2, 1).accepted  # Wrong expected source state.
    assert not f.transition("velocity_smoother", 5, 2).accepted  # No shutdown escape hatch.
    assert not f.transition("velocity_smoother", 2, 2, job="foreign").accepted
    assert f.cache("/velocity_smoother/tuning_cache")["max_velocity"] == [0.3, 0.0, 0.7]

    # Inactive SetParameters succeeds but the actual smoother cache stays OLD.
    f.set("velocity_smoother", {"max_velocity": [0.2, 0.0, 0.5],
                                "min_velocity": [-0.2, 0.0, -0.5]})
    assert f.cache("/velocity_smoother/tuning_cache")["max_velocity"] == [0.3, 0.0, 0.7]
    assert f.transition("velocity_smoother", 2, 2).accepted
    assert not f.lease("release").accepted  # Cannot drop ownership mid-cleanup.
    assert f.transition("velocity_smoother", 1, 1).accepted
    assert f.cache("/velocity_smoother/tuning_cache")["max_velocity"] == [0.2, 0.0, 0.5]
    assert f.cache("/velocity_smoother/tuning_cache")["min_velocity"] == [-0.2, 0.0, -0.5]
    assert f.sibling.transitions == ["configure"]

    # Spin is constructor-only; readback must expose old execution values until L.
    assert f.cache("/behavior_server/spin/tuning_cache")["max_rotational_vel"] == 0.7
    f.set("behavior_server", {"max_rotational_vel": 0.5, "min_rotational_vel": 0.3})
    assert f.cache("/behavior_server/spin/tuning_cache")["max_rotational_vel"] == 0.7
    assert f.transition("behavior_server", 2, 2).accepted
    assert f.transition("behavior_server", 1, 1).accepted
    assert f.cache("/behavior_server/spin/tuning_cache") == {
        "max_rotational_vel": 0.5, "min_rotational_vel": 0.3}

    rpp = "/controller_cache_probe/FollowPath/tuning_cache"
    assert f.cache(rpp)["approach_velocity_scaling_dist"] == 0.6
    f.set("controller_cache_probe", {"FollowPath.approach_velocity_scaling_dist": 0.8,
                                    "FollowPath.lookahead_dist": 0.7})
    cached = f.cache(rpp)
    assert cached["approach_velocity_scaling_dist"] == 0.6
    assert cached["lookahead_dist"] == 0.7  # Real dynamic callback consumed it.
    assert f.call(Trigger, "/controller_cache_probe/reconstruct").success
    assert f.cache(rpp)["approach_velocity_scaling_dist"] == 0.8
    assert f.sibling.transitions == ["configure"]
    assert f.lease("inspect").accepted
    assert f.lease("release").accepted
    assert f.lease("release").accepted
    assert not f.lease("acquire").accepted  # A completed job cannot reacquire.

    # Prior ACTIVE may be returned ACTIVE, or left INACTIVE without seed/TF/odom.
    assert f.manage(ManageLifecycleNodes.Request.RESUME)
    assert f.sibling.transitions == ["configure", "activate"]
    assert f.lease("acquire", job="native-job-2").accepted
    assert f.transition("velocity_smoother", 4, 3, job="native-job-2").accepted
    assert f.transition("velocity_smoother", 2, 2, job="native-job-2").accepted
    assert f.transition("velocity_smoother", 1, 1, job="native-job-2").accepted
    assert f.transition("velocity_smoother", 3, 2, job="native-job-2").accepted
    assert f.sibling.transitions == ["configure", "activate"]
    assert f.lease("release", job="native-job-2").accepted

    # Real ControllerServer cleanup/configure, not only the isolated handler probe.
    f.spawn("controller", ["ros2", "run", "nav2_controller", "controller_server",
                           "--ros-args", "--params-file", str(config)])
    f.spawn("controller_manager", ["ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager",
                                   "--ros-args", "-r", "__node:=controller_scoped_manager",
                                   "--params-file", str(config)])
    f.manager = "/controller_scoped_manager"
    assert f.manage(ManageLifecycleNodes.Request.CONFIGURE)
    assert f.lease("acquire").accepted
    actual_rpp = "/controller_server/FollowPath/tuning_cache"
    assert f.cache(actual_rpp)["approach_velocity_scaling_dist"] == 0.6
    f.set("controller_server", {"FollowPath.approach_velocity_scaling_dist": 0.8})
    assert f.cache(actual_rpp)["approach_velocity_scaling_dist"] == 0.6
    assert f.transition("controller_server", 2, 2).accepted
    assert f.transition("controller_server", 1, 1).accepted
    assert f.cache(actual_rpp)["approach_velocity_scaling_dist"] == 0.8
    assert f.lease("release").accepted

    # A failed native-manager transition keeps its exact token, never auto-retries.
    # The deliberately failing child is a local lifecycle fixture, not real AMCL.
    f.spawn("fault_manager", ["ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager",
                              "--ros-args", "-r", "__node:=fault_scoped_manager",
                              "--params-file", str(config)])
    f.manager = "/fault_scoped_manager"
    assert f.lease("acquire").accepted
    failed = f.transition("amcl", 1, 1)
    assert not failed.accepted and failed.detail == "transition_outcome_unknown"
    inspected = f.lease("inspect")
    assert not inspected.accepted
    fault_evidence = json.loads(inspected.evidence_json)
    assert fault_evidence["held"] and fault_evidence["faulted"]
    assert not f.lease("release").accepted
    assert not f.manage(ManageLifecycleNodes.Request.CONFIGURE)

    # Actual navigator lease endpoint exists even unconfigured. The muxer token
    # outlives plugin cleanup and is separate from the pre-existing Boolean hold.
    f.spawn("navigator", ["ros2", "run", "nav2_bt_navigator", "bt_navigator"])
    f.spawn("navigator_manager", ["ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager",
                                  "--ros-args", "-r", "__node:=navigator_scoped_manager",
                                  "--params-file", str(config)])
    def navlease(operation, job="nav-job"):
        return f.call(TuningMaintenance, "/bt_navigator/tuning_maintenance",
                      TuningMaintenance.Request(operation=operation, job_id=job,
                                                digest="a" * 64, plan_hash="b" * 64))
    # Reuse the manager's read-only discovery barrier. Never repeat an acquire
    # to compensate for service-name readiness predating the DDS reply reader.
    observed = f.reply_ready(TuningMaintenance, '/bt_navigator/tuning_maintenance',
        TuningMaintenance.Request(operation='inspect', job_id='nav-job', digest='a' * 64, plan_hash='b' * 64))
    assert not observed.accepted
    assert navlease("acquire").accepted
    assert not navlease("release", job="foreign").accepted
    f.manager = "/navigator_scoped_manager"
    assert f.manage(ManageLifecycleNodes.Request.CONFIGURE)
    assert f.call(SetBool, "/bt_navigator/tuning_hold", SetBool.Request(data=False)).success
    assert json.loads(navlease("inspect").evidence_json)["held"]
    assert f.manage(ManageLifecycleNodes.Request.CLEANUP)
    assert json.loads(navlease("inspect").evidence_json)["held"]
    assert f.manage(ManageLifecycleNodes.Request.CONFIGURE)
    assert f.call(SetBool, "/bt_navigator/tuning_hold", SetBool.Request(data=True)).success
    released_nav = navlease("release")
    assert released_nav.accepted and json.loads(released_nav.evidence_json)["legacy_hold"]

    # Source-audited AMCL hot fields consume a coherent active batch without a
    # particle-filter reset, map, odometry, localization or automatic seed.
    f.spawn('hot_amcl_probe', ['ros2', 'run', 'nav2_amcl', 'amcl', '--ros-args',
        '-r', '__node:=hot_amcl_probe', '--params-file', str(config)])
    def amcl_transition(transition):
        request = ChangeState.Request()
        request.transition.id = transition
        assert f.call(ChangeState, '/hot_amcl_probe/change_state', request).success
    assert f.call(GetState, '/hot_amcl_probe/get_state').current_state.id == 1
    amcl_transition(1)
    initial = f.call(Trigger, '/hot_amcl_probe/tuning_cache')
    assert initial.success
    initial = json.loads(initial.message)
    assert not initial['initial_pose_known']
    amcl_transition(3)
    changes = {'alpha1': .06, 'alpha2': .07, 'alpha3': .08, 'alpha4': .09,
               'update_min_d': .2, 'update_min_a': .15, 'max_beams': 64,
               'sigma_hit': .25, 'z_hit': .6, 'z_rand': .4}
    f.set('hot_amcl_probe', changes)
    consumed = f.call(Trigger, '/hot_amcl_probe/tuning_cache')
    assert consumed.success
    consumed = json.loads(consumed.message)
    assert all(consumed['values'][key] == value for key, value in changes.items())
    assert consumed['filter_generation'] == initial['filter_generation']
    assert consumed['motion_generation'] > initial['motion_generation']
    assert consumed['laser_generation'] > initial['laser_generation']
    assert not consumed['initial_pose_known']
    amcl_transition(4)
    f.set('hot_amcl_probe', {'alpha1': .12})
    # Inactive setter accepts the value but cannot refresh the removed callback.
    # The new policy must therefore review one complete Nav2 restart, not claim
    # that this successful SetParameters already changed execution.
    assert f.cache('/hot_amcl_probe/tuning_cache')['alpha1'] == .06
    print("PASS: native manager/cache regressions, active AMCL hot fields and inactive callback boundary")


def main():
    subprocess.run(["/opt/scoped-tests/probe-build/muxer_probe"], check=True)
    subprocess.run(["/opt/scoped-tests/probe-build/action_input_probe"], check=True, timeout=60)
    rclpy.init()
    fixture = Fixture()
    try:
        test_native(fixture)
    finally:
        fixture.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
