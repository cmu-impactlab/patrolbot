"""Command execution for the web bridge — Phase 3, opt-in.

Only constructed when WEB_BRIDGE_ENABLE_COMMANDS=1; otherwise the node stays
strictly subscribe-only and every command.request is declined. Commands are
goal-based only (Nav2 NavigateToPose, /initialpose, goal cancel) plus the
guarded dock-manager operations (the /patrolbot/undock action and the
/patrolbot/charge_release, /patrolbot/motor_enable and /patrolbot/motor_disable services) — there is
deliberately no /cmd_vel path, and nothing here talks to the SBC directly.

Undock is an *action on the robot*: this bridge asks for it and reports what
comes back. It never drives the robot itself, and the reverse distance and
speed come from this node's parameters rather than from the browser — the
dock manager bounds them again with its own hard caps.
"""
from __future__ import annotations

import logging
import math
import time
from typing import Any

from . import normalizers

log = logging.getLogger("web_bridge.commands")

DISABLED_REASON = (
    "Motion commands are disabled on the robot. An operator must set "
    "WEB_BRIDGE_ENABLE_COMMANDS=1 on the robot to allow them."
)

# How stale the drive base's own reading may be before it stops being a basis
# for driving, and how long ago this node may have received it. Two separate
# questions; both match the dashboard server's thresholds in
# server/app/commands/gates.py.
MAX_TELEMETRY_AGE_S = 2.0
MAX_RECEIPT_AGE_S = 3.0
# How stale the robot's own position stream may be before navigation is
# refused. /odom arrives at ~22.7 Hz, so 3 s is roughly seventy missed samples
# — generous enough to ride out a scheduling hiccup, tight enough that a dead
# pose stream is caught. This is NOT waivable by the localization override:
# "the robot is unsure where it is" and "the robot has stopped saying where it
# is" are different failures, and only the first is the operator's to accept.
MAX_ODOM_AGE_S = 3.0

# action_msgs/GoalStatus terminal codes.
_STATUS_SUCCEEDED = 4
_STATUS_CANCELED = 5
_STATUS_ABORTED = 6

# action_msgs/CancelGoal.Response return codes.
_CANCEL_REJECTED = 1
_CANCEL_GOAL_TERMINATED = 3


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) for a rotation of `yaw` about +z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def cancel_verdict(response: Any) -> str:
    """What a CancelGoal reply actually says: "canceling", "finished" or
    "refused".

    Only "canceling" means the server took the request on — and even then the
    robot has not stopped yet; its goal still has to reach a terminal state.
    ERROR_GOAL_TERMINATED is the one empty-list answer that is good news: the
    goal had already finished, so there was nothing left to cancel. Everything
    else — a refusal, an unknown goal id (the action server lost the goal, and
    with it any knowledge of whether the robot stopped), a malformed reply — is
    read as a refusal, because none of them is evidence the robot stopped.
    """
    if getattr(response, "goals_canceling", ()):
        return "canceling"
    if int(getattr(response, "return_code", _CANCEL_REJECTED)) == _CANCEL_GOAL_TERMINATED:
        return "finished"
    return "refused"


def stop_result(goal_outcome: str) -> tuple[str, str]:
    """Outcome and sentence for a stop, from the terminal state of the goal it
    was cancelling.

    Only a cancelled goal is a stop that did what it said. Arriving first is
    also a stopped robot, but a different story and the operator should be told
    which. An aborted goal is neither: Nav2 gave up for its own reasons, which
    is not confirmation that the robot came to rest, so the stop is reported as
    failed rather than quietly claiming success.
    """
    if goal_outcome == "canceled":
        return ("succeeded", "The robot has stopped.")
    if goal_outcome == "succeeded":
        return ("succeeded",
                "The robot reached its destination before it could be stopped.")
    return ("failed",
            "The robot's goal ended abnormally while it was being stopped — "
            "check the robot before assuming it has stopped.")


def map_goal_status(status: int) -> str:
    if status == _STATUS_SUCCEEDED:
        return "succeeded"
    if status == _STATUS_CANCELED:
        return "canceled"
    if status == _STATUS_ABORTED:
        return "failed"
    return "failed"


# patrolbot_interfaces/action/Undock result codes -> plain language. The dock
# manager returns a typed code AND a message; the code is the stable contract,
# so the operator-facing sentence is chosen from it and the manager's own
# message is appended when it adds detail.
UNDOCK_CODE_TEXT = {
    0: "The robot is clear of the dock.",
    1: "The robot refused the undock request.",
    2: "The robot's pre-move checks did not pass.",
    3: "The robot could not release its charger.",
    4: "The robot could not power its motors.",
    5: "A sensor the robot needs to back up safely is not reporting.",
    6: "The robot is not confident where it is, so it will not move.",
    7: "The robot's command path is not healthy.",
    8: "Undocking failed during the motion stage.",
    9: "Undocking was canceled.",
    10: "Undocking took too long and did not complete.",
    11: "The robot's drive base restarted mid-undock.",
    12: "The robot reported a safety fault. Stop is not confirmed — check the robot.",
}

# Result codes shared by ChargeRelease.srv and MotorEnable.srv.
SERVICE_CODE_TEXT = {
    0: "Done.",
    1: "The robot rejected the request as invalid.",
    2: "The robot's link to its drive base is unavailable.",
    3: "The robot's pre-conditions for this were not met.",
    4: "The robot's hardware refused the request.",
    5: "The robot did not answer in time.",
    6: "The robot's drive base restarted while the request was in flight.",
    7: "The robot and the dashboard disagreed on the protocol.",
}

# normalize_base_state's charge_state strings that mean current is flowing.
# This is a legacy fallback only: a valid dock_state is authoritative because
# raw charge can re-latch after CLEAR_CONFIRMED.
CHARGING_STATES = frozenset({"charging", "charge", "bulk", "float", "overcharge"})
# ...and those that mean physically on the dock, charging or not.
ON_DOCK_STATES = CHARGING_STATES | frozenset({"docked", "dock", "on_dock"})

# Undock feedback `state` -> what to show while it runs.
UNDOCK_STAGE_TEXT = {
    "IDLE": "Preparing to undock",
    "PRECHECK": "Checking it is safe to move",
    "VERIFY_LOCALIZATION": "Confirming the robot's location",
    "VERIFY_COMMAND_PATH": "Checking the guarded motion path",
    "HARDWARE_UNDOCK": "Undocking at the drive base",
    "VERIFY_HANDOFF": "Checking the robot is ready to turn",
    "RESTORE_LOCALIZATION": "Refreshing the robot's location",
    "TURN_AWAY_FROM_DOCK": "Turning away from the dock",
    "UNKNOWN": "Waiting for the robot to confirm the undock outcome",
    "STOPPING": "Waiting for the robot to confirm it has stopped",
    "STOPPED": "Reported stopped; awaiting the final result",
    "RELEASING": "Releasing the charger",
    "ENABLING_MOTORS": "Powering the motors",
    "SETTLING": "Waiting for the robot to settle",
    "MOVING": "Backing off the dock",
    "VERIFYING": "Confirming the robot is clear",
    "COMPLETE": "Clear of the dock",
}

HARDWARE_UNDOCK_STAGE_TEXT = {
    "PREPARING_UNDOCK": "Preparing the drive base",
    "RELEASING": "Releasing the charger",
    "ENABLING_MOTORS": "Powering the motors",
    "BACKING_OUT": "Backing clear of the dock",
    "VERIFYING_CLEARANCE": "Confirming physical clearance",
    "HANDOFF_READY": "Handing control back safely",
    "STOPPING": "Stopping the drive base",
}


def undock_detail(code: int, message: str, manual_recovery: bool = False) -> str:
    """Operator-facing sentence for an Undock result."""
    text = UNDOCK_CODE_TEXT.get(int(code), "Undocking did not complete.")
    if message and message.strip() and int(code) != 0:
        text = f"{text} ({message.strip()})"
    if manual_recovery:
        text += " Someone needs to check the robot at the dock before trying again."
    return text


def undock_outcome(success: bool, code: int) -> str:
    """Map an Undock result onto the dashboard's command outcomes."""
    if success and int(code) == 0:
        return "succeeded"
    if int(code) == 9:
        return "canceled"
    if int(code) == 10:
        return "timeout"
    return "failed"


def service_detail(code: int, message: str) -> str:
    text = SERVICE_CODE_TEXT.get(int(code), "The request did not complete.")
    if message and message.strip() and int(code) != 0:
        text = f"{text} ({message.strip()})"
    return text


def undock_stage(state: str, hardware_phase: str = "") -> str:
    if (state or "").upper() == "HARDWARE_UNDOCK" and hardware_phase:
        return HARDWARE_UNDOCK_STAGE_TEXT.get(
            hardware_phase.upper(), hardware_phase)
    return UNDOCK_STAGE_TEXT.get((state or "").upper(), state or "Undocking")


def _on_dock(base_state: dict) -> bool:
    """Is the robot on its charger? Mirrors StateFacts.on_dock on the server.

    A *usable* dock observer is authoritative in both directions: raw
    charge_state can re-latch after a proven departure, so CLEAR_CONFIRMED
    stays clear. An observer reporting itself invalid answers nothing and falls
    back to charge_state — reading it as "not docked" would clear the dock for
    a robot sitting on its charger with a broken observer.
    """
    if base_state.get("dock_state_valid"):
        return base_state.get("dock_state", "").strip().upper() == "DOCKED_CONFIRMED"
    return base_state.get("charge_state", "").strip().lower() in ON_DOCK_STATES


def localization_epoch_ready(base_state: Any | None,
                             amcl_stamp_ns: int,
                             base_state_age: float | None) -> bool:
    """Require a fresh, post-restart localization epoch before navigation."""
    if base_state is None or base_state_age is None:
        return False
    try:
        telemetry_age = float(base_state.get("telemetry_age", 999.0))
        receipt_age = float(base_state_age)
        seed_stamp_ns = int(base_state.get("localization_seed_stamp_ns", 0))
        pose_stamp_ns = int(amcl_stamp_ns)
    except (TypeError, ValueError, OverflowError):
        return False
    return (
        bool(base_state.get("link_connected", False))
        and 0.0 <= telemetry_age <= MAX_TELEMETRY_AGE_S
        and 0.0 <= receipt_age <= MAX_RECEIPT_AGE_S
        and bool(base_state.get("odom_epoch_valid", False))
        and not bool(base_state.get("localization_recovery_required", True))
        and seed_stamp_ns > 0
        and pose_stamp_ns > seed_stamp_ns
    )


def precheck(command: str, goal: dict | None, base_state: Any | None,
             localized: bool = False, base_state_age: float | None = None,
             allow_unlocalized: bool = False,
             odom_age: float | None = None,
             operator_authorized: bool = False) -> str | None:
    """Plain-language rejection reason, or None when the command may proceed.

    base_state is the latest normalized base_state payload (a dict) or None
    when the hardware bridge hasn't reported yet — fail closed for motion.
    `localized` is the node's own answer to "is the map-frame fix recent and
    confident" (BridgeNode._localization_is_usable); it defaults to False so a
    caller that cannot answer refuses rather than assumes. `base_state_age` is
    how long ago this node received that payload, which is a different question
    from the telemetry_age inside it: if the drive-base subscription dies, the
    last payload keeps reporting whatever age it had when it was sent, forever.

    These repeat the dashboard server's gates (server/app/commands/gates.py)
    deliberately, in the same words. The server refusing first is the normal
    case; this is what holds if a frame reaches the robot without having passed
    it — a replayed command.request, or a server that trusted stale telemetry.
    """
    if command in ("navigate_to_pose", "set_initial_pose") and goal is None:
        return "This command needs a destination."
    if command in {"navigate_to_pose", "set_initial_pose", "undock", "charge_release"}:
        try:
            age = float((base_state or {}).get("lidar_scan_age", -1))
            hardware_age = float((base_state or {}).get("telemetry_age", -1))
            receipt = float(base_state_age) if base_state_age is not None else -1.0
        except (TypeError, ValueError, OverflowError):
            age = receipt = hardware_age = -1.0
        if (not (base_state or {}).get("link_connected") or not (base_state or {}).get("hardware_state_valid")
                or not math.isfinite(hardware_age) or hardware_age < 0 or hardware_age + receipt > .5
                or (base_state or {}).get("lidar_state") != "READY" or not math.isfinite(age)
                or not math.isfinite(receipt) or age < 0 or receipt < 0 or age + receipt > .5):
            return "LiDAR is off, starting, stale or unavailable — wait for fresh scans."
    if command == "undock":
        if base_state.get("fault_flags"):
            return (f"The drive base is reporting a fault (code {int(base_state['fault_flags'])}). "
                    "Clear it on the robot first.")
        if base_state.get("dock_state_valid") is False:
            return "The robot's dock observer is not reporting valid data. Check the robot before trying to undock."
        if base_state.get("undock_profile_commissioned") is False:
            return "The robot's guarded undock profile has not been commissioned."
        if base_state.get("undock_active"):
            return "The robot is already undocking."
        departed = base_state.get("dock_state_valid") and base_state.get("dock_state", "").strip().upper() in {"CLEAR_CONFIRMED", "DEPARTING"}
        if departed or base_state.get("charge_state", "").strip().lower() not in CHARGING_STATES:
            return "The robot is not charging, so it cannot start undocking."
        if base_state.get("estop_pressed"):
            return "The emergency stop is pressed. Release it on the robot first."
        if base_state.get("bumpers_valid") is not True:
            return ("The robot cannot confirm its bumper readings, so it cannot "
                    "tell whether anything is behind it. Check the robot before undocking.")
        if base_state.get("bumpers_rear"):
            return "The rear bumper is pressed — clear whatever is behind the robot before backing it off the dock."
    if command == "navigate_to_pose":
        if base_state is None:
            return "The robot's drive base has not reported in yet — cannot navigate."
        if (not base_state.get("link_connected")
                or not 0.0 <= float(base_state.get("telemetry_age", 999.0))
                <= MAX_TELEMETRY_AGE_S
                or base_state_age is None
                or base_state_age > MAX_RECEIPT_AGE_S):
            # The drive base link is down, its last reading is old, or it
            # stopped reporting to us altogether. Either way what follows would
            # be judged from numbers that no longer describe the robot.
            return ("The robot's hardware readings are stale — wait for fresh "
                    "data before commanding the robot.")
        if not base_state.get("hardware_state_valid"):
            return "The robot's drive base is not reporting valid data — cannot navigate."
        if base_state.get("fault_flags"):
            return (f"The drive base is reporting a fault "
                    f"(code {int(base_state['fault_flags'])}). "
                    "Clear it on the robot first.")
        if base_state.get("estop_pressed"):
            return "The emergency stop is pressed. Release it on the robot first."
        # Coming off the dock is undock's job — it releases the charger and
        # backs off under the dock manager's guarded profile. Checked before
        # the motors because a hand-docked robot can charge with its motors
        # still reported enabled, which would otherwise look ready to drive.
        # It also names the real cause: on 2026-07-28 an operator hit "Enable
        # motors", saw it succeed, and was told a second later to enable the
        # motors — the charger had re-latched and tripped the interlock, and
        # they went round that loop five times.
        if _on_dock(base_state):
            return ("The robot is on its charger. Move it off the dock before "
                    "sending it anywhere.")
        if not base_state.get("motors_enabled"):
            return "The robot's motors are off. Enable them on the robot first."
        if not base_state.get("odom_epoch_valid", False):
            return ("The robot's odometry epoch is not valid — wait for its "
                    "localization to recover before sending it anywhere.")
        if base_state.get("localization_recovery_required", True):
            stage = str(base_state.get("localization_recovery_stage", "")).strip()
            suffix = f" ({stage})" if stage else ""
            return ("The robot is recovering localization"
                    f"{suffix} — wait before sending it anywhere.")
        try:
            seed_stamp_ns = int(base_state.get("localization_seed_stamp_ns", 0))
        except (TypeError, ValueError, OverflowError):
            seed_stamp_ns = 0
        if seed_stamp_ns <= 0:
            return ("The robot has no valid localization seed barrier — wait "
                    "for localization to recover before sending it anywhere.")
        # An exceptional flag needs the server's verified-role stamp, the same
        # way the guarded undock does. A browser cannot authorize itself past a
        # safety gate; anything arriving here unstamped is treated as if the
        # override had not been asked for.
        override = allow_unlocalized and operator_authorized
        # Bound to the override only, so the ordinary path keeps exactly the
        # behaviour it had. On that path `localized` already implies a recent
        # AMCL pose; the override is precisely what removes that implication,
        # and would otherwise leave the bridge with no pose-recency check at
        # all. "Unsure where it is" and "no longer saying where it is" are
        # different failures, and only the first is the operator's to accept.
        # This matters because the bridge is the barrier that has to hold when
        # a crafted or replayed frame arrives without passing the server.
        if override and (odom_age is None or odom_age > MAX_ODOM_AGE_S):
            return ("The robot has stopped reporting its position — wait for "
                    "fresh data before sending it anywhere.")
        if not localized and not override:
            return "The robot does not know where it is. Set its location first."
        # RViz has always been able to send a goal on a bad fix -- Nav2 has no
        # localization gate of its own -- so the override restores parity
        # rather than granting anything new. Every check above still applies.
    return None


class CommandExecutor:
    """Owns the Nav2 action client and /initialpose publisher.

    handle() is called from a ROS timer on the executor threads; WsClient.send
    is thread-safe, so replies can be emitted from any callback.
    """

    def __init__(self, node, ws, undock_config: dict | None = None) -> None:
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav2_msgs.action import NavigateToPose
        from patrolbot_interfaces.srv import GetNavigationPath
        from rclpy.action import ActionClient

        self._node = node
        self._ws = ws
        self._NavigateToPose = NavigateToPose
        self._PoseWithCovarianceStamped = PoseWithCovarianceStamped
        self._nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self._GetNavigationPath = GetNavigationPath
        self._path_client = node.create_client(GetNavigationPath, "/patrolbot/navigate_to_pose_path")
        self._path_future = None
        self._last_navigation_path = None
        node.create_timer(.5, self._poll_navigation_path)
        from patrolbot_interfaces.action import LocalizeOnMap
        self._LocalizeOnMap = LocalizeOnMap
        self._map_client = ActionClient(node, LocalizeOnMap, "/patrolbot/localize_on_map")
        self._map_active = None
        node.create_timer(.1, self._check_map_connection)
        self._active: dict | None = None  # {command_id, goal_handle}
        self._nav_pending = None
        self._last_progress_mono = 0.0
        self._navigation_context = None
        self._navigation_generation = 0
        node.get_logger().warning(
            "WEB_BRIDGE_ENABLE_COMMANDS=1 — motion commands are ENABLED")

        # Dock-manager interfaces. Optional: a robot without the dock manager
        # (or without patrolbot_interfaces built) simply never advertises the
        # capability, and the dashboard keeps the control greyed out.
        self._undock_config = undock_config or {}
        self._undock_client = None
        self._release_client = None
        self._motor_client = None
        self._motor_disable_client = None
        self._motor_disable_generation = 0
        self._undock_active: dict | None = None  # {command_id, goal_handle, started}
        self._last_undock_progress_mono = 0.0
        self._undock_server_misses = 0
        # Longer than the dock manager's own per-state timeouts, so this only
        # ever fires for a goal nobody is going to answer.
        self._undock_timeout_s = float(
            (undock_config or {}).get("timeout_s", 180.0))
        self._dock_pose_client = None
        self._lidar_client = None
        self._lidar_pending = None
        self._lidar_recovery_required = False
        self._lidar_completed_mono = None
        try:
            from patrolbot_interfaces.srv import MotorDisable
        except ImportError:
            node.get_logger().warning("MotorDisable interface unavailable — motor disable commands disabled")
        else:
            self._MotorDisable = MotorDisable
            self._motor_disable_client = node.create_client(MotorDisable, "/patrolbot/motor_disable")
        try:
            from patrolbot_interfaces.srv import SetLidarPower
        except ImportError:
            node.get_logger().warning("SetLidarPower interface unavailable — LiDAR power commands disabled")
        else:
            self._SetLidarPower = SetLidarPower
            self._lidar_client = node.create_client(SetLidarPower, "/patrolbot/set_lidar_power")
        try:
            from patrolbot_interfaces.action import Undock
            from patrolbot_interfaces.srv import (
                ChargeRelease, InitializeDockPose, MotorEnable,
            )
        except ImportError:
            node.get_logger().warning(
                "patrolbot_interfaces not available — dock commands disabled")
            return
        self._Undock = Undock
        self._ChargeRelease = ChargeRelease
        self._MotorEnable = MotorEnable
        self._InitializeDockPose = InitializeDockPose
        self._undock_client = ActionClient(node, Undock, "/patrolbot/undock")
        self._release_client = node.create_client(
            ChargeRelease, "/patrolbot/charge_release")
        self._motor_client = node.create_client(MotorEnable, "/patrolbot/motor_enable")
        self._dock_pose_client = node.create_client(
            InitializeDockPose, "/patrolbot/initialize_dock_pose")

    # -- dock-pose seeding -----------------------------------------------------

    def initialize_dock_pose(self, dock_id: str) -> bool:
        """Ask the dock manager to seed localization from its dock pose.

        Fire and forget: the manager refuses unless it can confirm the robot
        is actually docked and stationary, so a wrong guess cannot move or
        mislocate anything. Returns whether the request was sent at all.
        """
        if self._dock_pose_client is None or not self._dock_pose_client.service_is_ready():
            return False
        request = self._InitializeDockPose.Request()
        request.dock_id = dock_id
        request.operator_authorized = True
        future = self._dock_pose_client.call_async(request)
        future.add_done_callback(self._on_dock_pose_result)
        return True

    def _on_dock_pose_result(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(f"dock pose init failed: {exc}")
            return
        if response.success:
            self._node.get_logger().info(
                f"localization seeded from dock pose ({response.message})")
        else:
            self._node.get_logger().warning(
                f"dock pose init refused: {response.message}")

    # -- undock watchdog -------------------------------------------------------

    def check_undock_health(self) -> None:
        """Cancel and retain an undock whose action server stopped answering.

        Silence cannot resolve its movement outcome. Keep the owner until an
        actual result or explicit operator recovery; never offer a retry while
        the earlier operation may still be moving.

        Two consecutive misses before declaring, because the server briefly
        reads unavailable while launch respawns it.
        """
        active = self._undock_active
        if active is None:
            self._undock_server_misses = 0
            return
        if self._undock_client is not None and not self._undock_client.server_is_ready():
            self._undock_server_misses += 1
            if self._undock_server_misses >= 2:
                self._fail_undock(
                    "The robot's docking system stopped responding mid-undock. "
                    "Its motion outcome is unknown — check the robot and dock manager.")
            return
        self._undock_server_misses = 0
        if time.monotonic() - active["started"] > self._undock_timeout_s:
            self._fail_undock("Undocking did not finish in time; cancellation was requested.")

    def _fail_undock(self, detail: str) -> None:
        active = self._undock_active
        if active is None or active.get("uncertain", False):
            return
        active["uncertain"] = True
        self._cancel_navigation(active["command_id"])
        self._node.get_logger().warning(f"undock outcome unknown: {detail}")
        self._ws.send("command.progress", {"command_id": active["command_id"],
            "stage": "unknown", "distance_remaining": None,
            "detail": detail + " The outcome is unknown; do not retry. Use motor disable or the emergency stop."})

    # -- capability advertisement ----------------------------------------------

    def available_capabilities(self) -> list[str]:
        """Dock capabilities whose server is actually reachable right now.

        The dashboard offers a control only for a capability the robot claims,
        so this is the commissioning switch: nothing is advertised until the
        dock manager is up. `dock` is deliberately absent, and is no longer a
        command at all — the robot has no dock-in path to advertise.
        """
        capabilities: list[str] = []
        if self._map_client.server_is_ready():
            capabilities.append("map_context_v1")
        if self._undock_client is not None and self._undock_client.server_is_ready():
            capabilities.append("undock")
        if self._release_client is not None and self._release_client.service_is_ready():
            capabilities.append("charge_release")
        if self._motor_client is not None and self._motor_client.service_is_ready():
            capabilities.append("motor_enable")
        if getattr(self, "_motor_disable_client", None) is not None and self._motor_disable_client.service_is_ready():
            capabilities.append("motor_disable")
        if getattr(self, "_lidar_client", None) is not None and self._lidar_client.service_is_ready():
            capabilities.append("set_lidar_power")
        return capabilities

    # -- replies ---------------------------------------------------------------

    def _ack(self, command_id: str, accepted: bool, reason: str | None = None) -> None:
        self._ws.send("command.ack", {"command_id": command_id,
                                      "accepted": accepted, "reason": reason})

    def _result(self, command_id: str, outcome: str, detail: str | None = None,
                **result_fields) -> None:
        self._ws.send("command.result", {"command_id": command_id,
                                         "outcome": outcome, "detail": detail,
                                         **result_fields})

    # -- dispatch --------------------------------------------------------------

    def handle(self, data: dict, base_state: dict | None, current_yaw: float | None,
               localized: bool = False, base_state_age: float | None = None,
               odom_age: float | None = None) -> None:
        command_id = str(data.get("command_id", ""))
        command = data.get("command")
        goal = data.get("goal")
        seed_profile = data.get("seed_profile")
        if seed_profile is not None and (command != "set_initial_pose" or seed_profile != "measured"):
            self._ack(command_id, False, "Invalid measured seed profile.")
            return
        if command in {"cancel", "cancel_map"}:
            if self._map_active and self._map_active["command_id"] == command_id:
                self._check_map_connection(cancel=True)
            if command == "cancel":
                self._cancel_navigation(command_id)
            return

        if command in {"motor_enable", "motor_disable"}:
            if data.get("operator_authorized") is not True:
                self._ack(command_id, False, "Motor power requires operator authorization.")
                return
            if command == "motor_disable":
                self._motor_disable(command_id)
            else:
                self._motor_enable(command_id)
            return
        if getattr(self, "_lidar_pending", None) is not None and command not in {"stop", "motor_enable"}:
            self._ack(command_id, False, "LiDAR power is pending or its outcome is unknown. Wait for its result.")
            return
        if getattr(self, "_lidar_recovery_required", False) and command in {"navigate_to_pose", "set_initial_pose", "undock", "charge_release"}:
            self._ack(command_id, False, "LiDAR power recovery is required. Make a new explicit power request first.")
            return
        completed = getattr(self, "_lidar_completed_mono", None)
        if completed is not None and command not in {"stop", "motor_enable"}:
            if base_state_age is None or time.monotonic() - base_state_age <= completed:
                self._ack(command_id, False, "Waiting for fresh LiDAR state after the power command.")
                return
        if command in ("navigate_to_pose", "set_initial_pose"):
            if not goal or not goal.get("map_id") or not goal.get("map_revision"):
                self._ack(command_id, False, "Explicit map identity and revision required.")
                return
            if command == "navigate_to_pose" and any(goal.get(k) != (base_state or {}).get(k) for k in ("map_id", "map_revision")):
                self._ack(command_id, False, "Destination map differs from the active map.")
                return
        if self._map_active is not None:
            if command == "stop":
                self._check_map_connection(cancel=True)
                self._ack(command_id, True)
                self._result(command_id, "succeeded",
                             "Map cancellation requested; motion remains held.")
                return
            self._ack(command_id, False, "Map localization owns the robot; wait for its result.")
            return
        allow_unlocalized = bool(data.get("allow_unlocalized", False))
        operator_authorized = bool(data.get("operator_authorized", False))
        reason = precheck(command, goal, base_state, localized, base_state_age,
                          allow_unlocalized, odom_age, operator_authorized)
        if reason is not None:
            self._ack(command_id, False, reason)
            return
        # Logged only once the command is actually going ahead, so a goal
        # refused for some other reason leaves no misleading override record
        # behind. A suppressed safety gate has to be visible in the log.
        if (allow_unlocalized and operator_authorized
                and command == "navigate_to_pose" and not localized):
            self._node.get_logger().warning(
                "LOCALIZATION GATE OVERRIDDEN by operator: navigating with an "
                f"unusable map-frame fix (command {command_id})")

        if command == "navigate_to_pose":
            self._navigate(command_id, goal, current_yaw)
        elif command == "stop":
            self._stop(command_id)
        elif command == "set_lidar_power":
            self._set_lidar_power(command_id, data, base_state, base_state_age, odom_age)
        elif command == "set_initial_pose":
            self._set_initial_pose(command_id, goal, operator_authorized, seed_profile)
        elif command == "undock":
            # operator_authorized is set by the dashboard server from the
            # verified session role — never by the browser (see
            # server/app/commands/broker.py). Absent means unauthorized.
            self._undock(command_id, bool(data.get("operator_authorized", False)))
        elif command == "charge_release":
            self._charge_release(command_id)
        elif command == "motor_enable":
            self._motor_enable(command_id)
        else:
            self._ack(command_id, False, f"Unknown command: {command}")

    # -- navigate_to_pose ------------------------------------------------------

    def _navigate(self, command_id: str, goal: dict, current_yaw: float | None) -> None:
        if self._undock_active is not None:
            # Two motion owners at once is a state nothing downstream handles —
            # a stop would cancel one of them and report on the other.
            self._ack(command_id, False,
                      "The robot is moving off its dock. Wait for that to finish "
                      "before sending it somewhere.")
            return
        if not self._nav_client.server_is_ready():
            self._ack(command_id, False,
                      "The robot's navigation system is not running.")
            return

        nav_goal = self._NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = "map"
        nav_goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = float(goal["x"])
        nav_goal.pose.pose.position.y = float(goal["y"])
        # No requested yaw: keep the robot's current heading at the goal
        # rather than forcing a spin to zero.
        yaw = goal.get("yaw")
        if yaw is None:
            yaw = current_yaw if current_yaw is not None else 0.0
        qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
        nav_goal.pose.pose.orientation.x = qx
        nav_goal.pose.pose.orientation.y = qy
        nav_goal.pose.pose.orientation.z = qz
        nav_goal.pose.pose.orientation.w = qw

        self._clear_navigation_path()
        generation = getattr(self, "_motor_disable_generation", 0)
        context = self._navigation_context
        self._navigation_generation = getattr(self, "_navigation_generation", 0) + 1
        navigation_generation = self._navigation_generation
        self._nav_pending = {"command_id": command_id, "cancel": False}
        future = self._nav_client.send_goal_async(
            nav_goal, feedback_callback=lambda fb: self._on_feedback(command_id, fb))
        future.add_done_callback(lambda f: self._on_goal_response(
            command_id, f, generation, context, navigation_generation))

    def _on_goal_response(self, command_id: str, future, generation: int | None = None,
                          context: tuple | None = None,
                          navigation_generation: int | None = None) -> None:
        pending = getattr(self, "_nav_pending", None)
        if pending is None or pending["command_id"] != command_id:
            pending = None
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001 — surfaced to the operator
            self._ack(command_id, False, f"Navigation refused the request: {exc}")
            return
        if not goal_handle.accepted:
            if pending is not None and self._nav_pending is pending:
                self._nav_pending = None
            self._ack(command_id, False, "Navigation declined the destination.")
            return
        if generation is not None and generation != getattr(self, "_motor_disable_generation", 0):
            goal_handle.cancel_goal_async()
            self._ack(command_id, False, "Destination canceled by motor disable.")
            return
        if (navigation_generation is not None and navigation_generation !=
                getattr(self, "_navigation_generation", 0)):
            goal_handle.cancel_goal_async()
            self._ack(command_id, False, "A newer destination replaced this request.")
            return
        if context is not None and context != self._navigation_context:
            goal_handle.cancel_goal_async()
            self._ack(command_id, False, "Destination canceled because the robot context changed.")
            return
        # Nav2 preempts a prior goal itself; close ours out for the UI.
        if self._active is not None:
            self._result(self._active["command_id"], "canceled",
                         "A newer destination replaced this one.")
            # Any stop still waiting on that goal has lost its subject, and the
            # robot is now driving to a new one rather than standing still.
            for stop_command_id in self._claim_all_stops(self._active):
                self._result(stop_command_id, "failed",
                             "A new destination replaced the goal being stopped. "
                             "The robot is moving again — use the emergency stop "
                             "on the robot.")
        self._active = {"command_id": command_id, "goal_handle": goal_handle,
                        "goal_id": bytes(goal_handle.goal_id.uuid),
                        "navigation_context": self._navigation_context,
                        "path_valid": False, "path_blocked": False}
        if pending is not None and self._nav_pending is pending:
            self._nav_pending = None
        self._last_progress_mono = 0.0
        self._ack(command_id, True)
        self._clear_navigation_path()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_result(command_id, f))
        if pending is not None and pending["cancel"]:
            self._cancel_navigation(command_id)

    def _cancel_navigation(self, command_id: str) -> None:
        undock = self._undock_active
        if undock is not None and undock["command_id"] == command_id:
            if not undock.get("cancel_requested", False):
                undock["cancel_requested"] = True
                if undock.get("goal_handle") is not None:
                    try:
                        undock["goal_handle"].cancel_goal_async()
                    except Exception as exc:
                        self._ws.send("command.progress", {"command_id": command_id,
                            "stage": "stopping", "distance_remaining": None,
                            "detail": f"Undock could not be asked to stop: {exc}. Use Stop or the emergency stop."})
            return
        pending = getattr(self, "_nav_pending", None)
        if pending is not None and pending["command_id"] == command_id:
            pending["cancel"] = True
        active = self._active
        if (active is None or active["command_id"] != command_id
                or active.get("cancel_requested", False)):
            return
        active["cancel_requested"] = active["cancel_pending"] = active["path_blocked"] = True
        self._clear_navigation_path()
        self._ws.send("command.progress", {"command_id": command_id,
            "stage": "stopping", "path_valid": False, "distance_remaining": None,
            "detail": "Waiting for navigation to stop."})
        try:
            future = active["goal_handle"].cancel_goal_async()
        except Exception as exc:
            self._on_navigation_cancel_response(active, None, str(exc))
            return
        future.add_done_callback(lambda f: self._on_navigation_cancel_response(active, f))

    def _on_navigation_cancel_response(self, active: dict, future, error: str = "") -> None:
        if self._active is not active:
            return
        try:
            if future is not None and cancel_verdict(future.result()) in {"canceling", "finished"}:
                return  # Only the goal's terminal result closes the command.
        except Exception as exc:
            error = str(exc)
        active["cancel_pending"] = False
        self._ws.send("command.progress", {"command_id": active["command_id"],
            "stage": "stopping", "path_valid": False, "distance_remaining": None,
            "detail": "Navigation did not confirm cancellation. It may still be moving; use Stop or the emergency stop."
                      + (f" {error}" if error else "")})

    def update_navigation_context(self, base_state: dict) -> None:
        context = tuple(base_state.get(k) for k in (
            "map_id", "map_revision", "map_activated_stamp_ns", "session_generation",
            "odom_epoch_valid", "localization_recovery_required", "localization_seed_stamp_ns"))
        if context != self._navigation_context:
            self._navigation_context = context
            if self._active is not None:
                self._active["path_blocked"] = True
                self._clear_navigation_path()

    def _clear_navigation_path(self) -> None:
        active = self._active
        if active is None:
            return
        active["path_valid"] = False
        self._last_navigation_path = None
        context = active.get("navigation_context") or (None, None)
        self._ws.send("telemetry.path", {"frame_id": "map", "points": [], "goal": None,
                      "command_id": active["command_id"], "path_valid": False,
                      "map_id": context[0], "map_revision": context[1]})

    def _poll_navigation_path(self) -> None:
        if self._path_future is not None and not self._path_future.done():
            if time.monotonic() - self._path_sent_mono <= 2.0:
                return
            self._path_client.remove_pending_request(self._path_future)
            self._path_future = None
            self._clear_navigation_path()
        active = self._active
        if active is None or active.get("path_blocked", False):
            return
        if not self._path_client.service_is_ready():
            if active.get("path_valid", False):
                self._clear_navigation_path()
            return
        request = self._GetNavigationPath.Request()
        request.goal_id.uuid = list(active["goal_id"])
        self._path_sent_mono = time.monotonic()
        self._path_future = self._path_client.call_async(request)
        self._path_future.add_done_callback(
            lambda future: self._on_navigation_path(active, future))

    def _on_navigation_path(self, active: dict, future) -> None:
        if self._active is not active or future is not self._path_future:
            return
        try:
            response = future.result()
        except Exception:  # A lost navigator cannot retain a usable path.
            self._clear_navigation_path()
            return
        if not response.active:
            self._clear_navigation_path()
            return
        payload = self.navigation_path(active["goal_id"], response.path)
        if payload is not None and payload != self._last_navigation_path:
            self._last_navigation_path = payload
            self._ws.send("telemetry.path", payload)

    def navigation_path(self, goal_id: bytes, path) -> dict | None:
        active = self._active
        if (active is None or active.get("path_blocked", False)
                or goal_id != active.get("goal_id")
                or active.get("navigation_context") != self._navigation_context):
            return None
        context = self._navigation_context
        if (context is None or not context[0] or not context[1]
                or not context[4] or context[5]):
            return None
        valid = bool(path.poses)
        if valid:
            stamp = path.header.stamp.sec * 1_000_000_000 + path.header.stamp.nanosec
            valid = (path.header.frame_id == "map"
                     and stamp >= max(context[2] or 0, context[6] or 0) > 0
                     and all(math.isfinite(float(v)) for pose in path.poses for v in (
                         pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                         pose.pose.orientation.x, pose.pose.orientation.y,
                         pose.pose.orientation.z, pose.pose.orientation.w)))
        active["path_valid"] = valid
        payload = (normalizers.normalize_path(path, int(self._node.cfg["path_max_points"]))
                   if valid else {"frame_id": "map", "points": [], "goal": None})
        payload.update(command_id=active["command_id"], path_valid=valid,
                       map_id=context[0], map_revision=context[1])
        if payload.get("goal"):
            payload["goal"].update(map_id=context[0], map_revision=context[1])
        return payload

    def _on_feedback(self, command_id: str, feedback_msg) -> None:
        active = self._active
        if (active is None or active["command_id"] != command_id
                or bytes(feedback_msg.goal_id.uuid) != active.get("goal_id")):
            return
        now = time.monotonic()
        if now - self._last_progress_mono < 1.0:
            return
        self._last_progress_mono = now
        distance = getattr(feedback_msg.feedback, "distance_remaining", None)
        stopping = active.get("cancel_requested", False)
        valid = (not stopping and active.get("path_valid", False)
                 and distance is not None and math.isfinite(float(distance))
                 and float(distance) >= 0.0)
        self._ws.send("command.progress", {
            "command_id": command_id,
            "stage": "stopping" if stopping else "navigating" if valid else "planning",
            "detail": "Waiting for navigation to stop" if stopping else
                      "Heading to the destination" if valid else "Planning a path to the destination",
            "path_valid": bool(valid),
            "distance_remaining": round(float(distance), 2) if valid else None,
        })

    def _on_result(self, command_id: str, future) -> None:
        if self._active is None or self._active["command_id"] != command_id:
            return  # already superseded/canceled and reported
        stops = self._claim_all_stops(self._active)
        self._clear_navigation_path()
        self._active = None
        try:
            response = future.result()
            status = response.status
        except Exception as exc:  # noqa: BLE001
            self._result(command_id, "failed", f"Navigation ended abnormally: {exc}")
            outcome = "failed"
        else:
            outcome = map_goal_status(status)
            result = getattr(response, "result", None)
            code = getattr(result, "error_code", None)
            message = getattr(result, "error_msg", "") or ""
            detail = {"succeeded": "Arrived at the destination.",
                      "canceled": "Navigation was canceled.",
                      "failed": "The robot could not reach the destination."}[outcome]
            if outcome == "failed" and code:
                detail += f" Navigation error {int(code)}."
            if outcome == "failed" and message.strip():
                detail += f" {message.strip()}"
            self._result(command_id, outcome, detail, error_code=code,
                         error_message=message.strip() or None)
        # The goal is terminal, so Nav2 is no longer driving the robot — only
        # now can anything true be said about the robot having stopped.
        for stop_command_id in stops:
            self._result(stop_command_id, *stop_result(outcome))

    # -- stop ------------------------------------------------------------------

    def _stop(self, command_id: str) -> None:
        """Ask the running action to cancel, and say nothing about the robot
        having stopped until it actually has.

        The cancel request completing only means Nav2 (or the dock manager)
        answered. Reporting success there told the operator "the robot has
        stopped" for a cancellation the server had rejected, threw the goal
        away, and then ignored its result — while the robot kept driving.
        """
        self._ack(command_id, True)
        # An undock in progress is the thing to stop; otherwise the goal. The
        # two are mutually exclusive — _navigate refuses to start while an
        # undock is running, and _undock while one is.
        active = self._undock_active or self._active
        if active is None:
            self._result(command_id, "succeeded", "The robot was not navigating.")
            return
        already_cancelling = bool(active.get("stop_command_ids")) or active.get("cancel_pending", False)
        active["cancel_requested"] = True
        if active is self._active:
            active["path_blocked"] = True
            self._clear_navigation_path()
        stops = active.setdefault("stop_command_ids", [])
        stops.append(command_id)
        if active.get("goal_handle") is None:
            return  # Cancel an undock whose acceptance reply has not arrived.
        if already_cancelling:
            # A cancel is already in flight for this goal. Asking again would
            # get ERROR_GOAL_TERMINATED — which the server returns for a goal
            # that is merely CANCELING, not only for one that has finished — and
            # this stop would report "already stopped" while the robot is still
            # coming to a halt. Both stops are reported off the goal's terminal
            # result instead.
            return
        try:
            cancel_future = active["goal_handle"].cancel_goal_async()
        except Exception as exc:  # noqa: BLE001 — surfaced to the operator
            self._fail_stop(command_id,
                            f"The robot could not be asked to stop: {exc}.")
            return
        cancel_future.add_done_callback(
            lambda f: self._on_cancel_response(command_id, f))

    def _on_cancel_response(self, command_id: str, future) -> None:
        try:
            verdict = cancel_verdict(future.result())
        except Exception as exc:  # noqa: BLE001 — surfaced to the operator
            self._fail_stop(command_id,
                            f"The robot did not answer the stop request: {exc}.")
            return
        if verdict == "canceling":
            return  # Reported once the goal reaches a terminal state.
        if verdict == "finished" and self._claim_stop(command_id):
            self._result(command_id, "succeeded", "The robot had already stopped.")
            return
        if verdict != "finished":
            self._fail_stop(command_id, "The robot refused the stop request.")

    def _fail_stop(self, command_id: str, detail: str) -> None:
        """A stop that did not take. The goal is left alone — it is still the
        robot's active goal — and the operator is pointed at the one control
        that does not depend on this path."""
        if not self._claim_stop(command_id):
            return
        self._result(command_id, "failed",
                     f"{detail} It may still be moving — use the emergency "
                     "stop on the robot.")

    def _claim_stop(self, command_id: str) -> bool:
        """Claim the right to report this stop, if nothing has yet.

        A stop can be resolved from either end — the cancel reply or the goal's
        terminal result, whichever arrives first — so both paths claim before
        reporting and the loser stays quiet.
        """
        for active in (self._active, self._undock_active):
            if active is not None and command_id in active.get("stop_command_ids", ()):
                active["stop_command_ids"].remove(command_id)
                return True
        return False

    def _claim_all_stops(self, active: dict | None) -> list[str]:
        """Claim every stop still waiting on a goal that has just ended."""
        return [] if active is None else active.pop("stop_command_ids", [])

    # -- undock ----------------------------------------------------------------

    def _undock(self, command_id: str, operator_authorized: bool) -> None:
        if self._undock_client is None:
            self._ack(command_id, False,
                      "This robot does not have undocking commissioned.")
            return
        if not operator_authorized:
            # Belt and braces: the server already refuses observers. If an
            # unauthorized request somehow reaches the robot, it stops here
            # rather than being passed to the dock manager as authorized.
            self._ack(command_id, False,
                      "Your account is not authorized to move the robot.")
            return
        if not self._undock_client.server_is_ready():
            self._ack(command_id, False,
                      "The robot's docking system is not running.")
            return
        if self._undock_active is not None:
            self._ack(command_id, False, "The robot is already undocking.")
            return
        if self._active is not None or getattr(self, "_nav_pending", None) is not None:
            # The other half of _navigate's guard: never two motion owners.
            self._ack(command_id, False,
                      "The robot is driving to a destination. Stop it before "
                      "backing it off the dock.")
            return

        goal = self._Undock.Goal()
        goal.dock_id = str(self._undock_config.get("dock_id", ""))
        goal.reverse_distance = float(self._undock_config.get("reverse_distance", 0.53))
        goal.reverse_speed = float(self._undock_config.get("reverse_speed", 0.1))
        goal.validation_mode = bool(self._undock_config.get("validation_mode", False))
        goal.operator_authorized = True

        generation = getattr(self, "_motor_disable_generation", 0)
        self._undock_active = {"command_id": command_id, "goal_handle": None,
                               "started": time.monotonic()}
        try:
            future = self._undock_client.send_goal_async(
                goal, feedback_callback=lambda fb: self._on_undock_feedback(command_id, fb))
        except Exception as exc:
            self._undock_active = None
            self._ack(command_id, False, f"The undock request could not be sent: {exc}")
            return
        future.add_done_callback(lambda f: self._on_undock_response(command_id, f, generation))

    def _on_undock_response(self, command_id: str, future, generation: int | None = None) -> None:
        active = self._undock_active
        try:
            goal_handle = future.result()
            if goal_handle is None:
                raise RuntimeError("the action returned no acceptance reply")
        except Exception as exc:  # noqa: BLE001 — surfaced to the operator
            if active is None or active["command_id"] != command_id:
                return
            self._fail_undock(f"Undock acceptance is unknown: {exc}.")
            return
        if active is None or active["command_id"] != command_id:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self._undock_active = None
            # A rejected ROS action goal carries no message, and the dock
            # manager's real reason (unknown dock id, out-of-range distance or
            # speed, another goal active) only reaches its own log. Echo the
            # goal we sent so the mismatch is diagnosable from the dashboard
            # alone rather than requiring someone to SSH into the robot.
            config = self._undock_config
            self._ack(command_id, False,
                      "The robot declined to undock — it did not accept the request "
                      f"(dock “{config.get('dock_id', '')}”, "
                      f"{float(config.get('reverse_distance', 0)):.2f} m at "
                      f"{float(config.get('reverse_speed', 0)):.2f} m/s"
                      f"{', validation mode' if config.get('validation_mode') else ''}). "
                      "The robot's dock manager log has the exact reason.")
            for stop_id in self._claim_all_stops(active):
                self._result(stop_id, "succeeded", "The robot did not accept the undock request.")
            return
        if generation is not None and generation != getattr(self, "_motor_disable_generation", 0):
            active["cancel_requested"] = True
        active["goal_handle"] = goal_handle
        self._undock_server_misses = 0
        self._ack(command_id, True)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_undock_result(command_id, f))
        if active.get("cancel_requested", False):
            try:
                cancel_future = goal_handle.cancel_goal_async()
            except Exception as exc:
                for stop_id in list(active.get("stop_command_ids", [])):
                    self._fail_stop(stop_id, f"The robot could not be asked to stop: {exc}.")
                return
            stops = active.get("stop_command_ids", [])
            if stops:
                stop_id = stops[0]
                cancel_future.add_done_callback(lambda f: self._on_cancel_response(stop_id, f))

    def _on_undock_feedback(self, command_id: str, feedback_msg) -> None:
        active = self._undock_active
        if active is None or active["command_id"] != command_id or active.get("goal_handle") is None:
            return
        now = time.monotonic()
        if now - self._last_undock_progress_mono < 1.0:
            return
        self._last_undock_progress_mono = now
        feedback = feedback_msg.feedback
        state = "STOPPING" if active.get("cancel_requested", False) else getattr(feedback, "state", "")
        hardware_phase = getattr(feedback, "hardware_phase", "")
        self._ws.send("command.progress", {
            "command_id": command_id,
            "stage": state or "undocking",
            "detail": "Waiting for the robot to confirm it has stopped" if state == "STOPPING" else undock_stage(state, hardware_phase),
            "distance_remaining": None,
        })

    def _on_undock_result(self, command_id: str, future) -> None:
        if self._undock_active is None or self._undock_active["command_id"] != command_id:
            return
        try:
            wrapped = future.result()
            if getattr(wrapped, "status", None) not in (
                    _STATUS_SUCCEEDED, _STATUS_CANCELED, _STATUS_ABORTED):
                raise RuntimeError("the action returned no terminal result")
            result = wrapped.result
            code = int(result.code)
            outcome = undock_outcome(bool(result.success), code)
            detail = undock_detail(code, getattr(result, "message", ""),
                bool(getattr(result, "manual_recovery_required", False)))
            if ((outcome in {"succeeded", "canceled"}
                 and outcome != map_goal_status(wrapped.status))
                    or (code == 0 and not result.success)):
                outcome = "failed"
                detail = "The robot returned an inconsistent undock result. Check the robot before assuming it has stopped."
        except Exception as exc:  # noqa: BLE001
            self._fail_undock(f"Undock result is unknown: {exc}.")
            return
        stops = self._claim_all_stops(self._undock_active)
        self._undock_active = None
        self._result(command_id, outcome, detail)
        for stop_command_id in stops:
            self._result(stop_command_id, *stop_result(outcome))

    # -- charge_release / motor_enable -----------------------------------------

    def _charge_release(self, command_id: str) -> None:
        self._call_dock_service(command_id, self._release_client, self._ChargeRelease,
                                "release charging")

    def _motor_disable(self, command_id: str) -> None:
        if self._active is not None:
            self._active["path_blocked"] = True
            self._clear_navigation_path()
        self._motor_disable_generation = getattr(self, "_motor_disable_generation", 0) + 1
        self._call_dock_service(command_id, getattr(self, "_motor_disable_client", None),
                                getattr(self, "_MotorDisable", None), "turn the motors off")
        if self._map_active is not None:
            try:
                self._check_map_connection(cancel=True)
            except Exception as exc:
                self._node.get_logger().warning(f"Motor disable map cancellation failed: {exc}")
        for active in (self._active, self._undock_active):
            if active is not None:
                active["cancel_requested"] = True
                if active.get("goal_handle") is None:
                    continue
                try:
                    active["goal_handle"].cancel_goal_async()
                except Exception as exc:
                    self._node.get_logger().warning(f"Motor disable action cancellation failed: {exc}")

    def _motor_enable(self, command_id: str) -> None:
        if self._active is not None or self._undock_active is not None:
            self._ack(command_id, False, "Previous movement has not finished canceling. Stop it before enabling motors.")
            return
        self._call_dock_service(command_id, self._motor_client, self._MotorEnable,
                                "turn the motors on")

    def _call_dock_service(self, command_id: str, client, srv_type, what: str) -> None:
        """Both services share a shape: a caller-generated request_id whose
        reuse replays the original result, and a typed response code. Passing
        the dashboard's command_id straight through as request_id makes a
        replayed frame idempotent on the robot, not just at the server."""
        if client is None:
            self._ack(command_id, False, f"This robot cannot {what} from the dashboard.")
            return
        if not client.service_is_ready():
            self._ack(command_id, False, "The robot's drive-base service is not running.")
            return
        request = srv_type.Request()
        request.request_id = command_id
        future = client.call_async(request)
        future.add_done_callback(lambda f: self._on_dock_service_result(command_id, f, what))

    def _on_dock_service_result(self, command_id: str, future, what: str = "") -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self._ack(command_id, False, f"The robot did not answer: {exc}")
            return
        if not getattr(response, "accepted", False):
            self._ack(command_id, False,
                      service_detail(int(getattr(response, "code", 1)),
                                     getattr(response, "message", "")))
            return
        if getattr(response, "request_id", command_id) != command_id:
            self._ack(command_id, False, "The drive-base reply did not match this request.")
            return
        self._ack(command_id, True)
        code = int(getattr(response, "code", 0))
        if what == "turn the motors off" and response.success and getattr(response, "motors_enabled", True):
            self._result(command_id, "failed", "The drive base still reports motors enabled.")
            return
        self._result(command_id, "succeeded" if response.success else "failed",
                     service_detail(code, getattr(response, "message", "")))

    def _set_lidar_power(self, command_id, data, base, receipt_age, odom_age):
        if data.get("operator_authorized") is not True or type(data.get("enabled")) is not bool:
            self._ack(command_id, False, "LiDAR power requires operator authorization and an explicit boolean target.")
            return
        base = base or {}
        try:
            hardware_age = float(base.get("telemetry_age", -1))
        except (TypeError, ValueError, OverflowError):
            hardware_age = -1.0
        if (not base.get("lidar_power_supported") or not base.get("hardware_state_valid")
                or not base.get("link_connected") or base.get("fault_flags")
                or receipt_age is None or not 0 <= receipt_age <= .5
                or not math.isfinite(hardware_age) or hardware_age < 0
                or hardware_age + receipt_age > .5
                or base.get("lidar_state") not in {"OFF", "READY", "FAULT"}):
            self._ack(command_id, False, "LiDAR power control is unavailable, uncommissioned or stale.")
            return
        if self._active or self._undock_active or base.get("undock_active"):
            self._ack(command_id, False, "Stop the active operation before changing LiDAR power.")
            return
        client = getattr(self, "_lidar_client", None)
        if client is None or not client.service_is_ready():
            self._ack(command_id, False, "The robot's LiDAR power service is not running.")
            return
        # The typed robot service owns the measured-stationary check and motion hold.
        request = self._SetLidarPower.Request()
        request.request_id = command_id
        request.enabled = data["enabled"]
        self._lidar_pending = command_id
        try:
            future = client.call_async(request)
        except Exception as exc:
            self._result(command_id, "timeout", f"LiDAR power outcome is unknown: {exc}")
            return
        future.add_done_callback(lambda future: self._on_lidar_result(command_id, future))

    def _on_lidar_result(self, command_id, future):
        if self._lidar_pending != command_id:
            return
        try:
            response = future.result()
            if response.request_id != command_id:
                raise ValueError("LiDAR response belongs to a different request")
        except Exception as exc:
            self._result(command_id, "timeout", f"LiDAR power outcome is unknown: {exc}")
            return
        self._lidar_pending = None
        self._lidar_completed_mono = time.monotonic()
        if int(response.code) == 5:
            self._lidar_recovery_required = True
            self._ack(command_id, True)
            self._result(command_id, "failed", response.message or "LiDAR power timed out; explicit recovery is required.")
            return
        if response.accepted:
            self._lidar_recovery_required = not response.success
        self._ack(command_id, bool(response.accepted), response.message or None)
        if response.accepted:
            self._result(command_id, "succeeded" if response.success else "failed", response.message)

    # -- set_initial_pose ------------------------------------------------------

    def _set_initial_pose(self, command_id: str, goal: dict, operator_authorized: bool = False, seed_profile: str | None = None) -> None:
        msg = self._PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(goal["x"])
        msg.pose.pose.position.y = float(goal["y"])
        yaw = float(goal.get("yaw") or 0.0)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        measured = seed_profile == "measured"
        msg.pose.covariance[0] = 0.0009 if measured else 0.25
        msg.pose.covariance[7] = 0.0009 if measured else 0.25
        msg.pose.covariance[35] = 0.00030461742 if measured else 0.0685
        if not operator_authorized or self._active is not None or not self._map_client.server_is_ready():
            self._ack(command_id, False, "Map localization requires authorization, an idle robot and its action server.")
            return
        request = self._LocalizeOnMap.Goal()
        request.request_id = command_id
        request.map_id = goal["map_id"]
        request.map_revision = goal["map_revision"]
        request.pose = msg.pose
        request.operator_authorized = operator_authorized
        request.measured_pose = measured
        self._map_active = {"command_id": command_id, "handle": None, "cancel": False}
        future = self._map_client.send_goal_async(request, feedback_callback=self._map_feedback)
        active = self._map_active
        future.add_done_callback(lambda future: self._map_response(future, active))

    def _map_feedback(self, message):
        active = self._map_active
        if active and message.feedback.request_id == active["command_id"]:
            self._ws.send("command.progress", {"command_id": active["command_id"], "stage": message.feedback.stage})

    def _map_response(self, future, active):
        if active is not self._map_active: return
        try:
            handle = future.result()
            if not handle.accepted:
                self._ack(active["command_id"], False, "Robot refused map localization preconditions.")
                self._map_active = None
                return
            active["handle"] = handle
            self._ack(active["command_id"], True)
            handle.get_result_async().add_done_callback(lambda future: self._map_result(future, active))
            if active["cancel"]: handle.cancel_goal_async()
        except Exception as exc:
            self._result(active["command_id"], "failed", str(exc))
            self._map_active = None

    def _map_result(self, future, active):
        if active is not self._map_active: return
        try:
            result = future.result().result
            if result.request_id != active["command_id"]:
                raise ValueError("Map localization result belongs to a different request")
            self._result(active["command_id"], "succeeded" if result.localized and result.code == 0 else "failed", result.message)
        except Exception as exc:
            self._result(active["command_id"], "failed", str(exc))
        self._map_active = None

    def _check_map_connection(self, cancel=False):
        active = self._map_active
        if active and (cancel or not self._ws.connected.is_set()) and not active["cancel"]:
            active["cancel"] = True
            if active["handle"]: active["handle"].cancel_goal_async()
