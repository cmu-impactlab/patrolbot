#!/usr/bin/env python3
"""
PatrolBot Joystick Teleop — Logitech F710 (XInput, switch on 'X').

Hold **RB (deadman)** to take manual control; release to hand control back to
the operator. A rising RB edge cancels any active NavigateToPose goal before
the joystick input is allowed to time out, so autonomous motion never resumes
unexpectedly after manual takeover.

Drive with the **left analog stick** (smooth/proportional) OR the **D-pad**
(falls back to it if the stick is centered). Output is **acceleration-ramped**
at a fixed rate, so motion is smooth — never chunky — even from the digital D-pad.

  LB + RB + LT + RT = toggle main LiDAR power once; release all to rearm
  RB (5)  = deadman / enable
  left stick Y (axis 1) or D-pad Y (axis 7) = forward / reverse
  left stick X (axis 0) or D-pad X (axis 6) = turn
  A (0) / Y (3) = decrease / increase MAX LINEAR speed
  X (2) / B (1) = decrease / increase MAX ANGULAR speed

Drive indices verified live on the robot 2026-06-27. LB/trigger chord indices
and polarity still require live confirmation before feature acceptance. If
forward/back or left/right is reversed, flip the matching *_SIGN constant below and restart.
"""
import math
from uuid import uuid4

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from patrolbot_interfaces.msg import BaseState
from patrolbot_interfaces.srv import SetLidarPower
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

from joystick_policy import (
    ANG_SIGN,
    AX_ANG,
    AX_DPAD_ANG,
    AX_DPAD_LIN,
    AX_LIN,
    BTN_DEADMAN,
    DPAD_ANG_SIGN,
    DPAD_LIN_SIGN,
    JOY_TIMEOUT,
    LIN_SIGN,
    LidarChord,
    apply_deadzone,
    axis_value,
    button_pressed,
    ramp_toward,
    rising_edge,
)

# ---- Button map (Logitech F710, XInput) ----
BTN_LIN_DOWN = 0   # A  -> slower linear
BTN_LIN_UP = 3     # Y  -> faster linear
BTN_ANG_DOWN = 2   # X  -> slower angular
BTN_ANG_UP = 1     # B  -> faster angular

# Retry cancel-all while retaining the existing manual zero hold.
NAV_CANCEL_RETRY = 0.25
BASE_STATE_TIMEOUT = 0.5


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('p3dxJoyTeleop')

        # Live-adjustable speed limits (A/Y, X/B)
        # The SBC rejects ordinary DRIVE commands above 0.30 m/s. Keep both
        # the default and the operator-adjustable ceiling at that hardware
        # contract so full-stick input can never become an on/off command.
        self.declare_parameter('max_linear', 0.30)    # m/s at full stick
        self.declare_parameter('max_angular', 0.7)    # rad/s at full stick
        self.declare_parameter('min_linear', 0.05)
        self.declare_parameter('max_linear_cap', 0.30)
        self.declare_parameter('min_angular', 0.1)
        self.declare_parameter('max_angular_cap', 3.0)
        self.declare_parameter('linear_step', 0.05)
        self.declare_parameter('angular_step', 0.1)
        self.declare_parameter('deadzone', 0.12)
        self.declare_parameter('accel_linear', 0.7)   # m/s^2 ramp (smoothness)
        self.declare_parameter('accel_angular', 2.0)  # rad/s^2 ramp
        self.declare_parameter('rate', 30.0)          # Hz output

        g = lambda k: self.get_parameter(k).value
        self.max_lin = g('max_linear'); self.max_ang = g('max_angular')
        self.min_lin = g('min_linear'); self.lin_cap = g('max_linear_cap')
        self.min_ang = g('min_angular'); self.ang_cap = g('max_angular_cap')
        self.lin_step = g('linear_step'); self.ang_step = g('angular_step')
        self.deadzone = g('deadzone')
        self.accel_lin = g('accel_linear'); self.accel_ang = g('accel_angular')
        self.dt = 1.0 / g('rate')

        self.lin_tgt = 0.0; self.ang_tgt = 0.0   # target from sticks
        self.lin_cmd = 0.0; self.ang_cmd = 0.0   # ramped output
        self.active = False                       # RB held
        self.deadman_was_down = False
        self.prev_btns = []
        self.was_pub = False
        self._last_joy = None                     # time of last /joy msg (safety)
        # Manual takeover is fail-closed on release: until Nav2 accepts the
        # cancel-all request, keep publishing priority-8 zero rather than let a
        # previous autonomous goal resume underneath the operator.
        self._takeover_pending = False
        self._cancel_in_flight = False
        self._next_cancel_retry = 0.0
        self._cancel_unavailable_logged = False
        self._nav_status_received = False
        self._nav_goal_active = False
        self._takeover_generation = 0
        self._lidar_chord = LidarChord()
        self._lidar_toggle_pending = False
        self._lidar_in_flight = False
        self._base_state = None
        self._base_state_at = None
        self._lidar_generation = None

        self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.create_subscription(
            BaseState, '/patrolbot/base_state', self._base_state_cb, 10)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self._navigation_status, qos_profile_action_status_default)
        # Remapped to /input/joy in the launch (twist_mux arbitrates above nav).
        self.pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        # Deadman evidence for the bridge: lets a human drive during
        # localization recovery while autonomy stays blocked. Stale flag
        # fails closed at the bridge within MANUAL_ACTIVE_TIMEOUT.
        self.manual_pub = self.create_publisher(Bool, "/patrolbot/manual_active", 10)
        self.nav_cancel = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.lidar_power = self.create_client(
            SetLidarPower, '/patrolbot/set_lidar_power')
        self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            'Joy teleop ready (XInput). Hold RB + left stick (or D-pad) to drive. '
            'LB+RB+LT+RT=LiDAR power (release all to rearm). '
            f'A/Y=linear -/+, X/B=angular -/+. max {self.max_lin} m/s / {self.max_ang} rad/s.')

    def _base_state_cb(self, message):
        if ((self._lidar_generation is not None and
             message.session_generation != self._lidar_generation) or
                not message.link_connected):
            self._lidar_toggle_pending = False
            self._lidar_chord.reset()
            self.active = False
            self.deadman_was_down = False
            self.lin_tgt = self.ang_tgt = self.lin_cmd = self.ang_cmd = 0.0
        self._lidar_generation = message.session_generation
        self._base_state = message
        self._base_state_at = self.get_clock().now().nanoseconds / 1e9

    def _try_lidar_toggle(self):
        if not self._lidar_toggle_pending or self._takeover_pending:
            return
        # Consume the gesture even on refusal: an ambiguous toggle never retries.
        self._lidar_toggle_pending = False
        now = self.get_clock().now().nanoseconds / 1e9
        if (self._lidar_in_flight or not self._lidar_chord.all_pressed or
                self._last_joy is None or not 0 <= now - self._last_joy <= JOY_TIMEOUT or
                self._lidar_chord.last_stamp is None or
                not 0 <= now - self._lidar_chord.last_stamp <= JOY_TIMEOUT):
            return
        state = self._base_state
        if state is None:
            self.get_logger().warn('LiDAR power requires fresh known base state.')
            return
        age = now - self._base_state_at
        stamped_age = now - (state.header.stamp.sec + state.header.stamp.nanosec / 1e9)
        elapsed = max(age, stamped_age)
        if (not state.link_connected or not state.hardware_state_valid or
                not state.lidar_power_supported or
                not 0 <= age <= BASE_STATE_TIMEOUT or
                not 0 <= stamped_age <= BASE_STATE_TIMEOUT or
                not math.isfinite(state.telemetry_age) or
                not 0 <= state.telemetry_age + elapsed <= BASE_STATE_TIMEOUT or
                state.lidar_state not in ('READY', 'OFF') or
                (state.lidar_state == 'READY' and
                 (not math.isfinite(state.lidar_scan_age) or
                  not 0 <= state.lidar_scan_age + elapsed <= BASE_STATE_TIMEOUT))):
            self.get_logger().warn('LiDAR power requires fresh known commissioned state.')
            return
        if not self.lidar_power.service_is_ready():
            self.get_logger().warn('LiDAR power service is unavailable; repeat the chord to retry.')
            return
        request = SetLidarPower.Request()
        request.request_id = str(uuid4())
        request.enabled = state.lidar_state == 'OFF'
        self._lidar_in_flight = True
        try:
            future = self.lidar_power.call_async(request)
        except Exception as exc:  # noqa: BLE001 - never retry an ambiguous toggle
            self._lidar_in_flight = False
            self.get_logger().error(f'LiDAR power request failed: {exc}')
            return
        future.add_done_callback(self._lidar_power_done)

    def _lidar_power_done(self, future):
        self._lidar_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - no blind retry after lost result
            self.get_logger().error(f'LiDAR power result unavailable: {exc}')
            return
        if not response.accepted or not response.success:
            self.get_logger().warn(f'LiDAR power refused: {response.message}')
        else:
            self.get_logger().info(f'LiDAR power: {response.message}')

    def _edge(self, btns, i):
        return button_pressed(btns, i) and not button_pressed(self.prev_btns, i)

    def _navigation_status(self, message):
        self._nav_status_received = True
        self._nav_goal_active = any(
            status.status in (
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_CANCELING,
            )
            for status in message.status_list)

    def _begin_manual_takeover(self):
        # Always send cancel-all, even when the last status was empty, so a goal
        # accepted concurrently with the RB edge cannot slip through the race.
        self._takeover_pending = True
        self._takeover_generation += 1
        self._next_cancel_retry = 0.0
        self._cancel_unavailable_logged = False
        self.get_logger().info(
            'RB manual takeover requested — canceling any active Nav2 goal.')
        self._request_navigation_cancel()

    def _request_navigation_cancel(self):
        if not self._takeover_pending or self._cancel_in_flight:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now < self._next_cancel_retry:
            return
        if not self.nav_cancel.service_is_ready():
            self._next_cancel_retry = now + NAV_CANCEL_RETRY
            if not self._cancel_unavailable_logged:
                self._cancel_unavailable_logged = True
                self.get_logger().warn(
                    'Nav2 cancel service is unavailable; manual control remains '
                    'available, but priority-8 zero will stay latched after RB '
                    'release until cancellation can be confirmed.')
            return

        # A zero UUID with a zero timestamp is the ROS action protocol's
        # cancel-all request. It is safe when no goal exists.
        request = CancelGoal.Request()
        self._cancel_in_flight = True
        future = self.nav_cancel.call_async(request)
        generation = self._takeover_generation
        future.add_done_callback(
            lambda result: self._navigation_cancel_done(result, generation))

    def _navigation_cancel_done(self, future, generation):
        self._cancel_in_flight = False
        if generation != self._takeover_generation:
            self._request_navigation_cancel()
            return
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - keep manual hold fail-closed
            self.get_logger().error(
                f'Nav2 cancel request failed: {exc}; retaining manual zero hold.')
            self._next_cancel_retry = (
                self.get_clock().now().nanoseconds / 1e9 + NAV_CANCEL_RETRY)
            return
        if int(response.return_code) != 0:
            # CancelGoal returns ERROR_REJECTED when a cancel-all request finds
            # no matching goals. A fresh empty action status proves that case is
            # benign; other rejections retain the fail-closed manual zero hold.
            if self._nav_status_received and not self._nav_goal_active:
                self._takeover_pending = False
                self._cancel_unavailable_logged = False
                self.get_logger().info(
                    'Nav2 reported no active goal during manual takeover.')
                self._try_lidar_toggle()
                return
            self.get_logger().error(
                f'Nav2 rejected manual takeover cancellation '
                f'(return_code={response.return_code}); retaining zero hold.')
            self._next_cancel_retry = (
                self.get_clock().now().nanoseconds / 1e9 + NAV_CANCEL_RETRY)
            return

        self._takeover_pending = False
        self._cancel_unavailable_logged = False
        self.get_logger().info(
            'Nav2 manual takeover cancellation accepted; autonomous motion '
            'will not resume after RB release.')
        self._try_lidar_toggle()

    def joy_cb(self, m):
        b = m.buttons
        a = m.axes
        now = self.get_clock().now().nanoseconds / 1e9
        if self._last_joy is None or not 0 <= now - self._last_joy <= JOY_TIMEOUT:
            self._lidar_toggle_pending = False
        self._last_joy = now
        stamp = m.header.stamp.sec + m.header.stamp.nanosec / 1e9
        toggle = self._lidar_chord.update(a, b, now=now, stamp=stamp)
        if not self._lidar_chord.all_pressed:
            self._lidar_toggle_pending = False
        if not self._lidar_chord.valid:
            self.active = False
            self.deadman_was_down = False
            self.lin_tgt = self.ang_tgt = self.lin_cmd = self.ang_cmd = 0.0
            return
        if toggle:
            self._lidar_toggle_pending = not self._lidar_in_flight
        if self._lidar_chord.block_drive:
            self.lin_tgt = self.ang_tgt = self.lin_cmd = self.ang_cmd = 0.0

        # --- speed trim (edge-triggered, one step per press) ---
        changed = False
        if self._edge(b, BTN_LIN_UP):
            self.max_lin = round(min(self.max_lin + self.lin_step, self.lin_cap), 2); changed = True
        if self._edge(b, BTN_LIN_DOWN):
            self.max_lin = round(max(self.max_lin - self.lin_step, self.min_lin), 2); changed = True
        if self._edge(b, BTN_ANG_UP):
            self.max_ang = round(min(self.max_ang + self.ang_step, self.ang_cap), 2); changed = True
        if self._edge(b, BTN_ANG_DOWN):
            self.max_ang = round(max(self.max_ang - self.ang_step, self.min_ang), 2); changed = True
        if changed:
            self.get_logger().info(f'max_linear={self.max_lin} m/s   max_angular={self.max_ang} rad/s')
        self.prev_btns = list(b)

        # --- deadman ---
        deadman_down = button_pressed(b, BTN_DEADMAN)
        if rising_edge(deadman_down, self.deadman_was_down):
            self._begin_manual_takeover()
        self.deadman_was_down = deadman_down
        self.active = deadman_down
        self._try_lidar_toggle()
        if self._lidar_chord.block_drive:
            return
        if not self.active:
            self.lin_tgt = 0.0
            self.ang_tgt = 0.0
            return

        # --- direction: analog stick, fall back to D-pad if stick centered ---
        lin = apply_deadzone(axis_value(a, AX_LIN), self.deadzone) * LIN_SIGN
        ang = apply_deadzone(axis_value(a, AX_ANG), self.deadzone) * ANG_SIGN
        if lin == 0.0:
            lin = axis_value(a, AX_DPAD_LIN) * DPAD_LIN_SIGN
        if ang == 0.0:
            ang = axis_value(a, AX_DPAD_ANG) * DPAD_ANG_SIGN

        self.lin_tgt = max(-1.0, min(1.0, lin)) * self.max_lin
        self.ang_tgt = max(-1.0, min(1.0, ang)) * self.max_ang

    def tick(self):
        # Joy-loss safety: if /joy stopped arriving (controller unplugged or
        # joy_node died) while RB was held, drop the deadman and zero the target
        # so we ramp to a stop instead of republishing the last command forever.
        now = self.get_clock().now().nanoseconds / 1e9
        if self._last_joy is None or not 0 <= now - self._last_joy <= JOY_TIMEOUT:
            self.active = False
            # A fresh held-RB report after a receiver outage must be treated as
            # a new takeover and issue a new cancellation request.
            self.deadman_was_down = False
            self._lidar_toggle_pending = False
            self._lidar_chord.reset()
            self.lin_tgt = 0.0
            self.ang_tgt = 0.0

        self._request_navigation_cancel()

        # Acceleration-limited ramp toward the target == smooth motion.
        self.lin_cmd = ramp_toward(
            self.lin_cmd, self.lin_tgt, self.accel_lin * self.dt)
        self.ang_cmd = ramp_toward(
            self.ang_cmd, self.ang_tgt, self.accel_ang * self.dt)
        moving = abs(self.lin_cmd) > 1e-3 or abs(self.ang_cmd) > 1e-3

        # Deadman evidence first so the last message of a drive tick stays Twist.
        # Evidence is real manual intent only (held deadman or still coasting);
        # the fail-closed _takeover_pending zero hold is not intent and must
        # not keep the bridge's manual bypass open.
        self.manual_pub.publish(Bool(data=bool(self.active or moving)))
        if self.active or moving or self._takeover_pending:
            # While RB held (manual control) OR still coasting to a stop, keep
            # publishing at a steady rate. A pending takeover adds a fail-closed
            # zero hold after release until Nav2 cancellation is accepted.
            t = Twist()
            t.linear.x = float(self.lin_cmd)
            t.angular.z = float(self.ang_cmd)
            self.pub.publish(t)
            self.was_pub = True
        elif self.was_pub:
            # Fully stopped, RB released, and Nav2 canceled: one explicit zero,
            # then go silent. A new autonomous goal requires an explicit action.
            self.pub.publish(Twist())
            self.was_pub = False


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
