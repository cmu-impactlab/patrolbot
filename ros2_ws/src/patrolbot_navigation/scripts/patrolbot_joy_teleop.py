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

  RB (5)  = deadman / enable
  left stick Y (axis 1) or D-pad Y (axis 7) = forward / reverse
  left stick X (axis 0) or D-pad X (axis 6) = turn
  A (0) / Y (3) = decrease / increase MAX LINEAR speed
  X (2) / B (1) = decrease / increase MAX ANGULAR speed

Button/axis indices verified live on the robot 2026-06-27. If forward/back or
left/right is reversed, flip the matching *_SIGN constant below and restart.
"""
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from sensor_msgs.msg import Joy

from joystick_policy import (
    apply_deadzone,
    axis_value,
    button_pressed,
    ramp_toward,
    rising_edge,
)

# ---- Button map (Logitech F710, XInput) ----
BTN_DEADMAN = 5    # RB
BTN_LIN_DOWN = 0   # A  -> slower linear
BTN_LIN_UP = 3     # Y  -> faster linear
BTN_ANG_DOWN = 2   # X  -> slower angular
BTN_ANG_UP = 1     # B  -> faster angular

# ---- Axis map (analog left stick + D-pad fallback) ----
AX_LIN = 1         # left stick Y
AX_ANG = 0         # left stick X
AX_DPAD_LIN = 7    # D-pad Y
AX_DPAD_ANG = 6    # D-pad X

# ---- Direction signs (flip if a direction is reversed) ----
LIN_SIGN = 1.0          # stick up -> forward
ANG_SIGN = 1.0          # stick left -> turn left (CCW, +z)
DPAD_LIN_SIGN = -1.0    # D-pad up is -1 on joydev -> forward
DPAD_ANG_SIGN = -1.0    # D-pad left is -1 -> turn left

# Safety: if no /joy message arrives for this long (controller unplugged or
# joy_node died) we drop the deadman and ramp to a stop — never keep republishing
# the last command. joy_node autorepeats at ~20 Hz, so 0.4 s = several missed.
JOY_TIMEOUT = 0.4
NAV_CANCEL_RETRY = 0.25


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

        self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self._navigation_status, qos_profile_action_status_default)
        # Remapped to /input/joy in the launch (twist_mux arbitrates above nav).
        self.pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.nav_cancel = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            'Joy teleop ready (XInput). Hold RB + left stick (or D-pad) to drive. '
            f'A/Y=linear -/+, X/B=angular -/+. max {self.max_lin} m/s / {self.max_ang} rad/s.')

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
        future.add_done_callback(self._navigation_cancel_done)

    def _navigation_cancel_done(self, future):
        self._cancel_in_flight = False
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

    def joy_cb(self, m):
        b = m.buttons
        a = m.axes
        self._last_joy = self.get_clock().now().nanoseconds / 1e9

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
        if self._last_joy is None or (now - self._last_joy) > JOY_TIMEOUT:
            self.active = False
            # A fresh held-RB report after a receiver outage must be treated as
            # a new takeover and issue a new cancellation request.
            self.deadman_was_down = False
            self.lin_tgt = 0.0
            self.ang_tgt = 0.0

        self._request_navigation_cancel()

        # Acceleration-limited ramp toward the target == smooth motion.
        self.lin_cmd = ramp_toward(
            self.lin_cmd, self.lin_tgt, self.accel_lin * self.dt)
        self.ang_cmd = ramp_toward(
            self.ang_cmd, self.ang_tgt, self.accel_ang * self.dt)
        moving = abs(self.lin_cmd) > 1e-3 or abs(self.ang_cmd) > 1e-3

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
