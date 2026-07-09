#!/usr/bin/env python3
"""
PatrolBot Joystick Teleop — Logitech F710 (XInput, switch on 'X').

Hold **RB (deadman)** to take manual control; release to hand control back to
Nav2 (twist_mux: joy priority 8 > nav 5, joy times out after ~1 s of silence).

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
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

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


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('p3dxJoyTeleop')

        # Live-adjustable speed limits (A/Y, X/B)
        self.declare_parameter('max_linear', 0.35)    # m/s at full stick
        self.declare_parameter('max_angular', 0.7)    # rad/s at full stick
        self.declare_parameter('min_linear', 0.05)
        self.declare_parameter('max_linear_cap', 1.5)
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
        self.prev_btns = []
        self.was_pub = False
        self._last_joy = None                     # time of last /joy msg (safety)

        self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        # Remapped to /input/joy in the launch (twist_mux arbitrates above nav).
        self.pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            'Joy teleop ready (XInput). Hold RB + left stick (or D-pad) to drive. '
            f'A/Y=linear -/+, X/B=angular -/+. max {self.max_lin} m/s / {self.max_ang} rad/s.')

    def _dz(self, v):
        return 0.0 if abs(v) < self.deadzone else v

    def _edge(self, btns, i):
        return (len(btns) > i and btns[i] == 1
                and (len(self.prev_btns) <= i or self.prev_btns[i] == 0))

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
        self.active = len(b) > BTN_DEADMAN and b[BTN_DEADMAN] == 1
        if not self.active:
            self.lin_tgt = 0.0
            self.ang_tgt = 0.0
            return

        # --- direction: analog stick, fall back to D-pad if stick centered ---
        lin = self._dz(a[AX_LIN]) * LIN_SIGN if len(a) > AX_LIN else 0.0
        ang = self._dz(a[AX_ANG]) * ANG_SIGN if len(a) > AX_ANG else 0.0
        if lin == 0.0 and len(a) > AX_DPAD_LIN:
            lin = a[AX_DPAD_LIN] * DPAD_LIN_SIGN
        if ang == 0.0 and len(a) > AX_DPAD_ANG:
            ang = a[AX_DPAD_ANG] * DPAD_ANG_SIGN

        self.lin_tgt = max(-1.0, min(1.0, lin)) * self.max_lin
        self.ang_tgt = max(-1.0, min(1.0, ang)) * self.max_ang

    @staticmethod
    def _ramp(cur, tgt, step):
        if tgt > cur:
            return min(cur + step, tgt)
        if tgt < cur:
            return max(cur - step, tgt)
        return cur

    def tick(self):
        # Joy-loss safety: if /joy stopped arriving (controller unplugged or
        # joy_node died) while RB was held, drop the deadman and zero the target
        # so we ramp to a stop instead of republishing the last command forever.
        now = self.get_clock().now().nanoseconds / 1e9
        if self._last_joy is None or (now - self._last_joy) > JOY_TIMEOUT:
            self.active = False
            self.lin_tgt = 0.0
            self.ang_tgt = 0.0

        # Acceleration-limited ramp toward the target == smooth motion.
        self.lin_cmd = self._ramp(self.lin_cmd, self.lin_tgt, self.accel_lin * self.dt)
        self.ang_cmd = self._ramp(self.ang_cmd, self.ang_tgt, self.accel_ang * self.dt)
        moving = abs(self.lin_cmd) > 1e-3 or abs(self.ang_cmd) > 1e-3

        if self.active or moving:
            # While RB held (manual control) OR still coasting to a stop, keep
            # publishing at a steady rate so the motion is continuous & smooth.
            t = Twist()
            t.linear.x = float(self.lin_cmd)
            t.angular.z = float(self.ang_cmd)
            self.pub.publish(t)
            self.was_pub = True
        elif self.was_pub:
            # Fully stopped and RB released: one explicit zero, then go silent so
            # twist_mux times the joy input out and Nav2 resumes control.
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
