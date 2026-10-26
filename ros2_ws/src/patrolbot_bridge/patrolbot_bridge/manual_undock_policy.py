"""ROS-free, release-armed F710 Start gesture for the fixed manual undock."""

import math

START_BUTTON = 7  # F710 XInput Start; Back is 6, RB is 5.
HOLD_SECONDS = 1.0
INPUT_TIMEOUT = .4


def neutral_input(axes, buttons, *, allow_start=False):
    """Require centered sticks/D-pad, released triggers and no other buttons."""
    return (len(axes) >= 8 and len(buttons) > START_BUTTON
            and all(math.isfinite(v) and -1 <= v <= 1 for v in axes)
            and all(v in (0, 1) for v in buttons)
            and all(abs(axes[i]) <= .12 for i in (0, 1, 3, 4, 6, 7))
            and axes[2] >= .5 and axes[5] >= .5
            and not any(v for i, v in enumerate(buttons)
                        if not (allow_start and i == START_BUTTON)))


class StartGesture:
    """One request per deliberate hold; a subsequent fresh press cancels."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.armed = False
        self.pressed = True  # A held button at startup/reconnect is not a press.
        self.since = None
        self.last_at = self.last_stamp = None

    def update(self, axes, buttons, *, now, stamp, busy=False):
        """Return start/cancel once; input loss does not cancel a running job."""
        valid = (len(axes) >= 8 and len(buttons) > START_BUTTON
                 and all(math.isfinite(v) and -1 <= v <= 1 for v in axes)
                 and all(v in (0, 1) for v in buttons)
                 and math.isfinite(now) and math.isfinite(stamp)
                 and 0 <= now - stamp <= INPUT_TIMEOUT
                 and (self.last_stamp is None or stamp > self.last_stamp))
        if not valid:
            self.reset()
            return None
        if self.last_at is not None and not 0 <= now - self.last_at <= INPUT_TIMEOUT:
            self.reset()
        self.last_at, self.last_stamp = now, stamp
        down = bool(buttons[START_BUTTON])
        rising = down and not self.pressed
        self.pressed = down
        if busy:
            self.since = None
            self.armed = False
            return 'cancel' if rising else None
        if not down:
            self.since = None
            self.armed = neutral_input(axes, buttons)
        elif not neutral_input(axes, buttons, allow_start=True):
            self.armed, self.since = False, None
        elif self.armed:
            self.since = now if self.since is None else self.since
            if now - self.since >= HOLD_SECONDS:
                self.armed, self.since = False, None
                return 'start'
        return None
