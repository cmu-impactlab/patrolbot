"""Measured scanner freshness; OFF describes a command, not electrical readback."""
import math
import re


class LidarState:
    def __init__(self):
        self.reset()

    def reset(self):
        self.state, self.supported, self.age, self.received = 'UNKNOWN', False, -1., None

    def update(self, wire, received):
        if self.received is not None and received < self.received:
            return
        self.reset()
        fields = wire.split(',')
        if (len(fields) != 3 or fields[0] not in
                ('UNKNOWN', 'OFF', 'STARTING', 'READY', 'FAULT') or
                fields[1] not in ('0', '1') or
                re.fullmatch(r'-1|[0-9]+', fields[2]) is None or
                not math.isfinite(received)):
            raise ValueError('invalid LIDAR telemetry')
        try:
            age = int(fields[2]) / 1000.
        except (ValueError, OverflowError) as exc:
            raise ValueError('invalid LIDAR acquisition age') from exc
        if fields[0] == 'READY' and age < 0:
            raise ValueError('READY requires measured acquisition age')
        self.state, self.supported = fields[0], fields[1] == '1'
        self.age, self.received = age if fields[2] != '-1' else -1., received

    def snapshot(self, now):
        elapsed = now - self.received if self.received is not None else math.inf
        if not math.isfinite(elapsed) or not 0. <= elapsed <= .5:
            return 'UNKNOWN', False, -1.
        age = self.age + elapsed if self.age >= 0 else -1.
        state = 'FAULT' if self.state == 'READY' and age > .5 else self.state
        return state, self.supported, age

    def ready(self, now):
        state, _, age = self.snapshot(now)
        return state == 'READY' and 0. <= age <= .5
