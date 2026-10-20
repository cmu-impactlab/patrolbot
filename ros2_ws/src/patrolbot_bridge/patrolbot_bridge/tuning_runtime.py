"""Fresh, explicit observations for the existing tuning mutation owner."""

import math
import threading
import time

from .tuning_activation import Evidence


def fresh(timestamp, now, maximum):
    return (type(timestamp) in (float, int) and math.isfinite(timestamp)
            and 0 <= now - timestamp <= maximum)


class TuningObservations:
    """Track both released/held deadmen and the actual final velocity buffers."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.RLock()
        self.deadmen = {}
        self.velocities = {}
        self.health_at = None
        self.generation = None

    def reset(self, generation):
        with self.lock:
            if generation != self.generation:
                self.generation = generation
                self.deadmen.clear()
                self.velocities.clear()
                self.health_at = None

    def deadman(self, channel, held):
        if channel not in ('physical', 'web', 'web_session') or type(held) is not bool:
            raise ValueError('explicit typed deadman observation required')
        with self.lock:
            self.deadmen[channel] = (held, self.clock())

    def velocity(self, topic, linear, angular, lateral=0.0):
        if topic not in ('/cmd_vel', '/cmd_vel_out'):
            raise ValueError('unknown final velocity topic')
        valid = all(type(v) in (float, int) and math.isfinite(v)
                    for v in (linear, angular, lateral))
        zero = valid and all(abs(v) <= 1e-9 for v in (linear, angular, lateral))
        with self.lock:
            self.velocities[topic] = (zero, self.clock())

    def invalidate(self, *, channel=None, topic=None):
        with self.lock:
            if channel is not None:
                self.deadmen.pop(channel, None)
            if topic is not None:
                self.velocities.pop(topic, None)

    def healthy_publishers(self, generation):
        """Refresh only after fixed publisher identities/readiness are checked."""
        with self.lock:
            self.reset(generation)
            self.health_at = self.clock()

    def evidence(self, **facts):
        with self.lock:
            now = self.clock()
            for channel, prefix in (('physical', 'physical'), ('web', 'web')):
                value = self.deadmen.get(channel)
                current = value is not None and fresh(value[1], now, .5)
                facts[prefix + '_deadman_fresh'] = current
                facts[prefix + '_deadman_held'] = value[0] if current else None
            session = self.deadmen.get('web_session')
            session_fresh = session is not None and fresh(session[1], now, .5)
            if not session_fresh or not facts['web_deadman_fresh']:
                facts['web_deadman_fresh'] = False
                facts['web_deadman_held'] = None
            else:
                facts['web_deadman_held'] = bool(session[0] or facts['web_deadman_held'])
            # A retained zero is usable only while the SAME final publishers
            # remain healthy. Restart/generation change clears every old zero.
            facts['command_buffers_zero'] = (
                fresh(self.health_at, now, .5)
                and all(topic in self.velocities and self.velocities[topic][0]
                        and self.velocities[topic][1] <= now
                        for topic in ('/cmd_vel', '/cmd_vel_out')))
            return Evidence(**facts)
