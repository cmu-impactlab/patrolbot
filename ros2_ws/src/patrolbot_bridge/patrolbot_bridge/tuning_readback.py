"""Read-only parameter observations, independent of saved state and activation."""

import math
import threading
import time


MAX_AGE_S = 3.
READBACK_NODES = {
    name: name for name in (
        'teleop_velocity_smoother', 'amcl', 'controller_server', 'velocity_smoother',
        'behavior_server', 'patrolbot_joy_teleop', 'patrolbot_safety_watchdog',
        'collision_monitor', 'patrolbot_dock_manager', 'cmd_vel_mux')
}
READBACK_NODES['local_costmap.local_costmap'] = 'local_costmap/local_costmap'


def observed_value(entry, value):
    index = entry['source'].get('index')
    if index is not None:
        if not isinstance(value, (list, tuple)) or len(value) <= index:
            return None
        value = value[index]
    kind = entry['type']
    if kind == 'boolean':
        return value if type(value) is bool else None
    if kind == 'string':
        return value if isinstance(value, str) and len(value) <= 256 else None
    if type(value) not in (float, int) or not math.isfinite(value):
        return None
    if kind == 'integer' and (type(value) is not int or abs(value) > 2**53 - 1):
        return None
    # Readback can reveal drift outside write bounds; never clamp the observation.
    return value


class ParameterObserver:
    """Poll fixed ROS GetParameters only; never initialize state or set parameters."""

    def __init__(self, index, get, generation, *, clock=time.monotonic):
        self.index, self.get, self.generation, self.clock = index, get, generation, clock
        self._lock = threading.Lock()
        self._sample = None

    def poll(self):
        if not self._lock.acquire(blocking=False):
            return
        try:
            started, generation = self.clock(), self.generation()
            grouped = {}
            values = dict.fromkeys(self.index)
            for entry in self.index.values():
                source = entry['source']
                if source['node'] in READBACK_NODES:
                    grouped.setdefault(source['node'], set()).add(source['key'])
            actual = {}
            for node, names in grouped.items():
                try:
                    actual[node] = self.get(node, sorted(names))
                except Exception:
                    actual[node] = {}
            for pid, entry in self.index.items():
                source = entry['source']
                values[pid] = observed_value(
                    entry, actual.get(source['node'], {}).get(source['key']))
            linear, angular = values.get('max_linear_m_s'), values.get('max_angular_rad_s')
            limits = None
            if (type(linear) in (float, int) and 0 <= linear <= .30
                    and type(angular) in (float, int) and 0 <= angular <= .70):
                limits = {'max_linear_m_s': linear, 'max_angular_rad_s': angular}
            self._sample = (started, generation, values, limits)
        finally:
            self._lock.release()

    def current(self):
        sample = self._sample
        if sample is None:
            return None
        started, generation, values, limits = sample
        age = self.clock() - started
        if not 0 <= age <= MAX_AGE_S or generation != self.generation():
            return None
        return {'source': 'ros_parameters', 'age_s': age,
                'hardware_session_generation': generation, 'values': dict(values),
                'limits': dict(limits) if limits is not None else None}
