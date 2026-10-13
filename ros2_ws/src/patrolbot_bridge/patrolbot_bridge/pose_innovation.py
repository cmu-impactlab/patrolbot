"""Timestamped planar odometry comparisons; never extrapolate missing evidence."""
from collections import deque
import math


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def propagate(seed, start, end):
    angle = seed[2] - start[2]
    dx, dy = end[0] - start[0], end[1] - start[1]
    return (seed[0] + math.cos(angle)*dx - math.sin(angle)*dy,
            seed[1] + math.sin(angle)*dx + math.cos(angle)*dy,
            wrap(seed[2] + wrap(end[2] - start[2])))


def innovation(predicted, estimated, position_limit, heading_limit):
    if not all(math.isfinite(v) for v in (*predicted, *estimated)):
        return 'non-finite pose comparison'
    position = math.hypot(estimated[0]-predicted[0], estimated[1]-predicted[1])
    heading = abs(wrap(estimated[2]-predicted[2]))
    if position > position_limit + 1e-12 or heading > heading_limit + 1e-12:
        return (f'predicted={predicted} estimated={estimated} position_error_m={position:.6f} '
                f'heading_error_rad={heading:.6f} limits=({position_limit},{heading_limit})')
    return ''


class OdomHistory:
    def __init__(self):
        self.samples = deque(maxlen=12000)
        self.origin = None

    def add(self, stamp, pose, origin):
        if not all(math.isfinite(v) for v in pose):
            raise ValueError('non-finite odometry')
        if origin != self.origin or (self.samples and stamp < self.samples[-1][0]):
            self.samples.clear()
        self.origin = origin
        if self.samples and stamp == self.samples[-1][0]:
            self.samples.pop()
        self.samples.append((stamp, tuple(pose)))

    def at(self, stamp, origin):
        if origin != self.origin:
            raise ValueError('odometry origin mismatch')
        previous = None
        for current in self.samples:
            if current[0] == stamp:
                return current[1]
            if current[0] > stamp:
                if previous is None or current[0]-previous[0] > 500000000:
                    break
                fraction = (stamp-previous[0])/(current[0]-previous[0])
                a,b = previous[1],current[1]
                return (a[0]+fraction*(b[0]-a[0]), a[1]+fraction*(b[1]-a[1]),
                        wrap(a[2]+fraction*wrap(b[2]-a[2])))
            previous = current
        raise ValueError('timestamped odometry history unavailable')


def valid_planar_covariance(covariance):
    if len(covariance) != 36 or not all(math.isfinite(v) for v in covariance):
        return False
    axes = (0, 1, 5)
    for a in axes:
        if covariance[6*a+a] <= 0:
            return False
        for b in axes:
            if not math.isclose(covariance[6*a+b], covariance[6*b+a], rel_tol=1e-9, abs_tol=1e-10):
                return False
    a,b,c = covariance[0],covariance[1],covariance[5]
    d,e,f = covariance[7],covariance[11],covariance[35]
    return (a*d-b*b >= 0 and a*f-c*c >= 0 and d*f-e*e >= 0 and
            a*d*f+2*b*c*e-a*e*e-d*c*c-f*b*b >= 0)
