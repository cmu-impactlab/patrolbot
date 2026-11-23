"""Validated raw hardware evidence, independent of AMCL publication ordering."""

import math

from .odom_epoch import parse_frame_epoch
from .odom_validation import OdomSample, OdomValidator, parse_odom_source
from .scan_timing import parse_laser_age


class ManualTelemetry:
    """Never publish TF/odometry, acknowledge AMCL reset or change an origin."""

    def __init__(self):
        self.origin = None
        self.session = None
        self._sample_session = None
        self.sample = None
        self.received = self.scan_at = None
        self.validator = OdomValidator(max_linear_speed=.30, max_angular_speed=1.20,
                                       expected_period=.05)

    def observe(self, line, received, *, now, session, epoch, source_timing=False):
        self.sample = None
        self.received = self.scan_at = None
        try:
            if not math.isfinite(received) or not 0 <= now - received <= .5:
                return
            if parse_frame_epoch(line) != epoch:
                return
            if epoch != self.origin:
                self.validator.reset_epoch()
                self._sample_session = None
            self.origin, self.session = epoch, session
            fields = line.split('|')
            if not fields[0].startswith('ODOM:') or not fields[1].startswith('LASER:'):
                return
            values = tuple(float(v) for v in fields[0][5:].split(','))
            if len(values) != 5:
                return
            sample = OdomSample(*values)
            source = parse_odom_source(fields[2:], required=source_timing)
            validation = self.validator.check_and_accept(
                sample, received, source=source, queue_age=now - received,
            )
            if not validation.accepted:
                return
            if validation.repeated:
                if self._sample_session != session:
                    return
            else:
                self._sample_session = session
            scan = fields[1][6:].rstrip(',').split(',')
            if len(scan) < 2:
                return
            ranges = [float(v) for v in scan]
            if any(not math.isfinite(v) or v < 0 for v in ranges):
                return
            age = parse_laser_age(line)
            if not age.accepted or age.legacy or age.age_ms is None:
                return
            scan_at = received - age.age_ms / 1000.
            if not 0 <= now - scan_at <= .5:
                return
            self.sample, self.received, self.scan_at = sample, validation.fresh_at, scan_at
        except (IndexError, ValueError, TypeError, OverflowError):
            return

    def ready(self, *, now, session, epoch):
        return bool(self.sample is not None and self.session == session and self.origin == epoch
                    and self.received is not None and self.scan_at is not None
                    and 0 <= now - self.received <= .5 and 0 <= now - self.scan_at <= .5)
