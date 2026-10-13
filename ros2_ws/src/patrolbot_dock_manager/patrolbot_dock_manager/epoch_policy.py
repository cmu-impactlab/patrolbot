"""ROS-free readiness gates for motion and operator-requested pose recovery."""

import math


def epoch_hardware_ready(*, origin_valid=False, recovery_required=True,
                         recovery_stage='', for_pose_initialization=False):
    """Permit seed recovery without granting motion readiness.

    This is only the epoch part of the hardware gate. The caller still checks
    fresh physical telemetry; the pose service still checks operator consent,
    a commissioned dock pose, dock contact, stationary odometry and idle Nav2.
    """
    if origin_valid is not True:
        return False
    if recovery_required is False:
        return True
    return (for_pose_initialization is True
            and recovery_stage in ('wait-operator-seed', 'wait-amcl-pose'))



def lidar_ready(base, *, receipt_age, stamp_age):
    """Fresh measured scans are required independently of power commissioning."""
    if (getattr(base, 'lidar_state', '') != 'READY' or
            not all(math.isfinite(age) and 0 <= age <= .5
                    for age in (receipt_age, stamp_age))):
        return False
    elapsed = max(receipt_age, stamp_age)
    return all(math.isfinite(age) and 0 <= age and age + elapsed <= .5
               for age in (getattr(base, 'lidar_scan_age', -1.),
                           getattr(base, 'telemetry_age', math.inf)))
