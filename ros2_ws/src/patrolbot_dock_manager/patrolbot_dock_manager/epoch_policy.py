"""ROS-free epoch gate for motion and operator-requested pose recovery."""


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
