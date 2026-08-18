"""
Pure wheel-odometry covariance accumulation for /odom.

The bridge published 36 zeros for both covariance blocks. Zero covariance is
not "unknown" -- it is the claim that the wheel odometry is exact, and it is a
singular matrix, so any estimator that tries to weight the measurement by
inverting it fails outright. This module produces the honest replacement.

The model is deliberately small. Dead-reckoning error accumulates with the
distance driven and the angle turned, so variance grows linearly in each,
independently, from a floor and up to a ceiling. There is no cross-correlation
term because nothing here estimates one; the off-diagonal entries stay zero
rather than being invented.
"""

import math


# Conventional "this axis is not observed" marker. A planar differential base
# senses nothing about height, roll or pitch, and saying so with a huge
# variance is what stops a 3D consumer from fusing an axis that carries no
# information.
UNOBSERVED_VARIANCE = 1_000_000.0

# Diagonal indices into a row-major 6x6 covariance, in the order
# geometry_msgs/PoseWithCovariance uses: x, y, z, roll, pitch, yaw.
_X = 0
_Y = 7
_Z = 14
_ROLL = 21
_PITCH = 28
_YAW = 35


class OdomCovariance:
    """Track how far the wheel odometry has had to guess since its last reset."""

    def __init__(
            self, *, sigma_xy_per_m, sigma_yaw_per_rad, sigma_yaw_per_m,
            sigma_vx, sigma_vyaw, min_variance, max_variance,
            max_step_m=0.5, max_step_rad=1.0):
        """
        Configure growth rates, the variance clamp, and jump rejection.

        The sigmas are per-unit-of-motion, not absolute: sigma_xy_per_m is the
        1-sigma position error contributed by one metre driven, and
        sigma_yaw_per_rad the 1-sigma heading error contributed by one radian
        turned. They come from the operator's measurement, via
        config/bridge_params.yaml -- never from a guess in code.
        """
        self._sigma_xy_per_m = float(sigma_xy_per_m)
        self._sigma_yaw_per_rad = float(sigma_yaw_per_rad)
        self._sigma_yaw_per_m = float(sigma_yaw_per_m)
        self._sigma_vx = float(sigma_vx)
        self._sigma_vyaw = float(sigma_vyaw)
        self._min_variance = float(min_variance)
        self._max_variance = float(max_variance)
        self._max_step_m = float(max_step_m)
        self._max_step_rad = float(max_step_rad)
        self.reset()

    def update(self, x, y, th):
        """
        Fold one absolute planar pose sample into the accumulated motion.

        Distance is integrated as path length, not displacement: driving out
        and back is two metres of accumulated error, not zero.
        """
        pose = (float(x), float(y), float(th))
        if self._previous_pose is None:
            # The sample that establishes the reference measures nothing. At
            # ~22.7 Hz it costs at most one frame of motion.
            self._previous_pose = pose
            return

        previous_x, previous_y, previous_th = self._previous_pose
        step_m = math.hypot(pose[0] - previous_x, pose[1] - previous_y)
        step_rad = self._wrapped_delta(pose[2] - previous_th)

        # Re-anchor unconditionally, including on a rejected step. Holding the
        # pre-jump reference would make every healthy sample afterwards look
        # discontinuous too, and the accumulator would freeze for good.
        self._previous_pose = pose

        if not math.isfinite(step_m) or not math.isfinite(step_rad):
            return
        # A step this large is not motion. The bridge already withdraws
        # odom->base_link once the pose is 0.5 s stale, and this base tops out
        # near 1 m/s, so a real sample can never move more than ~0.5 m without
        # the transform having been withdrawn first. Anything bigger is an SBC
        # restart resetting ARIA's pose to the origin, or a corrupt frame.
        #
        # Rejection is unconditional and stays that way however often a jump
        # repeats: a flapping link produces exactly that pattern, and treating
        # a repeated jump as real motion would charge kilometres of imaginary
        # path and peg the covariance at its ceiling for the whole session.
        if step_m > self._max_step_m or abs(step_rad) > self._max_step_rad:
            return

        self._path_length += step_m
        self._rotation_length += abs(step_rad)

    def pose_covariance(self):
        """Return a fresh row-major 6x6 covariance for the integrated pose."""
        variance_xy = self._clamped(
            self._sigma_xy_per_m ** 2 * self._path_length)
        variance_yaw = self._clamped(
            self._sigma_yaw_per_rad ** 2 * self._rotation_length +
            self._sigma_yaw_per_m ** 2 * self._path_length)

        covariance = [0.0] * 36
        covariance[_X] = variance_xy
        covariance[_Y] = variance_xy
        covariance[_Z] = UNOBSERVED_VARIANCE
        covariance[_ROLL] = UNOBSERVED_VARIANCE
        covariance[_PITCH] = UNOBSERVED_VARIANCE
        covariance[_YAW] = variance_yaw
        return covariance

    def twist_covariance(self):
        """
        Return a fresh row-major 6x6 covariance for the instantaneous twist.

        Constant, unlike the pose block. Velocity is measured afresh every
        frame rather than integrated, so its error does not compound; growing
        it would be modelling a process that is not happening.
        """
        covariance = [0.0] * 36
        covariance[_X] = self._sigma_vx ** 2
        # vy is not unobserved -- it is constrained. A differential base has no
        # mechanism for sideways translation, so the honest statement is "zero,
        # to within the floor", not the sentinel used for the axes this robot
        # genuinely cannot sense. Handing an estimator the sentinel here would
        # let it explain lateral residuals away as real crabbing motion.
        covariance[_Y] = self._min_variance
        covariance[_Z] = UNOBSERVED_VARIANCE
        covariance[_ROLL] = UNOBSERVED_VARIANCE
        covariance[_PITCH] = UNOBSERVED_VARIANCE
        covariance[_YAW] = self._sigma_vyaw ** 2
        return covariance

    def reset(self):
        """
        Begin a new dead-reckoning run with no accumulated motion.

        Called when the SBC session changes: the pose stream restarts from
        ARIA's origin, so the drift accrued against the old one is finished.
        """
        self._path_length = 0.0
        self._rotation_length = 0.0
        self._previous_pose = None

    def _clamped(self, accumulated):
        """
        Apply the variance floor and ceiling.

        The floor keeps the matrix invertible before the robot has moved. The
        ceiling matters because this robot runs for weeks between restarts:
        unbounded linear growth would reach magnitudes that wreck the
        conditioning of anything the value is multiplied into.
        """
        return min(self._min_variance + accumulated, self._max_variance)

    @staticmethod
    def _wrapped_delta(delta):
        """
        Return the shortest signed angular difference.

        Without this, a heading crossing +/-pi reads as a ~6.28 rad turn and
        injects a full spurious rotation into the accumulator.
        """
        return (delta + math.pi) % (2.0 * math.pi) - math.pi
