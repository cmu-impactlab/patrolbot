"""
Unit tests for the /odom covariance accumulator.

The bridge published a 36-element block of zeros for both pose and twist
covariance. That is not a neutral "unknown" -- it is the assertion that the
wheel odometry is exact. Any estimator that takes /odom at its word has to
invert that matrix, and a zero matrix is singular, so the claim is not merely
wrong, it is unusable. These tests pin the accumulator that replaces it.
"""

import math

from patrolbot_bridge.odom_covariance import OdomCovariance, UNOBSERVED_VARIANCE
import pytest


# Row-major 6x6 diagonal indices, in the order sensor_msgs/Odometry uses:
# x, y, z, roll, pitch, yaw.
X, Y, Z, ROLL, PITCH, YAW = 0, 7, 14, 21, 28, 35

# Deliberately round numbers -- the point of every test below is a relation
# between entries, never a calibrated value. The real sigmas live in
# config/bridge_params.yaml and come from the operator's measurement.
SIGMAS = {
    'sigma_xy_per_m': 0.01,
    'sigma_yaw_per_rad': 0.02,
    'sigma_yaw_per_m': 0.005,
    'sigma_vx': 0.03,
    'sigma_vyaw': 0.04,
    'min_variance': 1e-6,
    'max_variance': 10.0,
}


def make(**overrides):
    """Build an accumulator from SIGMAS with per-test overrides."""
    params = dict(SIGMAS)
    params.update(overrides)
    return OdomCovariance(**params)


def drive_straight(acc, metres, step=0.1, start=0.0, heading=0.0):
    """
    Feed poses along +x from `start`, in `step` increments, not rotating.

    The first call anchors the accumulator, which by design charges no
    distance for the sample that establishes the reference. Anchoring at
    `start` explicitly keeps the accumulated path exactly `metres`, so the
    tests below can assert real arithmetic rather than an off-by-one-step
    approximation of it.

    Returns the final x, to be passed as the next call's `start`.
    """
    acc.update(start, 0.0, heading)
    travelled = 0.0
    while travelled < metres - 1e-9:
        travelled = min(travelled + step, metres)
        acc.update(start + travelled, 0.0, heading)
    return start + metres


def turn_in_place(acc, radians, step=0.05, start=0.0, position=(0.0, 0.0)):
    """Feed poses that rotate about a fixed point, without translating."""
    acc.update(position[0], position[1], start)
    turned = 0.0
    while turned < radians - 1e-9:
        turned = min(turned + step, radians)
        acc.update(position[0], position[1], start + turned)
    return start + radians


def test_a_fresh_accumulator_is_already_invertible():
    """
    Zero distance travelled is still not zero uncertainty.

    robot_localization inverts the measurement covariance on the very first
    sample it receives. A floor on the diagonal is what keeps the filter from
    dividing by zero before the robot has moved at all.
    """
    acc = make()
    pose = acc.pose_covariance()
    for axis in (X, Y, YAW):
        assert pose[axis] > 0.0


def test_no_diagonal_entry_is_ever_zero():
    """The original defect, stated directly, over a realistic drive."""
    acc = make()
    drive_straight(acc, 3.0)
    turn_in_place(acc, math.pi)
    for matrix in (acc.pose_covariance(), acc.twist_covariance()):
        for axis in (X, Y, Z, ROLL, PITCH, YAW):
            assert matrix[axis] > 0.0


def test_unmeasured_axes_are_marked_unmeasured_not_confident():
    """
    A planar base observes nothing about z, roll or pitch.

    Leaving those at a small number claims the robot knows it is level. The
    large sentinel is the conventional way to say "not observed", and it is
    what stops a 3D consumer from fusing an axis this robot cannot sense.
    """
    acc = make()
    pose = acc.pose_covariance()
    assert pose[Z] == UNOBSERVED_VARIANCE
    assert pose[ROLL] == UNOBSERVED_VARIANCE
    assert pose[PITCH] == UNOBSERVED_VARIANCE


def test_pose_uncertainty_grows_with_distance_travelled():
    """
    Dead reckoning drifts; the covariance has to say so.

    A constant pose covariance would claim the pose is as trustworthy after a
    full patrol as it was at the dock, which is the one thing wheel odometry
    is guaranteed not to be.
    """
    acc = make()
    start = acc.pose_covariance()[X]
    end = drive_straight(acc, 1.0)
    after_one = acc.pose_covariance()[X]
    drive_straight(acc, 1.0, start=end)
    after_two = acc.pose_covariance()[X]

    assert after_one > start
    assert after_two > after_one


def test_distance_growth_is_linear_in_path_length():
    """
    Variance accumulates per metre, so it is linear in metres.

    Independent per-step errors add in variance, not in standard deviation.
    Getting this backwards understates long-run drift badly.
    """
    acc = make(min_variance=0.0)
    end = drive_straight(acc, 1.0)
    one_metre = acc.pose_covariance()[X]
    drive_straight(acc, 3.0, start=end)   # four metres total
    four_metres = acc.pose_covariance()[X]

    assert four_metres == pytest.approx(4.0 * one_metre, rel=1e-6)


def test_path_length_counts_distance_not_displacement():
    """
    Driving out and back is two metres of drift, not zero.

    Displacement from the origin returns to zero; the accumulated error does
    not. Integrating |delta| rather than the net vector is the difference.
    """
    acc = make(min_variance=0.0)
    acc.update(0.0, 0.0, 0.0)
    for x in (0.25, 0.5, 0.75, 1.0):
        acc.update(x, 0.0, 0.0)
    for x in (0.75, 0.5, 0.25, 0.0):
        acc.update(x, 0.0, 0.0)

    there_and_back = acc.pose_covariance()[X]
    assert acc.pose_covariance()[X] > 0.0, 'net displacement is zero here'

    straight = make(min_variance=0.0)
    drive_straight(straight, 2.0, step=0.25)

    assert there_and_back == pytest.approx(straight.pose_covariance()[X],
                                           rel=1e-6)


def test_yaw_uncertainty_grows_with_rotation():
    """
    Heading error is what a gyro-less differential base accrues fastest.

    HasGyro=0 on this base, so yaw comes from differencing two wheel encoders
    and nothing corrects it. Turning has to cost more heading confidence than
    standing still does.
    """
    acc = make()
    before = acc.pose_covariance()[YAW]
    turn_in_place(acc, math.pi)
    assert acc.pose_covariance()[YAW] > before


def test_translation_alone_can_leave_heading_untouched():
    """
    The two growth terms are separable, and sigma_yaw_per_m controls one.

    Set it to zero and pure translation must not inflate yaw. Without this the
    per-metre and per-radian terms could be silently wired to the same
    accumulator and every other test here would still pass.
    """
    acc = make(sigma_yaw_per_m=0.0, min_variance=0.0)
    drive_straight(acc, 5.0)
    assert acc.pose_covariance()[YAW] == pytest.approx(0.0, abs=1e-12)


def test_translation_can_also_bleed_into_heading():
    """
    With the term enabled, driving straight does cost heading confidence.

    Unequal wheel diameters turn a commanded straight line into a slow arc, so
    a base that only ever drives forward still loses heading. This is AMCL's
    alpha4 term, expressed as a covariance.
    """
    acc = make(min_variance=0.0)
    drive_straight(acc, 5.0)
    assert acc.pose_covariance()[YAW] > 0.0


def test_growth_is_capped_so_the_matrix_stays_conditioned():
    """
    Unbounded growth eventually overflows the filter's arithmetic.

    This robot has been up for 19 days between restarts. Linear growth with no
    ceiling would reach numbers that wreck the conditioning of any matrix the
    value is multiplied into, so the cap is not cosmetic.
    """
    # Deliberately enormous sigmas so the cap is reached in a few metres --
    # stepping 5 km at 0.4 m a sample to prove the same point would only be
    # testing that Python can add.
    acc = make(max_variance=0.5, sigma_xy_per_m=1.0,
               sigma_yaw_per_rad=1.0, sigma_yaw_per_m=1.0)
    end = drive_straight(acc, 2.0)
    turn_in_place(acc, math.pi, start=0.0, position=(end, 0.0))
    pose = acc.pose_covariance()
    assert pose[X] == pytest.approx(0.5)
    assert pose[Y] == pytest.approx(0.5)
    assert pose[YAW] == pytest.approx(0.5)


def test_a_pose_discontinuity_is_not_charged_as_distance():
    """
    An SBC restart resets ARIA's pose to the origin in one sample.

    That jump is bookkeeping, not motion. Charged as real distance it would
    slam the covariance into its cap in a single frame and leave it there for
    the rest of the session.
    """
    acc = make(min_variance=0.0)
    drive_straight(acc, 1.0)
    honest = acc.pose_covariance()[X]

    acc.update(250.0, -400.0, 3.0)   # nothing physical moves this far in 44 ms
    assert acc.pose_covariance()[X] == pytest.approx(honest, rel=1e-6)


def test_motion_resumes_normally_after_a_discontinuity():
    """
    Rejecting the jump must not wedge the accumulator.

    The step after a discontinuity is measured from the new pose, so the
    accumulator has to re-anchor on the rejected sample rather than keep
    differencing against the pose from before the jump -- which would reject
    every subsequent sample too, freezing the covariance for good.
    """
    acc = make(min_variance=0.0)
    drive_straight(acc, 1.0)
    acc.update(250.0, -400.0, 3.0)
    before = acc.pose_covariance()[X]

    acc.update(250.05, -400.0, 3.0)
    acc.update(250.10, -400.0, 3.0)
    assert acc.pose_covariance()[X] > before


def test_repeated_jumps_are_rejected_every_time_not_eventually_believed():
    """
    A link that keeps jumping is broken, not driving fast.

    It is tempting to decide that a jump repeated often enough must be real
    motion after all. It must not be: a flapping SBC link produces exactly
    that pattern, and believing it would charge kilometres of imaginary path
    and peg the covariance at its cap for the rest of the session. Rejection
    has to stay rejection no matter how many times the jump repeats.
    """
    acc = make(min_variance=0.0)
    drive_straight(acc, 1.0)
    honest = acc.pose_covariance()[X]

    # The same large delta over and over -- a runaway, not a drive. This is the
    # exact shape a "it repeated, so it must be real" heuristic would start
    # believing, so it is the shape worth pinning.
    for step in range(1, 21):
        acc.update(80.0 * step, 0.0, 0.0)
    # And the same jump alternating back and forth, which is what a flapping
    # link between two poses looks like.
    for _ in range(20):
        acc.update(0.0, 0.0, 0.0)
        acc.update(80.0, 60.0, 0.0)

    assert acc.pose_covariance()[X] == pytest.approx(honest, rel=1e-9)


def test_twist_covariance_is_constant_and_matches_the_configured_sigmas():
    """
    Velocity is measured fresh every frame, so its error does not compound.

    Pose is an integral and drifts; twist is an instantaneous reading and does
    not. Growing the twist covariance would be modelling a process that is not
    happening.
    """
    acc = make()
    at_rest = list(acc.twist_covariance())
    drive_straight(acc, 25.0)
    turn_in_place(acc, 4 * math.pi)

    assert acc.twist_covariance() == at_rest
    assert at_rest[X] == pytest.approx(SIGMAS['sigma_vx'] ** 2)
    assert at_rest[YAW] == pytest.approx(SIGMAS['sigma_vyaw'] ** 2)


def test_sideways_velocity_is_known_to_be_zero_not_unknown():
    """
    A differential base cannot translate sideways, and that is information.

    Marking vy "unobserved" with the large sentinel throws away the
    nonholonomic constraint and invites an estimator to explain away lateral
    error as real sideways motion. The constraint is the single strongest
    thing this base knows about its own velocity -- it belongs in the matrix
    as a tight variance about zero, not as a shrug.

    Pose y is the opposite case: it is genuinely integrated and genuinely
    drifts, so it must keep growing with the rest of the pose block.
    """
    acc = make()
    twist = acc.twist_covariance()
    assert 0.0 < twist[Y] <= SIGMAS['sigma_vx'] ** 2
    assert twist[Y] != UNOBSERVED_VARIANCE

    before = acc.pose_covariance()[Y]
    drive_straight(acc, 4.0)
    assert acc.pose_covariance()[Y] > before
    assert acc.pose_covariance()[Y] == acc.pose_covariance()[X]


def test_both_matrices_are_36_element_row_major_with_zero_cross_terms():
    """
    ROS covariance blocks are flat row-major 6x6, and these axes are independent.

    A short list, or values off the diagonal that were never estimated, are
    both read by consumers without complaint and quietly corrupt the result.
    """
    acc = make()
    drive_straight(acc, 2.0)
    for matrix in (acc.pose_covariance(), acc.twist_covariance()):
        assert len(matrix) == 36
        assert all(isinstance(v, float) for v in matrix)
        diagonal = {X, Y, Z, ROLL, PITCH, YAW}
        assert all(matrix[i] == 0.0 for i in range(36) if i not in diagonal)


def test_reset_returns_the_pose_covariance_to_the_floor():
    """A new SBC session starts a new dead-reckoning run from zero drift."""
    fresh = make()
    acc = make()
    drive_straight(acc, 8.0)
    turn_in_place(acc, math.pi)
    assert acc.pose_covariance() != fresh.pose_covariance()

    acc.reset()
    assert acc.pose_covariance() == fresh.pose_covariance()


def test_the_accumulator_never_hands_out_its_own_mutable_state():
    """A consumer that edits the returned list must not corrupt the next one."""
    acc = make()
    first = acc.pose_covariance()
    first[X] = 12345.0
    assert acc.pose_covariance()[X] != 12345.0
