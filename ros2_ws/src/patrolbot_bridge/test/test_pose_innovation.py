import math
import pytest
from patrolbot_bridge.pose_innovation import OdomHistory, innovation, propagate


def test_measured_displacement_and_wrapped_heading():
    assert innovation((0,0,0), (.11,0,0), .1, math.radians(3))
    assert innovation((0,0,0), (0,0,math.radians(3.1)), .1, math.radians(3))
    assert not innovation((0,0,math.pi-.01), (0,0,-math.pi+.01), .1, math.radians(3))


def test_propagation_respects_seed_rotation_and_nonzero_odom_origin():
    assert propagate((10,20,math.pi/2), (3,4,0), (4,4,.1)) == pytest.approx((10,21,math.pi/2+.1))


def test_history_interpolates_only_same_origin_bracket_and_bounds_gap():
    h=OdomHistory()
    h.add(1000000000, (0,0,math.pi-.02), 'a')
    h.add(1100000000, (1,0,-math.pi+.02), 'a')
    assert h.at(1050000000, 'a') == pytest.approx((.5,0,-math.pi))
    for stamp, origin in [(900000000,'a'),(1200000000,'a'),(1050000000,'b')]:
        with pytest.raises(ValueError): h.at(stamp,origin)
    h.add(2000000000, (1,0,0), 'a')
    with pytest.raises(ValueError): h.at(1500000000,'a')


def test_history_regression_and_origin_change_invalidate_old_evidence():
    h=OdomHistory();h.add(100,(0,0,0),'a');h.add(90,(0,0,0),'a')
    with pytest.raises(ValueError): h.at(100,'a')
    h.add(110,(0,0,0),'b')
    with pytest.raises(ValueError): h.at(90,'a')


def test_planar_covariance_rejects_nonphysical_correlation():
    from patrolbot_bridge.pose_innovation import valid_planar_covariance
    c=[0.]*36
    for i in (0,7,35):c[i]=.0009
    assert valid_planar_covariance(c)
    c[1]=.0001
    assert not valid_planar_covariance(c)
    c[6]=c[1]=.01
    assert not valid_planar_covariance(c)


def test_amcl_covariance_roundoff_at_large_map_coordinates_is_valid():
    from patrolbot_bridge.pose_innovation import valid_planar_covariance
    covariance = [0.] * 36
    covariance[0], covariance[7], covariance[35] = .04078533539012463, .2226248095464189, .015069456232981525
    covariance[1], covariance[6] = .014099728983739486, .014099728985558476
    assert valid_planar_covariance(covariance)
    covariance[6] += 1e-6
    assert not valid_planar_covariance(covariance)
