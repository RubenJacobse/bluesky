"""
Test the objects and functions defined in plugins/thesis/area.py

© Ruben Jacobse, 2019
"""

# Third-party imports
import pytest
import numpy as np

# Thesis imports
import plugins.thesis.geo as geo

# Error tolerances for floating point comparisons
DIFF_DEG = 0.01  # [deg] Error tolerance for angle comparison
DIFF_DIST = 0.1  # [NM] Error tolerance for distance comparison
DIFF_VEL = 0.1  # [m/s] Error tolerance for velocity comparison

# Conversion factors
NM_TO_KM = 1.852  # Conversion factor from nautical miles to kilometers
KM_TO_NM = 1/1.852  # Conversion factor from kilometers to nautical miles


def test_crs_middle():
    """ Test the crs_middle function that returns the bisector course
        dividing the angle between two courses in half. """

    # Test result when crs_left_tangent < crs_right_tangent
    assert geo.crs_middle(0, 360) == 180
    assert geo.crs_middle(10, 50) == 30
    assert geo.crs_middle(160, 200) == 180

    # Test result when crs_right_tangent < crs_left_tangent
    assert geo.crs_middle(350, 10) == 0
    assert geo.crs_middle(330, 0) == 345
    assert geo.crs_middle(340, 40) == 10

    # Test rsult when crs_left_tangent = crs_right_tangent
    assert geo.crs_middle(220, 220) == 220
    assert geo.crs_middle(360, 360) == 0


def test_crs_is_between_true():
    """ Test the crs_is_between function that checks if a given course
        lies between a left and right most course. """

    # These calls should return True
    assert geo.crs_is_between(10, 0, 30)
    assert geo.crs_is_between(0, 359, 1)
    assert geo.crs_is_between(350, 190, 170)
    assert geo.crs_is_between(340, 330, 20)


def test_crs_is_between_false():
    """ Test the crs_is_between function that checks if a given course
        lies between a left and right most course. """

    # Thes calls should return False
    assert not geo.crs_is_between(0, 10, 20)
    assert not geo.crs_is_between(340, 355, 190)
    assert not geo.crs_is_between(22, 33.4, 359.1)
    assert not geo.crs_is_between(220, 220, 220)
    assert not geo.crs_is_between(180, 181, 179)


def test_enu2crs():
    """ Test the function that converts ENU angles to compass angles. """

    enu = np.array([-180, -135, -90, -45, 0, 45, 90, 135, 180])
    crs_corr = np.array([270, 225, 180, 135, 90, 45, 0, 315, 270])
    crs = geo.enu2crs(enu)

    assert np.array_equal(crs, crs_corr)


def test_ned2crs():
    """ Test the function that converts ENU angles to compass angles. """

    ned = np.array([-180, -135, -90, -45, 0, 45, 90, 135, 180])
    crs_corr = np.array([180, 225, 270, 315, 0, 45, 90, 135, 180])
    crs = geo.ned2crs(ned)

    assert np.array_equal(crs, crs_corr)


def test_crs_closest_scalar():
    """ Test the function that takes two courses and returns the course
        with the smallest angle difference with respect to a reference 
        course. """

    # Test for scalar input
    ref = 340
    crs_a = 10
    crs_b = 30

    crs = geo.crs_closest(ref, crs_a, crs_b)
    assert crs == crs_a


def test_crs_closest_numpy_scalar():
    """ Test the function that takes two courses and returns the course
        with the smallest angle difference with respect to a reference 
        course. """

    # Test for single value numpy array input
    ref = np.array([340])
    crs_a = np.array([10])
    crs_b = np.array([30])
    crs = geo.crs_closest(ref, crs_a, crs_b)

    crs_correct = np.array([10])
    assert np.array_equal(crs, crs_correct)


def test_crs_closest_numpy_vector():
    """ Test the function that takes two courses and returns the course
        with the smallest angle difference with respect to a reference 
        course. """

    # Test for single value numpy array input
    ref = np.array([340, 20, 0, 170, 185])
    crs_a = np.array([10, 360, 340, 350, 350])
    crs_b = np.array([30, 45, 10, 20, 10])
    crs = geo.crs_closest(ref, crs_a, crs_b)

    crs_correct = np.array([10, 360, 10, 20, 350])
    assert np.array_equal(crs, crs_correct)
