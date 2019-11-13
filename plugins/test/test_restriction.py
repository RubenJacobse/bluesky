"""
Test the objects and functions defined in plugins/thesis/restriction.py

© Ruben Jacobse, 2019
"""

# Third-party imports
import pytest
import numpy as np
import shapely.geometry as spgeom

# Thesis imports
from plugins.thesis.restriction import AreaRestriction

# Error tolerances for floating point comparisons
DIFF_DEG = 0.01 # [deg] Error tolerance for angle comparison
DIFF_DIST = 0.1 # [NM] Error tolerance for distance comparison
DIFF_VEL = 0.1 # [m/s] Error tolerance for velocity comparison

# Conversion factors
NM_TO_KM = 1.852 # Conversion factor from nautical miles to kilometers
KM_TO_NM = 1/1.852 # Conversion factor from kilometers to nautical miles

def test_raa_init(areafilter_):
    """ Tests if the AreaRestriction class is initialized correctly. """

    test_id = "RAA_01"
    test_status = True
    test_gseast = 10
    test_gsnorth = -30

    # Vertices in ccw order (lon,lat): (-1,-1), (1,-1), (1,1), (-1,1), (-1,-1)
    test_coords = [-1, -1, -1, 1, 1, 1, 1, -1, -1, -1]

    raa = AreaRestriction(test_id, test_status, test_coords)

    assert raa.area_id == test_id
    assert raa.status == test_status
    assert raa.coords == test_coords

    assert raa.ring.is_valid
    assert raa.ring.is_closed
    assert raa.ring.is_ccw
    assert raa.poly.is_valid
    assert raa.poly.exterior.coords.xy == raa.ring.coords.xy

def test_raa_is_ccw(areafilter_):
    """ Test the function that checks if a coordinate sequence is
        defined in counter-clockwise order. """

    # Test some coordinate lists
    coords1 = [0, 0, 0, 1, 1, 1, 1, 0, 0, 0]  # ccw
    coords2 = [0, 0, 1, 0, 1, 1, 0, 1, 0, 0]  # not ccw
    coords3 = [-1, -1, 1, -1, 1, 1, -1, 1, -1, -1]  # not ccw
    coords4 = [-1, -1, -1, 1, 1, 1, 1, -1, -1, -1]  # ccw

    # Create areas
    raa1 = AreaRestriction("RAA1", True, coords1)
    raa2 = AreaRestriction("RAA2", True, coords2)
    raa3 = AreaRestriction("RAA3", True, coords3)
    raa4 = AreaRestriction("RAA4", True, coords4)

    # Verify both are processed correctly
    assert raa1.ring.is_ccw
    assert raa1.coords == coords1
    assert raa2.ring.is_ccw
    assert raa2.coords != coords2
    assert raa1.coords == raa2.coords

    assert raa3.ring.is_ccw
    assert raa3.coords != coords3
    assert raa4.ring.is_ccw
    assert raa4.coords == coords4
    assert raa3.coords == raa4.coords

    assert raa1.is_ccw(coords1)
    assert raa1.is_ccw(raa1.coords)
    assert not raa2.is_ccw(coords2)
    assert raa1.is_ccw(raa2.coords)
    assert not raa3.is_ccw(coords3)
    assert raa3.is_ccw(raa3.coords)
    assert raa4.is_ccw(coords4)
    assert raa4.is_ccw(raa4.coords)


def test_raa_check_poly(areafilter_):
    """ Test the function that ensures a user-entered polygon is defined by a
        counterclockwise closed ring. """

    # Create an area
    raa = AreaRestriction("RAA", True, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])

    # Test some coordinate lists
    coords0_in = [0, 0, 1, 0, 1, 1, 0, 1]  # Not a closed ring, not ccw
    coords1_in = [0, 0, 0, 1, 1, 1, 1, 0, 0, 0]  # Correct, ccw
    coords2_in = [0, 0, 1, 0, 1, 1, 0, 1, 0, 0]  # Not ccw
    coords3_in = [-1, -1, 1, -1, 1, 1, -1, 1, -1, -1]  # Not ccw

    # The above lists should result in the following outputs:
    coords0_out = [0, 0, 0, 1, 1, 1, 1, 0, 0, 0]
    coords1_out = [0, 0, 0, 1, 1, 1, 1, 0, 0, 0]
    coords2_out = [0, 0, 0, 1, 1, 1, 1, 0, 0, 0]
    coords3_out = [-1, -1, -1, 1, 1, 1, 1, -1, -1, -1]

    assert raa._check_poly(coords0_in) == coords0_out
    assert raa._check_poly(coords1_in) == coords1_out
    assert raa._check_poly(coords2_in) == coords2_out
    assert raa._check_poly(coords3_in) == coords3_out


def test_raa_delete(areafilter_):
    """ Test the delete function. """

    # Create an area
    raa = AreaRestriction("RAA", True, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])

    raa.delete()


def test_raa_coords2verts(areafilter_):
    """ Test the correctness of the function that transforms an array of vertices in
        lon,lat order into a list of coordinates in lat, lon order. """

    # Create an area
    raa = AreaRestriction("RAA", True, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])

    coords = [1.1, 2.0, 1.0, -2.2, -1.5, 2.2]
    verts = np.array([[2.0, 1.1], [-2.2, 1.0], [2.2, -1.5]])

    assert np.array_equal(raa._coords2verts(coords), verts)


def test_raa_verts2coords(areafilter_):
    """ Test the correctness of the function that transforms a coordinate list in
        lat,lon order to an array of vertices in lon,lat order. """

    # Create an area
    raa = AreaRestriction("RAA", True, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])

    verts = np.array([[2.0, 1.1], [-2.2, 1.0], [2.2, -1.5]])
    coords = [1.1, 2.0, 1.0, -2.2, -1.5, 2.2]

    assert raa._verts2coords(verts) == coords


def test_raa_calc_tangents_four_aircraft(areafilter_):
    """ Test the correctness of the function that calculates the bearing
        and distance from aircraft positions to the tangent points of the
        area. """

    # Create area with vertices at (lon,lat): {(-1, -1), (1, -1), (1, 1), (-1, 1)}
    raa = AreaRestriction(
        "RAA", True, [-1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0])
    assert raa.ring.is_ccw

    # Create four aircraft at (lon,lat): {(0,2),(2,0),(0, -2),(-2, 0)}
    ntraf = 4
    ac_lon = np.array([0.0, 2.0, 0.0, -2.0])
    ac_lat = np.array([2.0, 0.0, -2.0, 0.0])

    # Correct qdr (defined as [-180 .. 180] degrees East-North-Up)
    # Headings 135, 225, 315, 45 in North-East-Down
    qdr_cor_l = np.array([135.0, -135.0, -45.0, 45.0])
    # Headings 225, 315, 45, 135 in North-East-Down
    qdr_cor_r = np.array([-135.0, -45.0, 45.0, 135.0])

    # Perform calculation
    qdr_res_l, qdr_res_r = raa.calc_tangents(ntraf, ac_lon, ac_lat)

    # Check that the results are within margin from the correct values
    assert np.allclose(qdr_res_l, qdr_cor_l, DIFF_DEG)
    assert np.allclose(qdr_res_r, qdr_cor_r, DIFF_DEG)


def test_raa_calc_tangents_two_aircraft(areafilter_):
    """ Test the correctness of the function that calculates the bearing
        and distance from aircraft positions to the tangent points of the
        area. """

    # Create area with vertices at (lon,lat): {(-3, 1), (-0.5, 1), (-0.5, -1), (-3, -1)}
    raa = AreaRestriction("RAA", True,
                                 [1.0, -3.0, 1.0, -0.5, -1.0, -0.5, -1.0, -3.0, 1.0, -3.0])

    # Create two aircraft at (lon,lat): {(0,2),(2,0),(0, -2),(-2, 0)}
    ntraf = 2
    ac_lon1 = np.array([-1.5, 1.5])
    ac_lat1 = np.array([-1.5, -1.5])

    # Correct qdr (defined as [-180 .. 180] degrees East-North-Up)
    qdr_cor_l1 = np.array([-71.69, -83.75])  # [deg] Correct left tangent headings
    qdr_cor_r1 = np.array([63.595, -38.857])  # [deg] Correct right tangent headings

    # Perform calculation
    qdr_res_l1, qdr_res_r1 = raa.calc_tangents(ntraf, ac_lon1, ac_lat1)

    # Check that the results are within margin from the correct values
    assert np.allclose(qdr_res_l1, qdr_cor_l1, DIFF_DEG)
    assert np.allclose(qdr_res_r1, qdr_cor_r1, DIFF_DEG)


def test_raa_calc_tangents_single_aircraft(areafilter_):
    """ Test the correctness of the function that calculates the bearing
        and distance from aircraft positions to the tangent points of the
        area. """

    # Create area with vertices at (lon,lat): {(-3, 1), (-0.5, 1), (-0.5, -1), (-3, -1)}
    raa = AreaRestriction("RAA", True,
                                 [1.0, -3.0, 1.0, -0.5, -1.0, -0.5, -1.0, -3.0, 1.0, -3.0])

    # Create one aircraft at (lon,lat): (-1.5,-1.5)
    ntraf = 1
    ac_lon1 = np.array([-1.5])
    ac_lat1 = np.array([-1.5])

    # Correct loxodrome azimuth (defined as [-180 .. 180] degrees East-North-Up)
    lox_az_cor_l1 = np.array([-71.67601713])  # [deg] Correct left tangent headings
    lox_ax_cor_r1 = np.array([63.58299887])  # [deg] Correct right tangent headings

    # Perform calculation
    lox_az_res_l1, lox_az_res_r1 = raa.calc_tangents(ntraf, ac_lon1, ac_lat1)

    # Check that the results are within margin from the correct values
    assert np.allclose(lox_az_res_l1, lox_az_cor_l1, DIFF_DEG)
    assert np.allclose(lox_az_res_r1, lox_ax_cor_r1, DIFF_DEG)


def test_raa_is_left_of_line_on_line(areafilter_):
    """ Test the correctness of the function that determines whether a point lies
        to the left of a given linepiece. """

    raa = AreaRestriction("RAA", True, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])

    # Test result when the point lies on line
    assert raa.is_left_of_line([0, 0], [1, 1], [2, 2]) == 0
    assert raa.is_left_of_line([0, 0], [0, 1], [0, 2]) == 0
    assert raa.is_left_of_line([0, 0], [0, 1], [0, 0.2]) == 0
    assert raa.is_left_of_line([0, 0], [1, 0], [2, 0]) == 0
    assert raa.is_left_of_line([0, 0], [1, 0], [0.1, 0]) == 0


def test_raa_is_left_of_line_false(areafilter_):
    """ Test the correctness of the function that determines whether a point lies
        to the left of a given linepiece. """

    raa = AreaRestriction("RAA", True, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])

    # Test result when the point lies to the right of the line
    assert raa.is_left_of_line([0, 0], [1, 1], [1, 0]) < 0
    assert raa.is_left_of_line([0, 0], [0, 1], [1, 0]) < 0
    assert raa.is_left_of_line([0, 0], [0, 1], [0.2, 0]) < 0
    assert raa.is_left_of_line([0, 0], [1, 0], [0, -1]) < 0
    assert raa.is_left_of_line([1, 0], [0, 0], [0, 0.1]) < 0


def test_raa_is_left_of_line_true(areafilter_):
    """ Test the correctness of the function that determines whether a point lies
        to the left of a given linepiece. """

    raa = AreaRestriction("RAA", True, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])

    # Test result when the point lies to the left of the line
    assert raa.is_left_of_line([0, 0], [1, 1], [0, 1]) > 0
    assert raa.is_left_of_line([0, 0], [0, 1], [-1, 1]) > 0
    assert raa.is_left_of_line([0, 0], [0, 1], [-1, 1]) > 0
    assert raa.is_left_of_line([0, 0], [1, 0], [0, 1]) > 0
    assert raa.is_left_of_line([1, 0], [0, 0], [0, -0.1]) > 0
