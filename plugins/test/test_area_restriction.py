""" Test the area_restriction plugin. """

import area_restriction as ar
from area_restriction import AreaRestrictionManager, RestrictedAirspaceArea


def test_plugin_init():
    """ Check if the methods specified in init_plugin are member functions of the SuaArray class """

    # Run the init_plugin function to see if it executes properly
    config, stackfunctions = ar.init_plugin()

    assert len(config) == 6
    assert "update" in config and "preupdate" in config and "reset" in config
    assert len(stackfunctions) == 3

def test_RAA_is_left(areafilter_):
    """ Test the correctness of the function that determines whether a point lies
        to the left of a given linepiece. """

    raa = RestrictedAirspaceArea("RAA1", True, 0, 0, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])

    # Test result when the point lies on line
    assert raa.is_left([0, 0], [1, 1], [2, 2]) == 0
    assert raa.is_left([0, 0], [0, 1], [0, 2]) == 0
    assert raa.is_left([0, 0], [0, 1], [0, 0.2]) == 0
    assert raa.is_left([0, 0], [1, 0], [2, 0]) == 0
    assert raa.is_left([0, 0], [1, 0], [0.1, 0]) == 0

    # Test result when the point lies to the right of the line
    assert raa.is_left([0, 0], [1, 1], [1, 0]) < 0
    assert raa.is_left([0, 0], [0, 1], [1, 0]) < 0
    assert raa.is_left([0, 0], [0, 1], [0.2, 0]) < 0
    assert raa.is_left([0, 0], [1, 0], [0, -1]) < 0
    assert raa.is_left([1, 0], [0, 0], [0, 0.1]) < 0

    # Test result when the point lies to the left of the line
    assert raa.is_left([0, 0], [1, 1], [0, 1]) > 0
    assert raa.is_left([0, 0], [0, 1], [-1, 1]) > 0
    assert raa.is_left([0, 0], [0, 1], [-1, 1]) > 0
    assert raa.is_left([0, 0], [1, 0], [0, 1]) > 0
    assert raa.is_left([1, 0], [0, 0], [0, -0.1]) > 0

def test_RAA_crs_mid(areafilter_):
    """ Test the crs_mid function that returns the bisector course
        dividing the angle between two courses in half. """

    raa = RestrictedAirspaceArea("RAA1", True, 0, 0, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])

    # Test result when crs_l < crs_r
    assert raa.crs_mid(0, 360) == 180
    assert raa.crs_mid(10, 50) == 30
    assert raa.crs_mid(160, 200) == 180

    # Test result when crs_r < crs_l
    assert raa.crs_mid(350, 10) == 0
    assert raa.crs_mid(330, 0) == 345
    assert raa.crs_mid(340, 40) == 10

def test_RAA_crs_is_between(areafilter_):
    """ Test the crs_is_between function that checks if a given course
        lies between a left and right most course. """

    raa = RestrictedAirspaceArea("RAA1", True, 0, 0, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])

    # These calls should return True
    assert raa.crs_is_between(10, 0, 30)
    assert raa.crs_is_between(0, 359, 1)
    assert raa.crs_is_between(350, 190, 170)
    assert raa.crs_is_between(340, 330, 20)

    # Thes calls should return False
    assert not raa.crs_is_between(0, 10, 20)
    assert not raa.crs_is_between(340, 355, 190)
    assert not raa.crs_is_between(22, 33.4, 359.1)
    assert not raa.crs_is_between(220, 220, 220)
    assert not raa.crs_is_between(180, 181, 179)
