"""
Test the objects and functions defined in area_restriction.py

© Ruben Jacobse, 2019
"""

# Third party imports
import pytest
import numpy as np
import shapely.geometry as spgeom

# BlueSky imports
import area_restriction as ar
from area_restriction import AreaRestrictionManager, RestrictedAirspaceArea

# Error tolerances for floating point comparisons
DIFF_DEG = 0.01 # [deg] Error tolerance for angle comparison
DIFF_DIST = 0.1 # [NM] Error tolerance for distance comparison
DIFF_VEL = 0.1 # [m/s] Error tolerance for velocity comparison

# Conversion factors
NM_TO_KM = 1.852 # Conversion factor from nautical miles to kilometers
KM_TO_NM = 1/1.852 # Conversion factor from kilometers to nautical miles

# NOTE: Code below commented out because it creates a second (unwanted) instance of the 
# AreaRestrictionManager class in the TrafficArrays tree. - To be fixed
# ##################################################################################################
# # Test that plugin correctly returns config and stackfunctions variables to BlueSky
# ##################################################################################################
# def test_plugin_init(MockTraf_, AreaRestrictionManager_, mocktraf_, areafilter_, mockarm_):
#     """ Check if the methods specified in init_plugin are member functions
#         of the SuaArray class. """

#     # Run the init_plugin function to see if it executes properly
#     config, stackfunctions = ar.init_plugin()

#     assert len(config) == 6
#     assert "update" in config and "preupdate" in config and "reset" in config
#     assert len(stackfunctions) == 3

##################################################################################################
# Tests for AreaRestrictionManager class
##################################################################################################
def test_arm_init(MockTraf_, AreaRestrictionManager_, areafilter_, mocktraf_):
    """ Verify that the AreaRestrictionManager initializes correctly. """

    # Check that root element exists (MockTraf in these tests) and
    # that the child-parent relationship is properly initialized
    assert AreaRestrictionManager_.root is MockTraf_
    assert AreaRestrictionManager_._parent is MockTraf_
    assert AreaRestrictionManager_ in MockTraf_._children

    # Check that other variables are initialized to their default values
    assert AreaRestrictionManager_.areas == []
    assert AreaRestrictionManager_.area_ids == []
    assert AreaRestrictionManager_.num_areas == 0
    assert AreaRestrictionManager_.num_traf == 0
    assert AreaRestrictionManager_.t_lookahead == 120

    # Check that all traffic variables have been registered properly
    lstVarList = ["closest_conflicting_area_idx", "current_position", "relative_track"]
    ArrVarList = ["idx_active_wp", "idx_next_wp", "crs_to_active_wp", "crs_to_next_wp", "reso_dv_east", "reso_dv_north", "is_in_area_reso", "is_in_aircraft_reso", "commanded_crs", "commanded_spd"]
    ndArrVarList = ["rel_gseast", "rel_gsnorth", "brg_left_tangent", "brg_right_tangent", \
                    "is_in_conflict", "is_inside", "time_to_intrusion"]
    assert all(x in AreaRestrictionManager_._LstVars for x in lstVarList)
    assert all(x in AreaRestrictionManager_._ArrVars for x in ArrVarList)
    assert all(x in AreaRestrictionManager_._ndArrVars for x in ndArrVarList)

def test_arm_set_t_lookahead(AreaRestrictionManager_, MockTraf_, areafilter_, mocktraf_):
    """ Test the set_t_lookahead() setter method. """

    assert AreaRestrictionManager_.t_lookahead == 120

    # Set new value
    AreaRestrictionManager_.set_t_lookahead(600)
    assert AreaRestrictionManager_.t_lookahead == 600

    # Give non-integer value, should return 'False'
    result, _ = AreaRestrictionManager_.set_t_lookahead(600.0)
    assert not result

    # Reset to default value
    AreaRestrictionManager_.set_t_lookahead(300)
    assert AreaRestrictionManager_.t_lookahead == 300

def test_arm_create_area(AreaRestrictionManager_, MockTraf_, areafilter_, mocktraf_):
    """ Verify that the create_area() method works correctly """

    # Add first area
    AreaRestrictionManager_.create_area("RAA_1", True, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0)
    assert AreaRestrictionManager_.num_areas == 1
    assert "RAA_1" in AreaRestrictionManager_.area_ids

    # Add another area (this will be deleted in next test)
    AreaRestrictionManager_.create_area("RAA_2", True, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0)
    assert AreaRestrictionManager_.num_areas == 2
    assert "RAA_2" in AreaRestrictionManager_.area_ids

    # Attempt to add an area with an id that already exists
    success, _ = AreaRestrictionManager_.create_area("RAA_1", True, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0)
    assert AreaRestrictionManager_.num_areas == 2
    assert not success

def test_arm_delete_area(AreaRestrictionManager_, MockTraf_, areafilter_, mocktraf_):
    """ Verify that the delete_area() method works correctly. """

    # Test deletion of an existing area
    success, _ = AreaRestrictionManager_.delete_area("RAA_2")
    assert success
    assert AreaRestrictionManager_.num_areas == 1

    # Test deletion of non-existing area
    success, _ = AreaRestrictionManager_.delete_area("RAA_3")
    assert not success
    assert AreaRestrictionManager_.num_areas == 1

def test_arm_create(AreaRestrictionManager_, MockTraf_, areafilter_, mocktraf_):
    """ Verify that the create() method works correctly. """

    # Create some fake traffic
    MockTraf_.fake_traf()
    nareas = AreaRestrictionManager_.num_areas
    ntraf = MockTraf_.ntraf

    assert nareas == 1
    assert ntraf == 4

    # Check that list variables have an entry for each aircraft and that
    # n-dimensional array variables have an entry for each area,aircraft combination
    for var in AreaRestrictionManager_._LstVars:
        assert len(AreaRestrictionManager_._Vars[var]) == ntraf
    for var in AreaRestrictionManager_._ArrVars:
        assert np.shape(AreaRestrictionManager_._Vars[var]) == (ntraf,)
    for var in AreaRestrictionManager_._ndArrVars:
        assert np.shape(AreaRestrictionManager_._Vars[var]) == (nareas, ntraf)

def test_arm_delete(AreaRestrictionManager_, MockTraf_, areafilter_, mocktraf_):
    """ Verify that the delete() method works correctly. """

    # Check that there are 4 aircraft
    assert AreaRestrictionManager_.num_areas == 1
    assert AreaRestrictionManager_.num_traf == 4
    assert MockTraf_.ntraf == 4
    for var in AreaRestrictionManager_._ArrVars:
        assert np.size(AreaRestrictionManager_._Vars[var]) == 4

    # In BlueSky aircraft will be deleted using traf.delete(), verify its working
    assert MockTraf_.delete(3)
    assert MockTraf_.ntraf == 3
    assert AreaRestrictionManager_.num_traf == 3

    # Delete another aircraft and verify the result
    assert MockTraf_.delete(1)
    assert MockTraf_.ntraf == 2
    assert AreaRestrictionManager_.num_traf == 2

def test_arm_reset(AreaRestrictionManager_, MockTraf_, areafilter_, mocktraf_):
    """ Verify that the reset() method results in the initial state
        with empty variable lists. """

    # Check that variables have values before reset
    for var in AreaRestrictionManager_._LstVars:
        assert AreaRestrictionManager_._Vars[var]
    for var in AreaRestrictionManager_._ArrVars:
        assert np.size(AreaRestrictionManager_._Vars[var])
    for var in AreaRestrictionManager_._ndArrVars:
        assert np.size(AreaRestrictionManager_._Vars[var])

    # Check that all traffic related variables are emptied after reset
    MockTraf_.reset()

    for var in AreaRestrictionManager_._LstVars:
        assert AreaRestrictionManager_._Vars[var] == []
    for var in AreaRestrictionManager_._ArrVars:
        assert np.size(AreaRestrictionManager_._Vars[var]) == 0
    for var in AreaRestrictionManager_._ndArrVars:
        assert np.size(AreaRestrictionManager_._Vars[var]) == 0

    # Check that all areas have been deleted
    assert AreaRestrictionManager_.areas == []
    assert AreaRestrictionManager_.area_ids == []
    assert AreaRestrictionManager_.num_areas == 0
    assert AreaRestrictionManager_.t_lookahead == 300

def test_arm_preupdate(AreaRestrictionManager_, MockTraf_, areafilter_, mocktraf_):
    """ Test the preupdate() method. """

    # Previous test called reset(), create area and traffic
    assert AreaRestrictionManager_.num_areas == 0
    AreaRestrictionManager_.create_area("RAA_1", True, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0)
    AreaRestrictionManager_.create_area("RAA_2", True, 20., -10., 0, 0, 1, 0, 1, 1, 0, 1, 0, 0)
    assert MockTraf_.ntraf == 0
    MockTraf_.fake_traf()
    assert MockTraf_.ntraf == 4
    assert AreaRestrictionManager_.num_traf == 4

    # Verify that vertices of non-moving area RAA_1 have not changed
    verts0_before = AreaRestrictionManager_.areas[0].verts
    coords0_before = AreaRestrictionManager_.areas[0].coords
    AreaRestrictionManager_.preupdate()
    verts0_after = AreaRestrictionManager_.areas[0].verts
    coords0_after = AreaRestrictionManager_.areas[0].coords

    assert np.allclose(verts0_before, verts0_after)
    assert coords0_before == coords0_after

    # Verify that vertices of moving area RAA_2 have changed
    # Actual verification of the position update correctness is done in test_raa_update_pos()
    verts1_before = AreaRestrictionManager_.areas[1].verts
    coords1_before = AreaRestrictionManager_.areas[1].coords
    AreaRestrictionManager_.preupdate()
    verts1_after = AreaRestrictionManager_.areas[1].verts
    coords1_after = AreaRestrictionManager_.areas[1].coords

    assert not np.allclose(verts1_before, verts1_after)
    assert not coords1_before == coords1_after

@pytest.mark.xfail
def test_arm_update(AreaRestrictionManager_, MockTraf_, areafilter_, mocktraf_):
    """ Test the update() method. """

    # Reset the objects
    MockTraf_.reset()

    # Create area and fake traffic
    AreaRestrictionManager_.create_area("RAA_4", True, 0, 0, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1)
    MockTraf_.fake_traf()
    AreaRestrictionManager_.update()

    # Verify that conflicts / inside conditions are evaluated correctly
    assert np.array_equal(AreaRestrictionManager_.is_inconflict, \
                            np.array([[False, True, True, True]]))
    assert np.array_equal(AreaRestrictionManager_.is_inside, \
                            np.array([[False, False, False, True]]))

    # Add extra tests to verify avoidance vector calculations
    # NOTE: To do!

def test_arm_traf_noarea(AreaRestrictionManager_, MockTraf_, areafilter_, mocktraf_):
    """ Tests whether traffic can be created before area creation """

    # Delete any existing traffic and areas
    MockTraf_.reset()
    assert MockTraf_._children
    assert not AreaRestrictionManager_.areas
    assert not MockTraf_.ntraf

    # Create fake traffic and then create area
    MockTraf_.fake_traf()

    AreaRestrictionManager_.create_area("RAA_4", True, 0, 0, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1)
    AreaRestrictionManager_.create_area("RAA_5", True, 0, 0, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1)

    assert AreaRestrictionManager_.num_traf == 4
    assert AreaRestrictionManager_.num_areas == 2

    # Verify all vars have 4 aircraft elements
    for var in AreaRestrictionManager_._LstVars:
        assert len(AreaRestrictionManager_._Vars[var]) == 4
    for var in AreaRestrictionManager_._ArrVars:
        assert AreaRestrictionManager_._Vars[var].shape == (4,)
    for var in AreaRestrictionManager_._ndArrVars:
        assert AreaRestrictionManager_._Vars[var].shape == (2, 4)

def test_arm_multi_delete(AreaRestrictionManager_, MockTraf_, areafilter_, mocktraf_):
    """ Test the deletion of multiple aircraft at the same time """

    MockTraf_.reset()

    # Creat the four fake aircraft and then delete the middle two
    AreaRestrictionManager_.create_area("RAA_1", True, 0, 0, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1)
    MockTraf_.fake_traf()
    assert MockTraf_.ntraf == 4
    assert AreaRestrictionManager_.num_traf == 4
    MockTraf_.delete([1, 2])
    assert MockTraf_.ntraf == 2
    assert AreaRestrictionManager_.num_traf == 2

    # Verify AC002 and AC003 were deleted
    assert [x in MockTraf_.id for x in ["AC001", "AC004"]]

    # Verify all aircraft variables in MockTraf_ have only 2 elements remaining
    for var in MockTraf_._LstVars:
        assert len(MockTraf_._Vars[var]) == 2
    for var in MockTraf_._ArrVars:
        assert len(MockTraf_._Vars[var]) == 2

    # Verify all aircraft variables in AreaRestrictionManager_ have only 2 elements for each area
    for var in AreaRestrictionManager_._LstVars:
        assert len(AreaRestrictionManager_._Vars[var]) == 2
    for var in AreaRestrictionManager_._ArrVars:
        assert AreaRestrictionManager_._Vars[var].shape == (2,)
    for var in AreaRestrictionManager_._ndArrVars:
        assert AreaRestrictionManager_._Vars[var].shape[1] == 2

@pytest.mark.xfail
def test_arm_area_findconf(AreaRestrictionManager_, MockTraf_, areafilter_, mocktraf_):
    assert 0

@pytest.mark.xfail
def test_arm_calc_reso(AreaRestrictionManager_, MockTraf_, areafilter_, mocktraf_):
    assert 0

@pytest.mark.xfail
def test_arm_stack_reso_apply(AreaRestrictionManager_, MockTraf_, areafilter_, mocktraf_):
    assert 0

##################################################################################################
# Tests for methods of RestrictedAirspaceArea class
##################################################################################################
def test_raa_init(areafilter_):
    """ Tests if the RestrictedAirspaceArea class is initialized correctly. """

    test_id = "RAA_01"
    test_status = True
    test_gseast = 10
    test_gsnorth = -30

    # Vertices in ccw order (lon,lat): (-1,-1), (1,-1), (1,1), (-1,1), (-1,-1)
    test_coords = [-1, -1, -1, 1, 1, 1, 1, -1, -1, -1]

    raa = RestrictedAirspaceArea(test_id, test_status, test_gseast, test_gsnorth, test_coords)

    assert raa.area_id == test_id
    assert raa.status == test_status
    assert raa.gseast == test_gseast
    assert raa.gsnorth == test_gsnorth
    assert raa.coords == test_coords

    assert raa.ring.is_valid
    assert raa.ring.is_closed
    assert raa.ring.is_ccw
    assert raa.poly.is_valid
    assert raa.poly.exterior.coords.xy == raa.ring.coords.xy

def test_raa_update_pos(areafilter_):
    """ Test the function that calculates the new coordinates of a moving
        area. """

    # Create a moving and a non-moving area with the same initial coordinates
    raa_0 = RestrictedAirspaceArea("RAA0", True, 0, 0, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])
    raa_1 = RestrictedAirspaceArea("RAA1", True, -50, 50, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])

    # Check that non-moving polygon does indeed not move
    raa_0_coords_orig = raa_0.coords 
    raa_0.update_pos(10)
    assert raa_0.coords == raa_0_coords_orig

    # Check that moving polygon is indeed moving 
    raa_1_coords_orig = raa_1.coords
    raa_1.update_pos(10)
    assert raa_1.coords != raa_1_coords_orig
    # NOTE: Verification of correctness of movement to be added

def test_raa_is_ccw(areafilter_):
    """ Test the function that checks if a coordinate sequence is
        defined in counter-clockwise order. """

    # Test some coordinate lists
    coords1 = [0, 0, 0, 1, 1, 1, 1, 0, 0, 0] # ccw
    coords2 = [0, 0, 1, 0, 1, 1, 0, 1, 0, 0] # not ccw
    coords3 = [-1, -1, 1, -1, 1, 1, -1, 1, -1, -1] # not ccw
    coords4 = [-1, -1, -1, 1, 1, 1, 1, -1, -1, -1] # ccw

    # Create areas
    raa1 = RestrictedAirspaceArea("RAA1", True, 0, 0, coords1)
    raa2 = RestrictedAirspaceArea("RAA2", True, 0, 0, coords2)
    raa3 = RestrictedAirspaceArea("RAA3", True, 0, 0, coords3)
    raa4 = RestrictedAirspaceArea("RAA4", True, 0, 0, coords4)

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
    raa = RestrictedAirspaceArea("RAA", True, 0, 0, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])

    # Test some coordinate lists
    coords0_in = [0, 0, 1, 0, 1, 1, 0, 1] # Not a closed ring, not ccw
    coords1_in = [0, 0, 0, 1, 1, 1, 1, 0, 0, 0] # Correct, ccw
    coords2_in = [0, 0, 1, 0, 1, 1, 0, 1, 0, 0] # Not ccw
    coords3_in = [-1, -1, 1, -1, 1, 1, -1, 1, -1, -1] # Not ccw

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
    raa = RestrictedAirspaceArea("RAA", True, 0, 0, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])

    raa.delete()

def test_raa_coords2verts(areafilter_):
    """ Test the correctness of the function that transforms an array of vertices in
        lon,lat order into a list of coordinates in lat, lon order. """

    # Create an area
    raa = RestrictedAirspaceArea("RAA", True, 0, 0, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])

    coords = [1.1, 2.0, 1.0, -2.2, -1.5, 2.2]
    verts = np.array([[2.0, 1.1], [-2.2, 1.0], [2.2, -1.5]])

    assert np.array_equal(raa._coords2verts(coords), verts)

def test_raa_verts2coords(areafilter_):
    """ Test the correctness of the function that transforms a coordinate list in
        lat,lon order to an array of vertices in lon,lat order. """

    # Create an area
    raa = RestrictedAirspaceArea("RAA", True, 0, 0, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])

    verts = np.array([[2.0, 1.1], [-2.2, 1.0], [2.2, -1.5]])
    coords = [1.1, 2.0, 1.0, -2.2, -1.5, 2.2]

    assert raa._verts2coords(verts) == coords

def test_raa_calc_qdr_tangents_four_aircraft(areafilter_):
    """ Test the correctness of the function that calculates the bearing
        and distance from aircraft positions to the tangent points of the
        area. """

    # Create area with vertices at (lon,lat): {(-1, -1), (1, -1), (1, 1), (-1, 1)}
    raa = RestrictedAirspaceArea("RAA", True, 0, 0, [-1, -1, -1, 1, 1, 1, 1, -1, -1, -1])
    assert raa.ring.is_ccw

    # Create four aircraft at (lon,lat): {(0,2),(2,0),(0, -2),(-2, 0)}
    ntraf = 4
    ac_lon = np.array([0, 2, 0, -2])
    ac_lat = np.array([2, 0, -2, 0])

    # Correct qdr (defined as [-180 .. 180] degrees East-North-Up)
    qdr_cor_l = np.array([135, -135, -45, 45]) # Headings 135, 225, 315, 45 in North-East-Down
    qdr_cor_r = np.array([-135, -45, 45, 135]) # Headings 225, 315, 45, 135 in North-East-Down

    # Perform calculation
    qdr_res_l, qdr_res_r = raa.calc_qdr_tangents(ntraf, ac_lon, ac_lat)

    # Check that the results are within margin from the correct values
    assert np.allclose(qdr_res_l, qdr_cor_l, DIFF_DEG)
    assert np.allclose(qdr_res_r, qdr_cor_r, DIFF_DEG)


def test_raa_calc_qdr_tangents_two_aircraft(areafilter_):
    """ Test the correctness of the function that calculates the bearing
        and distance from aircraft positions to the tangent points of the
        area. """

    # Create area with vertices at (lon,lat): {(-3, 1), (-0.5, 1), (-0.5, -1), (-3, -1)}
    raa = RestrictedAirspaceArea("RAA", True, 0, 0, \
                                    [1.0, -3.0, 1.0, -0.5, -1.0, -0.5, -1.0, -3.0, 1.0, -3.0])

    # Create two aircraft at (lon,lat): {(0,2),(2,0),(0, -2),(-2, 0)}
    ntraf = 2
    ac_lon1 = np.array([-1.5, 1.5])
    ac_lat1 = np.array([-1.5, -1.5])

    # Correct qdr (defined as [-180 .. 180] degrees East-North-Up)
    qdr_cor_l1 = np.array([-71.69, -83.75]) # [deg] Correct left tangent headings
    qdr_cor_r1 = np.array([63.595, -38.857]) # [deg] Correct right tangent headings

    # Perform calculation
    qdr_res_l1, qdr_res_r1 = raa.calc_qdr_tangents(ntraf, ac_lon1, ac_lat1)

    # Check that the results are within margin from the correct values
    assert np.allclose(qdr_res_l1, qdr_cor_l1, DIFF_DEG)
    assert np.allclose(qdr_res_r1, qdr_cor_r1, DIFF_DEG)

def test_raa_calc_rhumb_tangents_single_aircraft(areafilter_):
    """ Test the correctness of the function that calculates the bearing
        and distance from aircraft positions to the tangent points of the
        area. """

    # Create area with vertices at (lon,lat): {(-3, 1), (-0.5, 1), (-0.5, -1), (-3, -1)}
    raa = RestrictedAirspaceArea("RAA", True, 0, 0, \
                                    [1.0, -3.0, 1.0, -0.5, -1.0, -0.5, -1.0, -3.0, 1.0, -3.0])

    # Create two aircraft at (lon,lat): {(-1.5,-1.5),(1.5,-1.5)}
    ntraf = 1
    ac_lon1 = np.array([-1.5])
    ac_lat1 = np.array([-1.5])

    # Correct loxodrome azimuth (defined as [-180 .. 180] degrees East-North-Up)
    lox_az_cor_l1 = np.array([-71.67601713]) # [deg] Correct left tangent headings
    lox_ax_cor_r1 = np.array([63.58299887]) # [deg] Correct right tangent headings

    # Perform calculation
    lox_az_res_l1, lox_az_res_r1 = raa.calc_rhumb_tangents(ntraf, ac_lon1, ac_lat1)

    # Check that the results are within margin from the correct values
    assert np.allclose(lox_az_res_l1, lox_az_cor_l1, DIFF_DEG)
    assert np.allclose(lox_az_res_r1, lox_ax_cor_r1, DIFF_DEG)

def test_raa_calc_rhumb_tangents_two_aircraft(areafilter_):
    """ Test the correctness of the function that calculates the bearing
        and distance from aircraft positions to the tangent points of the
        area. """

    # Create area with vertices at (lon,lat): {(-3, 1), (-0.5, 1), (-0.5, -1), (-3, -1)}
    raa = RestrictedAirspaceArea("RAA", True, 0, 0, \
                                    [1.0, -3.0, 1.0, -0.5, -1.0, -0.5, -1.0, -3.0, 1.0, -3.0])

    # Create two aircraft at (lon,lat): {(-1.5,-1.5),(1.5,-1.5)}
    ntraf = 2
    ac_lon1 = np.array([-1.5, 1.5])
    ac_lat1 = np.array([-1.5, -1.5])

    # Correct loxodrome azimuth (defined as [-180 .. 180] degrees East-North-Up)
    lox_az_cor_l1 = np.array([-71.67601713, -83.70038260]) # [deg] Correct left tangent headings
    lox_ax_cor_r1 = np.array([63.58299887, -38.84515642]) # [deg] Correct right tangent headings

    # Perform calculation
    lox_az_res_l1, lox_az_res_r1 = raa.calc_rhumb_tangents(ntraf, ac_lon1, ac_lat1)

    # Check that the results are within margin from the correct values
    assert np.allclose(lox_az_res_l1, lox_az_cor_l1, DIFF_DEG)
    assert np.allclose(lox_az_res_r1, lox_ax_cor_r1, DIFF_DEG)

def test_raa_calc_rhumb_tangents_four_aircraft(areafilter_):
    """ Test the correctness of the function that calculates the bearing
        and distance from aircraft positions to the tangent points of the
        area. """

    # Create area with vertices at (lon,lat): {(-1, -1), (1, -1), (1, 1), (-1, 1)}
    raa = RestrictedAirspaceArea("RAA", True, 0, 0, [-1, -1, -1, 1, 1, 1, 1, -1, -1, -1])
    assert raa.ring.is_ccw

    # Create four aircraft at (lon,lat): {(0,2),(2,0),(0, -2),(-2, 0)}
    ntraf = 4
    ac_lon = np.array([0, 2, 0, -2])
    ac_lat = np.array([2, 0, -2, 0])

    # Correct loxodrome azimuth (defined as [-180 .. 180] degrees East-North-Up)
    lox_az_cor_l = np.array([134.81789556, -134.80905074, -45.18210444, 45.19094926])
    lox_az_cor_r = np.array([-134.81789556, -45.19094926, 45.18210444, 134.80905074])

    # Perform calculation
    lox_az_res_l, lox_az_res_r = raa.calc_rhumb_tangents(ntraf, ac_lon, ac_lat)

    # Check that the results are within margin from the correct values
    assert np.allclose(lox_az_res_l, lox_az_cor_l, DIFF_DEG)
    assert np.allclose(lox_az_res_r, lox_az_cor_r, DIFF_DEG)

def test_raa_is_left_of_line_on_line(areafilter_):
    """ Test the correctness of the function that determines whether a point lies
        to the left of a given linepiece. """

    raa = RestrictedAirspaceArea("RAA", True, 0, 0, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])

    # Test result when the point lies on line
    assert raa.is_left_of_line([0, 0], [1, 1], [2, 2]) == 0
    assert raa.is_left_of_line([0, 0], [0, 1], [0, 2]) == 0
    assert raa.is_left_of_line([0, 0], [0, 1], [0, 0.2]) == 0
    assert raa.is_left_of_line([0, 0], [1, 0], [2, 0]) == 0
    assert raa.is_left_of_line([0, 0], [1, 0], [0.1, 0]) == 0

def test_raa_is_left_of_line_false(areafilter_):
    """ Test the correctness of the function that determines whether a point lies
        to the left of a given linepiece. """

    raa = RestrictedAirspaceArea("RAA", True, 0, 0, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])

    # Test result when the point lies to the right of the line
    assert raa.is_left_of_line([0, 0], [1, 1], [1, 0]) < 0
    assert raa.is_left_of_line([0, 0], [0, 1], [1, 0]) < 0
    assert raa.is_left_of_line([0, 0], [0, 1], [0.2, 0]) < 0
    assert raa.is_left_of_line([0, 0], [1, 0], [0, -1]) < 0
    assert raa.is_left_of_line([1, 0], [0, 0], [0, 0.1]) < 0

def test_raa_is_left_of_line_true(areafilter_):
    """ Test the correctness of the function that determines whether a point lies
        to the left of a given linepiece. """

    raa = RestrictedAirspaceArea("RAA", True, 0, 0, [0, 0, 1, 0, 1, 1, 0, 1, 0, 0])

    # Test result when the point lies to the left of the line
    assert raa.is_left_of_line([0, 0], [1, 1], [0, 1]) > 0
    assert raa.is_left_of_line([0, 0], [0, 1], [-1, 1]) > 0
    assert raa.is_left_of_line([0, 0], [0, 1], [-1, 1]) > 0
    assert raa.is_left_of_line([0, 0], [1, 0], [0, 1]) > 0
    assert raa.is_left_of_line([1, 0], [0, 0], [0, -0.1]) > 0

##################################################################################################
# Tests for other functions defined in area_restriction.py
##################################################################################################
def test_crs_mid():
    """ Test the crs_mid function that returns the bisector course
        dividing the angle between two courses in half. """

    # Test result when crs_left_tangent < crs_right_tangent
    assert ar.crs_mid(0, 360) == 180
    assert ar.crs_mid(10, 50) == 30
    assert ar.crs_mid(160, 200) == 180

    # Test result when crs_right_tangent < crs_left_tangent
    assert ar.crs_mid(350, 10) == 0
    assert ar.crs_mid(330, 0) == 345
    assert ar.crs_mid(340, 40) == 10

    # Test rsult when crs_left_tangent = crs_right_tangent
    assert ar.crs_mid(220, 220) == 220
    assert ar.crs_mid(360, 360) == 0

def test_crs_is_between_true():
    """ Test the crs_is_between function that checks if a given course
        lies between a left and right most course. """

    # These calls should return True
    assert ar.crs_is_between(10, 0, 30)
    assert ar.crs_is_between(0, 359, 1)
    assert ar.crs_is_between(350, 190, 170)
    assert ar.crs_is_between(340, 330, 20)

def test_crs_is_between_false():
    """ Test the crs_is_between function that checks if a given course
        lies between a left and right most course. """

    # Thes calls should return False
    assert not ar.crs_is_between(0, 10, 20)
    assert not ar.crs_is_between(340, 355, 190)
    assert not ar.crs_is_between(22, 33.4, 359.1)
    assert not ar.crs_is_between(220, 220, 220)
    assert not ar.crs_is_between(180, 181, 179)

def test_enu2crs():
    """ Test the function that converts ENU angles to compass angles. """

    enu = np.array([-180, -135, -90, -45, 0, 45, 90, 135, 180])
    crs_corr = np.array([270, 225, 180, 135, 90, 45, 0, 315, 270])
    crs = ar.enu2crs(enu)

    assert np.array_equal(crs, crs_corr)

def test_ned2crs():
    """ Test the function that converts ENU angles to compass angles. """

    ned = np.array([-180, -135, -90, -45, 0, 45, 90, 135, 180])
    crs_corr = np.array([180, 225, 270, 315, 0, 45, 90, 135, 180])
    crs = ar.ned2crs(ned)

    assert np.array_equal(crs, crs_corr)

def test_crs_closest_scalar():
    """ Test the function that takes two courses and returns the course
        with the smallest angle difference with respect to a reference 
        course. """

    # Test for scalar input
    ref = 340
    crs_a = 10
    crs_b = 30

    crs = ar.crs_closest(ref, crs_a, crs_b)
    assert crs == crs_a


def test_crs_closest_numpy_scalar():
    """ Test the function that takes two courses and returns the course
        with the smallest angle difference with respect to a reference 
        course. """

    # Test for single value numpy array input
    ref = np.array([340])
    crs_a = np.array([10])
    crs_b = np.array([30])
    crs = ar.crs_closest(ref, crs_a, crs_b)

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
    crs = ar.crs_closest(ref, crs_a, crs_b)

    crs_correct = np.array([10, 360, 10, 20, 350])
    assert np.array_equal(crs, crs_correct)
