"""
Test the objects and functions defined in area_manager.py

© Ruben Jacobse, 2019
"""

# Third party imports
import pytest
import numpy as np
import shapely.geometry as spgeom

# BlueSky imports
from plugins.thesis import area_manager as ar
from plugins.thesis.area_manager import AreaRestrictionManager
from plugins.thesis.area import AreaRestriction

# Error tolerances for floating point comparisons
DIFF_DEG = 0.01 # [deg] Error tolerance for angle comparison
DIFF_DIST = 0.1 # [NM] Error tolerance for distance comparison
DIFF_VEL = 0.1 # [m/s] Error tolerance for velocity comparison

# Conversion factors
NM_TO_KM = 1.852 # Conversion factor from nautical miles to kilometers
KM_TO_NM = 1/NM_TO_KM # Conversion factor from kilometers to nautical miles


def test_arm_init(AreaRestrictionManager_, MockTraf_):
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


def test_arm_set_t_lookahead(AreaRestrictionManager_):
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


def test_arm_create_area(AreaRestrictionManager_):
    """ Verify that the create_area() method works correctly """

    # Add first area
    coords_1 = [0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0]
    AreaRestrictionManager_.create_area("RAA_1", True, *coords_1)
    assert AreaRestrictionManager_.num_areas == 1
    assert "RAA_1" in AreaRestrictionManager_.area_ids

    # Add another area (this will be deleted in next test)
    coords_2 = [0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0]
    AreaRestrictionManager_.create_area("RAA_2", True, *coords_2)
    assert AreaRestrictionManager_.num_areas == 2
    assert "RAA_2" in AreaRestrictionManager_.area_ids

    # Attempt to add an area with an id that already exists
    success, _ = AreaRestrictionManager_.create_area("RAA_1", True, *coords_1)
    assert AreaRestrictionManager_.num_areas == 2
    assert not success


def test_arm_delete_area(AreaRestrictionManager_):
    """ Verify that the delete_area() method works correctly. """

    # Test deletion of an existing area
    coords_1 = [0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0]
    AreaRestrictionManager_.create_area("RAA_1", True, *coords_1)
    success, _ = AreaRestrictionManager_.delete_area("RAA_1")
    assert success
    assert AreaRestrictionManager_.num_areas == 0

    # Test deletion of non-existing area
    success, _ = AreaRestrictionManager_.delete_area("RAA_3")
    assert not success
    assert AreaRestrictionManager_.num_areas == 0


def test_arm_create(AreaRestrictionManager_, MockTraf_):
    """ Verify that the create() method works correctly. """

    # Create 1 area some 4 fake aircraft
    coords_1 = [0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0]
    AreaRestrictionManager_.create_area("RAA_1", True, *coords_1)
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


def test_arm_delete(AreaRestrictionManager_, MockTraf_):
    """ Verify that the delete() method works correctly. """

    # Check that there are 0 aircraft and areas
    assert AreaRestrictionManager_.num_areas == 0
    assert AreaRestrictionManager_.num_traf == 0

    # Add four aircraft and 1 area
    coords_1 = [0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0]
    AreaRestrictionManager_.create_area("RAA_1", True, *coords_1)
    MockTraf_.fake_traf()

    assert MockTraf_.ntraf == 4
    assert AreaRestrictionManager_.num_areas == 1
    assert AreaRestrictionManager_.num_traf == 4
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


def test_arm_reset(AreaRestrictionManager_, MockTraf_):
    """ Verify that the reset() method results in the initial state
        with empty variable lists. """

    # Create some fake traffic
    coords_1 = [0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0]
    AreaRestrictionManager_.create_area("RAA_1", True, *coords_1)
    MockTraf_.fake_traf()
    nareas = AreaRestrictionManager_.num_areas
    ntraf = MockTraf_.ntraf

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
    assert AreaRestrictionManager_.t_lookahead == 120


def test_arm_preudate(AreaRestrictionManager_, MockTraf_):
    """ Test the preupdate() method. """

    # Initialize area and traffic
    coords_1 = [0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0]
    AreaRestrictionManager_.create_area("RAA_1", True, *coords_1)
    MockTraf_.fake_traf()

    # Call preupdate to perform a single time step
    AreaRestrictionManager_.preupdate()


@pytest.mark.xfail
def test_arm_update(AreaRestrictionManager_, MockTraf_):
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


def test_arm_traf_noarea(AreaRestrictionManager_, MockTraf_):
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


def test_arm_multi_delete(AreaRestrictionManager_, MockTraf_):
    """ Test the deletion of multiple aircraft at the same time """

    MockTraf_.reset()

    # Create the four fake aircraft and then delete the middle two

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


def test_arm_remove(AreaRestrictionManager_, MockTraf_):

    AreaRestrictionManager_.create_area(
        "RAA_1", True, 0, 0, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1)
    MockTraf_.fake_traf()

    AreaRestrictionManager_.remove()
    assert AreaRestrictionManager_ not in MockTraf_._children

@pytest.mark.xfail
def test_arm_area_findconf(AreaRestrictionManager_, MockTraf_, MockSim_, areafilter_, mocktraf_, mocksim_):
    assert 0


@pytest.mark.xfail
def test_arm_calc_reso(AreaRestrictionManager_, MockTraf_, MockSim_, areafilter_, mocktraf_, mocksim_):
    assert 0


@pytest.mark.xfail
def test_arm_stack_reso_apply(AreaRestrictionManager_, MockTraf_, MockSim_, areafilter_, mocktraf_, mocksim_):
    assert 0
