""" Pytest configuration for area restriction plugin tests. """

import pytest
import numpy as np
import bluesky as bs
from bluesky.tools import areafilter, TrafficArrays, RegisterElementParameters
from area_restriction import AreaRestrictionManager

class MockTraf(TrafficArrays):
    """ Mock class that emulates part of the BlueSky traffic object. """

    def __init__(self):
        super().__init__()

        # Set this object as the root of the TrafficArrays tree
        TrafficArrays.SetRoot(self)

        self.ntraf = 0

        # Register some variables to be updated when adding and
        # deleting aircraft
        with RegisterElementParameters(self):
            self.id = []
            self.lon = np.array([])
            self.lat = np.array([])
            self.gseast = np.array([])
            self.gsnorth = np.array([])
            self.gs = np.array([])

def mockfun(*args):
    """ Take any number of arguments and do nothing """
    pass

def mock_defineArea(area_id, area_type, area_coords):
    """ Mock areafilter.defineArea and ensure that input is formatted
        correctly. """

    assert isinstance(area_id, str)
    assert area_type == "POLY"
    assert all(isinstance(x, (int, float)) for x in area_coords)

def mock_deleteArea(area_id):
    """ Mock areafilter.deleteArea and ensure that input is formatted
        correctly. """

    assert isinstance(area_id, str)

@pytest.fixture
def areafilter_(monkeypatch):
    """ Fixture for all test functions naming AirspaceRestrictionManager_
        in their parameter lists. """

    # Replace subsequent calls to areafilter functions 'defineArea' and 'deleteArea'
    # with calls to 'mockfun'. (These functions are called in RestricedAirspaceArea
    # methods but require the screen object bs.scr to be initialized, which we will
    # avoid for now)
    monkeypatch.setattr(areafilter, "defineArea", mock_defineArea)
    monkeypatch.setattr(areafilter, "deleteArea", mock_deleteArea)

@pytest.fixture
def mocktraf_(monkeypatch):
    """ Fixture for all test functions naming bs.traf in their parameter lists. """

    MockTraf_ = MockTraf()

    # Create some traffic elements
    MockTraf_.create(4)

    # Set some variables that would be present in bs.traf and that are required by
    # the functions that are being tested
    MockTraf_.lon = np.array([0.0, 2.0, 0.0, -2.0])
    MockTraf_.lat = np.array([2.0, 0.0, -2.0, 0.0])
    MockTraf_.gseast = np.array([0, -250, 0, 250])
    MockTraf_.gsnorth = np.array([-250, 0, 250, 0])
    MockTraf_.gs = np.array([250, 250, 250, 250])

    # Replace subsequent calls to bs.traf with calls to the MockTraf_ class
    monkeypatch.setattr(bs, "traf", MockTraf_)

@pytest.fixture(scope = 'module')
def AreaRestrictionManager_():
    """ Fixture that yields an instance of the AreaRestrictionManager
        class that carries state between test functions. """

    yield AreaRestrictionManager()

# @pytest.fixture(scope = 'module')
# def MockTraf_():
#     """ Fixture that yields an instance of the MockTraf
#         class that carries state between test functions. """

#     yield MockTraf()
