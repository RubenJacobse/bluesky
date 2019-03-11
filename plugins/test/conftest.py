""" Pytest configuration for area restriction plugin tests. """

import pytest
from bluesky.tools import areafilter

def mockfun(*args):
    """ Take any number of arguments and do nothing """

    pass

def mock_defineArea(area_id, area_type, area_coords):
    """ Mock areafilter.defineArea and ensure that input is formatted
        correctly. """

    assert isinstance(area_id, str)
    assert area_type == 'POLY'
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
