""" Pytest configuration for area restriction plugin tests. """

# Python imports
from collections.abc import Collection

# Third-party imports
import numpy as np
import pytest

# BlueSky imports
import bluesky as bs
from bluesky import stack
from bluesky.tools import areafilter, datalog, TrafficArrays, RegisterElementParameters
from plugins.thesis.restriction_manager import AreaRestrictionManager


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


def mock_crelog(*args):
    """ Create an fake instance of the CSVLogger class """
    return MockCSVLogger()


class MockTraf(TrafficArrays):
    """ Mock class that emulates part of the BlueSky traffic object. """

    def __init__(self):
        super().__init__()

        # Set this object as the root of the TrafficArrays tree
        TrafficArrays.set_class_root(self)

        self.ntraf = 0

        self.asas = MockAsas()

        # Register some variables to be updated when adding and
        # deleting aircraft
        with RegisterElementParameters(self):
            self.id = []
            self.lon = np.array([])
            self.lat = np.array([])
            self.hdg = np.array([])
            self.gseast = np.array([])
            self.gsnorth = np.array([])
            self.gs = np.array([])

    def create(self, n = 1):
        """ Create n new aircraft and add elements to variables. """

        # Call actual create() method
        super().create(n)
        self.ntraf += n
        for child in self._children:
            child.create(n)

    def delete(self, idx):
        """ Delete aircraft at index idx from all variables. """

        if isinstance(idx, Collection):
            idx.sort()
            dec = len(idx)
        else:
            dec = 1

        # Call actual delete() method
        super().delete(idx)

        self.ntraf -= dec

        return True

    def reset(self):
        """ Delete all aircraft and clear variables """

        # Call actual delete() method in TrafficArrays class
        super().reset()
        self.ntraf = 0

    def fake_traf(self):
        """ Create 4 fake traffic elements """

        # Ensure that the create command is executed in all children as well
        self.create(4)

        # Set some variables that would be present in bs.traf and that are required by
        # the functions that are being tested
        self.id = ["AC001", "AC002", "AC003", "AC004"]
        self.lon = np.array([0.0, 2.0, 0.0, 0.0])
        self.lat = np.array([2.0, 0.0, -2.0, 0.0])
        self.gseast = np.array([0, -450, 0, 450])
        self.gsnorth = np.array([450, 0, 450, 0])
        self.gs = np.array([450, 450, 450, 450])
        self.hdg = np.array([360, 270, 360, 90])

        self.alt = np.array([6100, 6100, 6100, 6100])
        self.actwp = MockWaypoint()


class MockAsas():
    """
    Mock object that is used to replace all calls to bs.traf.asas
    """

    def update(self):
        pass


class MockWaypoint():
    def __init__(self):
        self.lat = np.array([4, 0, 4, 0])
        self.lon = np.array([0, 4, 0, -4])


class MockSim():
    """
    Mock object that is used to replace all calls to bs.sim
    """


    def __init__(self):
        self.simt = 0


class MockCSVLogger():
    """
    Mock object that is used to replace the CSVLogger class
    """

    scenname = ""

    def __init__(self):
        pass

    def start(self):
        pass


@pytest.fixture
def areafilter_(monkeypatch):
    """
    Fixture for all test functions naming AirspaceRestrictionManager_
    in their parameter lists.
    """

    monkeypatch.setattr(areafilter, "defineArea", mock_defineArea)
    monkeypatch.setattr(areafilter, "deleteArea", mock_deleteArea)


@pytest.fixture
def crelog_(monkeypatch):
    monkeypatch.setattr(datalog, "crelog", mock_crelog)


@pytest.fixture
def AreaRestrictionManager_(crelog_, areafilter_, mocktraf_):
    """ Fixture that yields an instance of the AreaRestrictionManager
        class that carries state between test functions. """

    yield AreaRestrictionManager()


@pytest.fixture
def MockTraf_():
    yield MockTraf()


@pytest.fixture
def MockSim_():
    yield MockSim()


@pytest.fixture
def MockAsas_():
    yield MockAsas


@pytest.fixture
def mocksim_(monkeypatch, MockSim_):
    """ Fixture for all test functions naming bs.sim in their parameter lists. """

    # Replace calls to bs.sim with calls to the MockSim class
    monkeypatch.setattr(bs, "sim", MockSim_)


@pytest.fixture
def mocktraf_(monkeypatch, MockTraf_):
    """
    Fixture for all test functions naming bs.traf in their parameter lists.
    """

    # Replace subsequent calls to bs.traf with calls to the MockTraf class
    monkeypatch.setattr(bs, "traf", MockTraf_)

    # Replace subsequent calls to bs.stack with calls to mockfun
    monkeypatch.setattr(stack, "stack", mockfun)


@pytest.fixture
def mockbs_(monkeypatch, MockTraf_, MockSim_):
    """
    Fixture for all test functions naming bs.traf in their parameter lists.
    """

    # Replace subsequent calls to bs.traf with calls to the MockTraf class
    monkeypatch.setattr(bs, "traf", MockTraf_)

    # Replace subsequent calls to bs.sim with calls to the MockSim class
    monkeypatch.setattr(bs, "sim", MockSim_)

    # Replace subsequent calls to bs.asas with calls to the MockAsas class
    monkeypatch.setattr(bs, "asas", MockAsas_)

    # Replace subsequent calls to bs.stack with calls to mockfun
    monkeypatch.setattr(stack, "stack", mockfun)
