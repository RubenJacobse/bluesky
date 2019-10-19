"""
Test the Leader-Following resolution method.

© Ruben Jacobse, 2019
"""

# Third party imports
import pytest
import numpy as np

# BlueSky imports
from bluesky.traffic.asas import LF


class MockTraffic:
    """ Mock the Traffic class attributes required in the tests. """

    def __init__(self):
        self.hdg = np.array([343.91, 358.18])
        self.lon = np.array([0.24743848, 0.0640334])
        self.lat = np.array([-0.48705884, -0.500544])
        self.gseast = np.array([-66.00, -8.17636])
        self.gsnorth = np.array([228.84, 257.564585])


# Setup the test data for the parametrized test
testtraf = MockTraffic()
testdata = [(testtraf, 0.24906, 214.2077, 0, 1, ("LF", "leader")),
            (testtraf, 0.24906, 214.2077, 1, 0, ("LF", "follower")), ]


@pytest.mark.parametrize("traf,delta_crs,tLOS,idx_ownship,idx_intruder,expected",
                         testdata)
def test_find_lf_status(traf, delta_crs, tLOS,
                        idx_ownship, idx_intruder, expected):

    result = LF.find_lf_status(traf, delta_crs, tLOS,
                               idx_ownship, idx_intruder)
    assert result == expected
