import sys
import os

# Third-party imports
import numpy as np

# BlueSky imports
from bluesky.tools import geo as py_geo
import bluesky.traffic.asas.StateBasedCD as py_cd
from bluesky.traffic.asas.src_c import cgeo as cy_geo
from bluesky.traffic.asas.src_c import StateBasedCD as cy_cd


def test_compare_py_cy_qdrdist():
    """
    Verify that python and cython qdr and dist functions return
    the exact same values.
    """

    lat = [52.3, 50.0]
    lon = [4.5, -1.0]

    pyqdr, pydist = py_geo.qdrdist(lat[0], lon[0], lat[1], lon[1])

    cyqdr = cy_geo.qdr(lat[0], lon[0], lat[1], lon[1])
    cydist = cy_geo.dist(lat[0], lon[0], lat[1], lon[1])

    assert pyqdr == cyqdr
    assert pydist == cydist


def test_statebasedcd():
    traf = MockTraf()

    pz_radius = 9260
    pz_height = 304.8
    t_lookahead = 300

    py_tuple = py_cd.detect(traf, traf, pz_radius, pz_height, t_lookahead)
    print("\n\npy_tuple:")
    print(py_tuple)
    assert py_tuple == traf.out_tuple

    cy_tuple = cy_cd.detect(traf, traf, pz_radius, pz_height, t_lookahead)
    print("\n\ncy_tuple:")
    print(cy_tuple)
    assert cy_tuple == traf.out_tuple

    # Technically redundant comparison
    assert py_tuple == cy_tuple


class MockTraf:
    def __init__(self):

        # Input, based on:
        # 20191021-1111022_L50_W30_A110_RESO_GV-CORRIDOR-BOTH_T-HIGH_SCEN001
        # bs.sim.simt = 372.0 s
        self.ntraf = 16
        self.id = ['AC000', 'AC001', 'AC002', 'AC003',
                   'AC004', 'AC005', 'AC006', 'AC007',
                   'AC008', 'AC009', 'AC010', 'AC011',
                   'AC012', 'AC013', 'AC014', 'AC015']
        self.lat = np.array([-0.90263743, -0.98339299, -0.9527116, -0.89771277,
                             -0.99829762, -1.04978902, -0.96362205, -1.14391533,
                             -0.99802094, -1.31062982, -1.3095438, -1.15996797,
                             -1.04025818, - 1.41510871, - 1.6056927, - 1.65355893])
        self.lon = np.array([0.09969293, 0.06190735, -0.27407323, -0.64128505,
                             0.46169351, -0.44782712, -0.55587418, -0.01322746,
                             -0.73188048, 0.13300892, -0.33015276, -0.83270959,
                             -1.0599241, -0.60826966, -0.05382854, -0.16379333])
        self.trk = np.array([352.5402963, 1.95094798, 23.84435847, 56.0304172,
                             320.86593571, 39.74383849, 47.42999846, 2.32806656,
                             47.8299476, 351.54410557, 20.2775614, 48.22211032,
                             59.50574727, 31.33370616, 2.59056915, 7.53932152])
        self.gs = np.array([239.2991127, 222.02400151, 246.63291407, 222.10715536,
                            232.47178037, 226.5491127, 254.91255828, 257.69400383,
                            246.23263015, 226.71709103, 255.36255828, 230.75518863,
                            237.17604506, 238.77356071, 226.71709103, 225.90715536])
        self.alt = np.array([10972.8, 10972.8, 10972.8, 10972.8, 10972.8, 10972.8,
                             10972.8, 10972.8, 10972.8, 10972.8, 10972.8, 10972.8,
                             10972.8, 10972.8, 10972.8, 10972.8, ])
        self.vs = np.array([0., 0., 0., 0., 0., 0., 0., 0.,
                            0., 0., 0., 0., 0., 0., 0., 0.])

        # Expected output
        confpairs = [('AC004', 'AC007'), ('AC005', 'AC006'),
                     ('AC006', 'AC005'), ('AC006', 'AC007'),
                     ('AC007', 'AC004'), ('AC007', 'AC006'),
                     ('AC014', 'AC015'), ('AC015', 'AC014')]
        lospairs = []
        inconf = [False, False, False, False,
                  True, True, True, True,
                  False, False, False, False,
                  False, False, True, True]
        tcpamax = np.array([0., 0., 0., 0.,
                            311.42333182, 288.74393647, 320.77391302,
                            320.77391302, 0., 0., 0., 0.,
                            0., 0., 584.47235726, 584.47235726])
        qdr = np.array([252.95088798, 308.57648993, 128.57648993,
                        108.38186306, 72.95088798, 288.38186306,
                        246.46865588, 66.46865588])
        dist = np.array([55226.57328299, 15365.55779063, 15365.55779063,
                         63573.12003922, 55226.57328299, 63573.12003922,
                         13331.17695854, 13331.17695854])
        dcpa = np.array([8730.66293937, 9083.55498246, 9083.55498246,
                         8018.24074422, 8730.66293937, 8018.24074422,
                         6859.585157, 6859.585157])
        tcpa = np.array([311.42333182, 288.74393647, 288.74393647,
                         320.77391302, 311.42333182, 320.77391302,
                         584.47235726, 584.47235726])
        tinconf = np.array([293.799974, 246.82804468, 246.82804468,
                            297.21386928, 293.799974, 297.21386928,
                            266.41741822, 266.41741822])

        self.out_tuple = (confpairs, lospairs, inconf, tcpamax,
                          qdr, dist, dcpa, tcpa, tinconf)
