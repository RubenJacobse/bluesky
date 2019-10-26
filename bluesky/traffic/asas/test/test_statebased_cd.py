import pytest
import sys
import os

# Enable BlueSky imports by adding the project folder to the path
sys.path.append(os.path.abspath(os.path.join('../../../..')))

import bluesky.traffic.asas.src_c.cgeo as cy_geo
import bluesky.tools.geo as py_geo


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


if __name__ == "__main__":
    test_compare_py_cy_qdrdist()
