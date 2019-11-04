"""
Tests to verify that python and cython version of geo module return
the exact same results.
"""

import math
import numpy as np

import py_geo
import cy_geo


def test_rwgslat_scalar():
    for lat in range(0, 101):
        lat = float(lat)
        py_rwgs84 = py_geo.rwgs84(lat)
        cy_rwgs84 = cy_geo.rwgs84(lat)

        py_rwgs84_r = np.around(py_rwgs84, 14)
        cy_rwgs84_r = np.around(cy_rwgs84, 14)

        diff = abs(py_rwgs84_r - cy_rwgs84_r)
        if diff > 0:
            print(f"lat={lat}: diff={diff}")
            print(f"\tpy:{py_rwgs84_r:.30f}")
            print(f"\tcy:{cy_rwgs84_r:.30f}")


def test_nprad2deg():
    for deg in range(-101, 101):
        deg = float(deg)

        npy_rad = np.deg2rad(deg)
        npi_rad = (deg * np.pi) / 180.0

        npy_pi = npy_rad * 180.0 / deg

        npy_rad_r = np.around(npy_rad, 14)
        npi_rad_r = np.around(npi_rad, 14)
        if not npy_rad_r == npi_rad_r:
            diff = abs(npy_rad - npi_rad)
            print(f"deg={deg}")
            print(f"\tnpy:    {npy_rad:.30f}")
            print(f"\tnpi:    {npi_rad:.30f}")
            print(f"\tdiff:   {diff}")
            print(f"\tnpy_r:  {npy_rad_r:.30f}")
            print(f"\tnpi_r:  {npi_rad_r:.30f}")
            print(f"\tnpi:    {np.pi:.50f}")
            print(f"\tnpy_pi: {npy_pi:.50f}")


def test_rwgslat_npndarray():
    pass


if __name__ == "__main__":
    test_rwgslat_scalar()
    # test_nprad2deg()
