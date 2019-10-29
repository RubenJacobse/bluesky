"""
Geography related functions used in conflict detection.
"""

# Cython imports
import cython
from libc.math cimport sin, cos, pi, atan2, sqrt, fabs
import numpy as np
cimport numpy as np

cdef double RAD_TO_DEG = 180 / pi
cdef double DEG_TO_RAD = pi / 180

ctypedef np.float32_t DTYPE_t


def qdr(lat0, lon0, lat1, lon1):
    return cy_qdr(lat0, lon0, lat1, lon1)


def dist(lat0, lon0, lat1, lon1):
    return cy_dist(lat0, lon0, lat1, lon1)


def rwgs84(lat):
    return cy_rwgs84(lat)


@cython.cdivision(True)
cdef double cy_qdr(double lat0, double lon0, double lat1, double lon1):
    """
    Calculate bearing from one point to another using WGS84.
    """

    cdef double a, r1, r2, sw, r
    cdef double sin0, sin1, coslat0, coslat1, root, qdr

    # Radius calculation depends on whether the latitudes are on
    # the same hemisphere
    if lat0 * lat1 >= 0.0:
        r = cy_rwgs84(0.5 * (lat0 + lat1))
    else:
        a = 6378137.0
        r1 = cy_rwgs84(lat0)
        r2 = cy_rwgs84(lat1)
        r = (0.5 * (fabs(lat0) * (r1 + a) + fabs(lat1) * (r2 + a)) /
             (fabs(lat0) + fabs(lat1)))

    # Convert to radians
    lat0 = lat0 * DEG_TO_RAD
    lon0 = lon0 * DEG_TO_RAD
    lat1 = lat1 * DEG_TO_RAD
    lon1 = lon1 * DEG_TO_RAD
    coslat0 = cos(lat0)
    coslat1 = cos(lat1)

    # Bearing from Ref. http://www.movable-type.co.uk/scripts/latlong.html
    qdr = RAD_TO_DEG * (atan2(sin(lon1 - lon0) * coslat1,
                        coslat0 * sin(lat1)
                        - sin(lat0) * coslat1 * cos(lon1 - lon0)))

    return qdr


@cython.cdivision(True)
cdef double cy_dist(double lat0, double lon0, double lat1, double lon1):
    """
    Calculate distance from one point to another using WGS84.
    """

    cdef double a, r1, r2, r
    cdef double sin0, sin1, coslat0, coslat1, root, dist

    # Radius calculation depends on whether the latitudes are on
    # the same hemisphere
    if lat0 * lat1 >= 0.0:
        r = cy_rwgs84(0.5 * (lat0 + lat1))
    else:
        a = 6378137.0
        r1 = cy_rwgs84(lat0)
        r2 = cy_rwgs84(lat1)
        r = (0.5 * (fabs(lat0) * (r1 + a) + fabs(lat1) * (r2 + a)) /
             (fabs(lat0) + fabs(lat1)))

    # Convert to radians
    lat0 = lat0 * DEG_TO_RAD
    lon0 = lon0 * DEG_TO_RAD
    lat1 = lat1 * DEG_TO_RAD
    lon1 = lon1 * DEG_TO_RAD
    sin0 = sin(0.5 * (lat1 - lat0))
    sin1 = sin(0.5 * (lon1 - lon0))
    coslat0 = cos(lat0)
    coslat1 = cos(lat1)

    # d = 2.0 * r * np.arcsin(np.sqrt(sin0*sin0 + coslat0*coslat1*sin1*sin1))
    root = sin0 * sin0 + coslat0 * coslat1 * sin1 * sin1
    dist = 2.0 * r * atan2(sqrt(root) , sqrt(1.0 - root))
    
    return dist / 1852.0


@cython.cdivision(True)
cdef double cy_rwgs84(double lat):
    cdef double a, b, coslat, sinlat, an, bn, ad, bd, r

    lat = lat * DEG_TO_RAD
    a = 6378137.0 # [m] Major semi-axis WGS-84
    b = 6356752.314245 # [m] Minor semi-axis WGS-84
    coslat = cos(lat)
    sinlat = sin(lat)

    an = a * a * coslat
    bn = b * b * sinlat
    ad = a * coslat
    bd = b * sinlat

    r = sqrt((an * an + bn * bn) / (ad * ad + bd * bd))

    return r
