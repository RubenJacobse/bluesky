"""
This module defines a set of standard geographic functions and constants for
easy use in BlueSky.
"""

cimport cython
import numpy as np
cimport numpy as np
from libc.math cimport sin, cos, sqrt, asin, atan2, fabs, pi

# Constants
cdef double nm = 1852.  # m       1 nautical mile
cdef double a = 6378137.0       # [m] Major semi-axis WGS-84
cdef double b = 6356752.314245  # [m] Minor semi-axis WGS-84

# Typedef the floating point type used in numpy arrays
DTYPE = np.double
ctypedef np.double_t DTYPE_t


@cython.nonecheck(False)
@cython.embedsignature(True)
def rwgs84(latd):
    """
    Calculate the earths radius with WGS'84 geoid definition
        In:  lat [deg] (latitude)
        Out: R   [m]   (earth radius)
    """

    if isinstance(latd, np.ndarray):
        return _rwgs84_arr(latd)
    else:
        return _rwgs84(latd)


@cython.nonecheck(False)
@cython.cdivision(True)
cdef DTYPE_t _rwgs84(DTYPE_t latd):
    cdef DTYPE_t lat, coslat, sinlat, an, bn, ad, bd, r

    # Using np.radians(latd) results in a python call (slow) each time
    # the function is used. Using "latd * pi / 180" is faster, but the
    # function result is NOT exactly the same as that of the native python
    # rwgs84 function (which does use np.radians) for all values of latd.
    # lat = np.radians(latd)
    lat = (latd * pi) / 180

    coslat = cos(lat)
    sinlat = sin(lat)

    an = a*a * coslat
    bn = b*b * sinlat
    ad = a * coslat
    bd = b * sinlat

    r = sqrt((an*an + bn*bn) / (ad*ad + bd*bd))

    return r


@cython.nonecheck(False)
@cython.cdivision(True)
@cython.boundscheck(False)
@cython.wraparound(False)
cdef np.ndarray[DTYPE_t, ndim=1] _rwgs84_arr(np.ndarray[DTYPE_t, ndim=1] latd):
    cdef Py_ssize_t ntraf = latd.shape[0]
    cdef np.ndarray[DTYPE_t, ndim=1] r = np.empty(ntraf, DTYPE)

    cdef Py_ssize_t ii
    for ii in range(ntraf):
        r[ii] = _rwgs84(latd[ii])

    return r


@cython.nonecheck(False)
@cython.cdivision(True)
@cython.boundscheck(False)
@cython.wraparound(False)
cpdef rwgs84_matrix(np.ndarray[DTYPE_t, ndim=2] latd):
    """ 
    Calculate the earths radius with WGS'84 geoid definition
        In:  lat [deg] (Vector of latitudes)
        Out: R   [m]   (Vector of radii)
    """
    cdef Py_ssize_t ntraf1 = latd.shape[0]
    cdef Py_ssize_t ntraf2 = latd.shape[1]
    cdef np.ndarray[DTYPE_t, ndim=2] r = np.empty((ntraf1, ntraf2), DTYPE)

    cdef Py_ssize_t ii, jj
    cdef DTYPE_t lat, coslat, sinlat, an, bn, ad, bd
    for ii in range(ntraf1):
        for jj in range(ntraf2):
            r[ii, jj] = _rwgs84(latd[ii, jj])

    return np.mat(r)


## Cythonized and checked until this point


@cython.nonecheck(False)
@cython.embedsignature(True)
def qdrdist(latd1, lond1, latd2, lond2):
    """
    Calculate bearing and distance, using WGS'84
        In:
            latd1,lond1 en latd2, lond2 [deg] :positions 1 & 2
        Out:
            qdr [deg] = heading from 1 to 2
            d [nm]    = distance from 1 to 2 in nm
    """

    if isinstance(latd1, np.ndarray):
        assert latd1.shape == lond1.shape == latd2.shape == lond2.shape
        return _qdrdist_arr(latd1, lond1, latd2, lond2)
    else:
        return _qdrdist(latd1, lond1, latd2, lond2)


@cython.cdivision(True)
cpdef _qdrdist(DTYPE_t latd1, DTYPE_t lond1, DTYPE_t latd2, DTYPE_t lond2):
    cdef DTYPE_t r, r1, r2
    cdef DTYPE_t lat1, lon1, lat2, lon2, sin1, sin2, coslat1, coslat2, root, d
    cdef DTYPE_t qdr_rad, qdr

    # Haversine with average radius
    # Check for hemisphere crossing,
    # when simple average would not work
    if latd1 * latd2 >= 0.0:
        # Same hemisphere
        r = _rwgs84(0.5 * (latd1 + latd2))
    else:
        # Different hemispheres
        r1   = _rwgs84(latd1)
        r2   = _rwgs84(latd2)
        r = 0.5 * (fabs(latd1) * (r1 + a) + fabs(latd2) * (r2 + a)) / \
                   (fabs(latd1) + fabs(latd2))

    # Convert to radians
    lat1 = np.radians(latd1)
    lon1 = np.radians(lond1)
    lat2 = np.radians(latd2)
    lon2 = np.radians(lond2)

    sin1 = sin(0.5 * (lat2 - lat1))
    sin2 = sin(0.5 * (lon2 - lon1))
    coslat1 = cos(lat1)
    coslat2 = cos(lat2)

    root = sin1*sin1 + coslat1 * coslat2 * sin2*sin2
    d    =  2.0 * r * atan2(sqrt(root) , sqrt(1.0 - root))
    #    d =2.*r*np.arcsin(np.sqrt(sin1*sin1 + coslat1*coslat2*sin2*sin2))

    # Bearing from Ref. http://www.movable-type.co.uk/scripts/latlong.html
    qdr = atan2(sin(lon2 - lon1) * coslat2,
                coslat1 * sin(lat2) - sin(lat1) * coslat2 * cos(lon2 - lon1))

    return qdr, d/nm


@cython.boundscheck(False)
@cython.wraparound(False)
cdef _qdrdist_arr(DTYPE_t[:] latd1,
                  DTYPE_t[:] lond1,
                  DTYPE_t[:] latd2,
                  DTYPE_t[:] lond2):
    cdef Py_ssize_t ntraf = latd1.shape[0]
    cdef np.ndarray[DTYPE_t, ndim=1] qdr = np.empty(ntraf, DTYPE)
    cdef np.ndarray[DTYPE_t, ndim=1] dist = np.empty(ntraf, DTYPE)

    cdef Py_ssize_t ii
    for ii in range(ntraf):
        (qdr[ii], dist[ii]) = _qdrdist(latd1[ii], lond1[ii], 
                                       latd2[ii], lond2[ii])

    return qdr, dist


@cython.boundscheck(False)
@cython.wraparound(False)
cpdef qdrdist_matrix(DTYPE_t[:] lat1, 
                     DTYPE_t[:] lon1,
                     DTYPE_t[:] lat2,
                     DTYPE_t[:] lon2):
    """
    Calculate bearing and distance vectors, using WGS'84
        In:
            latd1,lond1 en latd2, lond2 [deg] :positions 1 & 2 (vectors)
        Out:
            qdr [deg] = heading from 1 to 2 (matrix)
            d [nm]    = distance from 1 to 2 in nm (matrix) 
    """

    assert lat1.shape == lon1.shape and lat2.shape == lon2.shape

    cdef Py_ssize_t ntraf1 = lat1.shape[0]
    cdef Py_ssize_t ntraf2 = lat2.shape[0]
    cdef np.ndarray[DTYPE_t, ndim=2] qdr = np.empty((ntraf1, ntraf2), DTYPE)
    cdef np.ndarray[DTYPE_t, ndim=2] dist = np.empty((ntraf1, ntraf2), DTYPE)

    cdef Py_ssize_t ii, jj
    for ii in range(ntraf1):
        for jj in range(ntraf2):
            (qdr[ii, jj], dist[ii, jj]) = _qdrdist(lat1[ii], lon1[ii],
                                                   lat2[jj], lon2[jj])

    return np.mat(qdr), np.mat(dist)


@cython.cdivision(True)
cpdef DTYPE_t latlondist(DTYPE_t latd1,
                         DTYPE_t lond1,
                         DTYPE_t latd2,
                         DTYPE_t lond2):
    """
    Calculate bearing and distance, using WGS'84
        In:
            latd1,lond1 and latd2, lond2 [deg] :positions 1 & 2
        Out:
            d [m] = distance from 1 to 2 in meters
    """

    cdef DTYPE_t r, r1, r2
    cdef DTYPE_t lat1, lon1, lat2, lon2, sin1, sin2, coslat1, coslat2, root, d

    # Haversine with average radius
    # Check for hemisphere crossing,
    # when simple average would not work
    if latd1 * latd2 >= 0.0:
        # Same hemisphere
        r = _rwgs84(0.5 * (latd1 + latd2))
    else:
        # Different hemispheres
        r1   = _rwgs84(latd1)
        r2   = _rwgs84(latd2)
        r = 0.5 * (fabs(latd1) * (r1 + a) + fabs(latd2) * (r2 + a)) / \
                   (fabs(latd1) + fabs(latd2))

    # Convert to radians
    lat1 = np.radians(latd1)
    lon1 = np.radians(lond1)
    lat2 = np.radians(latd2)
    lon2 = np.radians(lond2)

    sin1 = sin(0.5 * (lat2 - lat1))
    sin2 = sin(0.5 * (lon2 - lon1))
    coslat1 = cos(lat1)
    coslat2 = cos(lat2)

    root = sin1*sin1 + coslat1 * coslat2 * sin2*sin2
    d    =  2.0 * r * atan2(sqrt(root) , sqrt(1.0 - root))
    #    d =2.*r*np.arcsin(np.sqrt(sin1*sin1 + coslat1*coslat2*sin2*sin2))

    return d


@cython.boundscheck(False)
@cython.wraparound(False)
cpdef latlondist_matrix(DTYPE_t[:] lat1,
                        DTYPE_t[:] lon1,
                        DTYPE_t[:] lat2,
                        DTYPE_t[:] lon2):
    """
    Calculates distance using haversine formulae and avaerage r from wgs'84
        Input:
              two lat/lon position vectors in degrees
        Out:
              distance vector in meters !!!!
    """

    assert lat1.shape == lon1.shape and lat2.shape == lon2.shape

    cdef Py_ssize_t ntraf1 = lat1.shape[0]
    cdef Py_ssize_t ntraf2 = lat2.shape[0]
    cdef np.ndarray[DTYPE_t, ndim=2] dist = np.empty((ntraf1, ntraf2), DTYPE)

    cdef Py_ssize_t ii, jj
    for ii in range(ntraf1):
        for jj in range(ntraf2):
            dist[ii, jj] = latlondist(lat1[ii], lon1[ii], lat2[jj], lon2[jj])

    return np.mat(dist)


@cython.cdivision(True)
cpdef DTYPE_t wgsg(DTYPE_t latd):
    """
    Gravity acceleration at a given latitude according to WGS'84
    """

    cdef DTYPE_t geq, e2, k, sinlat, g

    geq = 9.7803   # m/s2 g at equator
    e2 = 6.694e-3  # eccentricity
    k  = 0.001932  # derived from flattening f, 1/f = 298.257223563
 
    sinlat = sin(np.radians(latd))
    g = geq * (1.0 + k*sinlat*sinlat) / sqrt(1.0 - e2*sinlat*sinlat)

    return g


def qdrpos(latd1, lond1, qdr, dist):
    """ 
    Calculate vector with positions from vectors of reference position,
    bearing and distance.
        In:
             latd1,lond1  [deg]   ref position(s)
             qdr          [deg]   bearing (vector) from 1 to 2
             dist         [nm]    distance (vector) between 1 and 2
        Out:
             latd2,lond2 (IN DEGREES!)
    Ref for qdrpos: http://www.movable-type.co.uk/scripts/latlong.html
    """

    if isinstance(latd1, np.ndarray):
        assert latd1.shape == lond1.shape == qdr.shape == dist.shape
        return _qdrpos_arr(latd1, lond1, qdr, dist)
    else:
        return _qdrpos(latd1, lond1, qdr, dist)


@cython.cdivision(True)
cdef _qdrpos(DTYPE_t latd1,
             DTYPE_t lond1,
             DTYPE_t qdr,
             DTYPE_t dist):
    cdef DTYPE_t r, lat1, lon1, lat2, lon2, lat2_deg, lon2_deg, qdr_rad

    # Unit conversion
    r = _rwgs84(latd1) / nm
    lat1 = np.radians(latd1)
    lon1 = np.radians(lond1)
    qdr_rad = np.radians(qdr)

    # Calculate new position
    lat2 = asin(sin(lat1) * cos(dist/r)
                + cos(lat1) * sin(dist/r) * cos(qdr_rad))
    lon2 = lon1 + atan2(sin(qdr_rad) * sin(dist/r) * cos(lat1),
                        cos(dist/r) - sin(lat1) * sin(lat2))

    lat2_deg = np.radians(lat2)
    lon2_deg = np.radians(lon2) 

    return lat2_deg, lon2_deg


@cython.boundscheck(False)
@cython.wraparound(False)
cpdef _qdrpos_arr(DTYPE_t[:] latd1,
                  DTYPE_t[:] lond1,
                  DTYPE_t[:] qdr,
                  DTYPE_t[:] dist):
    cdef Py_ssize_t ntraf = latd1.shape[0]
    cdef np.ndarray[DTYPE_t, ndim=1] lat2_deg = np.empty(ntraf, DTYPE)
    cdef np.ndarray[DTYPE_t, ndim=1] lon2_deg = np.empty(ntraf, DTYPE)

    cdef Py_ssize_t ii
    for ii in range(ntraf):
        (lat2_deg[ii], lon2_deg[ii]) = _qdrpos(latd1[ii], lond1[ii],
                                               qdr[ii], dist[ii])

    return lat2_deg, lon2_deg


cpdef kwikdist(lata, lona, latb, lonb):
    """
    Quick and dirty dist [nm]
    In:
        lat/lon, lat/lon [deg]
    Out:
        dist [nm]
    """

    re      = 6371000.  # readius earth [m]
    dlat    = np.radians(latb - lata)
    dlon    = np.radians(lonb - lona)
    cavelat = np.cos(np.radians(lata + latb) * 0.5)

    dangle  = np.sqrt(dlat * dlat + dlon * dlon * cavelat * cavelat)
    dist    = re * dangle / nm

    return dist


def kwikdist_matrix(lata, lona, latb, lonb):
    """
    Quick and dirty dist [nm]
    In:
        lat/lon, lat/lon vectors [deg]
    Out:
        dist vector [nm]
    """

    re      = 6371000.  # readius earth [m]
    dlat    = np.radians(latb - lata.T)
    dlon    = np.radians(lonb - lona.T)
    cavelat = np.cos(np.radians(lata + latb.T) * 0.5)

    dangle  = np.sqrt(np.multiply(dlat, dlat) +
                      np.multiply(np.multiply(dlon, dlon),
                                  np.multiply(cavelat, cavelat)))
    dist    = re * dangle / nm

    return dist


def kwikqdrdist(lata, lona, latb, lonb):
    """Gives quick and dirty qdr[deg] and dist [nm]
       from lat/lon. (note: does not work well close to poles)"""

    re      = 6371000.  # radius earth [m]
    dlat    = np.radians(latb - lata)
    dlon    = np.radians(lonb - lona)
    cavelat = np.cos(np.radians(lata + latb) * 0.5)

    dangle  = np.sqrt(dlat * dlat + dlon * dlon * cavelat * cavelat)
    dist    = re * dangle / nm

    qdr     = np.degrees(np.arctan2(dlon * cavelat, dlat)) % 360.

    return qdr, dist


def kwikqdrdist_matrix(lata, lona, latb, lonb):
    """Gives quick and dirty qdr[deg] and dist [nm] matrices
       from lat/lon vectors. (note: does not work well close to poles)"""

    re      = 6371000.  # radius earth [m]
    dlat    = np.radians(latb - lata.T)
    dlon    = np.radians(lonb - lona.T)
    cavelat = np.cos(np.radians(lata + latb.T) * 0.5)

    dangle  = np.sqrt(np.multiply(dlat, dlat) +
                      np.multiply(np.multiply(dlon, dlon),
                                  np.multiply(cavelat, cavelat)))
    dist    = re * dangle / nm

    qdr     = np.degrees(np.arctan2(np.multiply(dlon, cavelat), dlat)) % 360.

    return qdr, dist

def kwikpos(latd1, lond1, qdr, dist):
    """ Fast, but quick and dirty, position calculation from vectors of reference position,
        bearing and distance using flat earth approximation
        In:
             latd1,lond1  [deg]   ref position(s)
             qdr          [deg]   bearing (vector) from 1 to 2
             dist         [nm]    distance (vector) between 1 and 2
        Out:
             latd2,lond2 [deg]
        Use for flat earth purposes e.g. flat display"""

    dx = dist*np.sin(np.radians(qdr))
    dy = dist*np.cos(np.radians(qdr))
    dlat = dy/60.
    dlon = dx/(np.maximum(0.01,60.*np.cos(np.radians(latd1))))
    latd2 = latd1 + dlat
    lond2 = lond1 + dlon
    return latd2,lond2
