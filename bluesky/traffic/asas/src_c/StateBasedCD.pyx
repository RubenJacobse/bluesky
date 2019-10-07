""" State-based conflict detection. """

# Cython imports
import cython
from libc.math cimport log, atan2, sqrt, tan, sin, cos, pi, fabs
import numpy as np
cimport numpy as np

cdef double RAD_TO_DEG = 180 / pi
cdef double DEG_TO_RAD = pi / 180

ctypedef np.float64_t DTYPE_t

cdef struct traf:
    int ntraf
    DTYPE_t *lat
    DTYPE_t *lon
    DTYPE_t *trk
    DTYPE_t *gs
    DTYPE_t *alt
    DTYPE_t *vs 

ctypedef traf traf_arr

cpdef detect(ownship, intruder, RPZ, HPZ, tlookahead):
    """ Conflict detection between ownship (traf) and intruder (traf/adsb)."""

    # Declare C type variables to which Python variables will be cast
    cdef traf_arr ownship_, intruder_
    cdef DTYPE_t pz_radius, pz_height, t_lookahead

    # Typecast double types
    pz_radius = <DTYPE_t> RPZ
    pz_height = <DTYPE_t> HPZ
    t_lookahead = <DTYPE_t> tlookahead

    # Store traffic variables in struct type
    ownship_ = convert_to_struct(ownship)
    intruder_ = convert_to_struct(intruder)

    # Call actual detection function
    detect_all(ownship_, intruder_, pz_radius, pz_height, t_lookahead)

@cython.boundscheck(False)
@cython.wraparound(False)
cdef traf_arr convert_to_struct(traf_obj):
    """ 
    Extract the relevant parameters of the traf object to a 
    traf_arr struct.
    """

    cdef traf_arr arr

    # Convert numpy arrays to memoryviews
    cdef DTYPE_t[::1] lat = np.ascontiguousarray(traf_obj.lat)
    cdef DTYPE_t[::1] lon = np.ascontiguousarray(traf_obj.lon)
    cdef DTYPE_t[::1] trk = np.ascontiguousarray(traf_obj.trk)
    cdef DTYPE_t[::1] gs = np.ascontiguousarray(traf_obj.gs)
    cdef DTYPE_t[::1] alt = np.ascontiguousarray(traf_obj.alt)
    cdef DTYPE_t[::1] vs = np.ascontiguousarray(traf_obj.vs)

    # Store data in struct
    arr.ntraf = <int> traf_obj.ntraf
    arr.lat = &lat[0]
    arr.lon = &lon[0]
    arr.trk = &trk[0]
    arr.gs = &gs[0]
    arr.alt = &alt[0]
    arr.vs = &vs[0]

    return arr


@cython.boundscheck(False)
@cython.wraparound(False)
cpdef detect_all(traf_arr ownship,
                 traf_arr intruder,
                 DTYPE_t pz_radius,
                 DTYPE_t pz_height,
                 DTYPE_t t_lookahead):

    cdef DTYPE_t qdr, dist, dx, dy, owntrkrad, ownu, ownv, inttrkrad, intu, \
    intv, du, dv, dv2, vrel, tcpa, dpa2, R2, dxinhor, dtinhor, tinhor, dalt, \
    dvs, tcrosslo, tcrosshi, tinver, toutver, tinconf, toutconf
    cdef bint is_hor_conf, is_conf

    cdef int ii, jj
    for ii in range(ownship.ntraf):
        for jj in range(intruder.ntraf):
            # Skip conflict with self
            if ii == jj:
                continue

            # Horizontal conflict ----------------------------------------------
            qdr = geo.qdr(ownship.lat[ii], 
                          ownship.lon[ii],
                          intruder.lat[jj],
                          intruder.lon[jj])
            dist = geo.dist(ownship.lat[ii], 
                            ownship.lon[ii],
                            intruder.lat[jj],
                            intruder.lon[jj])

            # Convert distance to nm and qdr to radians
            dist = dist * nm 
            qdrrad = qdr * DEG_TO_RAD

            # Calculate horizontal closest point of approach (CPA)
            dx = dist * sin(qdrrad)
            dy = dist * cos(qdrrad)

            # Ownship track angle and speed
            owntrkrad = ownship.trk[ii] * DEG_TO_RAD
            ownu = ownship.gs[ii] * sin(owntrkrad)
            ownv = ownship.gs[ii] * cos(owntrkrad)

            # Intruder track angle and speed
            inttrkrad = intruder.trk[jj] * DEG_TO_RAD
            intu = intruder.gs[jj] * sin(inttrkrad)
            intv = intruder.gs[jj] * cos(inttrkrad)

            # Relative velocities
            du = ownu - intu
            dv = ownv - intv
            dv2 = du * du + dv * dv
            vrel = sqrt(dv2)

            tcpa = -(du * dx + dv * dy) / dv2
            dcpa2 = fabs(dist * dist - tcpa * tcpa * dv2)
            R2 = pz_radius * pz_radius
            is_hor_conf = dcpa2 < R2

            # Calculate times of entering and leaving horizontal conflict
            dxinhor = sqrt(max(0.0, R2 - dcpa2))  # half the distance travelled inzide zone
            dtinhor = dxinhor / vrel

            tinhor, touthor
            if is_hor_conf:
                tinhor = tcpa - dtinhor
                touthor = tcpa + dtinhor
            else:
                tinhor = 1e8
                touthor = -1e8

            # Vertical crossing of disk (-dh,+dh)
            dalt = ownship.alt[ii] - intruder.alt[jj]
            dvs = ownship.vs[ii] - ownship.vs[jj]

            if abs(dvs) < 1e-6:
                dvs = 1e-6

            tcrosslo = (dalt - pz_height) / -dvs
            tcrosshi = (dalt + pz_height) / -dvs

            tinver = min(tcrosshi, tcrosslo)
            toutver = max(tcrosshi, tcrosslo)

            tinconf = max(tinver, tinhor)
            toutconf = min(toutver, touthor)

            is_conf = is_hor_conf * (tinconf <= toutconf) * (toutconf > 0.0) * (tinconf < t_lookahead)


               

# cpdef detect(ownship, intruder, RPZ, HPZ, tlookahead):
#     """ Conflict detection between ownship (traf) and intruder (traf/adsb)."""

#     # Identity matrix of order ntraf: avoid ownship-ownship detected conflicts
#     I = np.eye(ownship.ntraf)

#     # Horizontal conflict ------------------------------------------------------

#     # qdlst is for [i,j] qdr from i to j, from perception of ADSB and own coordinates
#     qdr, dist = geo.kwikqdrdist_matrix(np.mat(ownship.lat), np.mat(ownship.lon),
#                                        np.mat(intruder.lat), np.mat(intruder.lon))

#     # Convert back to array to allow element-wise array multiplications later on
#     # Convert to meters and add large value to own/own pairs
#     qdr = np.array(qdr)
#     dist = np.array(dist) * nm + 1e9 * I

#     # Calculate horizontal closest point of approach (CPA)
#     qdrrad = np.radians(qdr)
#     dx = dist * np.sin(qdrrad)  # is pos j rel to i
#     dy = dist * np.cos(qdrrad)  # is pos j rel to i

#     # Ownship track angle and speed
#     owntrkrad = np.radians(ownship.trk)
#     ownu = ownship.gs * np.sin(owntrkrad).reshape((1, ownship.ntraf))  # m/s
#     ownv = ownship.gs * np.cos(owntrkrad).reshape((1, ownship.ntraf))  # m/s
 
#     # Intruder track angle and speed
#     inttrkrad = np.radians(intruder.trk)
#     intu = intruder.gs * np.sin(inttrkrad).reshape((1, ownship.ntraf))  # m/s
#     intv = intruder.gs * np.cos(inttrkrad).reshape((1, ownship.ntraf))  # m/s

#     du = ownu - intu.T  # Speed du[i,j] is perceived eastern speed of i to j
#     dv = ownv - intv.T  # Speed dv[i,j] is perceived northern speed of i to j

#     dv2 = du * du + dv * dv
#     dv2 = np.where(np.abs(dv2) < 1e-6, 1e-6, dv2)  # limit lower absolute value
#     vrel = np.sqrt(dv2)

#     tcpa = -(du * dx + dv * dy) / dv2 + 1e9 * I

#     # Calculate distance^2 at CPA (minimum distance^2)
#     dcpa2 = np.abs(dist * dist - tcpa * tcpa * dv2)

#     # Check for horizontal conflict
#     R2 = RPZ * RPZ
#     swhorconf = dcpa2 < R2  # conflict or not

#     # Calculate times of entering and leaving horizontal conflict
#     dxinhor = np.sqrt(np.maximum(0., R2 - dcpa2))  # half the distance travelled inzide zone
#     dtinhor = dxinhor / vrel

#     tinhor = np.where(swhorconf, tcpa - dtinhor, 1e8)  # Set very large if no conf
#     touthor = np.where(swhorconf, tcpa + dtinhor, -1e8)  # set very large if no conf

#     # Vertical conflict --------------------------------------------------------

#     # Vertical crossing of disk (-dh,+dh)
#     dalt = ownship.alt.reshape((1, ownship.ntraf)) - \
#         intruder.alt.reshape((1, ownship.ntraf)).T  + 1e9 * I

#     dvs = ownship.vs.reshape(1, ownship.ntraf) - \
#         intruder.vs.reshape(1, ownship.ntraf).T
#     dvs = np.where(np.abs(dvs) < 1e-6, 1e-6, dvs)  # prevent division by zero

#     # Check for passing through each others zone
#     tcrosshi = (dalt + HPZ) / -dvs
#     tcrosslo = (dalt - HPZ) / -dvs
#     tinver = np.minimum(tcrosshi, tcrosslo)
#     toutver = np.maximum(tcrosshi, tcrosslo)

#     # Combine vertical and horizontal conflict----------------------------------
#     tinconf = np.maximum(tinver, tinhor)
#     toutconf = np.minimum(toutver, touthor)

#     swconfl = np.array(swhorconf * (tinconf <= toutconf) * (toutconf > 0.0) * \
#         (tinconf < tlookahead) * (1.0 - I), dtype=np.bool)

#     # --------------------------------------------------------------------------
#     # Update conflict lists
#     # --------------------------------------------------------------------------
#     # Ownship conflict flag and max tCPA
#     inconf = np.any(swconfl, 1)
#     tcpamax = np.max(tcpa * swconfl, 1)

#     # Select conflicting pairs: each a/c gets their own record
#     confpairs = [(ownship.id[i], ownship.id[j]) for i, j in zip(*np.where(swconfl))]
#     swlos = (dist < RPZ) * (np.abs(dalt) < HPZ)
#     lospairs = [(ownship.id[i], ownship.id[j]) for i, j in zip(*np.where(swlos))]

#     # bearing, dist, tcpa, tinconf, toutconf per conflict
#     qdr = qdr[swconfl]
#     dist = dist[swconfl]
#     tcpa = tcpa[swconfl]
#     dcpa = np.sqrt(dcpa2[swconfl])
#     tinconf = tinconf[swconfl]

#     return confpairs, lospairs, inconf, tcpamax, qdr, dist, dcpa, tcpa, tinconf
