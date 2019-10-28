""" State-based conflict detection. """

# Cython imports
import cython
from libc.math cimport log, atan2, sqrt, tan, sin, cos, pi, fabs
import numpy as np
cimport numpy as np
cimport cgeo

cdef double RAD_TO_DEG = 180 / pi
cdef double DEG_TO_RAD = pi / 180
cdef double nm = 1852.0

ctypedef np.float32_t DTYPE_t

cdef struct traf:
    int ntraf
    double *lat
    double *lon
    double *trk
    double *gs
    double *alt
    double *vs

ctypedef traf traf_arr


cpdef detect(ownship, intruder, RPZ, HPZ, tlookahead):
    """ Conflict detection between ownship (traf) and intruder (traf/adsb)."""

    # Declare C type variables to which Python variables will be cast
    cdef traf_arr ownship_, intruder_
    cdef double pz_radius, pz_height, t_lookahead

    # Ensure typecast to double types
    pz_radius = <double> RPZ
    pz_height = <double> HPZ
    t_lookahead = <double> tlookahead

    # Store traffic variables in struct type
    ownship_ = convert_to_struct(ownship)
    intruder_ = convert_to_struct(intruder)

    # Call actual detection function
    (confidxpairs, losidxpairs, inconf, tcpamax, conf_qdr, conf_dist, conf_dcpa,
     conf_tcpa, conf_tinconf) = detect_all(ownship_, intruder_, pz_radius,
                                           pz_height, t_lookahead)

    confpairs = [(ownship.id[i], ownship.id[j]) for (i, j) in confidxpairs]
    lospairs = [(ownship.id[i], ownship.id[j]) for (i, j) in losidxpairs]

    return (confpairs, lospairs, inconf, tcpamax, conf_qdr, conf_dist,
            conf_dcpa, conf_tcpa, conf_tinconf)

@cython.boundscheck(False)
@cython.wraparound(False)
cdef traf_arr convert_to_struct(traf_obj):
    """ 
    Extract the relevant parameters of the traf object to a 
    traf_arr struct.
    """

    cdef traf_arr arr

    # Convert numpy arrays to memoryviews
    cdef double[::1] lat = np.ascontiguousarray(traf_obj.lat)
    cdef double[::1] lon = np.ascontiguousarray(traf_obj.lon)
    cdef double[::1] trk = np.ascontiguousarray(traf_obj.trk)
    cdef double[::1] gs = np.ascontiguousarray(traf_obj.gs)
    cdef double[::1] alt = np.ascontiguousarray(traf_obj.alt)
    cdef double[::1] vs = np.ascontiguousarray(traf_obj.vs)

    # Store data in struct
    arr.ntraf = <int> traf_obj.ntraf
    arr.lat = &lat[0]
    arr.lon = &lon[0]
    arr.trk = &trk[0]
    arr.gs = &gs[0]
    arr.alt = &alt[0]
    arr.vs = &vs[0]

    return arr


@cython.cdivision(True)
@cython.boundscheck(False)
@cython.wraparound(False)
cdef detect_all(traf_arr ownship,
                traf_arr intruder,
                double pz_radius,
                double pz_height,
                double t_lookahead):

    cdef double qdr, dist, dx, dy, owntrkrad, ownu, ownv, inttrkrad, intu, \
    intv, du, dv, dv2, vrel, tcpa, dcpa2, R2, dxinhor, dtinhor, tinhor, \
    touthor, dalt, dvs, tcrosslo, tcrosshi, tinver, toutver, tinconf, toutconf
    cdef int is_hor_conf, is_conf, is_los

    # Fixed-size array with one element for each aircraft
    cdef np.ndarray[int, ndim = 1] inconf = np.zeros((ownship.ntraf), dtype=int)
    cdef np.ndarray[DTYPE_t, ndim=1] tcpamax = np.zeros((ownship.ntraf), dtype=np.float32)

    # Dynamically sized based on number of pairs in conflict
    cdef list confpairs = []
    cdef np.ndarray[DTYPE_t, ndim=1] conf_qdr = np.zeros((0), dtype=np.float32)
    cdef np.ndarray[DTYPE_t, ndim=1] conf_dist = np.zeros((0), dtype=np.float32)
    cdef np.ndarray[DTYPE_t, ndim=1] conf_tcpa = np.zeros((0), dtype=np.float32)
    cdef np.ndarray[DTYPE_t, ndim=1] conf_dcpa = np.zeros((0), dtype=np.float32)
    cdef np.ndarray[DTYPE_t, ndim=1] conf_tinconf = np.zeros((0), dtype=np.float32)

    # Dynamically sized based on number of pairs with loss of separation
    cdef list lospairs = []

    cdef int ii, jj
    for ii in range(ownship.ntraf):
        for jj in range(intruder.ntraf):
            # Skip conflict with self
            if ii == jj:
                continue

            # Horizontal conflict ----------------------------------------------
            qdr = cgeo.cy_qdr(ownship.lat[ii], 
                              ownship.lon[ii],
                              intruder.lat[jj],
                              intruder.lon[jj])
            dist = cgeo.cy_dist(ownship.lat[ii], 
                                ownship.lon[ii],
                                intruder.lat[jj],
                                intruder.lon[jj])

            # Convert distance to m and qdr to radians
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

            if is_hor_conf:
                tinhor = tcpa - dtinhor
                touthor = tcpa + dtinhor
            else:
                tinhor = 1e8
                touthor = -1e8

            # Vertical crossing of disk (-dh,+dh)
            dalt = ownship.alt[ii] - intruder.alt[jj]
            dvs = ownship.vs[ii] - ownship.vs[jj]

            if fabs(dvs) < 1e-6:
                dvs = 1e-6

            tcrosslo = (dalt - pz_height) / -dvs
            tcrosshi = (dalt + pz_height) / -dvs

            tinver = min(tcrosshi, tcrosslo)
            toutver = max(tcrosshi, tcrosslo)

            tinconf = max(tinver, tinhor)
            toutconf = min(toutver, touthor)

            is_conf = is_hor_conf * (tinconf <= toutconf) * (toutconf > 0.0) \
                        * (tinconf < t_lookahead)
            is_los = (dist < pz_radius) * (fabs(dalt) < pz_height)

            if is_conf:
                confpairs.append((ii, jj))
                inconf[ii] = True
                if tcpa > tcpamax[ii]:
                    tcpamax[ii] = tcpa
                conf_qdr = np.append(conf_qdr, [qdr])
                conf_dist = np.append(conf_dist, [dist])
                conf_dcpa = np.append(conf_tcpa, [sqrt(dcpa2)])
                conf_tcpa = np.append(conf_tcpa, [tcpa])
                conf_tinconf = np.append(conf_tinconf, [tinconf])

            if is_los:
                lospairs.append((ii, jj))

    return (confpairs, lospairs, inconf, tcpamax, conf_qdr, conf_dist,
            conf_dcpa, conf_tcpa, conf_tinconf)
