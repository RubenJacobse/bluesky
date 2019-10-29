""" State-based conflict detection. """

# Cython imports
import cython
from libc.math cimport log, atan2, sqrt, tan, sin, cos, pi, fabs
import numpy as np
cimport numpy as np
cimport cgeo

ctypedef np.float32_t DTYPE_t

cdef double RAD_TO_DEG = 180 / pi
cdef double DEG_TO_RAD = pi / 180
cdef double nm = 1852.0


cpdef detect(ownship, intruder, RPZ, HPZ, tlookahead):
    """ Conflict detection between ownship (traf) and intruder (traf/adsb)."""

    # Declare C type variables to which Python variables will be cast
    cdef double pz_radius, pz_height, t_lookahead

    # Ensure typecast to double types
    pz_radius = <double> RPZ
    pz_height = <double> HPZ
    t_lookahead = <double> tlookahead

    # Call actual detection function
    (confidxpairs, losidxpairs, inconf, tcpamax, conf_qdr, conf_dist, conf_dcpa,
     conf_tcpa, conf_tinconf) = detect_all(ownship, intruder, pz_radius,
                                           pz_height, t_lookahead)

    confpairs = [(ownship.id[ii], intruder.id[jj]) for (ii, jj) in confidxpairs]
    lospairs = [(ownship.id[ii], intruder.id[jj]) for (ii, jj) in losidxpairs]

    return (confpairs, lospairs, inconf, tcpamax, conf_qdr, conf_dist,
            conf_dcpa, conf_tcpa, conf_tinconf)


@cython.cdivision(True)
@cython.boundscheck(False)
@cython.wraparound(False)
cdef detect_all(ownship,
                intruder,
                double pz_radius,
                double pz_height,
                double t_lookahead):

    # Convert ownship variables to memoryviews
    cdef int ownship_ntraf = <int> ownship.ntraf
    cdef double[::1] ownship_lat = np.ascontiguousarray(ownship.lat)
    cdef double[::1] ownship_lon = np.ascontiguousarray(ownship.lon)
    cdef double[::1] ownship_trk = np.ascontiguousarray(ownship.trk)
    cdef double[::1] ownship_gs = np.ascontiguousarray(ownship.gs)
    cdef double[::1] ownship_alt = np.ascontiguousarray(ownship.alt)
    cdef double[::1] ownship_vs = np.ascontiguousarray(ownship.vs)

    # Convert intruder variables to memoryviews
    cdef int intruder_ntraf = <int> intruder.ntraf
    cdef double[::1] intruder_lat = np.ascontiguousarray(intruder.lat)
    cdef double[::1] intruder_lon = np.ascontiguousarray(intruder.lon)
    cdef double[::1] intruder_trk = np.ascontiguousarray(intruder.trk)
    cdef double[::1] intruder_gs = np.ascontiguousarray(intruder.gs)
    cdef double[::1] intruder_alt = np.ascontiguousarray(intruder.alt)
    cdef double[::1] intruder_vs = np.ascontiguousarray(intruder.vs)

    # Prepare fixed-size arrays with one element for each aircraft
    cdef np.ndarray[int, ndim = 1] inconf = np.zeros((ownship_ntraf), dtype=int)
    cdef np.ndarray[DTYPE_t, ndim=1] tcpamax = np.zeros((ownship_ntraf), dtype=np.float32)

    # Dynamically sized variables to store data for each conflict pair
    confpairs = []
    conf_qdr = np.zeros((0), dtype=np.float32)
    conf_dist = np.zeros((0), dtype=np.float32)
    conf_tcpa = np.zeros((0), dtype=np.float32)
    conf_dcpa = np.zeros((0), dtype=np.float32)
    conf_tinconf = np.zeros((0), dtype=np.float32)

    # Dynamically sized list to store each loss of separation pair
    lospairs = []

    # Declare variables used inside double loop
    cdef double qdr, dist, dx, dy, owntrkrad, ownu, ownv, inttrkrad, intu, \
    intv, du, dv, dv2, vrel, tcpa, dcpa2, R2, dxinhor, dtinhor, tinhor, \
    touthor, dalt, dvs, tcrosslo, tcrosshi, tinver, toutver, tinconf, toutconf
    cdef int is_hor_conf, is_conf, is_los, ii, jj

    # Check all ownship-intruder combinations for conflicts and
    # loss of separation
    for ii in range(ownship_ntraf):
        for jj in range(intruder_ntraf):
            # Skip conflict with self
            if ii == jj:
                continue
 
            print(f"{ii},{jj}")
            # Horizontal conflict ----------------------------------------------
            qdr = cgeo.cy_qdr(ownship_lat[ii], 
                              ownship_lon[ii],
                              intruder_lat[jj],
                              intruder_lon[jj])
            dist = cgeo.cy_dist(ownship_lat[ii], 
                                ownship_lon[ii],
                                intruder_lat[jj],
                                intruder_lon[jj])
            print(f"\tqdr: {qdr} dist:{dist}")

            # Convert distance to m and qdr to radians
            dist = dist * nm
            qdrrad = qdr * DEG_TO_RAD

            # Calculate horizontal closest point of approach (CPA)
            dx = dist * sin(qdrrad)
            dy = dist * cos(qdrrad)

            # Ownship track angle and speed
            owntrkrad = ownship_trk[ii] * DEG_TO_RAD
            ownu = ownship_gs[ii] * sin(owntrkrad)
            ownv = ownship_gs[ii] * cos(owntrkrad)

            # Intruder track angle and speed
            inttrkrad = intruder_trk[jj] * DEG_TO_RAD
            intu = intruder_gs[jj] * sin(inttrkrad)
            intv = intruder_gs[jj] * cos(inttrkrad)

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
            print(f"is_hor_conf: {is_hor_conf}")

            # Vertical crossing of disk (-dh,+dh)
            dalt = ownship_alt[ii] - intruder_alt[jj]
            dvs = ownship_vs[ii] - intruder_vs[jj]

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
                inconf[ii] = 1
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
