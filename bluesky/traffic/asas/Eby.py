# -*- coding: utf-8 -*-
"""
Created on Tue Mar 03 16:50:19 2015

@author: Jerom Maas
"""

import numpy as np
from bluesky.tools.aero import vtas2eas


def start(asas):
    pass


def resolve(asas, traf):
    """ Resolve all current conflicts """

    # Initialize an array to store the resolution velocity vector for all A/C
    delta_v = np.zeros((traf.ntraf, 3))

    # Stores resolution vector, also used in visualization
    asas.asasn = np.zeros(traf.ntraf, dtype=np.float32)
    asas.asase = np.zeros(traf.ntraf, dtype=np.float32)

    # Call Eby function to resolve conflicts
    for (ac1, ac2), qdr, dist in zip(asas.confpairs, asas.qdr, asas.dist):
        idx1 = traf.id2idx(ac1)
        idx2 = traf.id2idx(ac2)

        # Do not resolve current pair if:
        # - the pair contains an aircraft that no longer exists
        # - resolutions are switched off for ac1
        # - ac2 is a noreso aircraft which ac1 should not avoid
        if (not (idx1 > -1 and idx2 > -1)
                or (asas.swresooff and ac1 in asas.resoofflst)
                or (asas.swnoreso and ac2 in asas.noresolst)):
            continue

        dv_eby = Eby(traf, asas, qdr, dist, idx1, idx2)
        delta_v[idx1] -= dv_eby

    # Determine new speed and limit resolution direction for all aicraft-------

    # Resolution vector for all aircraft, cartesian coordinates
    delta_v = np.transpose(delta_v)

    # The old speed vector, cartesian coordinates
    v = np.array([traf.gseast, traf.gsnorth, traf.vs])

    # The new speed vector, cartesian coordinates
    new_v = delta_v + v

    # Get indices of aircraft that have a resolution
    idxs = delta_v[0, :]**2 + delta_v[1, :]**2 > 0

    # The new speed vector in polar coordinates
    new_track = np.degrees(np.arctan2(new_v[0, :], new_v[1, :])) % 360
    new_gs = np.sqrt(new_v[0, :]**2 + new_v[1, :]**2)
    new_vs = new_v[2, :]

    # Determine ASAS module commands for all aircraft--------------------------

    # Cap the velocity
    new_gs_capped = np.clip(new_gs, asas.vmin, asas.vmax)

    # Now assign resolutions to variables in the ASAS class
    asas.trk = new_track
    asas.tas = new_gs_capped
    asas.vs = new_vs
    asas.alt = traf.alt #np.sign(asas.vs) * 1e5

    # Stores resolution vector
    asas.asase[idxs] = asas.tas[idxs] * np.sin(asas.trk[idxs] / 180 * np.pi)
    asas.asasn[idxs] = asas.tas[idxs] * np.cos(asas.trk[idxs] / 180 * np.pi)
    # asaseval should be set to True now
    if not asas.asaseval:
        asas.asaseval = True


def Eby(traf, asas, qdr, dist, idx1, idx2):
    """
    Eby Voltage potential resolution method.
    """

    # Convert qdr from degrees to radians
    qdr = np.radians(qdr)

    # Relative position vector between idx1 and idx2
    drel = np.array([np.sin(qdr) * dist,
                     np.cos(qdr) * dist,
                     traf.alt[idx2] - traf.alt[idx1]])

    # Write velocities as vectors and find relative velocity vector
    v1 = np.array([traf.gseast[idx1], traf.gsnorth[idx1], traf.vs[idx1]])
    v2 = np.array([traf.gseast[idx2], traf.gsnorth[idx2], traf.vs[idx2]])
    vrel = np.array(v2 - v1)

    # bear in mind: the definition of vr (relative velocity) is opposite to
    # the velocity vector in the LOS_nominal method, this just has consequences
    # for the derivation of tstar following Eby method, not more
    """
    intrusion vector:
    i(t)=self.hsep-d(t)
    d(t)=sqrt((d[0]+v[0]*t)**2+(d[1]+v[1]*t)**2)
    find max(i(t)/t)
    -write the equation out
    -take derivative, set to zero
    -simplify, take square of everything so the sqrt disappears (creates two solutions)
    -write to the form a*t**2 + b*t + c = 0
    -Solve using the quadratic formula
    """
    # These terms are used to construct a,b,c of the quadratic formula
    R2 = asas.Rm**2 # in meters
    d2 = np.dot(drel, drel) # distance vector length squared
    v2 = np.dot(vrel, vrel) # velocity vector length squared
    dv = np.dot(drel, vrel) # dot product of distance and velocity

    # Solving the quadratic formula
    a = R2 * v2 - dv**2
    b = 2 * dv * (R2 - d2)
    c = R2 * d2 - d2**2
    discrim = b**2 - 4 * a * c

    # If the discriminant is negative, we're done as taking the square root
    # will result in an error
    if discrim < 0:
        discrim = 0
    time1 = ( -b + np.sqrt(discrim)) / (2 * a)
    time2 = ( -b - np.sqrt(discrim)) / (2 * a)

    # Time when the size of the conflict is largest relative to time to solve
    tstar = min(abs(time1), abs(time2))

    # Find drel and absolute distance at tstar
    drelstar = drel + vrel * tstar
    dstarabs = np.linalg.norm(drelstar)

    # Exception: if the two aircraft are on exact collision course
    # (passing eachother within 10 meter), change drelstar
    exactcourse = 10 # 10 meter
    dif = exactcourse - dstarabs
    if dif > 0:
        # rotate velocity 90 degrees in horizontal plane
        vperp = np.array([-vrel[1], vrel[0], 0])
        # normalize to 10 m and add to drelstar
        drelstar += dif * vperp / np.linalg.norm(vperp)
        dstarabs = np.linalg.norm(drelstar)

    # intrusion at tstar
    i = asas.Rm - dstarabs

    # desired change in the plane's speed vector:
    delta_v = i * drelstar / (dstarabs * tstar)
    return delta_v
