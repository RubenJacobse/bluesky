"""
LF
==

Implements Leader-Following behaviour method, based on the method
described in the MSc thesis of Arjen Kieskamp at Delft University
of Technology (March 5th, 2009).

Currently only supports 2-dimensional resolutions.

© Ruben Jacobse, 2019
"""

# Third-party imports
import numpy as np

# Local imports
from .MVP import MVP


def start(asas):
    pass


def resolve(asas, traf):
    """ Resolve all current conflicts """

    # Check if ASAS is ON first!
    if not asas.swasas:
        return

    # Store the resolution velocity change vector
    delta_v = np.zeros((traf.ntraf, 3))

    # Stores resolution vector, also used in visualization
    asas.asasn = np.zeros(traf.ntraf, dtype=np.float32)
    asas.asase = np.zeros(traf.ntraf, dtype=np.float32)

    # For each conflict pair calculate resolution for ac1 if necessary
    for ((ac1, ac2), qdr, dist, tcpa, tLOS) in zip(asas.confpairs,
                                                   asas.qdr,
                                                   asas.dist,
                                                   asas.tcpa,
                                                   asas.tLOS):
        idx1 = traf.id.index(ac1)
        idx2 = traf.id.index(ac2)

        # Do not resolve if:
        # - the pair contains an aircraft that no longer exists
        # - resolutions are switched off for ac1
        # - ac2 is a noreso aircraft which nobody avoids
        if idx1 == -1 or idx2 == -1:
            continue
        elif asas.swresooff and ac1 in asas.resoofflst:
            continue
        elif asas.swnoreso and ac2 in asas.noresolst:
            continue

        # Use velocity unit vectors to calculate angle between aircraft velocities
        velocity_ac1 = np.array([traf.gseast[idx1], traf.gsnorth[idx1]])
        velocity_ac2 = np.array([traf.gseast[idx2], traf.gsnorth[idx2]])
        unitvec_ac1 = velocity_ac1 / np.linalg.norm(velocity_ac1)
        unitvec_ac2 = velocity_ac2 / np.linalg.norm(velocity_ac2)
        cosine_angle = np.clip(np.dot(unitvec_ac1, unitvec_ac2), -1.0, 1.0)
        angle = np.arccos(cosine_angle)

        # Find the mode and status of ac1 based on the geometry conflict
        ac1_mode, ac1_status = find_lf_status(asas, traf, angle, tLOS, idx1, idx2)
        ac2_mode, ac2_status = find_lf_status(asas, traf, angle, tLOS, idx2, idx1)

        # Determine whether MVP or follow-through logic has to be used
        if ac1_mode == "MVP" or ac1_status == ac2_status or tLOS < 240 or dist < asas.R:
            dv_mvp, _ = MVP(traf, asas, qdr, dist, tcpa, tLOS, idx1, idx2)
            delta_v[idx1] -= dv_mvp
            # print("using MVP")
        elif ac1_status == "follower":
            dv_lf = LF(traf, asas, qdr, dist, tcpa, tLOS, idx2, idx1)
            delta_v[idx1] -= dv_lf
            # print("using LF")
        else:
            # Leader in "LF" mode takes no action
            # print("no reso")
            pass

    # Add resolution mandated velocity difference to current velocity
    delta_v = np.transpose(delta_v)
    current_v = np.array([traf.gseast, traf.gsnorth, traf.vs])
    new_v = current_v + delta_v

    # Get indices of aircraft that perform a resolution
    idxs = delta_v[0, :] ** 2 + delta_v[1, :] ** 2 > 0

    # Compute new speed in polar coordinates
    new_track = np.degrees(np.arctan2(new_v[0, :], new_v[1, :])) % 360
    new_gs = np.sqrt(new_v[0, :]**2 + new_v[1, :]**2)
    new_vs = traf.vs

    # Ensure the ground speed and vertical speed are within their allowed ranges
    new_gs_capped = np.clip(new_gs, asas.vmin, asas.vmax)
    new_vs_capped = np.clip(new_vs, asas.vsmin, asas.vsmax)

    # Now assign resolutions to variables in the ASAS class
    asas.trk = new_track
    asas.tas = new_gs_capped
    asas.vs = new_vs_capped

    # Stores resolution vector
    asas.asase[idxs] = asas.tas[idxs] * np.sin(np.radians(asas.trk[idxs]))
    asas.asasn[idxs] = asas.tas[idxs] * np.cos(np.radians(asas.trk[idxs]))

    # asaseval should be set to True now
    if not asas.asaseval:
        asas.asaseval = True

    # Resolution is horizontal only
    asas.alt = traf.alt


def find_lf_status(asas, traf, delta_crs, tLOS, idx_ownship, idx_intruder):
    """
    For a conflict between a given ownship and an intruder, find the mode
    and status of the ownship in the conflict.
    """

    # Calculate ownship velocity vector
    dx1_r = asas.R * np.cos(traf.hdg[idx_ownship])
    dx2_r = asas.R * np.sin(traf.hdg[idx_ownship])
    dxy_r = np.array([dx1_r, dx2_r])

    # Calculate intruder relative position and velocity wrt ownship
    dx1 = traf.lon[idx_intruder] - traf.lon[idx_ownship]
    dx2 = traf.lat[idx_intruder] - traf.lat[idx_ownship]
    dvx1 = traf.gseast[idx_intruder] - traf.gseast[idx_ownship]
    dvx2 = traf.gsnorth[idx_intruder] - traf.gsnorth[idx_ownship]

    # Calculate point of intrusion of intruder relative to ownship at tLOS
    dx1_in = dx1 + dvx1 * tLOS
    dx2_in = dx2 + dvx2 * tLOS
    dxy_in = np.array([dx1_in, dx2_in])

    # Calculate cosine of angle between vector to point of intrusion
    # and velocity vector
    cos_phi = (np.dot(dxy_in, dxy_r)
               / (np.linalg.norm(dxy_in) * np.linalg.norm(dxy_r)))

    # Determine status and mode of ac1
    if delta_crs >= np.pi/2:
        ownship_status = "leader"
        ownship_mode = "MVP"
    elif cos_phi >= 0:
        ownship_status = "leader"
        ownship_mode = "LF"
    elif cos_phi < 0:
        ownship_status = "follower"
        ownship_mode = "LF"

    return ownship_mode, ownship_status


def LF(traf, asas, qdr, dist, tcpa, idx_leader, idx_follower):
    """ Leader-Following (LF) resolution method """

    # Preliminary calculations-------------------------------------------------
    # Convert qdr from degrees to radians
    qdr = np.radians(qdr)

    # Relative position vector between id1 and id2
    drel = np.array([np.sin(qdr) * dist,
                     np.cos(qdr) * dist,
                     traf.alt[idx_leader] - traf.alt[idx_follower]])

    # Write velocities as vectors and find relative velocity vector
    v1 = np.array([traf.gseast[idx_follower],
                   traf.gsnorth[idx_follower],
                   traf.vs[idx_follower]])
    v2 = np.array([traf.gseast[idx_leader],
                   traf.gsnorth[idx_leader],
                   traf.vs[idx_leader]])
    vrel = np.array(v2 - v1)

    # Horizontal resolution----------------------------------------------------
    # Find horizontal distance at the tcpa (min horizontal distance)
    dcpa = drel + vrel*tcpa
    mindist = np.sqrt(dcpa[0]**2 + dcpa[1]**2)

    t_manv = max(30, tcpa)

    # Calculate CI vector
    v_f1 = mindist / t_manv
    crs_1 = np.arctan(dcpa[1]/dcpa[0])
    v_f1_vec = v_f1 * np.array([np.sin(crs_1), np.cos(crs_1)])

    # Calculate IX vector
    v_f2 = 1.2 * asas.R / t_manv
    crs_2 = np.radians(traf.trk[idx_leader] - 180)
    v_f2_vec = v_f2 * np.array([np.sin(crs_2), np.cos(crs_2)])

    # Calculate combined vector
    dv_reso = v_f1_vec + v_f2_vec
    dv = np.array([dv_reso[0], dv_reso[1], 0.0])

    return dv
