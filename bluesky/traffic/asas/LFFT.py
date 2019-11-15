"""
LFFT
==

Implements Leader-Following behaviour method with follow-through
maneuvering, based on the method described in the MSc thesis of
Arjen Kieskamp at Delft University of Technology (March 5th, 2009).

Currently only supports 2-dimensional resolutions.

© Ruben Jacobse, 2019
"""

# Third-party imports
import numpy as np

# BlueSky imports
import bluesky as bs
from bluesky.tools import geo

# Local imports
from .MVP import MVP
from .LF import find_lf_status

# Store follower, leader as key, value pairs for use over multiple time steps
ft_dict = {}


def start(asas):
    """ Ensure dict is empty after reset.  """
    ft_dict = {}


def resolve(asas, traf):
    """ Resolve all current conflicts. """

    # Check if ASAS is ON first!
    if not asas.swasas:
        return

    # Store the resolution velocity change vector
    delta_v = np.zeros((traf.ntraf, 3))

    # Stores resolution vector, also used in visualization
    asas.asasn = np.zeros(traf.ntraf, dtype=np.float32)
    asas.asase = np.zeros(traf.ntraf, dtype=np.float32)

    # print(f"t={bs.sim.simt}")
    # For each conflict pair calculate resolution for ac1 if necessary
    for ((ac1, ac2), qdr, dist, tcpa, tLOS) in zip(asas.confpairs,
                                                   asas.qdr,
                                                   asas.dist,
                                                   asas.tcpa,
                                                   asas.tLOS):
        idx1 = traf.id2idx(ac1)
        idx2 = traf.id2idx(ac2)

        # Do not resolve if:
        # - the pair contains an aircraft that no longer exists
        # - resolutions are switched off for ac1
        # - ac2 is a noreso aircraft which nobody avoids
        # - ac1 is in a follow-through maneuver (handled below in ft_dict loop)
        if idx1 == -1 or idx2 == -1:
            continue
        elif asas.swresooff and ac1 in asas.resoofflst:
            continue
        elif asas.swnoreso and ac2 in asas.noresolst:
            continue
        elif ac1 in ft_dict:
            continue

        # Find the mode and status of ac1 based on the geometry conflict
        ac1_mode, ac1_status = find_lf_status(traf, tLOS, idx2, idx1)
        ac2_mode, ac2_status = find_lf_status(traf, tLOS, idx1, idx2)

        # Determine whether to use MVP, act as follower, or act as leader.
        # If (ac2, ac1) is in ft_dict then ac2 is following ac1 and ac1 should
        # do nothing to avoid ac2.
        if ((ac2, ac1) not in ft_dict) and ((ac1_status == ac2_status) or ac1_mode == "MVP" or tLOS < 240 or dist < asas.R):
            dv_mvp, _ = MVP(traf, asas, qdr, dist, tcpa, tLOS, idx1, idx2)
            delta_v[idx1] -= dv_mvp
            reso_str = "using MVP"
        elif ((ac2, ac1) not in ft_dict) and ac1_status == "follower":
            dv_lf = LF(traf, asas, qdr, dist, tcpa, idx2, idx1)
            delta_v[idx1] -= dv_lf
            reso_str = "using LF"
        else:  # ac1_status, ac1_mode now guaranteed to be "leader" and "LF"
            reso_str = "no reso"
            pass

        # print(f"\t{traf.id[idx1]}-{traf.id[idx2]} {reso_str}" +
        #       f" (tlos: {tLOS:.0f} s, dist: {dist/1852:.1f} NM)")

    # Handle all aircraft in follow-through maneuver (these are not guaranteed
    # to be in conflict and thus need to be handled separately)
    ft_dict_delkeys = []  # aircraft of which follow-through is to be switched off
    for ac1, ac2 in ft_dict.items():
        idx1 = traf.id2idx(ac1)
        idx2 = traf.id2idx(ac2)

        # Check if either aircraft has been deleted
        if idx1 == -1 or idx2 == -1:
            ft_dict_delkeys.append(ac1)
            continue

        # Parameters that determine whether to continue follow-through
        delta_crs0_deg = ((traf.trk[idx1] - traf.trk[idx2] + 180) % 360 - 180)
        delta_crs0 = abs(delta_crs0_deg) / 180 * np.pi
        delta_v0 = abs(traf.gs[idx1] - traf.gs[idx2])
        qdr, dist = geo.kwikqdrdist(traf.lat[idx1], traf.lon[idx1],
                                    traf.lat[idx2], traf.lon[idx2])
        dist = dist * 1852  # convert to meters

        # Check if follow-through manveuver should be continued using LF or
        # if it should be switched off.
        if delta_crs0 < 0.01 * np.pi and delta_v0 < 1:
            ft_dict_delkeys.append(ac1)
            # print(
            #     f"\t{ac1} stopped following {ac2} (delta_crs:{delta_crs0:.2f} rad, delta_v0:{delta_v0:.2f} m/s)")
        elif delta_crs0 > 0.5 * np.pi:
            ft_dict_delkeys.append(ac1)
            # print(f"\t{ac1} stopped following {ac2} (delta_crs:{delta_crs0:.2f} rad)")
        elif dist <= asas.R:
            ft_dict_delkeys.append(ac1)
            # print(f"\t{ac1} stopped following {ac2} (dist:{dist:.1f} NM)")
        else:
            # We exploit the fact that tcpa (and derived values) are
            # not used inside LF() iff ac1 is in follow-through mode
            # but are overriden using t_mindist instead.
            tcpa = 1e9  # this value does not matter
            dv_lf = LF(traf, asas, qdr, dist, tcpa, idx2, idx1)
            delta_v[idx1] -= dv_lf
            # print(f"\t{ac1} following {ac2}")

    for key in ft_dict_delkeys:
        ft_dict.pop(key, None)

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

    # Use Kieskamp's method of calculating t_mindist using his
    # parameter settings for 'mu' and 'b'
    mu = 5 * asas.R
    b = 1000
    exp_term = (dist - mu) / b

    if dist < mu:
        t_mindist = 299*(0.5*np.exp(exp_term)) + 1
    else:
        t_mindist = 299*(1 - 0.5*np.exp(-exp_term)) + 1

    # If in follow-through maneuver use t_mindist
    if traf.id[idx_follower] in ft_dict.keys():
        t_manv = t_mindist
    else:
        t_manv = max(30, tcpa)

    # Calculate CI vector
    v_f1 = mindist / t_manv
    crs_1 = np.arctan(dcpa[1] / dcpa[0])
    v_f1_vec = v_f1 * np.array([np.sin(crs_1), np.cos(crs_1)])

    # Calculate IX vector
    v_f2 = 1.2 * asas.R / t_manv
    crs_2 = np.radians(traf.trk[idx_leader] - 180)
    v_f2_vec = v_f2 * np.array([np.sin(crs_2), np.cos(crs_2)])

    # Calculate combined vector
    dv_reso = v_f1_vec + v_f2_vec
    dv = np.array([dv_reso[0], dv_reso[1], 0.0])

    return dv
