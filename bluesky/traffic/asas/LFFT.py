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

# Store (follower, leader) as tuples for use over multiple time steps
FT_LIST = []


def start(asas):
    """ Ensure dict is empty after reset.  """

    FT_LIST = []


def resolve(asas, traf):
    """ Resolve all current conflicts. """

    # Store the resolution velocity change vector
    delta_v = np.zeros((traf.ntraf, 3))

    # Stores resolution vector, also used in visualization
    asas.asasn = np.zeros(traf.ntraf, dtype=np.float32)
    asas.asase = np.zeros(traf.ntraf, dtype=np.float32)

    print(f"t={bs.sim.simt}")
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
        if idx1 == -1 or idx2 == -1:
            continue
        elif asas.swresooff and ac1 in asas.resoofflst:
            continue
        elif asas.swnoreso and ac2 in asas.noresolst:
            continue
        # Also do not resolve if
        # - ac1 is in a follow-through maneuver with any aircraft
        #   (handled below in FT_LIST loop)
        for pair in FT_LIST:
            if pair[0] == ac1:
                continue

        # Find the ac mode and status based on the conflict geometry
        ac1_mode, ac1_status = find_lf_status(traf, tLOS, idx2, idx1)
        ac2_mode, ac2_status = find_lf_status(traf, tLOS, idx1, idx2)

        # Determine whether to use MVP, act as follower, or act as leader.
        if (ac2, ac1) in FT_LIST:
            # ac2 is already following ac1, thus ac1 is leader and
            # should not take evasive action
            reso_str = "no reso"
            # asas.confpairs.remove((ac1, ac2))
        else:
            # ac1 not in follow-through maneuver
            if ((ac1_status == ac2_status)
                  or ac1_mode == "MVP"
                  or tLOS < 240
                  or dist < asas.R):
                dv_mvp, _ = MVP(traf, asas, qdr, dist, tcpa, tLOS, idx1, idx2)
                delta_v[idx1] -= dv_mvp
                reso_str = "using MVP"
            elif ac1_status == "follower":
                reso_str = "using LF"
                if (ac1, ac2) not in FT_LIST:
                    FT_LIST.append((ac1, ac2))
            else:
                # ac1_status == "leader"
                reso_str = "none"
                # print(f"({ac1},{ac2}) condition mismatch")

        print(f"\t{traf.id[idx1]}-{traf.id[idx2]} {reso_str}" +
              f" (tlos: {tLOS:.0f} s, dist: {dist/1852:.1f} NM)")

    # Handle all aircraft in follow-through maneuver (these are not guaranteed
    # to be in conflict and thus need to be handled separately)
    # NOTE: Order of (follower, leader) = (ac1, ac2) instead of (ac2, ac1) above
    ft_list_delpairs = []  # aircraft of which follow-through is to be switched off
    for (ac1, ac2) in FT_LIST:
        idx1 = traf.id2idx(ac1)
        idx2 = traf.id2idx(ac2)

        # Skip further steps if either aircraft has been deleted
        if idx1 == -1 or idx2 == -1:
            ft_list_delpairs.append((ac1, ac2))
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
            ft_list_delpairs.append((ac1, ac2))
            print(
                f"\t{ac1} stopped following {ac2} (delta_crs:{delta_crs0:.2f} rad, delta_v0:{delta_v0:.2f} m/s)")
        elif delta_crs0 > 0.5 * np.pi:
            ft_list_delpairs.append((ac1, ac2))
            print(f"\t{ac1} stopped following {ac2} (delta_crs:{delta_crs0:.2f} rad)")
        elif dist <= asas.R:
            ft_list_delpairs.append((ac1, ac2))
            print(f"\t{ac1} stopped following {ac2} (dist:{dist/1852:.1f} NM)")
        else:
            # We exploit the fact that tcpa (and derived values) are
            # not used inside LF() iff ac1 is in follow-through mode
            # but are overridden using t_mindist instead.
            dv_lf = LF(traf, asas, qdr, dist, idx2, idx1)
            delta_v[idx1] -= dv_lf
            print(f"\t{ac1} following {ac2}")

    for pair in ft_list_delpairs:
        FT_LIST.remove(pair)

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


def LF(traf, asas, qdr, dist, idx_leader, idx_follower):
    """ Leader-Following (LF) resolution method """

    # Preliminary calculations-------------------------------------------------
    # Convert qdr from degrees to radians
    qdr_rad = np.radians(qdr)

    # Relative position vector between id1 and id2
    drel = np.array([np.sin(qdr_rad) * dist,
                     np.cos(qdr_rad) * dist,
                     traf.alt[idx_leader] - traf.alt[idx_follower]])

    # Write velocities as vectors and find relative velocity vector
    v1 = np.array([traf.gseast[idx_follower],
                   traf.gsnorth[idx_follower],
                   traf.vs[idx_follower]])
    v2 = np.array([traf.gseast[idx_leader],
                   traf.gsnorth[idx_leader],
                   traf.vs[idx_leader]])
    vrel = np.array(v2 - v1)
    vrel2 = vrel[0]**2 + vrel[1]**2
    tcpa = -(vrel[0] * drel[0] + vrel[1] * drel[1]) / vrel2

    # Use Kieskamp's method of calculating t_mindist using his
    # parameter settings for 'mu' and 'b'
    mu = 5 * asas.R
    b = 100000
    exp_term = (dist - mu) / b

    if dist < mu:
        t_mindist = 299 * (0.5 * np.exp(exp_term)) + 1
    else:
        t_mindist = 299 * (1 - 0.5 * np.exp(-exp_term)) + 1

    # If in follow-through maneuver use t_mindist
    id1 = bs.traf.id[idx_follower]
    id2 = bs.traf.id[idx_leader]
    if (id1, id2) in FT_LIST:
        t_manv = t_mindist
    else:
        t_manv = max(30, tcpa)

    # Calculate position of point on edge of leader's PZ at tcpa to which follower must aim
    leader_distflown = traf.tas[idx_leader] * tcpa
    aimpoint_dist = leader_distflown - 9260  # 5NM behind leader at tcpa
    (aim_lat, aim_lon) = geo.kwikpos(traf.lat[idx_leader], traf.lon[idx_leader], traf.trk[idx_leader], aimpoint_dist / 1852)
    qdr_aim, dist_aim = geo.kwikqdrdist(traf.lat[idx_follower], traf.lon[idx_follower], aim_lat, aim_lon)

    # Calculate follower velocity to aim at leader PZ point
    aim_gsnorth = dist_aim * 1852 / t_manv * np.cos(np.radians(qdr_aim))
    aim_gseast = dist_aim * 1852 / t_manv * np.sin(np.radians(qdr_aim))

    # Calculate required follower velocity change
    dv_east = traf.gseast[idx_follower] - aim_gseast
    dv_north = traf.gsnorth[idx_follower] - aim_gsnorth
    # dv_trk = np.degrees(np.arctan2(dv_east, dv_north))
    dv = np.array([dv_east, dv_north, 0.0])

    # asas.active[idx_follower] = True

    # # Calculate CI vector
    # v_f1 = mindist / t_manv
    # crs_1 = np.arctan(dcpa[1] / dcpa[0])
    # v_f1_vec = v_f1 * np.array([np.sin(crs_1), np.cos(crs_1)])
    #
    # # Calculate IX vector
    # v_f2 = 1.2 * asas.R / t_manv
    # crs_2 = np.radians(traf.trk[idx_leader] - 180)
    # v_f2_vec = v_f2 * np.array([np.sin(crs_2), np.cos(crs_2)])
    #
    # # Calculate combined vector
    # dv_reso = v_f1_vec + v_f2_vec
    # dv = np.array([dv_reso[0], dv_reso[1], 0.0])

    return dv
