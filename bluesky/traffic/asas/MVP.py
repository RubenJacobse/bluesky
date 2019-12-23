# -*- coding: utf-8 -*-
"""
Created on Tue Mar 03 16:50:19 2015

@author: Jerom Maas
"""

import numpy as np
from bluesky.tools.aero import ft, fpm, kts


def start(asas):
    pass


def resolve(asas, traf):
    """ Resolve all current conflicts """

    # Initialize an array to store the resolution velocity vector for all A/C
    delta_v = np.zeros((traf.ntraf, 3))

    # Stores resolution vector, also used in visualization
    asas.asasn = np.zeros(traf.ntraf, dtype=np.float32)
    asas.asase = np.zeros(traf.ntraf, dtype=np.float32)

    # Initialize an array to store time needed to resolve vertically
    t_solve_vertical = np.ones(traf.ntraf) * 1e9

    # Call MVP function to resolve conflicts-----------------------------------
    for ((ac1, ac2), qdr, dist, tcpa, tLOS) in zip(asas.confpairs,
                                                   asas.qdr,
                                                   asas.dist,
                                                   asas.tcpa,
                                                   asas.tLOS):
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

        dv_mvp, tsolV = MVP(traf, asas, qdr, dist, tcpa, tLOS, idx1, idx2)
        if tsolV < t_solve_vertical[idx1]:
            t_solve_vertical[idx1] = tsolV

        # Use priority rules if activated
        if asas.swprio:
            delta_v[idx1], _ = prioRules(traf, asas.priocode, dv_mvp,
                                         delta_v[idx1], delta_v[idx2],
                                         idx1, idx2)
        else:
            # Since cooperative, the vertical resolution component can be
            # halved, and then dv_mvp can be added
            dv_mvp[2] = 0.5 * dv_mvp[2]
            delta_v[idx1] = delta_v[idx1] - dv_mvp

    # Determine new speed and limit resolution direction for all aicraft-------

    # Resolution vector for all aircraft, cartesian coordinates
    delta_v = np.transpose(delta_v)

    # The old speed vector, cartesian coordinates
    v = np.array([traf.gseast, traf.gsnorth, traf.vs])

    # The new speed vector, cartesian coordinates
    new_v = v + delta_v

    # Get indices of aircraft that have a resolution
    idxs = delta_v[0, :]**2 + delta_v[1, :]**2 > 0

    # Limit resolution direction if required-----------------------------------

    # Compute new speed vector in polar coordinates based on desired resolution
    if asas.swresohoriz:  # horizontal resolutions
        if asas.swresospd and not asas.swresohdg:  # SPD only
            new_track = traf.trk
            new_gs = np.sqrt(new_v[0, :]**2 + new_v[1, :]**2)
            new_vs = traf.vs
        elif asas.swresohdg and not asas.swresospd:  # HDG only
            new_track = np.degrees(np.arctan2(new_v[0, :], new_v[1, :])) % 360
            new_gs = traf.gs
            new_vs = traf.vs
        else:  # SPD + HDG
            new_track = np.degrees(np.arctan2(new_v[0, :], new_v[1, :])) % 360
            new_gs = np.sqrt(new_v[0, :]**2 + new_v[1, :]**2)
            new_vs = traf.vs
    elif asas.swresovert:  # vertical resolutions
        new_track = traf.trk
        new_gs = traf.gs
        new_vs = new_v[2, :]
    else:  # horizontal + vertical
        new_track = np.degrees(np.arctan2(new_v[0, :], new_v[1, :])) % 360
        new_gs = np.sqrt(new_v[0, :]**2 + new_v[1, :]**2)
        new_vs = new_v[2, :]

    # Determine ASAS module commands for all aircraft--------------------------

    # Cap the horizontal and vrtical speed
    new_gs_capped = np.clip(new_gs, asas.vmin, asas.vmax)
    new_vs_capped = np.clip(new_vs, asas.vsmin, asas.vsmax)

    # Now assign resolutions to variables in the ASAS class
    asas.trk = new_track
    asas.tas = new_gs_capped
    asas.vs = new_vs_capped

    # Stores resolution vector
    asas.asase[idxs] = asas.tas[idxs] * np.sin(asas.trk[idxs] / 180 * np.pi)
    asas.asasn[idxs] = asas.tas[idxs] * np.cos(asas.trk[idxs] / 180 * np.pi)
    # asaseval should be set to True now
    if not asas.asaseval:
        asas.asaseval = True

    # Calculate if Autopilot selected altitude should be followed. This avoids ASAS from
    # climbing or descending longer than it needs to if the autopilot leveloff
    # altitude also resolves the conflict. Because ASAS.alt is calculated using
    # the time to resolve, it may result in climbing or descending more than the selected
    # altitude.
    signdvs = np.sign(asas.vs - traf.ap.vs * np.sign(traf.selalt - traf.alt))
    signalt = np.sign(asas.alt - traf.selalt)
    asas.alt = np.where(np.logical_or(signdvs == 0, signdvs == signalt), asas.alt, traf.selalt)

    # To compute asas alt, timesolveV is used. timesolveV is a really big value (1e9)
    # when there is no conflict. Therefore asas alt is only updated when its
    # value is less than the look-ahead time, because for those aircraft are in conflict
    altCondition = np.logical_and(t_solve_vertical < asas.dtlookahead, np.abs(delta_v[2, :]) > 0.0)
    asasalttemp = asas.vs * t_solve_vertical + traf.alt
    asas.alt[altCondition] = asasalttemp[altCondition]

    # If resolutions are limited in the horizontal direction, then asasalt should
    # be equal to auto pilot alt (aalt). This is to prevent a new asasalt being computed
    # using the auto pilot vertical speed (traf.avs) using the code in line 106 (asasalttemp)
    # when only horizontal resolutions are allowed.
    asas.alt = asas.alt*(1-asas.swresohoriz) + traf.selalt*asas.swresohoriz


def MVP(traf, asas, qdr, dist, tcpa, tLOS, id1, id2):
    """Modified Voltage Potential (MVP) resolution method"""

    # Convert qdr from degrees to radians
    qdr = np.radians(qdr)

    # Relative position vector between id1 and id2
    drel = np.array([np.sin(qdr) * dist,
                     np.cos(qdr) * dist,
                     traf.alt[id2] - traf.alt[id1]])

    # Write velocities as vectors and find relative velocity vector
    v1 = np.array([traf.gseast[id1], traf.gsnorth[id1], traf.vs[id1]])
    v2 = np.array([traf.gseast[id2], traf.gsnorth[id2], traf.vs[id2]])
    vrel = np.array(v2-v1)

    # Horizontal resolution----------------------------------------------------

    # Find horizontal distance at the tcpa (min horizontal distance)
    dcpa = drel + vrel * tcpa
    dabsH = np.sqrt(dcpa[0]**2 + dcpa[1]**2)

    # Compute horizontal intrusion
    iH = asas.Rm - dabsH

    # Exception handlers for head-on conflicts
    # This is done to prevent division by zero in the next step
    if dabsH <= 10.:
        dabsH = 10.
        dcpa[0] = drel[1] / dist * dabsH
        dcpa[1] = -drel[0] / dist * dabsH

        # Compute the resolution velocity vector in horizontal direction
        # abs(tcpa) because it becomes negative during intrusion
    if asas.Rm < dist and dabsH < dist:
        # If intruder is outside the ownship PZ, then apply extra factor
        # to make sure that resolution does not graze PZ
        erratum = np.cos(np.arcsin(asas.Rm/dist) - np.arcsin(dabsH/dist))
        dv1 = ((asas.Rm / erratum - dabsH) * dcpa[0]) / (abs(tcpa) * dabsH)
        dv2 = ((asas.Rm / erratum - dabsH) * dcpa[1]) / (abs(tcpa) * dabsH)
    else:
        dv1 = (iH * dcpa[0]) / (abs(tcpa) * dabsH)
        dv2 = (iH * dcpa[1]) / (abs(tcpa) * dabsH)

    # Vertical resolution------------------------------------------------------

    # Compute the  vertical intrusion
    # Amount of vertical intrusion dependent on vertical relative velocity
    iV = asas.dhm if abs(vrel[2]) > 0.0 else asas.dhm-abs(drel[2])

    # Get the time to solve the conflict vertically - tsolveV
    tsolV = abs(drel[2]/vrel[2]) if abs(vrel[2]) > 0.0 else tLOS

    # If the time to solve the conflict vertically is longer than the look-ahead time,
    # because the the relative vertical speed is very small, then solve the intrusion
    # within tinconf
    if tsolV > asas.dtlookahead:
        tsolV = tLOS
        iV = asas.dhm

    # Compute the resolution velocity vector in the vertical direction
    # The direction of the vertical resolution is such that the aircraft with
    # higher climb/decent rate reduces their climb/decent rate
    dv3 = np.where(abs(vrel[2]) > 0.0, (iV/tsolV)*(-vrel[2]/abs(vrel[2])), (iV/tsolV))

    # It is necessary to cap dv3 to prevent that a vertical conflict
    # is solved in 1 timestep, leading to a vertical separation that is too
    # high (high vs assumed in traf). If vertical dynamics are included to
    # aircraft  model in traffic.py, the below three lines should be deleted.
#    mindv3 = -400*fpm# ~ 2.016 [m/s]
#    maxdv3 = 400*fpm
#    dv3 = np.maximum(mindv3,np.minimum(maxdv3,dv3))

    # Combine resolutions------------------------------------------------------

    # combine the dv components
    dv = np.array([dv1, dv2, dv3])

    return dv, tsolV

#============================= Priority Rules =================================

def prioRules(traf, priocode, dv_mvp, dv1, dv2, id1, id2):
    ''' Apply the desired priority setting to the resolution '''

    # Primary Free Flight prio rules (no priority)
    if priocode == "FF1":
        # since cooperative, the vertical resolution component can be halved, and then dv_mvp can be added
        dv_mvp[2] = dv_mvp[2]/2.0
        dv1 = dv1 - dv_mvp
        dv2 = dv2 + dv_mvp

    # Secondary Free Flight (Cruising aircraft has priority, combined resolutions)
    if priocode == "FF2":
        # since cooperative, the vertical resolution component can be halved, and then dv_mvp can be added
        dv_mvp[2] = dv_mvp[2]/2.0
        # If aircraft 1 is cruising, and aircraft 2 is climbing/descending -> aircraft 2 solves conflict
        if abs(traf.vs[id1]) < 0.1 and abs(traf.vs[id2]) > 0.1:
            dv2 = dv2 + dv_mvp
        # If aircraft 2 is cruising, and aircraft 1 is climbing -> aircraft 1 solves conflict
        elif abs(traf.vs[id2]) < 0.1 and abs(traf.vs[id1]) > 0.1:
            dv1 = dv1 - dv_mvp
        else:  # both are climbing/descending/cruising -> both aircraft solves the conflict
            dv1 = dv1 - dv_mvp
            dv2 = dv2 + dv_mvp

    # Tertiary Free Flight (Climbing/descending aircraft have priority and crusing solves with horizontal resolutions)
    elif priocode == "FF3":
        # If aircraft 1 is cruising, and aircraft 2 is climbing/descending -> aircraft 1 solves conflict horizontally
        if abs(traf.vs[id1]) < 0.1 and abs(traf.vs[id2]) > 0.1:
            dv_mvp[2] = 0.0
            dv1 = dv1 - dv_mvp
        # If aircraft 2 is cruising, and aircraft 1 is climbing -> aircraft 2 solves conflict horizontally
        elif abs(traf.vs[id2]) < 0.1 and abs(traf.vs[id1]) > 0.1:
            dv_mvp[2] = 0.0
            dv2 = dv2 + dv_mvp
        else:  # both are climbing/descending/cruising -> both aircraft solves the conflict, combined
            dv_mvp[2] = dv_mvp[2]/2.0
            dv1 = dv1 - dv_mvp
            dv2 = dv2 + dv_mvp

    # Primary Layers (Cruising aircraft has priority and clmibing/descending solves. All conflicts solved horizontally)
    elif priocode == "LAY1":
        dv_mvp[2] = 0.0
        # If aircraft 1 is cruising, and aircraft 2 is climbing/descending -> aircraft 2 solves conflict horizontally
        if abs(traf.vs[id1]) < 0.1 and abs(traf.vs[id2]) > 0.1:
            dv2 = dv2 + dv_mvp
        # If aircraft 2 is cruising, and aircraft 1 is climbing -> aircraft 1 solves conflict horizontally
        elif abs(traf.vs[id2]) < 0.1 and abs(traf.vs[id1]) > 0.1:
            dv1 = dv1 - dv_mvp
        else:  # both are climbing/descending/cruising -> both aircraft solves the conflict horizontally
            dv1 = dv1 - dv_mvp
            dv2 = dv2 + dv_mvp

    # Secondary Layers (Climbing/descending aircraft has priority and cruising solves. All conflicts solved horizontally)
    elif priocode == "LAY2":
        dv_mvp[2] = 0.0
        # If aircraft 1 is cruising, and aircraft 2 is climbing/descending -> aircraft 1 solves conflict horizontally
        if abs(traf.vs[id1]) < 0.1 and abs(traf.vs[id2]) > 0.1:
            dv1 = dv1 - dv_mvp
        # If aircraft 2 is cruising, and aircraft 1 is climbing -> aircraft 2 solves conflict horizontally
        elif abs(traf.vs[id2]) < 0.1 and abs(traf.vs[id1]) > 0.1:
            dv2 = dv2 + dv_mvp
        else:  # both are climbing/descending/cruising -> both aircraft solve the conflict horizontally
            dv1 = dv1 - dv_mvp
            dv2 = dv2 + dv_mvp

    return dv1, dv2
