"""
Alternative implementation of the Swarming algorithm
(The default version in Swarm.py is rather broken.)

© Ruben Jacobse, 2019
"""

import numpy as np
import bluesky as bs
from bluesky.tools import geo
from bluesky.tools.aero import nm, ft

from . import MVP_alt


def start(asas):
    asas.swarm_radius = 15 * nm  # [m]
    asas.swarm_altitude = 1500 * ft  # [m]
    asas.swarm_weights = np.array([10, 3, 1])


def resolve(asas, traf):
    """ Resolve conflicts using the swarming principle. """

    # Check if ASAS is ON:
    if not asas.swasas:
        return

    # Use flat-earth approximation for qdr and distance calculation
    qdr, dist = geo.kwikqdrdist_matrix(np.mat(traf.lat), np.mat(traf.lon),
                                       np.mat(traf.lat), np.mat(traf.lon))
    qdrrad = np.radians(np.array(qdr))
    dist = np.array(dist) * nm

    # Calculate approximate relative distances
    delta_x = dist * np.sin(qdrrad)  # + np.eye(traf.ntraf) * 1e9
    delta_y = dist * np.cos(qdrrad)  # + np.eye(traf.ntraf) * 1e9
    delta_h = traf.alt.reshape((1, traf.ntraf)) \
        - traf.alt.reshape((1, traf.ntraf)).T

    # Calculate track differences
    delta_trk = traf.trk.reshape(1, traf.ntraf) - traf.trk.reshape(traf.ntraf, 1)
    delta_trk = (delta_trk + 180) % 360 - 180

    # Apply swarming criteria (distance and direction) to find all aircraft
    # combinations for which swarming rules have to be applied
    ac_are_close = np.logical_and(delta_x**2 + delta_y**2 < asas.swarm_radius**2,
                                  np.abs(delta_h) < asas.swarm_altitude)
    ac_in_samedirection = np.abs(delta_trk) < 90
    ac_selected = np.logical_and(ac_are_close, ac_in_samedirection)
    ac_own = np.eye(traf.ntraf, dtype='bool')
    Swarming = np.logical_or(ac_selected, ac_own)

    #
    tas_matrix = np.ones((traf.ntraf, traf.ntraf)) * traf.tas
    trk_matrix = np.ones((traf.ntraf, traf.ntraf)) * traf.trk
    vs_matrix = np.ones((traf.ntraf, traf.ntraf)) * traf.vs
    alt_matrix = np.ones((traf.ntraf, traf.ntraf)) * traf.alt

    # Calculate Collision Avoidance (CA) parameters
    ca_trk, ca_tas, ca_vs, _, _, _ = MVP_alt.resolve(asas, traf)

    # Calculate Velocity Alignment (VA) parameters
    va_tas = np.average(tas_matrix, axis=1, weights=Swarming)
    va_trk = np.average(trk_matrix, axis=1, weights=Swarming)
    va_vs = np.average(vs_matrix, axis=1, weights=Swarming)

    # Calculate Flock Centering (FC) parameters
    dx_to_flock_center = np.average(delta_x, axis=1, weights=Swarming)
    dy_to_flock_center = np.average(delta_y, axis=1, weights=Swarming)
    dz_to_flock_center = np.average(alt_matrix, axis=1, weights=Swarming) - traf.alt

    fc_trk = np.degrees(np.arctan2(dx_to_flock_center, dy_to_flock_center))
    fc_tas = traf.tas
    ttoreach = np.sqrt(dx_to_flock_center**2 + dy_to_flock_center**2) / fc_tas
    fc_vs = np.where(ttoreach == 0, 0, dz_to_flock_center / ttoreach)

    # Combine parameter vectors into multidimensional arrays
    trk_arr = np.array([ca_trk, va_trk, fc_trk])
    tas_arr = np.array([ca_tas, va_tas, fc_tas])
    vs_arr = np.array([ca_vs, va_vs, fc_vs])
    vx_arr = tas_arr * np.sin(np.radians(trk_arr))
    vy_arr = tas_arr * np.cos(np.radians(trk_arr))

    # Find final Swarming directions by taking weighted averageov CA, VA and FC
    swarm_gseast = np.average(vx_arr, axis=0, weights=asas.swarm_weights)
    swarm_gsnorth = np.average(vy_arr, axis=0, weights=asas.swarm_weights)
    swarm_vs = np.average(vs_arr, axis=0, weights=asas.swarm_weights)
    swarm_hdg = np.degrees(np.arctan2(swarm_gseast, swarm_gsnorth))

    # Cap the velocity
    swarm_tas = np.average(tas_arr, axis=0, weights=asas.swarm_weights)
    tas_arr = np.clip(swarm_tas, asas.vmin, asas.vmax)

    # Assign Final Swarming directions to traffic
    asas.hdg = swarm_hdg
    asas.tas = swarm_tas
    asas.vs = swarm_vs
    asas.alt = traf.alt
