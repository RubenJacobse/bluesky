"""
Alternative implementation of the Swarming algorithm
(The default version in Swarm.py is rather broken.)

© Ruben Jacobse, 2019
"""

# Third-party imports
import numpy as np

# Bluesky imports
import bluesky as bs
from bluesky.tools import geo, areafilter
from bluesky.tools.aero import nm, ft

# Local imports
from . import MVP_alt


def start(asas):
    asas.swarm_min_radius = 10 * nm
    asas.swarm_max_radius = 15 * nm
    asas.swarm_altitude = 1500 * ft  # [m]
    asas.swarm_weights = np.array([10, 3])


def resolve(asas, traf):
    """ Resolve conflicts using the swarming principle. """

    # Check if ASAS is ON:
    if not asas.swasas:
        raise RuntimeError("This should not be happening...")

    # Use flat-earth approximation for qdr and distance calculation
    _, dist = geo.kwikqdrdist_matrix(np.mat(traf.lat), np.mat(traf.lon),
                                     np.mat(traf.lat), np.mat(traf.lon))
    dist = np.array(dist) * nm + np.eye(traf.ntraf) * 1e9
    dist_sqrd = dist * dist
    delta_h = traf.alt.reshape((1, traf.ntraf)) \
        - traf.alt.reshape((1, traf.ntraf)).T

    # Calculate track differences
    delta_trk = traf.trk.reshape(1, traf.ntraf) - traf.trk.reshape(traf.ntraf, 1)
    delta_trk = (delta_trk + 180) % 360 - 180

    # Apply swarming criteria (distance and direction) to find all aircraft
    # combinations for which swarming rules have to be applied
    ac_in_swarm_range = np.logical_and(dist_sqrd > asas.swarm_min_radius**2,
                                       dist_sqrd < asas.swarm_max_radius**2,
                                       np.abs(delta_h) < asas.swarm_altitude)
    ac_in_swarm_direction = np.abs(delta_trk) < 90
    ac_in_swarm = np.logical_and(ac_in_swarm_range, ac_in_swarm_direction)
    ac_own = np.eye(traf.ntraf, dtype='bool')
    Swarming = np.logical_or(ac_in_swarm, ac_own)

    # For each aircraft determine whether they use swarming rules or mvp rules
    ac_in_swarm_zone = areafilter.checkInside("SWARMING_ZONE",
                                              bs.traf.lat,
                                              bs.traf.lon,
                                              bs.traf.alt)
    ac_use_swarm = np.logical_and(np.any(ac_in_swarm, axis=0), ac_in_swarm_zone)
    ac_use_mvp = np.logical_and(np.any(np.logical_not(ac_in_swarm_range),
                                       axis=0),
                                asas.inconf)

    # Prepare matrices to use for calculating averages
    gseast_matrix = np.ones((traf.ntraf, traf.ntraf)) * traf.gseast
    gsnorth_matrix = np.ones((traf.ntraf, traf.ntraf)) * traf.gsnorth
    tas_matrix = np.ones((traf.ntraf, traf.ntraf)) * traf.tas
    vs_matrix = np.ones((traf.ntraf, traf.ntraf)) * traf.vs
    alt_matrix = np.ones((traf.ntraf, traf.ntraf)) * traf.alt

    # Calculate Collision Avoidance (CA) parameters
    ca_trk, ca_tas, ca_vs, _, _, _ = MVP_alt.resolve(asas, traf)

    # Calculate Velocity Alignment (VA) parameters
    va_gsnorth = np.average(gsnorth_matrix, axis=1, weights=Swarming)
    va_gseast = np.average(gseast_matrix, axis=1, weights=Swarming)
    va_trk = np.degrees(np.arctan2(va_gseast, va_gsnorth)) % 360.
    va_tas = np.average(tas_matrix, axis=1, weights=Swarming)
    va_vs = np.average(vs_matrix, axis=1, weights=Swarming)

    # # Calculate Flock Centering (FC) parameters
    # dx_to_flock_center = np.average(delta_x, axis=1, weights=Swarming)
    # dy_to_flock_center = np.average(delta_y, axis=1, weights=Swarming)
    # dz_to_flock_center = np.average(alt_matrix, axis=1, weights=Swarming) - traf.alt

    # fc_trk = np.degrees(np.arctan2(dx_to_flock_center, dy_to_flock_center))
    # fc_tas = traf.tas
    # ttoreach = np.sqrt(dx_to_flock_center**2 + dy_to_flock_center**2) / fc_tas
    # fc_vs = np.where(ttoreach == 0, 0, dz_to_flock_center / ttoreach)

    # Combine parameter vectors into multidimensional arrays
    tas_arr = np.array([ca_tas, va_tas])
    trk_arr = np.array([ca_trk, va_trk])
    vs_arr = np.array([ca_vs, va_vs])
    vx_arr = tas_arr * np.sin(np.radians(trk_arr))
    vy_arr = tas_arr * np.cos(np.radians(trk_arr))

    # Find final Swarming directions by taking weighted average of CA and VA
    swarm_gseast = np.average(vx_arr, axis=0, weights=asas.swarm_weights)
    swarm_gsnorth = np.average(vy_arr, axis=0, weights=asas.swarm_weights)
    swarm_hdg = np.degrees(np.arctan2(swarm_gseast, swarm_gsnorth)) % 360
    swarm_vs = np.average(vs_arr, axis=0, weights=asas.swarm_weights)

    # Cap the velocity
    swarm_tas = np.average(tas_arr, axis=0, weights=asas.swarm_weights)
    tas_arr = np.clip(swarm_tas, asas.vmin, asas.vmax)

    # Assign either swarming or MVP resolution
    new_hdg = np.where(ac_use_mvp, ca_trk, swarm_hdg) % 360
    new_tas = np.where(ac_use_mvp, ca_tas, swarm_tas)
    new_vs = np.where(ac_use_mvp, ca_vs, swarm_vs)
    new_alt = traf.alt

    # Set asas attributes
    asas.trk = new_hdg
    asas.tas = new_tas
    asas.vs = new_vs
    asas.alt = new_alt
    asas.swarm_active = ac_use_swarm
