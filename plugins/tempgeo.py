"""
Module that implements azimuth angle calculation for rhumb lines
between two points.

Based on adaptations of equations given by: https://planetcalc.com/713
The arctan(y/x) term given there is incorrect and has been replaced here
by arctan2(y,x) to return angles in the correct quadrant.

© Ruben Jacobse, 2019
"""

# Third-party imports
import numpy as np


def rhumb_azimuth(lat_start, lon_start, lat_end, lon_end):
    """
    Calculate azimuth angle from a given start point along the rhumb
    line connecting connecting it to an end point. Uses the WGS84
    reference ellipsoid for calculations. Accepts both scalar and numpy
    array input.

    In:
        lat_start, lon_start, lat_end, lon_end [deg]
    Out:
        azimuth [deg] = course from start to end along loxodrome
    """

    # Wrap around the functions doing the actual calculations
    if isinstance(lat_start, np.ndarray):
        return _rhumb_azimuth_vector(lat_start, lon_start, lat_end, lon_end)
    else:
        return _rhumb_azimuth_scalar(lat_start, lon_start, lat_end, lon_end)


def _rhumb_azimuth_scalar(lat_start, lon_start, lat_end, lon_end):
    """
    Perform the rhumb line azimuth calculation for scalar input
    """

    # NOTE: Equations from https://planetcalc.com/713/

    # Longitude difference value depends on whether the points are on the
    # same hemisphere
    if abs(lon_end - lon_start) <= 180:
        delta_lon = lon_end - lon_start
    elif lon_end - lon_start < -180:
        delta_lon = 360 + lon_end - lon_start
    else:
        delta_lon = lon_end - lon_start - 360

    # Further equations use radians
    lat_begin_rad = np.radians(lat_start)
    lat_end_rad = np.radians(lat_end)
    delta_lon_rad = np.radians(delta_lon)

    # Azimuth angle calculation
    # NOTE: If the start and end latitude are exactly equal, then the denominator in the
    # azimuth_rad calculation can become zero leading to division by zero. To prevent
    # this, a very small safety factor DIV_ZERO_SAFETY_FACTOR is added to this term.
    azimuth_rad = np.arctan2(delta_lon_rad, (natlog_factor(lat_end_rad) \
        - natlog_factor(lat_begin_rad)))

    azimuth_deg = np.degrees(azimuth_rad)

    return azimuth_deg


def _rhumb_azimuth_vector(lat_start, lon_start, lat_end, lon_end):
    """
    Perform the rhumb line azimuth calculation for vector input
    """

    # NOTE: Equations from https://planetcalc.com/713/

    delta_lon = np.zeros(np.shape(lat_start))

    # Longitude difference value depends on whether the points are on the
    # same hemisphere
    case_1 = np.where(abs(lon_end - lon_start) <= 180)
    delta_lon[case_1] = lon_end[case_1] - lon_start[case_1]

    case_2 = np.where(lon_end - lon_start < -180)
    delta_lon[case_2] = 360 + lon_end[case_2] - lon_start[case_2]

    case_3 = np.where(lon_end - lon_start > 180)
    delta_lon[case_3] = lon_end[case_3] - lon_start[case_3] - 360

    # Further equations use radians
    lat_begin_rad = np.radians(lat_start)
    lat_end_rad = np.radians(lat_end)
    delta_lon_rad = np.radians(delta_lon)

    # Azimuth angle calculation
    # NOTE: If the start and end latitude are exactly equal, then the denominator in the
    # azimuth_rad calculation can become zero leading to division by zero. To prevent
    # this, a very small safety factor DIV_ZERO_SAFETY_FACTOR is added to this term.
    azimuth_rad = np.arctan2(delta_lon_rad, (natlog_factor(lat_end_rad) \
        - natlog_factor(lat_begin_rad)))

    azimuth_deg = np.degrees(azimuth_rad)

    return azimuth_deg


def natlog_factor(lat):
    """
    Calculate the result of the following equation:

    ln(tan(pi/4 + lat/2) * ((1 - e * sin(lat)) / (1 + e * sin(lat))) ** (e / 2))

    """

    # NOTE: Equations from https://planetcalc.com/713/

    # Calculate eccentricity
    a = 6378137.0 # [m] WGS84 Earth semi-major axis
    b = 6356752.314245 # [m] WGS84 Earth semi-minor axis
    a_squared = a ** 2
    b_squared = b ** 2
    eccentricity = np.sqrt(1 - (b_squared / a_squared)) # [-] WGS84 Earth eccentricity factor

    tan_term = np.tan(np.pi / 4 + lat / 2)
    lat_term = ((1 - eccentricity * np.sin(lat)) / (1 + eccentricity * np.sin(lat))) \
        ** (eccentricity / 2)

    return np.log(tan_term * lat_term)
