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

# BlueSky imports
from bluesky.tools.aero import Rearth

# Module constants
ITER_CALC_LAT_MARGIN = 1e-6


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
    Perform the rhumb line azimuth angle calculation for scalar input
    """

    # Longitude difference value depends on whether the points are on the
    # same hemisphere
    if abs(lon_end - lon_start) <= 180:
        delta_lon = lon_end - lon_start
    elif lon_end - lon_start < -180:
        delta_lon = 360 + lon_end - lon_start
    else:
        delta_lon = lon_end - lon_start - 360

    # Subsequent equations use radians
    lat_begin_rad = np.radians(lat_start)
    lat_end_rad = np.radians(lat_end)
    delta_lon_rad = np.radians(delta_lon)

    # Azimuth angle calculation
    azimuth_rad = np.arctan2(delta_lon_rad, (natlog_factor(lat_end_rad)
                                             - natlog_factor(lat_begin_rad)))

    azimuth_deg = np.degrees(azimuth_rad)

    return azimuth_deg


def _rhumb_azimuth_vector(lat_start, lon_start, lat_end, lon_end):
    """
    Perform the rhumb line azimuth angle calculation for vector input
    """

    delta_lon = np.zeros(np.shape(lat_start))

    # Longitude difference value depends on whether the points are on the
    # same hemisphere
    case_1 = np.where(abs(lon_end - lon_start) <= 180)
    delta_lon[case_1] = lon_end[case_1] - lon_start[case_1]

    case_2 = np.where(lon_end - lon_start < -180)
    delta_lon[case_2] = 360 + lon_end[case_2] - lon_start[case_2]

    case_3 = np.where(lon_end - lon_start > 180)
    delta_lon[case_3] = lon_end[case_3] - lon_start[case_3] - 360

    # Subsequent equations use radians
    lat_begin_rad = np.radians(lat_start)
    lat_end_rad = np.radians(lat_end)
    delta_lon_rad = np.radians(delta_lon)

    # Azimuth angle calculation
    azimuth_rad = np.arctan2(delta_lon_rad, (natlog_factor(lat_end_rad)
                                             - natlog_factor(lat_begin_rad)))

    azimuth_deg = np.degrees(azimuth_rad)

    return azimuth_deg


def natlog_factor(lat):
    """
    Calculate the result of the following equation:

    ln(tan(pi/4 + lat/2) * ((1 - e * sin(lat)) / (1 + e * sin(lat))) ** (e / 2))

    """

    # Calculate eccentricity
    a = 6378137.0  # [m] WGS84 Earth semi-major axis
    b = 6356752.314245  # [m] WGS84 Earth semi-minor axis
    a_squared = a ** 2
    b_squared = b ** 2
    # [-] WGS84 Earth eccentricity factor
    eccentricity = np.sqrt(1 - (b_squared / a_squared))

    tan_term = np.tan(np.pi / 4 + lat / 2)
    lat_term = ((1 - eccentricity * np.sin(lat)) / (1 + eccentricity * np.sin(lat))) \
        ** (eccentricity / 2)

    return np.log(tan_term * lat_term)


def sphere_greatcircle_intersect(p1_lat, p1_lon, p2_lat, p2_lon, p3_lat, p3_lon, p4_lat, p4_lon):
    """
    Calculate the intersection point of two great circle arcs, the
    first defined by points: (p1, p2) and the second defined by the
    points: (p3, p4).

    Returns (lat, lon) of the intersection point if it exists, and
    raises a ValueError if the arcs do not intersect.
    """

    def latlon_to_unitvector(lat, lon):
        """
        Convert a point defined by a pair of (lat, lon) coordinates on
        the surface of a sphere to a three-dimensional unit vector in
        cartesian coordinates (x, y, z).

        Returns a one-dimensional numpy array containing [x, y, z].
        """

        x = np.cos(np.radians(lon)) * np.cos(np.radians(lat))
        y = np.sin(np.radians(lon)) * np.cos(np.radians(lat))
        z = np.sin(np.radians(lat))

        return np.array([x, y, z])

    def unitvector_to_latlon(cart_x, cart_y, cart_z):
        """
        Convert a point on the unit sphere defined by a unit vector with
        cartesian coordinates (x, y, z) to a pair of (lat, lon) coordinates
        on the sphere surface.
        """

        sphere_lon = np.arctan2(cart_y, cart_x)
        sphere_lat = np.arctan2(cart_z, np.sqrt(cart_x ** 2 + cart_y ** 2))

        return sphere_lat, sphere_lon

    # Convert each point to a unit vector
    vec_a0 = latlon_to_unitvector(p1_lat, p1_lon)
    vec_a1 = latlon_to_unitvector(p2_lat, p2_lon)
    vec_b0 = latlon_to_unitvector(p3_lat, p3_lon)
    vec_b1 = latlon_to_unitvector(p4_lat, p4_lon)

    # Calculate normal vectors to the planes defining arcs a and b
    vec_p = np.cross(vec_a0, vec_a1)
    vec_q = np.cross(vec_b0, vec_b1)

    # Calculate normalized vector along the line of plane intersection
    vec_t = np.cross(vec_p, vec_q)
    unit_t = vec_t

    # Projections of t along arcs a and b
    s_1 = np.dot(np.cross(vec_a0, vec_p), unit_t)
    s_2 = np.dot(np.cross(vec_a1, vec_p), unit_t)
    s_3 = np.dot(np.cross(vec_b0, vec_q), unit_t)
    s_4 = np.dot(np.cross(vec_b1, vec_q), unit_t)

    # Test if arcs intersect and if so, determine at which location
    if -s_1 < 0 and s_2 < 0 and -s_3 < 0 and s_4 < 0:
        # Intersection occurs at -t
        intersect_t = -1 * unit_t
    elif -s_1 >= 0 and s_2 >= 0 and -s_3 >= 0 and s_4 >= 0:
        # Intersection occurs at t
        intersect_t = unit_t
    else:
        # No intersection:
        raise ValueError("Great circle arcs do not intersect at any point.")

    # Convert intersect_t back to lat, lon
    intersect_x = intersect_t[0]
    intersect_y = intersect_t[1]
    intersect_z = intersect_t[2]

    intersect_lat, intersect_lon = unitvector_to_latlon(
        intersect_x, intersect_y, intersect_z)

    return np.degrees(intersect_lat), np.degrees(intersect_lon)


def ellipsoid_greatcircle_intersect(p1_lat, p1_lon, p2_lat, p2_lon, p3_lat, p3_lon, p4_lat, p4_lon):
    """
    Calculate the intersection point of two great circle arcs, the
    first defined by points: (p1, p2) and the second defined by the
    points: (p3, p4).

    Returns (lat, lon) of the intersection point if it exists, and
    raises a ValueError if the arcs do not intersect.

    Equations adapted from:
    https://gssc.esa.int/navipedia/index.php/Ellipsoidal_and_Cartesian_Coordinates_Conversion
    """

    def latlon_to_vector(lat, lon):
        """
        Convert a point defined by a pair of (lat, lon) coordinates on
        the surface of an ellipsoid to a three-dimensional unit vector in
        cartesian coordinates (x, y, z).

        Returns a one-dimensional numpy array containing [x, y, z].
        """

        e = 0.08181919084262149  # [-] WGS84 Earth eccentricity factor

        # Radius of curvature in prime vertical
        N = 1 / np.sqrt(1 - (e ** 2 * np.sin(np.radians(lat)) ** 2))

        # Calculate cartesian coordinates and normalize to unit vector
        cart_x = N * np.cos(np.radians(lat)) * np.cos(np.radians(lon))
        cart_y = N * np.cos(np.radians(lat)) * np.sin(np.radians(lon))
        cart_z = (1 - e ** 2) * N * np.sin(np.radians(lat))

        cart_vec = np.array([cart_x, cart_y, cart_z])
        cart_unit_vec = cart_vec / np.linalg.norm(cart_vec)

        return cart_unit_vec
        # return np.array([cart_x, cart_y, cart_z])

    def vector_to_latlon(cart_x, cart_y, cart_z):
        """
        Convert a point on the unit sphere defined by a unit vector with
        cartesian coordinates (x, y, z) to a pair of (lat, lon) coordinates
        on the ellipsoid surface.
        """

        e = 0.08181919084262149  # [-] WGS84 Earth eccentricity factor

        p = np.sqrt(cart_x ** 2 + cart_y ** 2)

        # Finding longitude is simple
        ellipse_lon = np.arctan2(cart_y, cart_x)

        # For latitude: iterate until successive latitude values converge within margin
        lat_init = np.arctan2(cart_z, (1 - e ** 2) * p)

        lat_prev = lat_init
        while True:
            N = 1 / np.sqrt(1 - (e ** 2) * (np.sin(lat_prev) ** 2))
            h = (p / np.cos(lat_prev)) - N

            lat_curr = np.arctan2(cart_z, (1 - (e ** 2) * (N / (N + h))) * p)

            if abs(lat_curr - lat_prev) < ITER_CALC_LAT_MARGIN:
                ellipse_lat = lat_curr
                break
            else:
                lat_prev = lat_curr

        return ellipse_lat, ellipse_lon

    # Convert each point to a unit vector
    vec_a0 = latlon_to_vector(p1_lat, p1_lon)
    vec_a1 = latlon_to_vector(p2_lat, p2_lon)
    vec_b0 = latlon_to_vector(p3_lat, p3_lon)
    vec_b1 = latlon_to_vector(p4_lat, p4_lon)

    # Calculate normal vectors to the planes defining arcs a and b
    vec_p = np.cross(vec_a0, vec_a1)
    vec_q = np.cross(vec_b0, vec_b1)

    # Calculate normalized vector along the line of plane intersection
    vec_t = np.cross(vec_p, vec_q)
    unit_t = vec_t / np.linalg.norm(vec_t)

    # Projections of t along arcs a and b
    s_1 = np.dot(np.cross(vec_a0, vec_p), unit_t)
    s_2 = np.dot(np.cross(vec_a1, vec_p), unit_t)
    s_3 = np.dot(np.cross(vec_b0, vec_q), unit_t)
    s_4 = np.dot(np.cross(vec_b1, vec_q), unit_t)

    # Test if arcs intersect and if so, determine at which location
    if -s_1 < 0 and s_2 < 0 and -s_3 < 0 and s_4 < 0:
        # Intersection occurs at -t
        intersect_t = -1 * unit_t
    elif -s_1 >= 0 and s_2 >= 0 and -s_3 >= 0 and s_4 >= 0:
        # Intersection occurs at t
        intersect_t = unit_t
    else:
        # No intersection:
        raise ValueError("Great circle arcs do not intersect at any point.")

    # Convert intersect_t back to lat, lon
    intersect_x = intersect_t[0]
    intersect_y = intersect_t[1]
    intersect_z = intersect_t[2]

    intersect_lat, intersect_lon = vector_to_latlon(
        intersect_x, intersect_y, intersect_z)

    return np.degrees(intersect_lat), np.degrees(intersect_lon)


def calc_midpoint(lat_1, lon_1, lat_2, lon_2):
    """
    Calculate the midpoint coordinates along a great circle path between two
    points (lat_1, lon_1) and (lat_2, lon_2) on a sphere.

    Equation from: https://www.movable-type.co.uk/scripts/latlong.html
    """

    delta_lon = np.radians(lon_2 - lon_1)
    Bx = np.cos(np.radians(lat_2)) * np.cos(delta_lon)
    By = np.cos(np.radians(lat_2)) * np.sin(delta_lon)

    lat_mid = np.degrees(np.arctan2(np.sin(np.radians(lat_1)) + np.sin(np.radians(lat_2)),
                                    np.sqrt((np.cos(np.radians(lat_1)) + Bx) ** 2 + By ** 2)))
    lon_mid = lon_1 + np.degrees(np.arctan2(By, np.cos(np.radians(lat_1)) + Bx))

    return lat_mid, lon_mid


def crs_middle(crs_left, crs_right):
    """
    Find the course that forms the bisector of the angle
    between crs_left and crs_right (in clockwise direction).
    """

    if crs_left < crs_right:
        crs_mid = 0.5 * (crs_left + crs_right)
    elif crs_left > crs_right:
        # North in between crs_l and crs_r
        crs_mid = (crs_left + 0.5 * (360 - crs_left + crs_right)) % 360
    else:
        # Ensure when crs_l,crs_r = 360 then crs_mid = 0
        crs_mid = crs_left % 360

    return crs_mid


def crs_is_between(crs, crs_left, crs_right):
    """
    Check if a given magnetic course crs on [0 .. 360] deg lies
    in between crs_left and crs_right (in clockwise direction).
    """

    is_between = ((crs_left > crs_right) and (crs > crs_left or crs < crs_right)) or \
        ((crs_left < crs_right) and (crs > crs_left and crs < crs_right))

    return is_between


def calc_future_pos(dt, lon, lat, gseast, gsnorth):
    """
    Calculate future lon and lat vectors after time dt based on
    current position and velocity vectors.
    """

    newlat = lat + np.degrees(dt * gsnorth / Rearth)
    newcoslat = np.cos(np.deg2rad(newlat))
    newlon = lon + np.degrees(dt * gseast / newcoslat / Rearth)

    return newlon, newlat


def enu2crs(enu):
    """
    Convert an array of angles defined in East-North-Up on
    [-180,180] degrees to compass angles on [0,360].
    """

    crs = ((90 - enu) + 360) % 360

    return crs


def ned2crs(ned):
    """
    Convert an array of angles defined in North-East-Down on
    [-180,180] degrees to compass angles on [0,360].
    """

    crs = (ned + 360) % 360

    return crs


def crs_closest(ref_crs, crs_a, crs_b):
    """
    Takes three course vectors ref_rs, crs_a, and crs_b and per element
    returns the value of either crs_a or crs_b depending on which has
    the smallest angle difference with respect to ref_crs.
    """

    # Calculate absolute angle difference between both courses and the reference
    diff_ref_a = np.absolute(np.remainder(ref_crs - crs_a + 180, 360) - 180)
    diff_ref_b = np.absolute(np.remainder(ref_crs - crs_b + 180, 360) - 180)

    # Select the course with the smallest angle difference
    crs = np.where(diff_ref_a < diff_ref_b, crs_a, crs_b)

    return crs


# if __name__ == "__main__":
#     (lat00, lon00) = sphere_greatcircle_intersect(52.2, 5.0, 51.7, 5.4, 51.5, 4.5, 52.0, 5.5)

#     # (lat, lon) = ellipse_greatcircle_intersect(51.88158, 4.60602,
#     #                                    52.18404, 5.44647,
#     #                                    51.5, 4.5,
#     #                                    52.0, 5.5)
#     # print("Lat: {:.30f}, Lon {:.30f}".format(lat, lon))

#     (lat02, lon02) = sphere_greatcircle_intersect(52.33534, 2.32361,
#                                               51.53950, 7.26196,
#                                               51.5, 4.5,
#                                               53.03461, 5.41626)

#     (lat03, lon03) = sphere_greatcircle_intersect(42.94034, -74.70703,
#                                               -7.18810, 84.90234,
#                                               -18.64625, -10.72266,
#                                               55.77657, 104.76563)

#     (lat04, lon04) = sphere_greatcircle_intersect(52, 5,
#                                               37, -120,
#                                               0, -60,
#                                               70, 0)

# # Ellipsoid
#     (lat10, lon10) = ellipsoid_greatcircle_intersect(52.2, 5.0, 51.7, 5.4, 51.5, 4.5, 52.0, 5.5)

#     # (lat, lon) = ellipse_greatcircle_intersect(51.88158, 4.60602,
#     #                                    52.18404, 5.44647,
#     #                                    51.5, 4.5,
#     #                                    52.0, 5.5)
#     # print("Lat: {:.30f}, Lon {:.30f}".format(lat, lon))

#     (lat12, lon12) = ellipsoid_greatcircle_intersect(52.33534, 2.32361,
#                                                  51.53950, 7.26196,
#                                                  51.5, 4.5,
#                                                  53.03461, 5.41626)

#     (lat13, lon13) = ellipsoid_greatcircle_intersect(42.94034, -74.70703,
#                                                  -7.18810, 84.90234,
#                                                  -18.64625, -10.72266,
#                                                  55.77657, 104.76563)

#     (lat14, lon14) = ellipsoid_greatcircle_intersect(52, 5,
#                                                  37, -120,
#                                                  0, -60,
#                                                  70, 0)

#     (lat15, lon15) = ellipsoid_greatcircle_intersect(55, 4,
#                                                  55, 22,
#                                                  0, 8,
#                                                  88, 8)
#     # print("Lat: {:.30f}, Lon {:.30f}".format(lat15, lon15))

# print("\nOn sphere:")
# print("Lat: {:.30f}, Lon {:.30f}".format(lat00, lon00))
# print("Lat: {:.30f}, Lon {:.30f}".format(lat02, lon02))
# print("Lat: {:.30f}, Lon {:.30f}".format(lat03, lon03))
# print("Lat: {:.30f}, Lon {:.30f}".format(lat04, lon04))

# print("\nOn ellipsoid:")
# print("Lat: {:.30f}, Lon {:.30f}".format(lat10, lon10))
# print("Lat: {:.30f}, Lon {:.30f}".format(lat12, lon12))
# print("Lat: {:.30f}, Lon {:.30f}".format(lat13, lon13))
# print("Lat: {:.30f}, Lon {:.30f}".format(lat14, lon14))

# print("\nDifferences:")
# print("Lat: {:.30f}, Lon: {:.30f}".format(abs(lat10 - lat00), abs(lon10 - lon00)))
# print("Lat: {:.30f}, Lon: {:.30f}".format(abs(lat12 - lat02), abs(lon12 - lon02)))
# print("Lat: {:.30f}, Lon: {:.30f}".format(abs(lat13 - lat03), abs(lon13 - lon03)))
# print("Lat: {:.30f}, Lon: {:.30f}".format(abs(lat14 - lat04), abs(lon14 - lon04)))
