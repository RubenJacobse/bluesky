import numpy as np

DIV_ZERO_SAFETY_FACTOR = 1e-15

# NOTE: Function does not currently support numpy arrays!
def rhumb_azimuth(lat_begin, lon_begin, lat_end, lon_end):
    """
    Calculate azimuth angle from a given start point along the rhumb
    line connecting connecting it to an end point. Uses the WGS84
    reference ellipsoid for calculations.

    In:
        lat_begin, lon_begin, lat_end, lon_end [deg]
    Out:
        azimuth [deg] = course from begin to end along loxodrome
    """

    # NOTE: Equations from https://planetcalc.com/713/

    # Calculate eccentricity
    a = 6378137.0 # [m] WGS84 Earth semi-major axis
    b = 6356752.314245 # [m] WGS84 Earth semi-minor axis
    a_squared = a ** 2
    b_squared = b ** 2
    eccentricity = np.sqrt(1 - (b_squared / a_squared)) # [-] WGS84 Earth eccentricity factor

    # Longitude difference value depends on whether the points are on the
    # same side of the 180th meridian
    if abs(lon_end - lon_begin) <= 180:
        delta_lon = lon_end - lon_begin
    elif lon_end - lon_begin < -180:
        delta_lon = 360 + lon_end - lon_begin
    else:
        delta_lon = lon_end - lon_begin - 360

    # Further equations use radians
    lat_begin_rad = np.radians(lat_begin)
    lat_end_rad = np.radians(lat_end)
    delta_lon_rad = np.radians(delta_lon)

    # Following function is used twice to calculate the denominator inside the arctan()
    # for the azimuth calculation
    def natlog_factor(lat):
        """
        Calculate the result of the following equation:

        ln(tan(pi/4 + lat/2) * ((1 - e * sin(lat)) / (1 + e * sin(lat))) ** (e / 2))

        """

        tan_term = np.tan(np.pi / 4 + lat / 2)
        lat_term = ((1 - eccentricity * np.sin(lat)) / (1 + eccentricity * np.sin(lat))) \
            ** (eccentricity / 2)

        return np.log(tan_term * lat_term)

    # Azimuth angle calculation
    # NOTE: If the start and end latitude are exactly equal, then the denominator in the
    # azimuth_rad calculation can become zero leading to division by zero. To prevent
    # this, a very small safety factor DIV_ZERO_SAFETY_FACTOR is added to this term.
    azimuth_rad = np.arctan(delta_lon_rad / (natlog_factor(lat_end_rad) \
        - natlog_factor(lat_begin_rad) + DIV_ZERO_SAFETY_FACTOR))

    azimuth_deg = np.degrees(azimuth_rad)

    return azimuth_deg

if __name__ == "__main__":

    # az = rhumb_azimuth(np.array([-1.5]), np.array([-1.5]), np.array([-1.0]), np.array([-0.5]))
    # print(az)

    # az2 = rhumb_azimuth(np.array([33.95]), np.array([-118.4]), np.array([40.63333333]), np.array([-73.78333333]))
    # print(az2)

    az3 = rhumb_azimuth(np.array([60.0]), np.array([170]), np.array([60.0]), np.array([-170]))
    print(az3)

    az4 = rhumb_azimuth(np.array([-1.5, 33.95, 60.0]),
                        np.array([-1.5, -118.4, 170]),
                        np.array([-1.0, 40.6333, 60.0]),
                        np.array([-0.5, -73.783333, -170]))
    print(az4)
