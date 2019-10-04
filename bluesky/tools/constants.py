"""
Globally used physical constants. When importing use the following syntax:

    >from bluesky.tools.constants import *

such that constants always exist in the local namespace.
"""

# ========================================================================
# Physical constants
# ========================================================================

# ICAO Standard Atmosphere
ISA_GASCONST_AIR = 287.058       # [J/kg/K] Specific gas constant for dry air
ISA_GAMMA_AIR = 1.4              # [-] Ratio of cp/cv for air
ISA_DENS_MSL = 1.225             # [kg/m^3] Air density at sea level
ISA_PRES_MSL = 101325.           # [N/m^2] Air pressure at sea level
ISA_SPDSOUND_MSL = 340.27        # [m/s] Speed of sound at sea level
ISA_TEMP_MSL = 288.15            # [K] Temperature at sea level
ISA_TEMP_STRATS_BASE = 216.65    # [K] Temperature at stratosphere base (20000 m)
# Temperature gradients
ISA_TEMPGRAD_TROPOS = -0.0065    # [K/m] Troposphere (0-11000 m)
ISA_TEMPGRAD_STRATS_LO = 0.0010  # [K/m] Lower stratosphere (20000-32000 m)
ISA_TEMPGRAD_STRATS_UP = 0.0028  # [K/m] Upper stratosphere (32000-47000 m)

# WGS-84 Earth reference ellipsoid
# Defining parameters
WGS84_SEMIMAJ = 6378137.0           # [m] Earth semi-major axis
WGS84_FLATTENING = 1/298.257223563  # [-] Flattening factor
# Derived parameters
WGS84_SEMIMIN = (1 - WGS84_FLATTENING) * WGS84_SEMIMAJ        # [m] Earth semi-minor axis
WGS84_ECCEN_SQRD = (2 - WGS84_FLATTENING) * WGS84_FLATTENING  # [-] First eccentricity squared
WGS84_ECCEN = WGS84_ECCEN_SQRD ** 0.5                         # [-] First eccentricity

# Other constants
AVG_RADIUS_EARTH = 6371000.     # [m] Average radius of Earth
GRAV_ACCEL_AVG = 9.80665        # [m/s^2] Average gravitational acceleration
GRAV_ACCEL_EQTR = 9.7803253359  # [m/s^2] Gravitational acceleration at equator
GRAV_ACCEL_POLE = 9.8321849378  # [m/s^2] Gravitational acceleration at pole
# [-] WGS84 Gravity formula constant 'k'
GRAV_CONST_K = ((WGS84_SEMIMIN * GRAV_ACCEL_POLE - WGS84_SEMIMAJ * GRAV_ACCEL_EQTR)
                / (WGS84_SEMIMAJ * GRAV_ACCEL_EQTR))

# ========================================================================
# Conversion factors
# ========================================================================

# Length
NM_TO_KM = 1.852             # [km] Nautical miles to kilometres
KM_TO_NM = 1 / NM_TO_KM      # [NM] Kilometres to nautical miles
NM_TO_M = 1852.              # [m] Nautical miles to metres
M_TO_NM = 1 / NM_TO_M        # [NM] Metres to nautical miles
FT_TO_M = 0.3048             # [m] Feet to metres
M_TO_FT = 1 / FT_TO_M        # [ft] Metres to feet
INCH_TO_M = 0.0254           # [m] Inches to metres
M_TO_INCH = 1 / INCH_TO_M    # [inch] Metres to inches
LVL_TO_FT = 100              # [ft] Flight level to feet
FT_TO_LVL = 1 / LVL_TO_FT    # [-] Feet to flight level

# Area
SQFT_TO_SQM = FT_TO_M ** 2   # [m^2] Square feet to square metres
SQM_TO_SQFT = M_TO_FT ** 2   # [ft^2] Square metres to square feet

# Time
MIN_TO_SEC = 60.             # [s] Minutes to seconds
SEC_TO_MIN = 1 / MIN_TO_SEC  # [min] Seconds to minutes
HR_TO_SEC = 3600.            # [s] Hours to seconds
SEC_TO_HR = 1 / HR_TO_SEC    # [hr] Seconds to hours
HR_TO_MIN = 60.              # [min] Hours to minutes
MIN_TO_HR = 1 / HR_TO_MIN    # [hr] Minutes to hours
DAY_TO_SEC = 86000.          # [sec] Days to seconds

# Speed
KTS_TO_MPS = 0.514444              # [m/s] Knots to metres per second
MPS_TO_KTS = 1 / KTS_TO_MPS        # [kt] Metres per second to knots
KTS_TO_KMH = 1.852                 # [km/h] Knots to kilometres per hour
KMH_TO_KTS = 1 / KTS_TO_KMH        # [kt] Kilometres per hour to knots
MPS_TO_FPM = MIN_TO_SEC * M_TO_FT  # [fpm] Metres per second to feet per minute
FPM_TO_MPS = 1 / MPS_TO_FPM        # [m/s] Feet per minute to metres per second

# Weight
LBS_TO_KG = 0.453592         # [kg] Pounds to kilograms
KG_TO_LBS = 1 / LBS_TO_KG    # [lbs] Kilograms to pounds
