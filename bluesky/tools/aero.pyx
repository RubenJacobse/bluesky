""" 
This module defines a set of standard aerodynamic functions and constants.

It contains scalar and vectorized versions of aero conversion routines.
International standard atmosphere calculations are supported only up to
an altitude of 72000 ft / 22 km.
"""

from libc.math import *
import numpy as np
cimport numpy as np


# Constants Aeronautics
cdef double kts = 0.514444              # m/s  of 1 knot
cdef double ft  = 0.3048                # m    of 1 foot
cdef double fpm = ft/60.                # feet per minute
cdef double inch = 0.0254               # m    of 1 inch
cdef double sqft = 0.09290304           # 1sqft
cdef double nm  = 1852.                 # m    of 1 nautical mile
cdef double lbs = 0.453592              # kg   of 1 pound mass
cdef double g0  = 9.80665               # m/s2    Sea level gravity constant
cdef double R   = 287.05287             # Used in wikipedia table: checked with 11000 m
cdef double p0 = 101325.                # Pa     Sea level pressure ISA
cdef double rho0 = 1.225                # kg/m3  Sea level density ISA
cdef double T0   = 288.15               # K   Sea level temperature ISA
cdef double Tstrat = 216.65             # K Stratosphere temperature (until alt=22km)
cdef double gamma = 1.40                # cp/cv: adiabatic index for air
cdef double gamma1 =  0.2               # (gamma-1)/2 for air
cdef double gamma2 = 3.5                # gamma/(gamma-1) for air
cdef double beta = -0.0065              # [K/m] ISA temp gradient below tropopause
cdef double Rearth = 6371000.           # m  Average earth radius
cdef double a0  = np.sqrt(gamma*R*T0)   # sea level speed of sound ISA


# Functions for aeronautics in this module
#  - physical quantities always in SI units
#  - lat,lon,course and heading in degrees
#
#  International Standard Atmosphere up to 22 km
#
#   p,rho,T = vatmos(h)    # atmos as function of geopotential altitude h [m]
#   a = vvsound(h)         # speed of sound [m/s] as function of h[m]
#   p = vpressure(h)       # calls atmos but retruns only pressure [Pa]
#   T = vtemperature(h)    # calculates temperature [K] (saves time rel to atmos)
#   rho = vdensity(h)      # calls atmos but retruns only pressure [Pa]
#
#  Speed conversion at altitude h[m] in ISA:
#
# M   = vtas2mach(tas,h)  # true airspeed (tas) to mach number conversion
# tas = vmach2tas(M,h)    # true airspeed (tas) to mach number conversion
# tas = veas2tas(eas,h)   # equivalent airspeed to true airspeed, h in [m]
# eas = vtas2eas(tas,h)   # true airspeed to equivent airspeed, h in [m]
# tas = vcas2tas(cas,h)   # cas  to tas conversion both m/s, h in [m]
# cas = vtas2cas(tas,h)   # tas to cas conversion both m/s, h in [m]
# cas = vmach2cas(M,h)    # Mach to cas conversion cas in m/s, h in [m]
# M   = vcas2mach(cas,h)   # cas to mach copnversion cas in m/s, h in [m]


# ------------------------------------------------------------------------------
# Vectorized aero functions
# ------------------------------------------------------------------------------
def vatmos(h):  # h in m
    # Temp
    T = vtemp(h)

    # Density
    rhotrop = 1.225 * (T / 288.15)**4.256848030018761
    dhstrat = np.maximum(0., h - 11000.)
    rho     = rhotrop * np.exp(-dhstrat / 6341.552161)  # = *g0/(287.05*216.65))

    # Pressure
    p = rho * R * T

    return p, rho, T


def vtemp(h):         # h [m]
    T = np.maximum(288.15 - 0.0065 * h, Tstrat)
    return T


# Atmos wrappings:
def vpressure(h):          # h [m]
    p, r, T = vatmos(h)
    return p


def vdensity(h):   # air density at given altitude h [m]
    p, r, T = vatmos(h)
    return r


def vvsound(h):  # Speed of sound for given altitude h [m]
    T = vtemp(h)
    a = np.sqrt(gamma * R * T)
    return a


# ---------Speed conversions---h in [m]------------------
def vtas2mach(tas, h):
    """ True airspeed (tas) to mach number conversion """
    a = vvsound(h)
    M = tas / a
    return M


def vmach2tas(M, h):
    """ True airspeed (tas) to mach number conversion """
    a = vvsound(h)
    tas = M * a
    return tas


def veas2tas(eas, h):
    """ Equivalent airspeed to true airspeed """
    rho = vdensity(h)
    tas = eas * np.sqrt(rho0 / rho)
    return tas


def vtas2eas(tas, h):
    """ True airspeed to equivent airspeed """
    rho = vdensity(h)
    eas = tas*np.sqrt(rho / rho0)
    return eas


def vcas2tas(cas, h):
    """ cas2tas conversion both m/s """
    p, rho, T = vatmos(h)
    qdyn = p0*((1.+rho0*cas*cas/(7.*p0))**3.5-1.)
    tas = np.sqrt(7.*p/rho*((1.+qdyn/p)**(2./7.)-1.))

    # cope with negative speed
    tas = np.where(cas<0, -1*tas, tas)
    return tas


def vtas2cas(tas, h):
    """ tas2cas conversion both m/s """
    p, rho, T = vatmos(h)
    qdyn = p*((1.+rho*tas*tas/(7.*p))**3.5-1.)
    cas = np.sqrt(7.*p0/rho0*((qdyn/p0+1.)**(2./7.)-1.))

    # cope with negative speed
    cas = np.where(tas<0, -1*cas, cas)
    return cas


def vmach2cas(M, h):
    """ Mach to CAS conversion """
    tas = vmach2tas(M, h)
    cas = vtas2cas(tas, h)
    return cas


def vcas2mach(cas, h):
    """ CAS to Mach conversion """
    tas = vcas2tas(cas, h)
    M   = vtas2mach(tas, h)
    return M

def vcasormach(spd, h):
    ismach = np.logical_and(0.1 < spd, spd < 2.0)
    tas = np.where(ismach, vmach2tas(spd, h), vcas2tas(spd, h))
    cas = np.where(ismach, vtas2cas(tas, h), spd)
    m   = np.where(ismach, spd, vtas2mach(tas, h))
    return tas, cas, m

def vcasormach2tas(spd, h):
    tas = np.where(np.abs(spd) < 2.0, vmach2tas(spd, h), vcas2tas(spd, h))
    return tas


def crossoveralt(vcas, mach):
    ''' Calculate crossover altitude for given CAS and Mach number. 
    
        Calculates the altitude where the given CAS and Mach values
        correspond to the same true airspeed.

        (BADA User Manual 3.12, p. 12)

        Returns: altitude in meters.
    '''
    # Delta: pressure ratio at the transition altitude
    delta = (((1.0 + 0.5 * (gamma - 1.0) * (vcas / a0) ** 2) **
                (gamma / (gamma - 1.0)) - 1.0) /
                ((1.0 + 0.5 * (gamma - 1.0) * mach ** 2) **
                (gamma / (gamma - 1.0)) - 1.0))
    # Theta: Temperature ratio at the transition altitude
    theta = delta ** (-beta * R / g0)
    return 1000.0 / 6.5 * T0 * (1.0 - theta)


# ------------------------------------------------------------------------------
# Scalar aero functions
# ------------------------------------------------------------------------------
cpdef double atmos(double h):
    """
    atmos(altitude): International Standard Atmosphere calculator

    Input:
          h = altitude in meters 0.0 < h < 84852.0
    (will be clipped when outside range, integer input allowed)
    
    Output:
          [p,rho,T]    (in SI-units: Pa, kg/m3 and K) 
    """

    cdef double h0[8], p0[8], T0[8], a[8]
    cdef int i
    cdef double p, T, rho

    # Constants
    # Base values and gradient in table from hand-out
    # (but corrected to avoid small discontinuities at borders of layers)
    h0[:] = [0.0, 11000., 20000., 32000., 47000., 51000., 71000., 86852.]

    p0[:] = [101325.,                 # Sea level
             22631.7009099,           # 11 km
              5474.71768857,          # 20 km
               867.974468302,         # 32 km
               110.898214043,         # 47 km
                66.939,               # 51 km
                 3.9564 ]             # 71 km

    T0[:] = [288.15,  # Sea level
             216.65,  # 11 km
             216.65,  # 20 km
             228.65,  # 32 km
             270.65,  # 47 km
             270.65,  # 51 km
             214.65]  # 71 km

    # a = lapse rate (temp gradient)
    # integer 0 indicates isothermic layer!
    a[:]  = [-0.0065, # 0-11 km
               0,     # 11-20 km
             0.001,   # 20-32 km
             0.0028,  # 32-47 km
               0,     # 47-51 km
             -0.0028, # 51-71 km
             -0.002]  # 71-   km

    # Clip altitude to maximum!
    h = max(0.0, min(float(h), h0[-1]))

    # Find correct layer
    i = 0
    while h > h0[i+1] and i < len(h0) - 2:
        i = i+1

    # Calculate if isothermic layer
    if a[i] == 0:
        T   = T0[i]
        p   = p0[i]*exp(-g0/(R*T)*(h-h0[i]))
        rho = p/(R*T)

    # Calculate for temperature gradient
    else:
        T   = T0[i] + a[i]*(h-h0[i])
        p   = p0[i]*((T/T0[i])**(-g0/(a[i]*R)))
        rho = p/(R*T)

    return p, rho, T


cpdef double temp(double h):
    """
    temp (altitude): Temperature only version of ISA atmos

    Input:
            h =  altitude in meters 0.0 < h < 84852.
            (will be clipped when outside range, integer input allowed)
    Output:
            T    (in SI-unit: K 
    """

    cdef double h0[8], T0[7], a[7]
    cdef int i
    cdef double T

    # Base values and gradient in table from hand-out
    # (but corrected to avoid small discontinuities at borders of layers)
    h0[:] = [0.0, 11000., 20000., 32000., 47000., 51000., 71000., 86852.]

    T0[:] = [288.15,  # Sea level
             216.65,  # 11 km
             216.65,  # 20 km
             228.65,  # 32 km
             270.65,  # 47 km
             270.65,  # 51 km
             214.65]  # 71 km

    # a = lapse rate (temp gradient)
    # integer 0 indicates isothermic layer!
    a[:] = [-0.0065, # 0-11 km
              0 ,    # 11-20 km
            0.001,   # 20-32 km
            0.0028,  # 32-47 km
              0 ,    # 47-51 km
            -0.0028, # 51-71 km
            -0.002]  # 71-   km
    
    # Clip altitude to maximum!
    h = max(0.0, min(float(h), h0[-1]))

    # Find correct layer
    i = 0
    while h > h0[i+1] and i < len(h0)-2:
        i = i + 1

    # Calculate if sothermic layer
    if a[i]==0:
        T = T0[i]

    # Calculate for temperature gradient
    else:
        T = T0[i] + a[i]*(h - h0[i])

    return T


# wrappers around atmos(h) for pressure, density and speed of sound
cpdef double pressure(double h):
    """
    Calculate air pressure [N/m^2] at altitude h [m]
    """
    cdef double p, r, T
    p, r, T = atmos(h)
    return p


cpdef double density(double h):
    """
    Calculate air density [kg/m^3] at altitude h [m]
    """
    cdef double p, r, T
    p, r, T = atmos(h)
    return r


cpdef double vsound(double h):
    """ 
    Calculates speed of sound [m/s] at altitude h [m]
    """
    cdef double T, a
    T = temp(h)
    a = sqrt(gamma*R*T)
    return a


# ---------Speed conversions---h in [m]------------------
cpdef double tas2mach(double tas, double h):
    """
    True airspeed (tas) to mach number conversion
    """
    cdef double a = vsound(h)
    cdef double M = tas / a
    return M


cpdef double mach2tas(double M, double h):
    """
    True airspeed (tas) to mach number conversion
    """
    cdef double a = vsound(h)
    cdef double tas = M * a
    return tas


cpdef double eas2tas(double eas, double h):
    """
    Equivalent airspeed to true airspeed
    """
    cdef double rho = density(h)
    cdef double tas = eas * sqrt(rho0 / rho)
    return tas


cdef double tas2eas(double tas, double h):
    """
    True airspeed to equivalent airspeed
    """
    cdef double rho = density(h)
    cdef double eas = tas * sqrt(rho / rho0)
    return eas


cdef double cas2tas(double cas, double h):
    """
    cas2tas conversion both [m/s], h in [m]
    """
    p, rho, T = atmos(h)
    qdyn = p0*((1.+rho0*cas*cas/(7.*p0))**3.5-1.)
    tas = sqrt(7.*p/rho*((1.+qdyn/p)**(2./7.)-1.))
    tas = -1 * tas if cas < 0 else tas
    return tas


def tas2cas(tas, h):
    """ tas2cas conversion both m/s """
    p, rho, T = atmos(h)
    qdyn = p*((1.+rho*tas*tas/(7.*p))**3.5-1.)
    cas = sqrt(7.*p0/rho0*((qdyn/p0+1.)**(2./7.)-1.))
    cas = -1 * cas if tas < 0 else cas
    return cas


def mach2cas(M, h):
    """ Mach to CAS conversion """
    tas = mach2tas(M, h)
    cas = tas2cas(tas, h)
    return cas


def cas2mach(cas, h):
    """ CAS Mach conversion """
    tas = cas2tas(cas, h)
    M   = tas2mach(tas, h)
    return M

def casormach(spd,h):
    if 0.1 < spd < 1:
        # Interpret spd as Mach number
        tas = mach2tas(spd, h)
        cas = mach2cas(spd, h)
        m   = spd
    else:
        # Interpret spd as CAS
        tas = cas2tas(spd,h)
        cas = spd
        m   = cas2mach(spd, h)
    return tas, cas, m

def casormach2tas(spd,h):
    if 0.1 < spd < 1:
        # Interpret spd as Mach number
        tas = mach2tas(spd, h)
    else:
        # Interpret spd as CAS
        tas = cas2tas(spd,h)
    return tas


def metres_to_feet_rounded(metres):
    """
    Converts metres to feet.
    Returns feet as rounded integer.
    """
    return int(round(metres / ft))


def metric_spd_to_knots_rounded(speed):
    """
    Converts speed in m/s to knots.
    Returns knots as rounded integer.
    """
    return int(round(speed / kts))
