"""
Alternate way to manage the functionalities
"""

from enum import IntEnum


class SteeringMode(IntEnum):
    """
    Object used to signal which steering mode is to be used
    by an aircraft.
    Valid modes that can be used: LNAV, ASAS, and AREA
    """

    AREA = 1  # Using area avoidance rules
    ASAS = 2  # Using aircraft avoidance rules
    LNAV = 3  # Using route following rules
