"""
Read the settings defined in "scenario_generation.ini".
"""

# Python imports
import itertools
from configparser import ConfigParser


def parse(section):
    """
    Parses the configuration settings in the specified section of the
    configuration file "scenario_generation.ini" and returns a list with
    with all combinations of value settings.
    """

    config = ConfigParser()
    config.read("scenario_generation.ini")

    # Split the comma separated strings and convert each numeric value
    # to integer type
    random_seed = [int(x) for x in
                   config[section]["random_seed"].split(",")]
    traffic_level = config[section]["traffic_level"].split(",")
    reso_method = config[section]["reso_method"].split(",")
    corridor_length = [int(x) for x in
                       config[section]["corridor_length"].split(",")]
    corridor_width = [int(x) for x in
                      config[section]["corridor_width"].split(",")]
    arc_angle = [int(x) for x in
                 config[section]["arc_angle"].split(",")]

    # Make a list of all combinations
    combination_lst = list(itertools.product(random_seed,
                                             traffic_level,
                                             reso_method,
                                             corridor_length,
                                             corridor_width,
                                             arc_angle))
    return combination_lst
