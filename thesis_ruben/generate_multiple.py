"""
Generate BlueSky scenario files using scenario_generator.py for specified
combinations of input variables.
"""

# Python imports
import itertools
import datetime

# Local imports
from scenario_generator import create


def main():
    current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    # Input variables and
    random_seed = [x for x in range(1, 11)]
    corridor_length = [40]
    corridor_width = [25]
    ac_creation_arc_angle = [90]
    asas_reso_method = ["OFF", "LF", "MVP", "SWARM_V2"]

    combination_lst = list(itertools.product(random_seed,
                                             asas_reso_method,
                                             corridor_length,
                                             corridor_width,
                                             ac_creation_arc_angle))

    # Create a scenario file for each combination of variables
    for (seed, reso_method, length, width, angle) in combination_lst:
        create(current_time,
               seed,
               reso_method,
               length,
               width,
               angle,
               is_edge_angle=False)


if __name__ == "__main__":
    main()
