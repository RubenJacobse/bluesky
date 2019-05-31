"""
Bla
"""

import itertools
import datetime
from scenario_generator import create

if __name__ == "__main__":
    curr_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    # Input variables and
    random_seed = [1]
    corridor_length = [10, 20, 30, 40, 50]
    corridor_width = [5, 10, 15, 20, 25]
    ac_creation_arc_angle = [80, 90, 100, 110, 120]
    asas_reso_method = ["OFF", "LF", "MVP", "SWARM_V2"]

    combination_lst = list(itertools.product(random_seed,
                                             asas_reso_method,
                                             corridor_length,
                                             corridor_width,
                                             ac_creation_arc_angle))

    for (seed, reso_method, length, width, angle) in combination_lst:
        create(curr_time,
               seed,
               reso_method,
               length,
               width,
               angle,
               is_restriction_angle=False)
