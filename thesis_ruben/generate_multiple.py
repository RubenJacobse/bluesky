"""
Bla
"""

import itertools
from scenario_generator import create

random_seed = [1]
corridor_length = [10, 20, 30, 40, 50]
corridor_width = [5, 10, 15, 20, 25]
ac_creation_arc_angle = [80, 90, 100, 110, 120]
asas_reso_method = ["OFF", "LF", "MVP"]

combination_lst = list(itertools.product(corridor_length, corridor_width,
                                         ac_creation_arc_angle, asas_reso_method))

for (length, width, angle, reso_method) in combination_lst:
    create(random_seed, reso_method, length, width, angle, is_restriction_angle=True)
