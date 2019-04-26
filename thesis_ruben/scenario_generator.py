"""
This module creates the scenario files necessary for my experiments.
"""

# Python imports
import math
import sys, os
sys.path.append(os.path.abspath(os.path.join('..')))

# Third-party imports
import numpy as np

# BlueSky imports
import bluesky.tools.geo as bsgeo
import plugins.tempgeo as tg

# Using these as constants for now, will probably become
# function arguments later.
CENTER_LAT = 0.0 # [deg]
CENTER_LON = 0.0 # [deg]
CORRIDOR_LENGTH = 50 # [NM]
CORRIDOR_WIDTH = 25 # [NM]
AREA_RADIUS = 100 # [NM]
EXPERIMENT_RADIUS = 75 # [NM]
RESTRICTION_ANGLE = 60 # [deg]
NUM_DEP_DEST_POINTS = 5 #

PLUGIN_NAME = "RAA"    
SIM_TIME_STEP = 0.05    # [s] Simulation time step
SHOW_AC_TRAILS = True
ASAS_ON = False
ASAS_RESO_METHOD = "MVP"


def main():
    file_name = "C{}_{}_L{}_W{}_R{}_A{}_P{}.scn".format(CENTER_LAT,
                                                        CENTER_LON,
                                                        CORRIDOR_LENGTH,
                                                        CORRIDOR_WIDTH,
                                                        AREA_RADIUS,
                                                        RESTRICTION_ANGLE,
                                                        NUM_DEP_DEST_POINTS)

    # Open file, overwrite if existing
    with open(file_name, "w+") as scnfile:
        zero_time_str = "0:00:00.00>"

        scnfile.write("# Sim commands\n")
        scnfile.write(zero_time_str + "PAN {},{}\n".format(CENTER_LAT, CENTER_LON))
        scnfile.write(zero_time_str + "DT {}\n".format(SIM_TIME_STEP))
        scnfile.write(zero_time_str + "TRAIL {}\n".format("ON" if SHOW_AC_TRAILS else "OFF"))

        scnfile.write("\n# Setup circular experiment area and activate it as a traffic area in BlueSky\n")
        scnfile.write(zero_time_str + "PLUGINS LOAD AREA\n")
        scnfile.write(zero_time_str + "CIRCLE EXPERIMENT {},{},{}\n"\
            .format(CENTER_LAT, CENTER_LON, AREA_RADIUS))
        scnfile.write(zero_time_str + "AREA EXPERIMENT\n")

        scnfile.write("\n# Setup BlueSky ASAS module options\n")
        scnfile.write(zero_time_str + "ASAS {}\n".format("ON" if ASAS_ON else "OFF"))
        scnfile.write(zero_time_str + "RESO {}\n".format(ASAS_RESO_METHOD))

        scnfile.write("\n# LOAD RAA plugin and create area restrictions\n")
        scnfile.write(zero_time_str + "PLUGINS LOAD {}\n".format(PLUGIN_NAME))

        # Area on left side of corridor
        #
        inner_left_top_lat, _ = bsgeo.qdrpos(CENTER_LAT, CENTER_LON, 0, CORRIDOR_LENGTH)
        _, inner_left_top_lon = bsgeo.qdrpos(CENTER_LAT, CENTER_LON, 270, CORRIDOR_WIDTH)

        inner_left_bottom_lat, _ = bsgeo.qdrpos(CENTER_LAT, CENTER_LON, 180, CORRIDOR_LENGTH)
        _, inner_left_bottom_lon = bsgeo.qdrpos(CENTER_LAT, CENTER_LON, 270, CORRIDOR_WIDTH)

        ext_dist = AREA_RADIUS / math.sin(math.radians(RESTRICTION_ANGLE))
        ext_left_top_lat, ext_left_top_lon = bsgeo.qdrpos(inner_left_top_lat, inner_left_top_lon, 360 - RESTRICTION_ANGLE, ext_dist)
        
        vert_dist = ext_dist * 2
        _, left_outer_lon = bsgeo.qdrpos(CENTER_LAT, CENTER_LON, 270, AREA_RADIUS)
        vert_left_top_lat, vert_left_top_lon = bsgeo.qdrpos(CENTER_LAT, left_outer_lon, 360, vert_dist)
        
        outer_left_top_lat, outer_left_top_lon = tg.sphere_greatcircle_intersect(inner_left_top_lat,
                                                                                   inner_left_top_lon,
                                                                                   ext_left_top_lat,
                                                                                   ext_left_top_lon,
                                                                                   CENTER_LAT,
                                                                                   left_outer_lon,
                                                                                   vert_left_top_lat,
                                                                                   vert_left_top_lon)
        
        ext_dist = AREA_RADIUS / math.sin(math.radians(RESTRICTION_ANGLE))
        ext_left_bottom_lat, ext_left_bottom_lon = bsgeo.qdrpos(inner_left_bottom_lat, inner_left_bottom_lon, 180 + RESTRICTION_ANGLE, ext_dist)
        
        vert_dist = ext_dist * 2
        _, left_outer_lon = bsgeo.qdrpos(CENTER_LAT, CENTER_LON, 270, AREA_RADIUS)
        vert_left_bottom_lat, vert_left_bottom_lon = bsgeo.qdrpos(CENTER_LAT, left_outer_lon, 180, vert_dist)
        
        outer_left_bottom_lat, outer_left_bottom_lon = tg.sphere_greatcircle_intersect(inner_left_bottom_lat,
                                                                                   inner_left_bottom_lon,
                                                                                   ext_left_bottom_lat,
                                                                                   ext_left_bottom_lon,
                                                                                   CENTER_LAT,
                                                                                   left_outer_lon,
                                                                                   vert_left_bottom_lat,
                                                                                   vert_left_bottom_lon)
        
        left_coords = "{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}"\
                                                .format(inner_left_top_lat, inner_left_top_lon,
                                                        outer_left_top_lat, outer_left_top_lon,
                                                        outer_left_bottom_lat, outer_left_bottom_lon,
                                                        inner_left_bottom_lat, inner_left_bottom_lon)
        scnfile.write(zero_time_str + "RAA RAA1,ON,{},{},{}\n".format(0, 0, left_coords))
        scnfile.write(zero_time_str + "DEFWPT RAA_1,{},{},FIX\n".format(CENTER_LAT, (inner_left_bottom_lon + left_outer_lon) / 2))

        # Calculate all coordinate values
        # Area on right side of corridor
        #
        inner_right_top_lat, _ = bsgeo.qdrpos(CENTER_LAT, CENTER_LON, 0, CORRIDOR_LENGTH)
        _, inner_right_top_lon = bsgeo.qdrpos(CENTER_LAT, CENTER_LON, 90, CORRIDOR_WIDTH)

        inner_right_bottom_lat, _ = bsgeo.qdrpos(CENTER_LAT, CENTER_LON, 180, CORRIDOR_LENGTH)
        _, inner_right_bottom_lon = bsgeo.qdrpos(CENTER_LAT, CENTER_LON, 90, CORRIDOR_WIDTH)

        ext_dist = AREA_RADIUS / math.sin(math.radians(RESTRICTION_ANGLE))
        ext_right_top_lat, ext_right_top_lon = bsgeo.qdrpos(inner_right_top_lat, inner_right_top_lon, RESTRICTION_ANGLE, ext_dist)
        
        vert_dist = ext_dist * 2
        _, right_outer_lon = bsgeo.qdrpos(CENTER_LAT, CENTER_LON, 90, AREA_RADIUS)
        vert_right_top_lat, vert_right_top_lon = bsgeo.qdrpos(CENTER_LAT, right_outer_lon, 360, vert_dist)
        
        outer_right_top_lat, outer_right_top_lon = tg.sphere_greatcircle_intersect(inner_right_top_lat,
                                                                                   inner_right_top_lon,
                                                                                   ext_right_top_lat,
                                                                                   ext_right_top_lon,
                                                                                   CENTER_LAT,
                                                                                   right_outer_lon,
                                                                                   vert_right_top_lat,
                                                                                   vert_right_top_lon)
        
        ext_dist = AREA_RADIUS / math.sin(math.radians(RESTRICTION_ANGLE))
        ext_right_bottom_lat, ext_right_bottom_lon = bsgeo.qdrpos(inner_right_bottom_lat, inner_right_bottom_lon, 180 - RESTRICTION_ANGLE, ext_dist)
        
        vert_dist = ext_dist * 2
        _, right_outer_lon = bsgeo.qdrpos(CENTER_LAT, CENTER_LON, 90, AREA_RADIUS)
        vert_right_bottom_lat, vert_right_bottom_lon = bsgeo.qdrpos(CENTER_LAT, right_outer_lon, 180, vert_dist)
        
        outer_right_bottom_lat, outer_right_bottom_lon = tg.sphere_greatcircle_intersect(inner_right_bottom_lat,
                                                                                   inner_right_bottom_lon,
                                                                                   ext_right_bottom_lat,
                                                                                   ext_right_bottom_lon,
                                                                                   CENTER_LAT,
                                                                                   right_outer_lon,
                                                                                   vert_right_bottom_lat,
                                                                                   vert_right_bottom_lon)
        
        right_coords = "{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}"\
                                                .format(inner_right_top_lat, inner_right_top_lon,
                                                        outer_right_top_lat, outer_right_top_lon,
                                                        outer_right_bottom_lat, outer_right_bottom_lon,
                                                        inner_right_bottom_lat, inner_right_bottom_lon)
        scnfile.write(zero_time_str + "RAA RAA2,ON,{},{},{}\n".format(0, 0, right_coords))
        scnfile.write(zero_time_str + "DEFWPT RAA_2,{},{},FIX\n".format(CENTER_LAT, (inner_right_bottom_lon + right_outer_lon) / 2))


        scnfile.write("\n# Create waypoints")
        # for area in area_list:
        #     scnfile.write(zero_time_str + "RAA {},ON,{},{},{}".format(area.name,
        #                                                               area.gseast,
        #                                                               area.gsnorth,
        #                                                               coords))

# def create_waypoint_defs():
#     # Corridor
#     CENTER_LAT, inner lat
#     CETNER_LON, -inner lat

#     # Deps
#     max angle
#     on arc dist AREA_RADIUS from CENTER_LAT,CENTER_LON between -angle_max and angle_max

#     # Dests
#     mirroring Deps around CENTER_LAT, CENTER_LON

# def create_restrictions():
#     # Positive directions
#     inner lat = CENTER_LAT,CENTER_LON + CORRIDOR_LENGTH to north
#     inner lon = CENTER_LAT,CENTER_LON + CORRIDOR_WIDTH to east
#     outer lon = CENTER_LAT,CENTER_LON + AREA_RADIUS to east
#     outer lat =
#         intersection of lines
#             (inner lat, inner lon) + RESTRICTION_ANGLE to east
#             and meridian through outer lon

# def point_angles():
#     angle_diff = RESTRICTION_ANGLE / NUM_DEP_DEST_POINTS

#     return 

if __name__ == "__main__":
    main()
