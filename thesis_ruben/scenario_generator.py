"""
This module creates the scenario files necessary for the experiments
involving the area_restriction.py module.

© Ruben Jacobse, 2019
"""

# Python imports
import math
import sys
import os
import time
import random

# Third-party imports
import numpy as np

# Enable BlueSky imports by adding the project folder to the path
sys.path.append(os.path.abspath(os.path.join('..')))
sys.path.append(os.path.abspath(os.path.join('../plugins')))

# BlueSky imports
import plugins.area_restriction as ar
import bluesky.tools.geo as bsgeo
import plugins.tempgeo as tg

# Using these as constants for now, will probably become
# function arguments later.
CENTER_LAT = 0.0  # [deg]
CENTER_LON = 0.0  # [deg]
CORRIDOR_LENGTH = 50  # [NM]
CORRIDOR_WIDTH = 20  # [NM]
AREA_RADIUS = 100  # [NM]
EXPERIMENT_RADIUS = 75  # [NM]
RESTRICTION_ANGLE = 60  # [deg]
NUM_DEP_DEST_POINTS = 10  # Number of departure and destination points

PLUGIN_NAME = "RAA"
SIM_TIME_STEP = 0.05  # [s] Simulation time step
SHOW_AC_TRAILS = True
ASAS_ON = True
ASAS_RESO_METHOD = "MVP"
NUM_EXPERIMENT_AIRCRAFT = 100
RMETHH = "BOTH"
LOOKAHEAD_TIME = 120

random.seed(1)


def main():
    """
    Create a scenario file which name reflects the properties of the
    experiment geometry.
    """

    file_name = "C{}_{}_L{}_W{}_AR{}_A{}_NP{}_D-{}_R-{}.scn"\
        .format(CENTER_LAT,
                CENTER_LON,
                CORRIDOR_LENGTH,
                CORRIDOR_WIDTH,
                AREA_RADIUS,
                RESTRICTION_ANGLE,
                NUM_DEP_DEST_POINTS,
                "ON" if ASAS_ON else "OFF",
                ASAS_RESO_METHOD)

    # Open file, overwrite if existing
    with open(file_name, "w+") as scnfile:
        zero_time = "0:00:00.00>"

        scnfile.write("# Sim commands\n")
        scnfile.write(zero_time + "PAN {},{}\n".format(CENTER_LAT,
                                                       CENTER_LON))
        scnfile.write(zero_time + "DT {}\n".format(SIM_TIME_STEP))
        scnfile.write(zero_time + "TRAIL {}\n".format(
            "ON" if SHOW_AC_TRAILS else "OFF"))
        scnfile.write(zero_time + "SWRAD {}\n".format("SYM"))
        scnfile.write(zero_time + "SWRAD {}\n".format("LABEL"))
        scnfile.write(zero_time + "SWRAD {}\n".format("WPT"))
        scnfile.write(zero_time + "FF\n")

        scnfile.write("\n# Setup circular experiment area and activate it"
                      + " as a traffic area in BlueSky\n")
        scnfile.write(zero_time + "PLUGINS LOAD AREA\n")
        scnfile.write(zero_time + "CIRCLE EXPERIMENT {},{},{}\n"
                      .format(CENTER_LAT, CENTER_LON, AREA_RADIUS))
        scnfile.write(zero_time + "AREA EXPERIMENT\n")
        scnfile.write(zero_time + "COLOR EXPERIMENT 0,128,0\n")

        scnfile.write("\n# Setup BlueSky ASAS module options\n")
        scnfile.write(zero_time + "ASAS {}\n".format("ON" if ASAS_ON else "OFF"))
        scnfile.write(zero_time + "RESO {}\n".format(ASAS_RESO_METHOD))
        scnfile.write(zero_time + "RMETHH {}\n".format(RMETHH))

        scnfile.write("\n# LOAD RAA plugin and create area restrictions\n")
        scnfile.write(zero_time + "PLUGINS LOAD {}\n".format(PLUGIN_NAME))
        scnfile.write(zero_time + "RAACONF {}\n".format(LOOKAHEAD_TIME))

        # Area on left side of corridor
        inner_left_top_lat, _ = bsgeo.qdrpos(CENTER_LAT, CENTER_LON,
                                             0, CORRIDOR_LENGTH / 2)
        _, inner_left_top_lon = bsgeo.qdrpos(CENTER_LAT, CENTER_LON,
                                             270, CORRIDOR_WIDTH / 2)

        inner_left_bottom_lat, _ = bsgeo.qdrpos(CENTER_LAT, CENTER_LON,
                                                180, CORRIDOR_LENGTH / 2)
        _, inner_left_bottom_lon = bsgeo.qdrpos(CENTER_LAT, CENTER_LON,
                                                270, CORRIDOR_WIDTH / 2)

        ext_dist = AREA_RADIUS / math.sin(math.radians(RESTRICTION_ANGLE))
        ext_left_top_lat, ext_left_top_lon = bsgeo.qdrpos(inner_left_top_lat,
                                                          inner_left_top_lon,
                                                          360 - RESTRICTION_ANGLE,
                                                          ext_dist)

        vert_dist = ext_dist * 2
        _, left_outer_lon = bsgeo.qdrpos(CENTER_LAT, CENTER_LON, 270, AREA_RADIUS)
        vert_left_top_lat, vert_left_top_lon = bsgeo.qdrpos(CENTER_LAT,
                                                            left_outer_lon,
                                                            360,
                                                            vert_dist)

        outer_left_top_lat, outer_left_top_lon = \
            tg.sphere_greatcircle_intersect(inner_left_top_lat,
                                            inner_left_top_lon,
                                            ext_left_top_lat,
                                            ext_left_top_lon,
                                            CENTER_LAT,
                                            left_outer_lon,
                                            vert_left_top_lat,
                                            vert_left_top_lon)

        ext_dist = AREA_RADIUS / math.sin(math.radians(RESTRICTION_ANGLE))
        ext_left_bottom_lat, ext_left_bottom_lon = \
            bsgeo.qdrpos(inner_left_bottom_lat, inner_left_bottom_lon,
                         180 + RESTRICTION_ANGLE, ext_dist)

        vert_dist = ext_dist * 2
        _, left_outer_lon = bsgeo.qdrpos(CENTER_LAT, CENTER_LON,
                                         270, AREA_RADIUS)
        vert_left_bottom_lat, vert_left_bottom_lon = \
            bsgeo.qdrpos(CENTER_LAT, left_outer_lon, 180, vert_dist)

        outer_left_bottom_lat, outer_left_bottom_lon = \
            tg.sphere_greatcircle_intersect(inner_left_bottom_lat,
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
        scnfile.write(zero_time + "RAA RAA1,ON,{},{},{}\n".format(0, 0, left_coords))
        scnfile.write(zero_time + "COLOR RAA1,164,0,0\n")
        scnfile.write(zero_time + "DEFWPT RAA_1,{:.6f},{:.6f},FIX\n".format(
            CENTER_LAT, (inner_left_bottom_lon + left_outer_lon) / 2))

        # Area on right side of corridor
        inner_right_top_lat, _ = bsgeo.qdrpos(CENTER_LAT, CENTER_LON,
                                              0, CORRIDOR_LENGTH / 2)
        _, inner_right_top_lon = bsgeo.qdrpos(CENTER_LAT, CENTER_LON,
                                              90, CORRIDOR_WIDTH / 2)

        inner_right_bottom_lat, _ = bsgeo.qdrpos(CENTER_LAT, CENTER_LON,
                                                 180, CORRIDOR_LENGTH / 2)
        _, inner_right_bottom_lon = bsgeo.qdrpos(CENTER_LAT, CENTER_LON,
                                                 90, CORRIDOR_WIDTH / 2)

        ext_dist = AREA_RADIUS / math.sin(math.radians(RESTRICTION_ANGLE))
        ext_right_top_lat, ext_right_top_lon = bsgeo.qdrpos(
            inner_right_top_lat, inner_right_top_lon, RESTRICTION_ANGLE, ext_dist)

        vert_dist = ext_dist * 2
        _, right_outer_lon = bsgeo.qdrpos(CENTER_LAT, CENTER_LON, 90, AREA_RADIUS)
        vert_right_top_lat, vert_right_top_lon = bsgeo.qdrpos(
            CENTER_LAT, right_outer_lon, 360, vert_dist)

        outer_right_top_lat, outer_right_top_lon = \
            tg.sphere_greatcircle_intersect(inner_right_top_lat,
                                            inner_right_top_lon,
                                            ext_right_top_lat,
                                            ext_right_top_lon,
                                            CENTER_LAT,
                                            right_outer_lon,
                                            vert_right_top_lat,
                                            vert_right_top_lon)

        ext_dist = AREA_RADIUS / math.sin(math.radians(RESTRICTION_ANGLE))
        ext_right_bottom_lat, ext_right_bottom_lon = bsgeo.qdrpos(
            inner_right_bottom_lat, inner_right_bottom_lon,
            180 - RESTRICTION_ANGLE, ext_dist)

        vert_dist = ext_dist * 2
        _, right_outer_lon = bsgeo.qdrpos(CENTER_LAT, CENTER_LON, 90, AREA_RADIUS)
        vert_right_bottom_lat, vert_right_bottom_lon = bsgeo.qdrpos(
            CENTER_LAT, right_outer_lon, 180, vert_dist)

        outer_right_bottom_lat, outer_right_bottom_lon = \
            tg.sphere_greatcircle_intersect(inner_right_bottom_lat,
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
        scnfile.write(zero_time + "RAA RAA2,ON,{},{},{}\n".format(0, 0, right_coords))
        scnfile.write(zero_time + "COLOR RAA2,164,0,0\n")
        scnfile.write(zero_time + "DEFWPT RAA_2,{:.6f},{:.6f},FIX\n".format(
            CENTER_LAT, (inner_right_bottom_lon + right_outer_lon) / 2))

        # First find angle to intersection point of angled restriction
        # edge and experiment area then find angle from center point
        _, _, angle = \
            calculate_line_ring_intersection(CENTER_LAT,
                                             CENTER_LON,
                                             AREA_RADIUS,
                                             inner_right_top_lat,
                                             inner_right_top_lon,
                                             outer_right_top_lat,
                                             outer_right_top_lon)

        # Departure waypoints
        scnfile.write("\n# Departure waypoints\n")
        dep_waypoints = calc_departure_waypoints(
            NUM_DEP_DEST_POINTS, CENTER_LAT, CENTER_LON, AREA_RADIUS, angle)

        for fix_idx, dep_fix in enumerate(dep_waypoints):
            fix_lat, fix_lon = dep_fix[0], dep_fix[1]
            scnfile.write(zero_time + "DEFWPT DEP{:03d},{:.6f},{:.6f},FIX\n".format(
                fix_idx, fix_lat, fix_lon))

        # Destination waypoints
        scnfile.write("\n# Destination waypoints\n")
        dest_waypoints = calc_destination_waypoints(
            NUM_DEP_DEST_POINTS, CENTER_LAT, CENTER_LON, AREA_RADIUS, angle)

        for fix_idx, dep_fix in enumerate(dest_waypoints):
            fix_lat, fix_lon = dep_fix[0], dep_fix[1]
            scnfile.write(zero_time + "DEFWPT DST{:03d},{:.6f},{:.6f},FIX\n".format(
                fix_idx, fix_lat, fix_lon))

        # Corridor waypoints
        scnfile.write("\n# Corridor waypoints\n")
        scnfile.write(zero_time + "DEFWPT COR101,{:.6f},{:.6f},FIX\n".format(
            inner_left_bottom_lat, CENTER_LON))
        scnfile.write(zero_time + "DEFWPT COR201,{:.6f},{:.6f},FIX\n".format(
            inner_left_top_lat, CENTER_LON))

        # Create aircaft
        scnfile.write("\n# Create aircraft\n")

        ac_spd = 280  # [kts] Speed

        # Minimum distance and time differences at creation
        min_dist_diff = 6  # [nm]
        min_time_diff = min_dist_diff / 280 * 3600  # [s]

        # Store parameters of created aircraft
        creation_time = []
        creation_pos = []
        creation_hdg = []
        creation_dest = []
        creation_dest_idx = []

        num_created_ac = 0
        while num_created_ac < NUM_EXPERIMENT_AIRCRAFT:
            # Will be set True if creation results in a conflict
            conflict = False

            # Create an aircraft at random time and position
            prev_time = creation_time[-1] if num_created_ac else 0
            curr_time = prev_time + random.randint(30, 60)

            curr_dep_wp_idx = random.randint(0, NUM_DEP_DEST_POINTS - 1)
            (curr_lat, curr_lon) = dep_waypoints[curr_dep_wp_idx]
            curr_dest_wp_idx = random.randint(0, NUM_DEP_DEST_POINTS - 1)
            (dest_lat, dest_lon) = dest_waypoints[curr_dest_wp_idx]

            # Heading to waypoint at start of corridor
            curr_hdg, _ = bsgeo.qdrdist(curr_lat, curr_lon,
                                        inner_left_bottom_lat, CENTER_LON)

            # First generated aircraft is always accepted. Each aircraft
            # thereafter has to have at least minimum separation (in space
            # OR time) with ALL preceeding aircraft at the time of its creation.
            if num_created_ac > 0:
                time_diff_list = [curr_time - t for t in creation_time]
                dist_diff_list = [bsgeo.kwikdist(lat, lon, curr_lat, curr_lon)
                                  for (lat, lon) in creation_pos]

                for time_diff, dist_diff in zip(time_diff_list, dist_diff_list):
                    # Either time OR distance difference must be smaller than minimum
                    if not (((dist_diff < min_dist_diff) and (time_diff > min_time_diff))
                            or ((dist_diff > min_dist_diff) and (time_diff < min_time_diff))
                            or ((dist_diff > min_dist_diff) and (time_diff > min_time_diff))):
                        conflict = True
                        break

                # If the current aircraft is in conflict, continue the while loop
                if conflict:
                    continue

            # Keep track of created aircraft that are not in conflict
            creation_time.append(curr_time)
            creation_pos.append((curr_lat, curr_lon))
            creation_hdg.append(curr_hdg % 360)
            creation_dest.append((dest_lat, dest_lon))
            creation_dest_idx.append(curr_dest_wp_idx)

            num_created_ac += 1

        for ac_idx in range(num_created_ac):
            # Type and altitude are always the same
            ac_type = "B744"
            ac_alt = "36000"

            time_str = time.strftime('%H:%M:%S', time.gmtime(creation_time[ac_idx]))
            time_str = "{}.00>".format(time_str)
            aircraft_str = time_str + "CRE AC{:03d} {},{:.6f},{:.6f},{:.2f},{},{}\n"\
                .format(ac_idx,
                        ac_type,
                        creation_pos[ac_idx][0],
                        creation_pos[ac_idx][1],
                        creation_hdg[ac_idx],
                        ac_alt,
                        ac_spd)

            aircraft_str += time_str + "AC{:03d} ADDWPT COR101\n".format(ac_idx)
            aircraft_str += time_str + "AC{:03d} ADDWPT COR201\n".format(ac_idx)
            aircraft_str += time_str + "AC{:03d} ADDWPT DST{:03d}\n".format(
                ac_idx, creation_dest_idx[ac_idx])

            scnfile.write("\n" + aircraft_str)


def calculate_line_ring_intersection(ring_center_lat,
                                     ring_center_lon,
                                     ring_radius,
                                     line_start_lat,
                                     line_start_lon,
                                     line_end_lat,
                                     line_end_lon):
    """
    Calculate the intersection point and angle from the center coordinate
    to that point between the area radius and the upper angled side of the
    restricted area to the right of the corridor.
    """

    # First guess is midpoint of line
    intersect_lat, intersect_lon = tg.calc_midpoint(line_start_lat, line_start_lon,
                                                    line_end_lat, line_end_lon)
    qdr_from_center, dist_from_center = bsgeo.qdrdist(ring_center_lat, ring_center_lat,
                                                      intersect_lat, intersect_lon)

    while True:
        if abs(dist_from_center - ring_radius) < 0.1:
            break

        if dist_from_center > ring_radius:
            line_end_lat, line_end_lon = intersect_lat, intersect_lon
        elif dist_from_center < ring_radius:
            line_start_lat, line_start_lon = intersect_lat, intersect_lon

        intersect_lat, intersect_lon = tg.calc_midpoint(line_start_lat, line_start_lon,
                                                        line_end_lat, line_end_lon)
        qdr_from_center, dist_from_center = bsgeo.qdrdist(ring_center_lat, ring_center_lon,
                                                          intersect_lat, intersect_lon)

    angle_from_center = ar.ned2crs(qdr_from_center)

    return intersect_lat, intersect_lon, angle_from_center


def calc_departure_waypoints(NUM_DEP_DEST_POINTS,
                             CENTER_LAT,
                             CENTER_LON,
                             AREA_RADIUS,
                             angle):
    """
    Calculate the locations of all departure waypoints on the area edge.
    """

    angles = list(np.linspace(180 - angle + 3, 180 + angle - 3, NUM_DEP_DEST_POINTS))
    waypoints = [bsgeo.qdrpos(CENTER_LAT, CENTER_LON, angle, AREA_RADIUS)
                 for angle in angles]

    return waypoints


def calc_destination_waypoints(NUM_DEP_DEST_POINTS,
                               CENTER_LAT,
                               CENTER_LON,
                               AREA_RADIUS,
                               angle):
    """
    Calculate the locations of all destination waypoints on the area edge.
    """

    angles = list(np.linspace(-angle + 3, angle - 3, NUM_DEP_DEST_POINTS))
    waypoints = [bsgeo.qdrpos(CENTER_LAT, CENTER_LON, angle, AREA_RADIUS)
                 for angle in angles]

    return waypoints


if __name__ == "__main__":
    main()
