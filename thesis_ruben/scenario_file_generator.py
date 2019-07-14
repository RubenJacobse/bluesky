"""
This module creates the scenario files necessary for the experiments
involving the area_restriction.py module.

© Ruben Jacobse, 2019
"""

# Python imports
import os
import sys
import math
import time
import random
import datetime

# Enable BlueSky imports by adding the project folder to the path
sys.path.append(os.path.abspath(os.path.join('..')))
sys.path.append(os.path.abspath(os.path.join('../plugins')))

# BlueSky imports
import bluesky.tools.geo as bsgeo
import plugins.area_restriction as ar
import plugins.tempgeo as tg

# BlueSky simulator settings
SIM_TIME_STEP = 0.1  # [s] Simulation time step
SIM_PAUSE_HOUR = 5  # [h] Pause simulation after this number of hours
SHOW_AC_TRAILS = True  # Plot trails in the radar window

# Area restriction settings
CENTER_LAT = 0.0  # [deg]
CENTER_LON = 0.0  # [deg]
PLUGIN_NAME = "RAA"
AREA_LOOKAHEAD_TIME = 120  # [s] Look-ahead time used to detect area conflicts
RMETHH = "BOTH"  # Horizontal resolution method, allow both spd and hdg changes
AREA_RADIUS = 100  # [NM]

# Aircraft creation settings
NUM_AIRCRAFT = 25
# AC_TYPES = ["A320", "B738", "A333", "B744"]
# AC_TYPES_SPD = [258, 260, 273, 286]
AC_TYPES = ["B744"]
AC_TYPES_SPD = [280]
SPD_STDDEV = 5  # [kts] Standard deviation of cruise speed distributions


def create_scenfile(target_dir,
                    timestamp,
                    random_seed,
                    traffic_level,
                    resolution_method,
                    corridor_length,
                    corridor_width,
                    angle,
                    is_edge_angle=False):
    """
    Create a scenario file which name reflects the properties of the
    experiment geometry.
    """

    # Initialize random number generator before we do anything where
    # random numbers are used.
    random.seed(random_seed)

    # Set the path of the resulting scenario file and geo file
    scnfile_name = (("{}_L{}_W{}_A{}_RESO-{}_T-{}_SCEN{:03d}.scn")
                    .format(timestamp,
                            corridor_length,
                            corridor_width,
                            angle,
                            resolution_method,
                            traffic_level,
                            random_seed))
    scnfile_path = os.path.join(target_dir, scnfile_name)

    geofile_name = (("{}_L{}_W{}_A{}_geo.csv")
                    .format(timestamp,
                            corridor_length,
                            corridor_width,
                            angle))
    geofile_path = os.path.join(target_dir, geofile_name)

    # Set aircraft creation interval minimum and maximum values.
    # (We do this here so we can print this in the scenario header)
    if traffic_level == "LOW":  # Average 72 seconds
        cre_interval_min = 52
        cre_interval_max = 92
    elif traffic_level == "MID":  # Average 36 seconds
        cre_interval_min = 26
        cre_interval_max = 46
    elif traffic_level == "HIGH":  # Average 24 seconds
        cre_interval_min = 14
        cre_interval_max = 34

    # Create a header to simplify traceability of variable values
    scen_header = \
        ("##################################################\n"
         + f"# File created at: {timestamp}\n"
         + f"# Center latitude: {CENTER_LAT} deg\n"
         + f"# Center longitude: {CENTER_LON} deg\n"
         + f"# Corridor length: {corridor_length} NM\n"
         + f"# Corridor width: {corridor_width} NM\n"
         + f"# Angle: {angle} deg "
         + (" (Defined by area edge angle)\n" if is_edge_angle
            else "(Defined by arc angle)\n")
         + f"# Experiment area radius: {AREA_RADIUS} NM\n"
         + f"# Aircraft types: {AC_TYPES}\n"
         + f"# Aircraft average speed: {AC_TYPES_SPD} kts\n"
         + f"# Aircraft speed std dev: {SPD_STDDEV} kts\n"
         + f"# Number of aircraft created: {NUM_AIRCRAFT}\n"
         + f"# Traffic level: {traffic_level}\n"
         + f"# Aircraft creation interval min: {cre_interval_min} s\n"
         + f"# Aircraft creation interval max: {cre_interval_max} s\n"
         + f"# Random number generator seed: {random_seed}\n"
         + "##################################################\n")

    # Create scenario file and geo file, overwrite if existing
    with open(scnfile_path, "w+") as scnfile, open(geofile_path, "w+") as geofile:
        scnfile.write(scen_header)
        zero_time = "\n0:00:00.00>"

        scnfile.write("\n# Sim commands")
        scnfile.write(zero_time + f"PAN {CENTER_LAT},{CENTER_LON}")
        scnfile.write(zero_time + f"DT {SIM_TIME_STEP}")
        scnfile.write(zero_time + "TRAIL {}".format("ON" if SHOW_AC_TRAILS
                                                    else "OFF"))
        scnfile.write(zero_time + "SWRAD SYM")
        scnfile.write(zero_time + "SWRAD LABEL 0")
        scnfile.write(zero_time + "SWRAD WPT")
        scnfile.write(zero_time + "SWRAD SAT")
        scnfile.write(zero_time + "SWRAD APT")
        scnfile.write(zero_time + "FF")

        scnfile.write("\n\n# Setup circular experiment area and activate it"
                      + " as a traffic area in BlueSky")
        scnfile.write(zero_time + "PLUGINS LOAD AREA")
        scnfile.write(zero_time + "CIRCLE EXPERIMENT {},{},{}"
                      .format(CENTER_LAT, CENTER_LON, AREA_RADIUS))
        scnfile.write(zero_time + "AREA EXPERIMENT")
        scnfile.write(zero_time + "COLOR EXPERIMENT 0,128,0")

        scnfile.write("\n\n# Setup BlueSky ASAS module options")
        scnfile.write(zero_time + "ASAS ON")
        scnfile.write(zero_time + f"RESO {resolution_method}")
        scnfile.write(zero_time + f"RMETHH {RMETHH}")

        # Create csv file to allow generation of area restriction graphs
        geofile.write("{},{},{},{}\n".format(
            "ring", CENTER_LAT, CENTER_LON, AREA_RADIUS))

        # Create restricted areas
        scnfile.write("\n\n# LOAD RAA plugin and create area restrictions")
        scnfile.write(zero_time + f"PLUGINS LOAD {PLUGIN_NAME}")
        scnfile.write(zero_time + f"RAALOG {timestamp}")
        scnfile.write(zero_time + f"RAACONF {AREA_LOOKAHEAD_TIME}")

        _ = create_area(scnfile,
                        geofile,
                        zero_time,
                        corridor_width,
                        corridor_length,
                        "LEFT",
                        angle,
                        is_edge_angle)
        angle_to_ring_intersect = create_area(scnfile,
                                              geofile,
                                              zero_time,
                                              corridor_width,
                                              corridor_length,
                                              "RIGHT",
                                              angle,
                                              is_edge_angle)

        # Angle range for traffic creation
        if is_edge_angle:
            angle_from_centerpoint = angle_to_ring_intersect
        else:
            angle_from_centerpoint = angle / 2

        # Corridor waypoints
        scnfile.write("\n\n# Corridor waypoints")
        corridor_top_lat, _ = bsgeo.qdrpos(CENTER_LAT,
                                           CENTER_LON,
                                           0,
                                           corridor_length/2)
        corridor_bottom_lat, _ = bsgeo.qdrpos(CENTER_LAT,
                                              CENTER_LON,
                                              180,
                                              corridor_length/2)

        scnfile.write(zero_time + "DEFWPT COR101,{:.6f},{:.6f},FIX"
                      .format(corridor_bottom_lat, CENTER_LON))
        scnfile.write(zero_time + "DEFWPT COR201,{:.6f},{:.6f},FIX"
                      .format(corridor_top_lat, CENTER_LON))

        # Create aircaft
        scnfile.write("\n\n# Create {} aircraft".format(NUM_AIRCRAFT))
        create_aircraft(scnfile,
                        traffic_level,
                        cre_interval_min,
                        cre_interval_max,
                        NUM_AIRCRAFT,
                        angle_from_centerpoint,
                        corridor_bottom_lat,
                        CENTER_LON)


def create_area(scnfile,
                geofile,
                zero_time,
                corridor_width,
                corridor_length,
                corridor_side,
                angle,
                is_edge_angle=False):
    """
    Create the restricted area on a given corridor side.
    """

    # Area on left side is located at 270 and right side at 90
    # degrees relative to the corridor center.
    east_west_angle = 270 if corridor_side == "LEFT" else 90

    # Calculate coordinates of area edge bordering the corridor
    inner_top_lat, _ = bsgeo.qdrpos(CENTER_LAT,
                                    CENTER_LON,
                                    0,
                                    corridor_length/2)
    _, inner_top_lon = bsgeo.qdrpos(CENTER_LAT,
                                    CENTER_LON,
                                    east_west_angle,
                                    corridor_width/2)
    inner_bottom_lat, _ = bsgeo.qdrpos(CENTER_LAT,
                                       CENTER_LON,
                                       180,
                                       corridor_length/2)
    _, inner_bottom_lon = bsgeo.qdrpos(CENTER_LAT,
                                       CENTER_LON,
                                       east_west_angle,
                                       corridor_width/2)

    # Determine the angle of the area edge, either by using the
    # input edge angle or by calculating the intersection of the
    # area edge with the ring
    if is_edge_angle:
        edge_angle = (angle if corridor_side == "RIGHT" else -angle)
    else:
        arc_angle = (angle/2 if corridor_side == "RIGHT" else -angle/2)
        arc_ext_lat, arc_ext_lon = bsgeo.qdrpos(CENTER_LAT,
                                                CENTER_LON,
                                                arc_angle,
                                                AREA_RADIUS*1.1)
        arc_point_lat, arc_point_lon, _ \
            = calc_line_ring_intersection(CENTER_LAT,
                                          CENTER_LON,
                                          AREA_RADIUS,
                                          CENTER_LAT,
                                          CENTER_LON,
                                          arc_ext_lat,
                                          arc_ext_lon)
        edge_angle, _ = bsgeo.qdrdist(inner_top_lat,
                                      inner_top_lon,
                                      arc_point_lat,
                                      arc_point_lon)

    # Calculate coordinates of points on the extended angled edges
    ext_dist = abs(2 * AREA_RADIUS / math.sin(math.radians(edge_angle)))
    ext_top_angle = edge_angle
    ext_bottom_angle = 180 - edge_angle
    ext_top_lat, ext_top_lon = bsgeo.qdrpos(inner_top_lat,
                                            inner_top_lon,
                                            ext_top_angle,
                                            ext_dist)
    ext_bottom_lat, ext_bottom_lon = bsgeo.qdrpos(inner_bottom_lat,
                                                  inner_bottom_lon,
                                                  ext_bottom_angle,
                                                  ext_dist)

    # Calculate coordinates of points on north and south lines extending the
    # edge furthest from the corridor
    vert_dist = ext_dist * 2
    _, outer_lon = bsgeo.qdrpos(CENTER_LAT,
                                CENTER_LON,
                                east_west_angle,
                                AREA_RADIUS)
    vert_top_lat, vert_top_lon = bsgeo.qdrpos(CENTER_LAT,
                                              outer_lon,
                                              360,
                                              vert_dist)
    vert_bottom_lat, vert_bottom_lon = bsgeo.qdrpos(CENTER_LAT,
                                                    outer_lon,
                                                    180,
                                                    vert_dist)

    # Calculate the coordinates of the edge furthest from the corridor by
    # intersecting the extended lines with the vertical lines
    outer_top_lat, outer_top_lon = \
        tg.sphere_greatcircle_intersect(inner_top_lat,
                                        inner_top_lon,
                                        ext_top_lat,
                                        ext_top_lon,
                                        CENTER_LAT,
                                        outer_lon,
                                        vert_top_lat,
                                        vert_top_lon)
    outer_bottom_lat, outer_bottom_lon = \
        tg.sphere_greatcircle_intersect(inner_bottom_lat,
                                        inner_bottom_lon,
                                        ext_bottom_lat,
                                        ext_bottom_lon,
                                        CENTER_LAT,
                                        outer_lon,
                                        vert_bottom_lat,
                                        vert_bottom_lon)

    # Write the area and waypoint with area name to the scenario file
    area_idx = 1 if corridor_side == "LEFT" else 2
    area_coords = "{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}"\
        .format(inner_top_lat,
                inner_top_lon,
                outer_top_lat,
                outer_top_lon,
                outer_bottom_lat,
                outer_bottom_lon,
                inner_bottom_lat,
                inner_bottom_lon)
    scnfile.write(zero_time + f"RAA RAA{area_idx},ON,{0},{0},{area_coords}")
    scnfile.write(zero_time + f"COLOR RAA{area_idx},164,0,0")
    scnfile.write(zero_time + "DEFWPT RAA_{},{:.6f},{:.6f},FIX".format(
        area_idx, CENTER_LAT, (inner_bottom_lon + outer_lon) / 2))

    # Write coordinates to geo file
    geofile.write(f"RAA{area_idx},{area_coords}")

    # Calculate angle from the center point to intersection between ring and
    # area edge
    _, _, top_angle_from_center = calc_line_ring_intersection(CENTER_LAT,
                                                              CENTER_LON,
                                                              AREA_RADIUS,
                                                              inner_top_lat,
                                                              inner_top_lon,
                                                              outer_top_lat,
                                                              outer_top_lon)
    return top_angle_from_center


def create_aircraft(scnfile,
                    traffic_level,
                    cre_interval_min,
                    cre_interval_max,
                    num_total_ac,
                    angle,
                    corridor_entry_lat,
                    corridor_entry_lon):
    """
    Create all aircraft
    """

    ac_spd = 280  # [kts] Speed

    # Minimum distance and time differences at creation
    min_dist_diff = 6  # [nm]
    min_time_diff = min_dist_diff / ac_spd * 3600  # [s]

    # Store parameters of all created aircraft
    creation_time = []
    creation_type = []
    creation_spd = []
    creation_dep_lat = []
    creation_dep_lon = []
    creation_dest_lat = []
    creation_dest_lon = []
    creation_hdg = []

    num_created_ac = 0
    while num_created_ac < num_total_ac:
        # Will be set True if creation results in a conflict
        in_conflict_at_creation = False

        # Create an aircraft at random time and position
        prev_time = creation_time[-1] if num_created_ac else 0
        curr_time = prev_time + random.randint(cre_interval_min,
                                               cre_interval_max)

        # Select an aircraft type and velocity
        type_idx = random.randint(0, len(AC_TYPES) - 1)
        curr_type = AC_TYPES[type_idx]
        curr_spd = random.gauss(AC_TYPES_SPD[type_idx], SPD_STDDEV)

        # Select random departure and destination waypoint
        curr_dep_angle = random.uniform(180 - angle + 3, 180 + angle - 3)
        (curr_lat, curr_lon) = bsgeo.qdrpos(CENTER_LAT,
                                            CENTER_LON,
                                            curr_dep_angle,
                                            AREA_RADIUS)
        curr_dest_angle = random.uniform(-angle + 3, angle - 3)
        (dest_lat, dest_lon) = bsgeo.qdrpos(CENTER_LAT,
                                            CENTER_LON,
                                            curr_dest_angle,
                                            AREA_RADIUS + 0.5)

        # Heading to waypoint at start of corridor
        curr_hdg, _ = bsgeo.qdrdist(curr_lat,
                                    curr_lon,
                                    corridor_entry_lat,
                                    corridor_entry_lon)

        # The first generated aircraft is always accepted. Each aircraft
        # thereafter has to have at least minimum separation in space
        # OR time with ALL existing aircraft at the time of its creation.
        if num_created_ac:
            time_diff_list = [curr_time - t for t in creation_time]
            dist_diff_list = \
                [bsgeo.kwikdist(lat, lon, curr_lat, curr_lon)
                 for (lat, lon) in zip(creation_dep_lat, creation_dep_lon)]

            for time_diff, dist_diff in zip(time_diff_list, dist_diff_list):
                # Either time OR distance difference must be smaller than minimum
                if not (((dist_diff < min_dist_diff) and (time_diff > min_time_diff))
                        or ((dist_diff > min_dist_diff) and (time_diff < min_time_diff))
                        or ((dist_diff > min_dist_diff) and (time_diff > min_time_diff))):
                    in_conflict_at_creation = True
                    break

            # If the current aircraft is in conflict, continue the while loop
            # and try another random creation.
            if in_conflict_at_creation:
                continue

        # Keep track of created aircraft that are not in conflict
        creation_time.append(curr_time)
        creation_type.append(curr_type)
        creation_spd.append(curr_spd)
        creation_dep_lat.append(curr_lat)
        creation_dep_lon.append(curr_lon)
        creation_hdg.append(curr_hdg % 360)
        creation_dest_lat.append(dest_lat)
        creation_dest_lon.append(dest_lon)
        num_created_ac += 1

    # Write each aircraft to file
    for ac_idx in range(num_created_ac):
        # Altitude is the same for all aircraft
        ac_alt = "36000"

        time_str = time.strftime('%H:%M:%S',
                                 time.gmtime(creation_time[ac_idx]))
        time_str = "{}.00>".format(time_str)
        aircraft_str = (time_str
                        + "CRE AC{:03d} {},{:.6f},{:.6f},{:.2f},{},{:.0f}\n"
                        .format(ac_idx,
                                creation_type[ac_idx],
                                creation_dep_lat[ac_idx],
                                creation_dep_lon[ac_idx],
                                creation_hdg[ac_idx],
                                ac_alt,
                                creation_spd[ac_idx]))

        aircraft_str += time_str + ("DEFWPT DEP{:03d},{:06f},{:06f},FIX\n"
                                    .format(ac_idx,
                                            creation_dep_lat[ac_idx],
                                            creation_dep_lon[ac_idx]))
        aircraft_str += time_str + ("DEFWPT DST{:03d},{:06f},{:06f},FIX\n"
                                    .format(ac_idx,
                                            creation_dest_lat[ac_idx],
                                            creation_dest_lon[ac_idx]))
        aircraft_str += time_str + ("AC{:03d} ADDWPT DEP{:03d}\n"
                                    .format(ac_idx, ac_idx))
        aircraft_str += time_str + "AC{:03d} ADDWPT COR101\n".format(ac_idx)
        aircraft_str += time_str + "AC{:03d} ADDWPT COR201\n".format(ac_idx)
        aircraft_str += time_str + ("AC{:03d} ADDWPT DST{:03d}\n"
                                    .format(ac_idx, ac_idx))

        scnfile.write("\n" + aircraft_str)


def calc_line_ring_intersection(ring_center_lat,
                                ring_center_lon,
                                ring_radius,
                                line_start_lat,
                                line_start_lon,
                                line_end_lat,
                                line_end_lon):
    """
    For a ring defined by a center latitude, longitude, and radius calculate
    the latitude and longitude of the intersection point of this ring with a
    line defined by a start and end point. Also calculate the angle to that
    intersection point from the ring center.

    Returns:
        lat [deg], lon [deg], angle [deg]
    """

    # Use recursive bisection until the intersection point is within
    # distance margin from the ring edge
    while True:
        intersect_lat, intersect_lon = tg.calc_midpoint(line_start_lat,
                                                        line_start_lon,
                                                        line_end_lat,
                                                        line_end_lon)
        qdr_from_center, dist_from_center = bsgeo.qdrdist(ring_center_lat,
                                                          ring_center_lon,
                                                          intersect_lat,
                                                          intersect_lon)

        if abs(dist_from_center - ring_radius) < 0.1:
            break

        if dist_from_center > ring_radius:
            line_end_lat, line_end_lon = intersect_lat, intersect_lon
        elif dist_from_center < ring_radius:
            line_start_lat, line_start_lon = intersect_lat, intersect_lon

    angle_from_center = ar.ned2crs(qdr_from_center)

    return intersect_lat, intersect_lon, angle_from_center


if __name__ == "__main__":
    # Passing these as arguments to create() when called as __main__
    SEED = 1
    TRAFFIC_LEVEL = "LOW"
    CORRIDOR_LENGTH = 40  # [NM]
    CORRIDOR_WIDTH = 25  # [NM]
    ARC_ANGLE = 90  # [deg]

    # When running as main, generate a file for each relevant resolution
    # method using the default values of the other parameters.
    current_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    test_folder = "scengen_test"

    for reso_method in ["OFF", "MVP", "LF", "SWARM-V2"]:
        create_scenfile(test_folder,
                        current_time,
                        SEED,
                        TRAFFIC_LEVEL,
                        reso_method,
                        CORRIDOR_LENGTH,
                        CORRIDOR_WIDTH,
                        ARC_ANGLE,
                        is_edge_angle=False)
