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
import datetime

# Third-party imports
import numpy as np

# Enable BlueSky imports by adding the project folder to the path
sys.path.append(os.path.abspath(os.path.join('..')))
sys.path.append(os.path.abspath(os.path.join('../plugins')))

# BlueSky imports
import plugins.area_restriction as ar
import bluesky.tools.geo as bsgeo
import plugins.tempgeo as tg

# Module level constants
CENTER_LAT = 0.0  # [deg]
CENTER_LON = 0.0  # [deg]
PLUGIN_NAME = "RAA"
AREA_LOOKAHEAD_TIME = 120
SIM_TIME_STEP = 0.05  # [s] Simulation time step
SIM_PAUSE_HOUR = 3  # [h] Pause simulation after this time
RMETHH = "BOTH"  # Horizontal resolution method, allow both spd and hdg changes
SHOW_AC_TRAILS = True
AREA_RADIUS = 100  # [NM]
EXPERIMENT_RADIUS = 75  # [NM]
NUM_DEP_DEST_POINTS = 10  # Number of departure and destination points
NUM_EXPERIMENT_AIRCRAFT = 100
# AC_TYPES = ["B744", "A320", "B738", "A343"]
# AC_TYPES_SPD = [286, 258, 260, 273]
AC_TYPES = ["B744"]
AC_TYPES_SPD = [280]
SPD_STDDEV = 5  # [kts]
AC_CRE_INTERVAL_MIN = 30  # [s] Minimum interval between creation of aircraft
AC_CRE_INTERVAL_MAX = 90  # [s] Maximum interval between creation of aircraft

# Passing these as arguments to create() when called as __main__
SEED = 1
CORRIDOR_LENGTH = 40  # [NM]
CORRIDOR_WIDTH = 25  # [NM]
RESTRICTION_ANGLE = 45  # [deg]
ARC_ANGLE = 90  # [deg]
RESO_METHOD = "MVP"  # Conflict resolution method


def create(creation_time,
           random_seed,
           asas_reso_method,
           corridor_length,
           corridor_width,
           angle,
           is_restriction_angle=False):
    """
    Create a scenario file which name reflects the properties of the
    experiment geometry.
    """

    random.seed(random_seed)

    # Use different file name format for restriction angle source
    if is_restriction_angle:
        file_name = "{}_SCEN_{}_C{}_{}_L{}_W{}_R{}_ANG{}_NP{}_RESO-{}.scn"\
            .format(creation_time,
                    SEED,
                    CENTER_LAT,
                    CENTER_LON,
                    corridor_length,
                    corridor_width,
                    AREA_RADIUS,
                    angle,
                    NUM_DEP_DEST_POINTS,
                    asas_reso_method)
    else:
        file_name = "{}_SCEN_{}_C{}_{}_L{}_W{}_R{}_ARC{}_NP{}_RESO-{}.scn"\
            .format(creation_time,
                    SEED,
                    CENTER_LAT,
                    CENTER_LON,
                    corridor_length,
                    corridor_width,
                    AREA_RADIUS,
                    angle,
                    NUM_DEP_DEST_POINTS,
                    asas_reso_method)

    # Open file, overwrite if existing
    with open(file_name, "w+") as scnfile:
        zero_time = "0:00:00.00>"

        scnfile.write("# Sim commands\n")
        scnfile.write(zero_time + "PAN {},{}\n".format(CENTER_LAT, CENTER_LON))
        scnfile.write(zero_time + "DT {}\n".format(SIM_TIME_STEP))
        scnfile.write(zero_time + "TRAIL {}\n".format("ON" if SHOW_AC_TRAILS
                                                      else "OFF"))
        scnfile.write(zero_time + "SWRAD SYM\n")
        scnfile.write(zero_time + "SWRAD LABEL 1\n")
        scnfile.write(zero_time + "SWRAD WPT\n")
        scnfile.write(zero_time + "SWRAD SAT\n")
        scnfile.write(zero_time + "SWRAD APT\n")
        scnfile.write(zero_time + "FF\n")
        scnfile.write("{}:00:00.00>PAUSE\n".format(SIM_PAUSE_HOUR))

        scnfile.write("\n# Setup circular experiment area and activate it"
                      + " as a traffic area in BlueSky\n")
        scnfile.write(zero_time + "PLUGINS LOAD AREA\n")
        scnfile.write(zero_time + "CIRCLE EXPERIMENT {},{},{}\n"
                      .format(CENTER_LAT, CENTER_LON, AREA_RADIUS))
        scnfile.write(zero_time + "AREA EXPERIMENT\n")
        scnfile.write(zero_time + "COLOR EXPERIMENT 0,128,0\n")

        scnfile.write("\n# Setup BlueSky ASAS module options\n")
        scnfile.write(zero_time + "ASAS ON\n")
        scnfile.write(zero_time + "RESO {}\n".format(asas_reso_method))
        scnfile.write(zero_time + "RMETHH {}\n".format(RMETHH))

        # Create restricted areas
        scnfile.write("\n# LOAD RAA plugin and create area restrictions\n")
        scnfile.write(zero_time + "PLUGINS LOAD {}\n".format(PLUGIN_NAME))
        scnfile.write(zero_time + "RAACONF {}\n".format(AREA_LOOKAHEAD_TIME))

        _ = create_area(scnfile,
                        zero_time,
                        corridor_width,
                        corridor_length,
                        "LEFT",
                        angle,
                        is_restriction_angle)
        angle_to_ring_intersect = create_area(scnfile,
                                              zero_time,
                                              corridor_width,
                                              corridor_length,
                                              "RIGHT",
                                              angle,
                                              is_restriction_angle)

        # Angle range for traffic creation
        if is_restriction_angle:
            angle_from_centerpoint = angle_to_ring_intersect
        else:
            angle_from_centerpoint = angle / 2

        # Departure waypoints
        scnfile.write("\n# Departure waypoints\n")
        dep_waypoints = calc_departure_waypoints(NUM_DEP_DEST_POINTS,
                                                 CENTER_LAT,
                                                 CENTER_LON,
                                                 AREA_RADIUS,
                                                 angle_from_centerpoint)

        for fix_idx, (fix_lat, fix_lon) in enumerate(dep_waypoints):
            scnfile.write(zero_time + "DEFWPT DEP{:03d},{:.6f},{:.6f},FIX\n"
                          .format(fix_idx, fix_lat, fix_lon))

        # Destination waypoints
        scnfile.write("\n# Destination waypoints\n")
        dest_waypoints = calc_destination_waypoints(NUM_DEP_DEST_POINTS,
                                                    CENTER_LAT,
                                                    CENTER_LON,
                                                    AREA_RADIUS,
                                                    angle_from_centerpoint)

        for fix_idx, (fix_lat, fix_lon) in enumerate(dest_waypoints):
            scnfile.write(zero_time + "DEFWPT DST{:03d},{:.6f},{:.6f},FIX\n"
                          .format(fix_idx, fix_lat, fix_lon))

        # Corridor waypoints
        scnfile.write("\n# Corridor waypoints\n")
        corridor_top_lat, _ = bsgeo.qdrpos(CENTER_LAT,
                                           CENTER_LON,
                                           0,
                                           corridor_length/2)
        corridor_bottom_lat, _ = bsgeo.qdrpos(CENTER_LAT,
                                              CENTER_LON,
                                              180,
                                              corridor_length/2)

        scnfile.write(zero_time + "DEFWPT COR101,{:.6f},{:.6f},FIX\n"
                      .format(corridor_bottom_lat, CENTER_LON))
        scnfile.write(zero_time + "DEFWPT COR201,{:.6f},{:.6f},FIX\n"
                      .format(corridor_top_lat, CENTER_LON))

        # Create aircaft
        scnfile.write("\n# Create {} aircraft".format(NUM_EXPERIMENT_AIRCRAFT))
        create_aircraft(scnfile,
                        NUM_EXPERIMENT_AIRCRAFT,
                        dep_waypoints,
                        dest_waypoints,
                        corridor_bottom_lat,
                        CENTER_LON)


def create_area(scnfile,
                zero_time,
                corridor_width,
                corridor_length,
                corridor_side,
                angle,
                is_restriction_angle=False):
    """
    Create the restricted area on a given coridor side
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

    # Determine the angle of the area edge
    if is_restriction_angle:
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
    scnfile.write(zero_time + "RAA RAA{},ON,{},{},{}\n"
                  .format(area_idx, 0, 0, area_coords))
    scnfile.write(zero_time + "COLOR RAA{},164,0,0\n".format(area_idx))
    scnfile.write(zero_time + "DEFWPT RAA_{},{:.6f},{:.6f},FIX\n".format(
        area_idx, CENTER_LAT, (inner_bottom_lon + outer_lon) / 2))

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
                    num_total_ac,
                    dep_waypoints,
                    dest_waypoints,
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
    creation_lat = []
    creation_lon = []
    creation_hdg = []
    creation_dest = []
    creation_dest_idx = []

    num_created_ac = 0
    while num_created_ac < num_total_ac:
        # Will be set True if creation results in a conflict
        in_conflict_at_creation = False

        # Create an aircraft at random time and position
        prev_time = creation_time[-1] if num_created_ac else 0
        curr_time = prev_time + random.randint(AC_CRE_INTERVAL_MIN,
                                               AC_CRE_INTERVAL_MAX)

        # Select an aircraft type and velocity
        type_idx = random.randint(0, len(AC_TYPES) - 1)
        curr_type = AC_TYPES[type_idx]
        curr_spd = random.gauss(AC_TYPES_SPD[type_idx], SPD_STDDEV)

        # Select random departure and destination waypoint
        curr_dep_wp_idx = random.randint(0, NUM_DEP_DEST_POINTS - 1)
        (curr_lat, curr_lon) = dep_waypoints[curr_dep_wp_idx]
        curr_dest_wp_idx = random.randint(0, NUM_DEP_DEST_POINTS - 1)
        (dest_lat, dest_lon) = dest_waypoints[curr_dest_wp_idx]

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
            dist_diff_list = [bsgeo.kwikdist(lat, lon, curr_lat, curr_lon)
                              for (lat, lon) in zip(creation_lat, creation_lon)]

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
        creation_lat.append(curr_lat)
        creation_lon.append(curr_lon)
        creation_hdg.append(curr_hdg % 360)
        creation_dest.append((dest_lat, dest_lon))
        creation_dest_idx.append(curr_dest_wp_idx)
        num_created_ac += 1

    # Write each aircraft to file
    for ac_idx in range(num_created_ac):
        # Altitude is the same for all aircraft
        ac_alt = "36000"

        time_str = time.strftime('%H:%M:%S', time.gmtime(creation_time[ac_idx]))
        time_str = "{}.00>".format(time_str)
        aircraft_str = time_str + "CRE AC{:03d} {},{:.6f},{:.6f},{:.2f},{},{:.0f}\n"\
            .format(ac_idx,
                    creation_type[ac_idx],
                    creation_lat[ac_idx],
                    creation_lon[ac_idx],
                    creation_hdg[ac_idx],
                    ac_alt,
                    creation_spd[ac_idx])

        aircraft_str += time_str + "AC{:03d} ADDWPT COR101\n".format(ac_idx)
        aircraft_str += time_str + "AC{:03d} ADDWPT COR201\n".format(ac_idx)
        aircraft_str += time_str + "AC{:03d} ADDWPT DST{:03d}\n" \
            .format(ac_idx, creation_dest_idx[ac_idx])

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

    # First guess for intersection is midpoint of line
    intersect_lat, intersect_lon = tg.calc_midpoint(line_start_lat,
                                                    line_start_lon,
                                                    line_end_lat,
                                                    line_end_lon)
    qdr_from_center, dist_from_center = bsgeo.qdrdist(ring_center_lat,
                                                      ring_center_lon,
                                                      intersect_lat,
                                                      intersect_lon)

    # Use recursive bisection until the intersection point is within
    # distance margin from the ring edge
    while True:
        if abs(dist_from_center - ring_radius) < 0.1:
            break

        if dist_from_center > ring_radius:
            line_end_lat, line_end_lon = intersect_lat, intersect_lon
        elif dist_from_center < ring_radius:
            line_start_lat, line_start_lon = intersect_lat, intersect_lon

        intersect_lat, intersect_lon = tg.calc_midpoint(line_start_lat,
                                                        line_start_lon,
                                                        line_end_lat,
                                                        line_end_lon)
        qdr_from_center, dist_from_center = bsgeo.qdrdist(ring_center_lat,
                                                          ring_center_lon,
                                                          intersect_lat,
                                                          intersect_lon)

    angle_from_center = ar.ned2crs(qdr_from_center)

    return intersect_lat, intersect_lon, angle_from_center


def calc_departure_waypoints(num_dep_points,
                             area_center_lat,
                             area_center_lon,
                             area_radius,
                             angle):
    """
    Calculate the locations of all departure waypoints on the area edge.
    """

    angles = list(np.linspace(180 - angle + 3, 180 + angle - 3, num_dep_points))
    waypoints = [bsgeo.qdrpos(area_center_lat, area_center_lon, angle, area_radius)
                 for angle in angles]

    return waypoints


def calc_destination_waypoints(num_dest_points,
                               area_center_lat,
                               area_center_lon,
                               area_radius,
                               angle):
    """
    Calculate the locations of all destination waypoints on the area edge.
    """

    angles = list(np.linspace(-angle + 3, angle - 3, num_dest_points))
    waypoints = [bsgeo.qdrpos(area_center_lat, area_center_lon, angle, area_radius)
                 for angle in angles]

    return waypoints


if __name__ == "__main__":
    # When running as main, generate a file for each relevant resolution method
    curr_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    create(curr_time,
           1,
           "OFF",
           CORRIDOR_LENGTH,
           CORRIDOR_WIDTH,
           ARC_ANGLE,
           is_restriction_angle=False)

    create(curr_time,
           1,
           "MVP",
           CORRIDOR_LENGTH,
           CORRIDOR_WIDTH,
           ARC_ANGLE,
           is_restriction_angle=False)

    create(curr_time,
           1,
           "LF",
           CORRIDOR_LENGTH,
           CORRIDOR_WIDTH,
           ARC_ANGLE,
           is_restriction_angle=False)

    create(curr_time,
           1,
           "SWARM_V2",
           CORRIDOR_LENGTH,
           CORRIDOR_WIDTH,
           ARC_ANGLE,
           is_restriction_angle=False)
