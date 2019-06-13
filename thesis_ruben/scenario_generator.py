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
AREA_LOOKAHEAD_TIME = 120  # [s] Look-ahead time used to detect area conflicts
SIM_TIME_STEP = 0.1  # [s] Simulation time step
SIM_PAUSE_HOUR = 4  # [h] Pause simulation after this number of hours
RMETHH = "BOTH"  # Horizontal resolution method, allow both spd and hdg changes
SHOW_AC_TRAILS = True  # Plot trails in the radar window
AREA_RADIUS = 100  # [NM]
NUM_AIRCRAFT = 200
# AC_TYPES = ["A320", "B738", "A333", "B744"]
# AC_TYPES_SPD = [258, 260, 273, 286]
AC_TYPES = ["B744"]
AC_TYPES_SPD = [280]
SPD_STDDEV = 5  # [kts] Standard deviation of cruise speed distributions
CRE_INTERVAL_MIN = 30  # [s] Minimum interval between creation of aircraft
CRE_INTERVAL_MAX = 90  # [s] Maximum interval between creation of aircraft

# Passing these as arguments to create() when called as __main__
SEED = 1
CORRIDOR_LENGTH = 40  # [NM]
CORRIDOR_WIDTH = 25  # [NM]
RESTRICTION_ANGLE = 45  # [deg]
ARC_ANGLE = 90  # [deg]
RESO_METHOD = "MVP"  # Conflict resolution method


def create_scenfile(timestamp,
                    random_seed,
                    asas_reso_method,
                    corridor_length,
                    corridor_width,
                    angle,
                    is_edge_angle=False):
    """
    Create a scenario file which name reflects the properties of the
    experiment geometry.
    """

    random.seed(random_seed)

    # Write scenario files to different folders based on angle types
    curr_dir = os.path.dirname(__file__)
    folder_rel_path = "../scenario/thesis_ruben/{}".format(timestamp)
    folder_abs_path = os.path.join(curr_dir, folder_rel_path)
    if not os.path.exists(folder_abs_path):
        os.makedirs(folder_abs_path)

    file_path = ("{}/L{}_W{}_RESO-{}_SCEN_{:03d}.scn")\
        .format(folder_abs_path,
                corridor_length,
                corridor_width,
                asas_reso_method,
                random_seed)

    # Create a header to simplify traceability of variable values
    scen_header = ("##################################################\n"
                   + "# File created at: {}\n".format(timestamp)
                   + "# Center latitude: {}\n".format(CENTER_LAT)
                   + "# Center longitude: {}\n".format(CENTER_LON)
                   + "# Corridor length: {} NM\n".format(CORRIDOR_LENGTH)
                   + "# Corridor width: {} NM\n".format(CORRIDOR_WIDTH)
                   + "# Angle: {} deg ".format(angle)
                   + (" (Defined by area edge angle)\n" if is_edge_angle
                      else "(Defined by arc angle)\n")
                   + "# Experiment area radius: {} NM\n".format(AREA_RADIUS)
                   + "# Aircraft types: {}\n".format(AC_TYPES)
                   + "# Aircraft average speed: {} kts\n".format(AC_TYPES_SPD)
                   + "# Aircraft speed std dev: {} kts\n".format(SPD_STDDEV)
                   + "# Number of aircraft created: {}\n".format(NUM_AIRCRAFT)
                   + "# Min creation interval: {} s\n".format(CRE_INTERVAL_MIN)
                   + "# Max creation interval: {} s\n".format(CRE_INTERVAL_MAX)
                   + "# Random number generator seed: {}\n".format(SEED)
                   + "##################################################\n\n")

    # Open file, overwrite if existing
    with open(file_path, "w+") as scnfile:
        scnfile.write(scen_header)
        zero_time = "0:00:00.00>"

        scnfile.write("# Sim commands\n")
        scnfile.write(zero_time + "PAN {},{}\n".format(CENTER_LAT, CENTER_LON))
        scnfile.write(zero_time + "DT {}\n".format(SIM_TIME_STEP))
        scnfile.write(zero_time + "TRAIL {}\n".format("ON" if SHOW_AC_TRAILS
                                                      else "OFF"))
        scnfile.write(zero_time + "SWRAD SYM\n")
        scnfile.write(zero_time + "SWRAD LABEL 0\n")
        scnfile.write(zero_time + "SWRAD WPT\n")
        scnfile.write(zero_time + "SWRAD SAT\n")
        scnfile.write(zero_time + "SWRAD APT\n")
        scnfile.write(zero_time + "FF\n")
        scnfile.write("{}:00:00.00>HOLD\n".format(SIM_PAUSE_HOUR))

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
                        is_edge_angle)
        angle_to_ring_intersect = create_area(scnfile,
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
        scnfile.write("\n# Create {} aircraft".format(NUM_AIRCRAFT))
        create_aircraft(scnfile,
                        NUM_AIRCRAFT,
                        angle_from_centerpoint,
                        corridor_bottom_lat,
                        CENTER_LON)


def create_area(scnfile,
                zero_time,
                corridor_width,
                corridor_length,
                corridor_side,
                angle,
                is_edge_angle=False):
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
        curr_time = prev_time + random.randint(CRE_INTERVAL_MIN,
                                               CRE_INTERVAL_MAX)

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


if __name__ == "__main__":
    # When running as main, generate a file for each relevant resolution
    # method using the default values of the other parameters.
    creation_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    for reso_method in ["OFF", "MVP", "LF", "SWARM_V2"]:
        create_scenfile(creation_time,
                        SEED,
                        reso_method,
                        CORRIDOR_LENGTH,
                        CORRIDOR_WIDTH,
                        ARC_ANGLE,
                        is_edge_angle=False)
