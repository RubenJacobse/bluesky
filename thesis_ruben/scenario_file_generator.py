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


class ScenarioGenerator():
    """
    Create a scenario file which name reflects the properties of the
    experiment geometry.
    """

    def __init__(self,
                 target_dir,
                 timestamp,
                 random_seed,
                 traffic_level,
                 resolution_method,
                 corridor_length,
                 corridor_width,
                 angle,
                 is_edge_angle=False):

        # Set input variables as attributes
        self.target_dir = target_dir
        self.timestamp = timestamp
        self.random_seed = random_seed
        self.traffic_level = traffic_level
        self.resolution_method = resolution_method
        self.corridor_length = corridor_length
        self.corridor_width = corridor_width
        self.angle = angle
        self.is_edge_angle = is_edge_angle

        # Initialize random number generator before we do anything where
        # random numbers are used.
        random.seed(self.random_seed)

        # Calculate airspace restriction and corridor parameters
        self.airspace_restrictions = []
        self.create_airspace_restriction("LEFT")
        self.create_airspace_restriction("RIGHT")
        self.calculate_corridor_parameters()

        # Create aircraft and store for further use
        self.set_ac_creation_intervals()
        self.calculate_creation_arc_angle_range()
        self.aircraft_list = []
        self.create_all_aircraft()

        # Save the results to file
        self.write_scenfile()
        self.write_geofile()

    def create_airspace_restriction(self, corridor_side):
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
                                        self.corridor_length/2)
        _, inner_top_lon = bsgeo.qdrpos(CENTER_LAT,
                                        CENTER_LON,
                                        east_west_angle,
                                        self.corridor_width/2)
        inner_bottom_lat, _ = bsgeo.qdrpos(CENTER_LAT,
                                           CENTER_LON,
                                           180,
                                           self.corridor_length/2)
        _, inner_bottom_lon = bsgeo.qdrpos(CENTER_LAT,
                                           CENTER_LON,
                                           east_west_angle,
                                           self.corridor_width/2)

        # Determine the angle of the area edge, either by using the
        # input edge angle or by calculating the intersection of the
        # area edge with the ring
        if self.is_edge_angle:
            edge_angle = (self.angle if corridor_side == "RIGHT"
                          else -self.angle)
        else:
            arc_angle = (self.angle/2 if corridor_side == "RIGHT"
                         else -self.angle/2)
            arc_ext_lat, arc_ext_lon = bsgeo.qdrpos(CENTER_LAT,
                                                    CENTER_LON,
                                                    arc_angle,
                                                    AREA_RADIUS*1.1)
            arc_point_lat, arc_point_lon, _\
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

        # Calculate coordinates of points on north and south lines extending
        # the edge furthest from the corridor
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

        # Calculate the approximate midpoint of the area, this is used to
        # place a waypoint with the area name in the scenario.
        midpoint_lat = CENTER_LAT
        midpoint_lon = (inner_bottom_lon + outer_lon) / 2

        # Store the area data
        area = {}
        area["inner_top"] = (inner_top_lat, inner_top_lon)
        area["inner_bottom"] = (inner_bottom_lat, inner_bottom_lon)
        area["outer_top"] = (outer_top_lat, outer_top_lon)
        area["outer_bottom"] = (outer_bottom_lat, outer_bottom_lon)
        area["coord_str"] = \
            "{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}"\
            .format(inner_top_lat,
                    inner_top_lon,
                    outer_top_lat,
                    outer_top_lon,
                    outer_bottom_lat,
                    outer_bottom_lon,
                    inner_bottom_lat,
                    inner_bottom_lon)
        area["midpoint_str"] = f"{midpoint_lat:.6f},{midpoint_lon:.6f}"
        self.airspace_restrictions.append(area)

    def calculate_corridor_parameters(self):
        """
        Calculate the 'top' and 'bottom' latitudes of the corridor.
        """

        self.corridor["north_lat"], _ = bsgeo.qdrpos(CENTER_LAT,
                                                     CENTER_LON,
                                                     0,
                                                     self.corridor_length/2)
        self.corridor["south_lat"], _ = bsgeo.qdrpos(CENTER_LAT,
                                                     CENTER_LON,
                                                     180,
                                                     self.corridor_length/2)
        _, self.corridor["left_lon"] = bsgeo.qdrpos(CENTER_LAT,
                                                    CENTER_LON,
                                                    270,
                                                    self.corridor_width/2)
        _, self.corridor["right_lon"] = bsgeo.qdrpos(CENTER_LAT,
                                                     CENTER_LON,
                                                     90,
                                                     self.corridor_width/2)

    def calculate_creation_arc_angle_range(self):
        """
        Calculate angle from the center point to intersection between
        ring and area edge.
        """
        area = self.airspace_restrictions[1]
        _, _, top_angle_from_center = \
            calc_line_ring_intersection(CENTER_LAT,
                                        CENTER_LON,
                                        AREA_RADIUS,
                                        area["inner_top"][0],
                                        area["inner_top"][1],
                                        area["outer_top"][0],
                                        area["outer_top"][1])
        if self.is_edge_angle:
            self.ac_creation_arc_angle_range = top_angle_from_center
        else:
            self.ac_creation_arc_angle_range = self.angle / 2

    def set_ac_creation_intervals(self):
        """
        Set aircraft creation interval minimum and maximum values. (We do
        this here so we can print these values in the scenario header)
        """

        if self.traffic_level == "LOW":  # Average 72 seconds = ~50 ac/hr
            cre_interval_min = 52
            cre_interval_max = 92
        elif self.traffic_level == "MID":  # Average 36 seconds = ~100 ac/hr
            cre_interval_min = 26
            cre_interval_max = 46
        elif self.traffic_level == "HIGH":  # Average 24 seconds = ~150 ac/hr
            cre_interval_min = 14
            cre_interval_max = 34

        self.cre_interval_min = cre_interval_min
        self.cre_interval_max = cre_interval_max

    def create_all_aircraft(self):
        """
        Create all aircraft in this scenario
        """

        angle = self.ac_creation_arc_angle_range
        ac_spd = 280  # [kts] Speed

        # Minimum distance and time differences at creation
        min_dist = 6  # [nm]
        min_dt = min_dist / ac_spd * 3600  # [s]

        num_created_ac = 0
        while num_created_ac < NUM_AIRCRAFT:
            # Will be set True if creation results in a conflict
            in_conflict_at_creation = False

            # Create an aircraft at random time and position
            prev_time = self.aircraft_list[-1]["time"] if num_created_ac else 0
            curr_time = prev_time + random.randint(self.cre_interval_min,
                                                   self.cre_interval_max)

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
                                        self.corridor["south_lat"],
                                        CENTER_LON)

            # The first generated aircraft is always accepted. Each aircraft
            # thereafter has to have at least minimum separation in space
            # OR time with ALL existing aircraft at the time of its creation.
            if num_created_ac:
                time_diff_list = [curr_time - aircraft["time"]
                                  for aircraft in self.aircraft_list]
                dist_diff_list = [bsgeo.kwikdist(aircraft["dep_lat"],
                                                 aircraft["dep_lon"],
                                                 curr_lat,
                                                 curr_lon)
                                  for aircraft in self.aircraft_list]

                for time_diff, dist_diff in zip(time_diff_list, dist_diff_list):
                    # Either time OR distance difference must be smaller than
                    # the minimum value; if not: the aircraft is in conflict
                    if not (((dist_diff < min_dist) and (time_diff > min_dt))
                            or ((dist_diff > min_dist) and (time_diff < min_dt))
                            or ((dist_diff > min_dist) and (time_diff > min_dt))):
                        in_conflict_at_creation = True
                        break

                # If the current aircraft is in conflict, continue the
                # while loop and try another random creation.
                if in_conflict_at_creation:
                    continue

            # Store the current aircraft as a dictionary in the aircraft list
            curr_ac = {}
            curr_ac["time"] = curr_time
            curr_ac["type"] = curr_type
            curr_ac["spd"] = curr_spd
            curr_ac["dep_lat"] = curr_lat
            curr_ac["dep_lon"] = curr_lon
            curr_ac["hdg"] = curr_hdg % 360
            curr_ac["dest_lat"] = dest_lat
            curr_ac["dest_lon"] = dest_lon
            self.aircraft_list.append(curr_ac)
            num_created_ac += 1

    def write_scenfile(self):
        """
        Create the .scn file containing all commands to be executed by
        BlueSky for the given scenario.
        """

        scnfile_name = (("{}_L{}_W{}_A{}_RESO-{}_T-{}_SCEN{:03d}.scn")
                        .format(self.timestamp,
                                self.corridor_length,
                                self.corridor_width,
                                self.angle,
                                self.resolution_method,
                                self.traffic_level,
                                self.random_seed))
        scnfile_path = os.path.join(self.target_dir, scnfile_name)

        # Create a header to simplify traceability of variable values
        # after creation.
        scen_header = \
            ("##################################################\n"
             + f"# File created at: {self.timestamp}\n"
             + f"# Center latitude: {CENTER_LAT} deg\n"
             + f"# Center longitude: {CENTER_LON} deg\n"
             + f"# Corridor length: {self.corridor_length} NM\n"
             + f"# Corridor width: {self.corridor_width} NM\n"
             + f"# Angle: {self.angle} deg "
             + (" (Defined by area edge angle)\n" if self.is_edge_angle
                else "(Defined by arc angle)\n")
             + f"# Experiment area radius: {AREA_RADIUS} NM\n"
             + f"# Aircraft types: {AC_TYPES}\n"
             + f"# Aircraft average speed: {AC_TYPES_SPD} kts\n"
             + f"# Aircraft speed std dev: {SPD_STDDEV} kts\n"
             + f"# Number of aircraft created: {NUM_AIRCRAFT}\n"
             + f"# Traffic level: {self.traffic_level}\n"
             + f"# Aircraft creation interval min: {self.cre_interval_min} s\n"
             + f"# Aircraft creation interval max: {self.cre_interval_max} s\n"
             + f"# Random number generator seed: {self.random_seed}\n"
             + "##################################################\n")

        with open(scnfile_path, "w+") as scnfile:
            # All commands not related to aircraft creation are executed at
            # t = 0
            zero_time = "\n0:00:00.00>"
            scnfile.write(scen_header)

            scnfile.write("\n# Sim commands")
            scnfile.write(zero_time + f"PAN {CENTER_LAT},{CENTER_LON}")
            scnfile.write(zero_time + f"DT {SIM_TIME_STEP}")
            scnfile.write(zero_time + "TRAIL {}"
                          .format("ON" if SHOW_AC_TRAILS else "OFF"))
            scnfile.write(zero_time + "SWRAD SYM")
            scnfile.write(zero_time + "SWRAD LABEL 0")
            scnfile.write(zero_time + "SWRAD WPT")
            scnfile.write(zero_time + "SWRAD SAT")
            scnfile.write(zero_time + "SWRAD APT")
            scnfile.write(zero_time + "FF")

            scnfile.write("\n\n# Setup circular experiment area and activate"
                          + " it as a traffic area in BlueSky")
            scnfile.write(zero_time + "PLUGINS LOAD AREA")
            scnfile.write(zero_time + "CIRCLE EXPERIMENT {},{},{}"
                          .format(CENTER_LAT, CENTER_LON, AREA_RADIUS))
            scnfile.write(zero_time + "AREA EXPERIMENT")
            scnfile.write(zero_time + "COLOR EXPERIMENT 0,128,0")

            scnfile.write("\n\n# Setup BlueSky ASAS module options")
            scnfile.write(zero_time + "ASAS ON")
            scnfile.write(zero_time + f"RESO {self.resolution_method}")
            scnfile.write(zero_time + f"RMETHH {RMETHH}")

            # Restricted airspaces
            scnfile.write("\n\n# LOAD RAA plugin and create area restrictions")
            scnfile.write(zero_time + f"PLUGINS LOAD {PLUGIN_NAME}")
            scnfile.write(zero_time + f"RAALOG {self.timestamp}")
            scnfile.write(zero_time + f"RAACONF {AREA_LOOKAHEAD_TIME}")

            for idx, area in enumerate(self.airspace_restrictions):
                scnfile.write(zero_time + f"RAA RAA{idx + 1},ON,{0},{0},"
                              + f"{area['coord_str']}")
                scnfile.write(zero_time + f"COLOR RAA{idx + 1},164,0,0")
                scnfile.write(zero_time + f"DEFWPT RAA_{idx + 1},"
                              + f"{area['midpoint_str']},FIX")

            # Corridor waypoints
            scnfile.write("\n\n# Corridor waypoints")
            scnfile.write(zero_time + "DEFWPT COR101,{:.6f},{:.6f},FIX"
                          .format(self.corridor["south_lat"], CENTER_LON))
            scnfile.write(zero_time + "DEFWPT COR201,{:.6f},{:.6f},FIX"
                          .format(self.corridor["north_lat"], CENTER_LON))

            # Write each aircraft to file
            scnfile.write("\n\n# Create {} aircraft".format(NUM_AIRCRAFT))
            for idx, ac in enumerate(self.aircraft_list):
                # Altitude is the same for all aircraft
                ac_alt = "36000"

                # Write the aircraft creation and route commands to file
                time_str = time.strftime('%H:%M:%S', time.gmtime(ac["time"]))
                time_str = "{}.00>".format(time_str)
                ac_str = (time_str +
                          "CRE AC{:03d} {},{:.6f},{:.6f},{:.2f},{},{:.0f}\n"
                          .format(idx,
                                  ac["type"],
                                  ac["dep_lat"],
                                  ac["dep_lon"],
                                  ac["hdg"],
                                  ac_alt,
                                  ac["spd"]))

                ac_str += time_str + ("DEFWPT DEP{:03d},{:06f},{:06f},FIX\n"
                                      .format(idx,
                                              ac["dep_lat"],
                                              ac["dep_lon"]))
                ac_str += time_str + ("DEFWPT DST{:03d},{:06f},{:06f},FIX\n"
                                      .format(idx,
                                              ac["dest_lat"],
                                              ac["dest_lon"]))
                ac_str += time_str + f"AC{idx:03d} ADDWPT DEP{idx:03d}\n"
                ac_str += time_str + f"AC{idx:03d} ADDWPT COR101\n"
                ac_str += time_str + f"AC{idx:03d} ADDWPT COR201\n"
                ac_str += time_str + f"AC{idx:03d} ADDWPT DST{idx:03d}\n"

                scnfile.write("\n" + ac_str)

    def write_geofile(self):
        """
        Create csv file containing the experiment area and airspace
        restriction geometric parameters to allow generation of area
        restriction graphs during post-processing.
        """

        geofile_name = (("{}_L{}_W{}_A{}_geo.csv")
                        .format(self.timestamp,
                                self.corridor_length,
                                self.corridor_width,
                                self.angle))
        geofile_path = os.path.join(self.target_dir, geofile_name)

        with open(geofile_path, 'w+') as geofile:
            # Write experiment area parameters to file
            geofile.write("{},{},{},{}\n".format(
                "ring", CENTER_LAT, CENTER_LON, AREA_RADIUS))

            # Write restriction coordinates to file
            for idx, area in enumerate(self.airspace_restrictions):
                geofile.write(f"RAA{idx+1},{area['coord_str']}\n")


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
        ScenarioGenerator(test_folder,
                          current_time,
                          SEED,
                          TRAFFIC_LEVEL,
                          reso_method,
                          CORRIDOR_LENGTH,
                          CORRIDOR_WIDTH,
                          ARC_ANGLE,
                          is_edge_angle=False)
