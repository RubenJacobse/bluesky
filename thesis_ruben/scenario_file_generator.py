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

# Third party imports
from shapely.geometry import Polygon, JOIN_STYLE

# Enable BlueSky imports by adding the project folder to the path
sys.path.append(os.path.abspath(os.path.join('..')))
sys.path.append(os.path.abspath(os.path.join('../plugins')))

# BlueSky imports
from bluesky.tools.aero import cas2tas
import bluesky.tools.geo as bsgeo
import plugins.thesis.geo as tg

# Local imports
import scenario_config

# BlueSky simulator settings
SIM_TIME_STEP = 0.1  # [s] Simulation time step
SIM_SHOW_TRAILS = False  # Plot trails in the radar window

# Area restriction settings
CENTER_LAT = 0.0  # [deg] Scenario area center latitude
CENTER_LON = 0.0  # [deg] Scenario area center longitude
AREA_RADIUS = 100  # [NM] Scenario area radius
PLUGIN_NAME = "RAA"
AREA_LOOKAHEAD_TIME = 120  # [s] Look-ahead time used to detect area conflicts
RMETHH = "BOTH"  # Horizontal resolution method, allow both spd and hdg changes

# Aircraft creation settings
AC_TYPES = ["A320", "B738", "A333", "B744"]
AC_TYPES_SPD = [258, 260, 273, 284]  # [kts] Cruise speed (CAS) for each type
AC_SPD_STDDEV = 5  # [kts] Standard deviation of cruise speed distributions

# Specify whether a scenario contains a fixed number of aircraft or
# is defined by a fixed simulation time period.
SCEN_AC_MODE = "RUNTIME"  # "TOT_AC"
SCEN_TOT_AC = 50  # Total number of aircraft in the scenario
SCEN_RUNTIME = 10800  # Scenario run duration in seconds


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

        # Calculate all geometric parameters
        self.airspace_restrictions = []
        self.geovectors = []
        self.corridor = {}
        self.polygons = {}

        self.create_airspace_restriction("LEFT")
        self.create_airspace_restriction("RIGHT")
        self.calculate_corridor_parameters()
        self.create_polygons()
        if "GV" in self.resolution_method:
            self.create_geovectors()
        self.swarm_zones = []
        if "SWARM-V3" in self.resolution_method:
            self.create_swarm_zones()

        # Create aircraft and store for further use
        self.set_ac_creation_intervals()
        self.calculate_creation_arc_angle_range()
        self.aircraft_list = []
        self.create_all_aircraft()
        self.num_created_ac = len(self.aircraft_list)

        # Save the results to file
        self.write_scenfile()
        self.write_geofile()
        self.write_geovectorfile()

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

    def create_polygons(self):
        """
        Create a number of relevant shapely polygons that can be
        used as basis for geovector areas or swarming zones.
        """

        # Experiment area
        ring_radius = 100  # [NM]
        ring_coords = [bsgeo.qdrpos(CENTER_LAT, CENTER_LON, angle, ring_radius)
                       for angle in range(0, 360)]
        ring_poly = Polygon(ring_coords)
        self.polygons["experiment_area"] = ring_poly

        # Left-side airspace restriction
        left_area = self.airspace_restrictions[0]
        left_coords = [left_area["inner_top"], left_area["inner_bottom"],
                       left_area["outer_bottom"], left_area["outer_top"]]
        left_poly = Polygon(left_coords).intersection(ring_poly)
        self.polygons["left_restriction"] = left_poly
        self.airspace_restrictions[0]["poly"] = left_poly

        # Right-side airspace restriction
        right_area = self.airspace_restrictions[1]
        right_coords = [right_area["inner_top"], right_area["inner_bottom"],
                        right_area["outer_bottom"], right_area["outer_top"]]
        right_poly = Polygon(right_coords).intersection(ring_poly)
        self.polygons["right_restriction"] = right_poly
        self.airspace_restrictions[1]["poly"] = right_poly

        # Corridor rectangular area
        corridor_coords = [(self.corridor["south_lat"],
                            self.corridor["left_lon"]),
                           (self.corridor["south_lat"],
                            self.corridor["right_lon"]),
                           (self.corridor["north_lat"],
                            self.corridor["right_lon"]),
                           (self.corridor["north_lat"],
                            self.corridor["left_lon"])]
        corridor_poly = Polygon(corridor_coords)
        self.polygons["corridor"] = corridor_poly

        # Coordinates of wedge shaped merging area
        # First find bounding box using corridor south latitude and
        # experiment area bounds
        (min_lat, min_lon, max_lat, max_lon) = ring_poly.bounds
        merge_box_coords = [(self.corridor["south_lat"], min_lon),
                            (min_lat, min_lon),
                            (min_lat, max_lon),
                            (self.corridor["south_lat"], max_lon)]
        merge_box_poly = Polygon(merge_box_coords)
        merge_circle_poly = merge_box_poly.intersection(ring_poly)
        wedge_poly = merge_circle_poly.difference(left_poly).difference(right_poly)
        # Use Polygon.buffer() to fix potential slivers
        wedge_poly = wedge_poly \
                        .buffer(0.0001, 1, join_style=JOIN_STYLE.mitre) \
                        .buffer(-0.0001, 1, join_style=JOIN_STYLE.mitre)
        self.polygons["merge_wedge"] = Polygon(wedge_poly)

        # Corwedge polygon: union of merge wedge and corridor area
        corwedge_poly = wedge_poly.union(corridor_poly)
        self.polygons["corwedge"] = Polygon(corwedge_poly)

    def create_geovectors(self):
        """
        If required, creates all geovector restrictions for the scenario.

        NOTE: Currently only creates a single geovector area!
        """

        # Set up the dictionary, relevant values will be overwritten depending
        # on the type of geovector that will be used
        geovector = {"name": self.resolution_method,
                     "coords": [],
                     "gs_min": "",
                     "gs_max": "",
                     "crs_min": "",
                     "crs_max": ""}

        # Set the geo vector coordinates
        if "CORRIDOR" in self.resolution_method:
            gv_poly = self.polygons["corridor"]
        elif "CIRCLE" in self.resolution_method:
            gv_poly = self.polygons["ring"]
        elif "WEDGE" in self.resolution_method:
            gv_poly = self.polygons["merge_wedge"]
        elif "CORWEDGE" in self.resolution_method:
            gv_poly = self.polygons["corwedge"]

        geovector["coords"] = [x for (lat, lon) in gv_poly.exterior.coords
                               for x in (lat, lon)]

        # A two dimensional geovector can restrict ground speed and/or course
        gs_min_cas = 260  # [kts]
        gs_max_cas = 270  # [kts]
        crs_min = 359  # [deg]
        crs_max = 1  # [deg]

        # Convert speed limits from CAS [kts] to TAS [kts] for use in geovector
        # cas2tas() is entirely in metric, so we need to convert kts and ft
        gs_min = cas2tas(gs_min_cas * 0.51444, 36000 * 0.3048) / 0.51444
        gs_max = cas2tas(gs_max_cas * 0.51444, 36000 * 0.3048) / 0.51444
        if "SPD" in self.resolution_method:
            geovector["gs_min"] = gs_min
            geovector["gs_max"] = gs_max
        elif "CRS" in self.resolution_method:
            geovector["crs_min"] = crs_min
            geovector["crs_max"] = crs_max
        elif "BOTH" in self.resolution_method:
            geovector["gs_min"] = gs_min
            geovector["gs_max"] = gs_max
            geovector["crs_min"] = crs_min
            geovector["crs_max"] = crs_max

        self.geovectors.append(geovector)

    def create_swarm_zones(self):
        """
        Create
        """

        self.swarm_zones.append(self.polygons["corwedge"])

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
            cre_interval_min = 62
            cre_interval_max = 82
        elif self.traffic_level == "MID":  # Average 36 seconds = ~100 ac/hr
            cre_interval_min = 26
            cre_interval_max = 46
        elif self.traffic_level == "HIGH":  # Average 24 seconds = ~150 ac/hr
            cre_interval_min = 14
            cre_interval_max = 34
        else:
            interval_avg = round(3600 / int(self.traffic_level))
            cre_interval_min = max(0, interval_avg - 10)
            cre_interval_max = interval_avg + 10

        self.cre_interval_min = cre_interval_min
        self.cre_interval_max = cre_interval_max

    def create_all_aircraft(self):
        """
        Create all aircraft in this scenario
        """

        angle = self.ac_creation_arc_angle_range

        # Minimum distance and time differences at creation
        min_dist = 6  # [nm]
        ac_spd = 411  # [kts] Lowest possible ground speed in scenario (B738)
        min_dt = min_dist / ac_spd * 3600  # [s]

        # Generate all aircraft using either a fixed total number or a fixed
        # time interval.
        num_created_ac = 0
        while ((SCEN_AC_MODE == "TOT_AC" and num_created_ac < SCEN_TOT_AC)
               or (SCEN_AC_MODE == "RUNTIME")):
            # Will be set True if creation results in a loss of separation
            in_los_at_creation = False

            # Create an aircraft at random time and position
            prev_time = self.aircraft_list[-1]["time"] if num_created_ac else 0
            curr_time = prev_time + random.randint(self.cre_interval_min,
                                                   self.cre_interval_max)

            # Exit the while loop
            if SCEN_AC_MODE == "RUNTIME" and curr_time >= SCEN_RUNTIME:
                break

            # Select an aircraft type and velocity
            type_idx = random.randint(0, len(AC_TYPES) - 1)
            curr_type = AC_TYPES[type_idx]
            curr_spd = random.gauss(AC_TYPES_SPD[type_idx], AC_SPD_STDDEV)

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
                                                2 * AREA_RADIUS)

            # Heading to waypoint at start of corridor
            curr_hdg, _ = bsgeo.qdrdist(curr_lat,
                                        curr_lon,
                                        self.corridor["south_lat"],
                                        CENTER_LON)

            # The first generated aircraft is always accepted. Each aircraft
            # thereafter has to have at least minimum separation in space
            # OR time with ALL existing aircraft at the time of its creation.
            if num_created_ac:
                # Make list of tuples with (time, distance) differences
                # between aircraft created less than < min_dt apart in time
                diff_list = [(curr_time - aircraft["time"],
                              bsgeo.kwikdist(aircraft["dep_lat"],
                                             aircraft["dep_lon"],
                                             curr_lat, curr_lon))
                             for aircraft in self.aircraft_list
                             if (curr_time - aircraft["time"]) < min_dt]

                for (time_diff, dist_diff) in diff_list:
                    # Either time OR distance difference must be larger than
                    # its minimum value; if not: the aircraft is in LoS
                    if time_diff < min_dt and dist_diff < min_dist:
                        in_los_at_creation = True
                        break

                # If the current aircraft is in LoS, continue the
                # while loop and try another random creation.
                if in_los_at_creation:
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
             + f"# Aircraft speed std dev: {AC_SPD_STDDEV} kts\n"
             + f"# Number of aircraft created: {self.num_created_ac}\n"
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
                          .format("ON" if SIM_SHOW_TRAILS else "OFF"))
            scnfile.write(zero_time + "SWRAD SYM")
            scnfile.write(zero_time + "SWRAD LABEL 1")
            scnfile.write(zero_time + "SWRAD WPT")
            scnfile.write(zero_time + "SWRAD SAT")
            scnfile.write(zero_time + "SWRAD APT")
            scnfile.write(zero_time + "FF")

            scnfile.write("\n\n# Setup BlueSky ASAS module options")
            scnfile.write(zero_time + "ASAS ON")
            if self.resolution_method.startswith("GV"):
                scnfile.write(zero_time + f"RESO MVP")
            else:
                scnfile.write(zero_time + f"RESO {self.resolution_method}")
            scnfile.write(zero_time + f"RMETHH {RMETHH}")

            # Experiment area
            scnfile.write("\n\n# Setup circular experiment area and activate"
                          + " it as a traffic area in BlueSky")
            scnfile.write(zero_time + "PLUGINS LOAD AREA")
            coords = [x for (lat, lon)
                      in self.polygons["experiment_area"].exterior.coords
                      for x in (lat, lon)]
            coords_str = ",".join(str(f"{x:.6f}") for x in coords)
            scnfile.write(zero_time + f"POLY EXPERIMENT {coords_str}")
            scnfile.write(zero_time + "AREA EXPERIMENT")
            scnfile.write(zero_time + "COLOR EXPERIMENT 0,128,0")

            # Restricted airspaces
            scnfile.write("\n\n# LOAD RAA plugin and create area restrictions")
            scnfile.write(zero_time + f"PLUGINS LOAD {PLUGIN_NAME}")
            scnfile.write(zero_time + f"RAALOG {self.timestamp}")
            scnfile.write(zero_time + f"RAACONF {AREA_LOOKAHEAD_TIME}")
            for idx, area in enumerate(self.airspace_restrictions):
                coords = [x for (lat, lon) in area["poly"].exterior.coords
                          for x in (lat, lon)]
                coords_str = ",".join(str(f"{x:.6f}") for x in coords)
                scnfile.write(zero_time + f"RAA RAA{idx + 1},ON,{coords_str}")
                scnfile.write(zero_time + f"COLOR RAA{idx + 1},164,0,0")
                scnfile.write(zero_time + f"DEFWPT RAA_{idx + 1},"
                              + f"{area['midpoint_str']},FIX")

            # Swarming zones (skipped if no swarming zones are defined)
            if self.swarm_zones:
                scnfile.write("\n\n# Create SWARMING zone(s)")
            for swarm_zone in self.swarm_zones:
                coords = [x for (lat, lon)
                          in swarm_zone.exterior.coords
                          for x in (lat, lon)]
                coords_str = ",".join(str(f"{x:.6f}") for x in coords)
                scnfile.write(zero_time + f"POLY SWARMING_ZONE {coords_str}")
                scnfile.write(zero_time + "COLOR SWARMING_ZONE 255,255,255")

            # Geovectors (skipped if no geovectors are defined)
            if self.geovectors:
                scnfile.write("\n\n# Create GEOVECTOR area(s)")
            for geovector in self.geovectors:
                gv_coords = geovector["coords"]
                gv_coord_str = ",".join(str(f"{x:.6f}") for x in gv_coords)
                area_str = f"POLY {geovector['name']} {gv_coord_str}"
                scnfile.write(zero_time + area_str)
                scnfile.write(zero_time + (f"COLOR {geovector['name']},"
                                           + "102,178,255"))
                scnfile.write(zero_time + f"GEOVECTOR {geovector['name']},"
                              + f"{geovector['gs_min']},"
                              + f"{geovector['gs_max']},"
                              + f"{geovector['crs_min']},"
                              + f"{geovector['crs_max']},,")

            # Corridor waypoints
            scnfile.write("\n\n# Corridor waypoints")
            scnfile.write(zero_time + "DEFWPT COR101,{:.6f},{:.6f},FIX"
                          .format(self.corridor["south_lat"], CENTER_LON))
            scnfile.write(zero_time + "DEFWPT COR201,{:.6f},{:.6f},FIX"
                          .format(self.corridor["north_lat"], CENTER_LON))

            # Write each aircraft to file
            scnfile.write(f"\n\n# Create {self.num_created_ac} aircraft")
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

        with open(geofile_path, "w+") as geofile:
            # Write experiment area parameters to file
            geofile.write("{},{},{},{}\n".format(
                "ring", CENTER_LAT, CENTER_LON, AREA_RADIUS))

            # Write restriction coordinates to file
            for idx, area in enumerate(self.airspace_restrictions):
                geofile.write(f"RAA{idx+1},{area['coord_str']}\n")

    def write_geovectorfile(self):
        """
        Create csv file containing the geovector areas geometric parameters
        to allow generation of figures during post-processing.
        """

        if not self.geovectors:
            return

        file_name = (("{}_L{}_W{}_A{}_RESO-{}_geovector.csv")
                     .format(self.timestamp,
                             self.corridor_length,
                             self.corridor_width,
                             self.angle,
                             self.resolution_method))
        file_path = os.path.join(self.target_dir, file_name)

        with open(file_path, "w+") as geovectorfile:
            for idx, geovector in enumerate(self.geovectors):
                gs_min, gs_max = geovector["gs_min"], geovector["gs_max"]
                crs_min, crs_max = geovector["crs_min"], geovector["crs_max"]
                coords = geovector["coords"]
                coord_str = ",".join(str(f"{x:.6f}") for x in coords)
                geovectorfile.write(f"GV{idx+1},{gs_min},{gs_max},"
                                    + f"{crs_min},{crs_max},{coord_str}\n")


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

    angle_from_center = tg.ned2crs(qdr_from_center)

    return intersect_lat, intersect_lon, angle_from_center


if __name__ == "__main__":
    # Set the time stamp and save directory
    timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    output_dir = "scengen_test"

    # Get the settings combinations from the "test" section of the config file
    combination_list = scenario_config.parse("test")

    # Create a scenario file for each combination of variables
    for (random_seed, traffic_level, reso_method, corridor_length,
         corridor_width, arc_angle) in combination_list:

        ScenarioGenerator(output_dir,
                          timestamp,
                          random_seed,
                          traffic_level,
                          reso_method,
                          corridor_length,
                          corridor_width,
                          arc_angle,
                          is_edge_angle=False)
