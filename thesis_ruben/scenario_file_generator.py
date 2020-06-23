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

from matplotlib import pyplot as plt

class Geovector():
    """
    Class that stores and prints the data of a geovector.
    """

    def __init__(self, name, crs_min=None, crs_max=None,
                 gs_min_cas=None, gs_max_cas=None, poly=None, coords=None):
        self.name = name
        self.crs_min = crs_min
        self.crs_max = crs_max

        # Convert groundspeed from CAS [kts] to TAS [m/s]
        self.gs_min = cas2tas(gs_min_cas * 0.51444, 36000 * 0.3048) / 0.51444
        self.gs_max = cas2tas(gs_max_cas * 0.51444, 36000 * 0.3048) / 0.51444

        # Set up the polygon and coordinate representations based on
        # the input received
        if poly:
            self.poly = poly
            self.coords = [x for (lat, lon) in poly.exterior.coords
                           for x in (lat, lon)]
        elif coords:
            self.coords = coords
            self.poly = Polygon(coords)
        else:
            raise ValueError("No polygon or coordinates defined")

    def scn_str(self):
        """
        Print the geovector and its attributes in TrafScript format.
        """
        zero_time = "\n0:00:00.00>"
        coord_str = ",".join(str(f"{x:.6f}") for x in self.coords)
        poly_str = zero_time + f"POLY {self.name} {coord_str}"
        color_str = zero_time + f"COLOR {self.name},102,178,255"
        gv_str = (zero_time + f"GEOVECTOR {self.name}"
                  + "," + ("" if not self.gs_min else f"{self.gs_min:.0f}")
                  + "," + ("" if not self.gs_max else f"{self.gs_max:.0f}")
                  + "," + ("" if not self.crs_min else f"{self.crs_min:.0f}")
                  + "," + ("" if not self.crs_max else f"{self.crs_max:.0f}"))

        return poly_str + color_str + gv_str


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
        if "VELAVG" in self.resolution_method:
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
        self.write_swarmzonefile()

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

        def ring_polygon(center_lat, center_lon, radius):
            """
            Create a shapely Polygon for a ring with center at
            (center_lat, center_lon) with 'radius' in Nautical Miles.
            """
            return Polygon([bsgeo.qdrpos(center_lat, center_lon, angle, radius)
                            for angle in range(0, 360, 3)])

        # Experiment area
        ring_poly = ring_polygon(CENTER_LAT, CENTER_LON, AREA_RADIUS)
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
        merge_ring_poly = ring_polygon(CENTER_LON, CENTER_LON, 80)
        merge_circle_poly = merge_box_poly.intersection(merge_ring_poly)
        merge_wedge_poly = merge_circle_poly.difference(left_poly).difference(right_poly)
        self.polygons["merge_wedge"] = merge_wedge_poly

        # Coordinates of wedge shaped diverging area
        # First find bounding box using corridor north latitude and
        # experiment area bounds
        (min_lat, min_lon, max_lat, max_lon) = ring_poly.bounds
        div_box_coords = [(self.corridor["north_lat"], max_lon),
                          (max_lat, max_lon),
                          (max_lat, min_lon),
                          (self.corridor["north_lat"], min_lon)]
        div_box_poly = Polygon(div_box_coords)
        div_ring_poly = ring_polygon(CENTER_LON, CENTER_LON, 80)
        div_circle_poly = div_box_poly.intersection(div_ring_poly)
        div_wedge_poly = div_circle_poly.difference(left_poly).difference(right_poly)
        self.polygons["diverge_wedge"] = div_wedge_poly

        # Corwedge polygon: union of merge wedge and corridor area
        merge_wedge_cor_poly = merge_wedge_poly.union(corridor_poly)
        self.polygons["mergecorwedge"] = merge_wedge_cor_poly

        # Corwedge polygon: union of diverge wedge and corridor area
        div_wedge_cor_poly = div_wedge_poly.union(corridor_poly)
        self.polygons["divcorwedge"] = div_wedge_cor_poly

        # Non-overlapping concentric rings in wedge on converging side
        ring_radii = [40, 60, 80]  # [NM]
        prev_ring = None
        for radius in ring_radii:
            radius_poly = ring_polygon(CENTER_LAT, CENTER_LON, radius)
            wedge_ring_poly = radius_poly.intersection(merge_wedge_poly)
            if prev_ring:
                # Ensure no overlap with previous smaller ring
                wedge_ring_poly = wedge_ring_poly.difference(prev_ring)
            self.polygons[f"convring_{radius}"] = wedge_ring_poly
            prev_ring = radius_poly

        # Non-overlapping concentric rings in wedge on diverging side
        ring_radii = [40, 60, 80]  # [NM]
        prev_ring = None
        for radius in ring_radii:
            radius_poly = ring_polygon(CENTER_LAT, CENTER_LON, radius)
            wedge_ring_poly = radius_poly.intersection(div_wedge_poly)
            if prev_ring:
                # Ensure no overlap with previous smaller ring
                wedge_ring_poly = wedge_ring_poly.difference(prev_ring)
            self.polygons[f"divring_{radius}"] = wedge_ring_poly
            prev_ring = radius_poly

        # Triangles from center point splitting the merge zone in equal areas
        angle_list = [[135, 157.5], [157.5, 180], [180, 202.5], [202.5, 225]]
        for idx, anglepair in enumerate(angle_list):
            p1_lat, p1_lon = bsgeo.qdrpos(CENTER_LAT, CENTER_LON, anglepair[0], 150)
            p2_lat, p2_lon = bsgeo.qdrpos(CENTER_LAT, CENTER_LON, anglepair[1], 150)
            triangle_coords = [(CENTER_LAT, CENTER_LON),
                               (p1_lat, p1_lon),
                               (p2_lat, p2_lon)]
            triangle_poly = Polygon(triangle_coords)
            # cutoff_triangle_poly = triangle_poly.intersection(merge_wedge_poly)
            self.polygons[f"radtri_{idx}"] = triangle_poly

        # Box that encloses divergence speed box
        div_spd_box_south_lat, _ = bsgeo.qdrpos(
            CENTER_LAT, CENTER_LON, 0, self.corridor_length / 2 - 2
        )
        div_spd_box_coords = [(div_spd_box_south_lat, max_lon),
                              (max_lat, max_lon),
                              (max_lat, min_lon),
                              (div_spd_box_south_lat, min_lon)]
        div_spd_box = Polygon(div_spd_box_coords)
        self.polygons["divspd_box"] = div_spd_box

        # Diverging speed wedge
        radius = 45  # [NM]
        radius_poly = ring_polygon(CENTER_LAT, CENTER_LON, radius)
        div_spd_wedge = div_spd_box.difference(left_poly).difference(right_poly)
        wedge_ring_poly = radius_poly.intersection(div_spd_wedge)
        self.polygons[f"divspdring_{radius}"] = wedge_ring_poly

        # For debugging purposes
        # plt.plot(*ring_poly.exterior.xy)
        # plt.plot(*corridor_poly.exterior.xy)
        # plt.plot(*left_poly.exterior.xy)
        # plt.plot(*right_poly.exterior.xy)
        # plt.plot(*merge_wedge_poly.exterior.xy)
        # plt.plot(*merge_wedge_cor_poly.exterior.xy)
        # plt.plot(*self.polygons["radtri_0"].exterior.xy)
        # plt.plot(*self.polygons["radtri_1"].exterior.xy)
        # plt.plot(*self.polygons["radtri_2"].exterior.xy)
        # plt.plot(*self.polygons["radtri_3"].exterior.xy)
        # plt.plot(*div_spd_box.exterior.xy)
        # plt.plot(*div_wedge_poly.exterior.xy)
        # plt.plot(*div_wedge_cor_poly.exterior.xy)
        # plt.plot(*self.polygons["divring_40"].exterior.xy)
        # plt.plot(*self.polygons["divring_60"].exterior.xy)
        # plt.plot(*self.polygons["divring_80"].exterior.xy)
        # plt.show()

    def create_geovectors(self):
        """
        Create all geovector restrictions for the scenario.
        """

        # Default ground speed and course restrictions
        gs_min_cas = 260  # [kts]
        gs_max_cas = 270  # [kts]
        crs_min = 359  # [deg]
        crs_max = 1  # [deg]

        # Create geovector for resolution methods than only require a single
        # geovector.
        onevec_methods = ["CORRIDOR", "CIRCLE", "CORWEDGE", "WEDGE"]
        if any(substr in self.resolution_method for substr in onevec_methods):
            # Set the geovector coordinates
            if "CORRIDOR" in self.resolution_method:
                poly_name = "corridor"
            elif "CIRCLE" in self.resolution_method:
                poly_name = "ring"
            elif "CORWEDGE" in self.resolution_method:
                poly_name = "mergecorwedge"
            elif "WEDGE" in self.resolution_method:
                poly_name = "merge_wedge"

            geovector = Geovector(poly_name,
                                  crs_min=crs_min, crs_max=crs_max,
                                  gs_min_cas=gs_min_cas, gs_max_cas=gs_max_cas,
                                  poly=self.polygons[poly_name])
            self.geovectors.append(geovector)

            raise ValueError("Deprecated geovectoring method!")

        # Create geovectors for resolution methods that require multiple
        # geovectors.
        elif "METHOD1" in self.resolution_method:
            merge_area = self.polygons["mergecorwedge"].difference(
                self.polygons["divspd_box"]
            )
            div_area = self.polygons["divspdring_45"].intersection(
                self.polygons["divspd_box"]
            )

            geovector = Geovector("MERGE",
                                  gs_min_cas=265,
                                  gs_max_cas=265,
                                  poly=merge_area)
            self.geovectors.append(geovector)
            geovector = Geovector("DIVERGE",
                                  gs_min_cas=275,
                                  gs_max_cas=275,
                                  poly=div_area)
            self.geovectors.append(geovector)

        elif "METHOD2" in self.resolution_method:
            merge_area = self.polygons["mergecorwedge"].difference(
                self.polygons["corridor"]
            )
            cor_area = self.polygons["corridor"].difference(
                self.polygons["divspd_box"]
            )
            div_area = self.polygons["divspdring_45"].intersection(
                self.polygons["divspd_box"]
            )

            geovector = Geovector("MERGE",
                                  gs_min_cas=269,
                                  gs_max_cas=269,
                                  poly=merge_area)
            self.geovectors.append(geovector)
            geovector = Geovector("CORRIDOR",
                                  crs_min=355,
                                  crs_max=5,
                                  gs_min_cas=269,
                                  gs_max_cas=269,
                                  poly=cor_area)
            self.geovectors.append(geovector)
            geovector = Geovector("DIVERGE",
                                  gs_min_cas=275,
                                  gs_max_cas=275,
                                  poly=div_area)
            self.geovectors.append(geovector)

        elif "METHOD3" in self.resolution_method:
            convring80 = Geovector("convring_80",
                                   gs_min_cas=262,
                                   gs_max_cas=276,
                                   poly=self.polygons["convring_80"])
            convring60 = Geovector("convring_60",
                                   gs_min_cas=264,
                                   gs_max_cas=274,
                                   poly=self.polygons["convring_60"])
            convring40 = Geovector("convring_40",
                                   gs_min_cas=266,
                                   gs_max_cas=272,
                                   poly=self.polygons["convring_40"].difference(
                                       self.polygons["corridor"]))
            corridor = Geovector("corridor",
                                 gs_min_cas=269,
                                 gs_max_cas=269,
                                 crs_min=355,
                                 crs_max=5,
                                 poly=self.polygons["corridor"])
            divring40 = Geovector("divring_40",
                                  gs_min_cas=262,
                                  gs_max_cas=276,
                                  poly=self.polygons["divring_40"].difference(
                                       self.polygons["corridor"]))
            divring60 = Geovector("divring_60",
                                  gs_min_cas=264,
                                  gs_max_cas=274,
                                  poly=self.polygons["divring_60"])
            divring80 = Geovector("divring_80",
                                  gs_min_cas=262,
                                  gs_max_cas=276,
                                  poly=self.polygons["divring_80"])

            self.geovectors.append(convring40)
            self.geovectors.append(convring60)
            self.geovectors.append(convring80)
            self.geovectors.append(corridor)
            self.geovectors.append(divring40)
            self.geovectors.append(divring60)
            self.geovectors.append(divring80)

        # elif "CONCMERGE" in self.resolution_method:
        #     convring80 = Geovector("convring_80",
        #                            gs_min_cas=262,
        #                            gs_max_cas=276,
        #                            poly=self.polygons["convring_80"])
        #     convring60 = Geovector("convring_60",
        #                            gs_min_cas=264,
        #                            gs_max_cas=274,
        #                            poly=self.polygons["convring_60"])
        #     convring40 = Geovector("convring_40",
        #                            gs_min_cas=266,
        #                            gs_max_cas=272,
        #                            poly=self.polygons["convring_40"].difference(
        #                                self.polygons["corridor"]))
        #     corridor = Geovector("corridor",
        #                          gs_min_cas=269,
        #                          gs_max_cas=269,
        #                          crs_min=359,
        #                          crs_max=1,
        #                          poly=self.polygons["corridor"])

        #     self.geovectors.append(convring40)
        #     self.geovectors.append(convring60)
        #     self.geovectors.append(convring80)
        #     self.geovectors.append(corridor)

        elif "METHOD4" in self.resolution_method:
            # Corridor area
            cor_area = self.polygons["corridor"].difference(
                self.polygons["divspd_box"]
            )
            corridor = Geovector("corridor",
                                 gs_min_cas=269,
                                 gs_max_cas=269,
                                 crs_min=355,
                                 crs_max=5,
                                 poly=cor_area)
            self.geovectors.append(corridor)

            # Diverging area
            div_area = self.polygons["divspdring_45"]
            geovector = Geovector("DIVERGE",
                                  gs_min_cas=276,
                                  gs_max_cas=276,
                                  poly=div_area)
            self.geovectors.append(geovector)

            # Merging zone
            # 80-60 NM ring
            ring80_tri0 = self.polygons["radtri_0"].intersection(self.polygons["convring_80"])
            ring80_tri1 = self.polygons["radtri_1"].intersection(self.polygons["convring_80"])
            ring80_tri2 = self.polygons["radtri_2"].intersection(self.polygons["convring_80"])
            ring80_tri3 = self.polygons["radtri_3"].intersection(self.polygons["convring_80"])
            r80t0 = Geovector("ring80_tri0",
                              gs_min_cas=262, gs_max_cas=276,
                              crs_min=325, crs_max=327,
                              poly=ring80_tri0)
            self.geovectors.append(r80t0)
            r80t1 = Geovector("ring80_tri1",
                              gs_min_cas=262, gs_max_cas=276,
                              crs_min=348, crs_max=349,
                              poly=ring80_tri1)
            self.geovectors.append(r80t1)
            r80t2 = Geovector("ring80_tri2",
                              gs_min_cas=262, gs_max_cas=276,
                              crs_min=11, crs_max=12,
                              poly=ring80_tri2)
            self.geovectors.append(r80t2)
            r80t3 = Geovector("ring80_tri3",
                              gs_min_cas=262, gs_max_cas=276,
                              crs_min=33, crs_max=35,
                              poly=ring80_tri3)
            self.geovectors.append(r80t3)

            # 60-40 NM ring
            ring60_tri0 = self.polygons["radtri_0"].intersection(self.polygons["convring_60"])
            ring60_tri1 = self.polygons["radtri_1"].intersection(self.polygons["convring_60"])
            ring60_tri2 = self.polygons["radtri_2"].intersection(self.polygons["convring_60"])
            ring60_tri3 = self.polygons["radtri_3"].intersection(self.polygons["convring_60"])
            r60t0 = Geovector("ring60_tri0",
                              gs_min_cas=264, gs_max_cas=274,
                              crs_min=325, crs_max=327,
                              poly=ring60_tri0)
            self.geovectors.append(r60t0)
            r60t1 = Geovector("ring60_tri1",
                              gs_min_cas=264, gs_max_cas=274,
                              crs_min=348, crs_max=349,
                              poly=ring60_tri1)
            self.geovectors.append(r60t1)
            r60t2 = Geovector("ring60_tri2",
                              gs_min_cas=264, gs_max_cas=274,
                              crs_min=11, crs_max=12,
                              poly=ring60_tri2)
            self.geovectors.append(r60t2)
            r60t3 = Geovector("ring60_tri3",
                              gs_min_cas=264, gs_max_cas=274,
                              crs_min=33, crs_max=35,
                              poly=ring60_tri3)
            self.geovectors.append(r60t3)

            # 40 NM ring
            ring40_tri0 = self.polygons["radtri_0"].intersection(self.polygons["convring_40"])
            ring40_tri1 = self.polygons["radtri_1"].intersection(self.polygons["convring_40"])
            ring40_tri2 = self.polygons["radtri_2"].intersection(self.polygons["convring_40"])
            ring40_tri3 = self.polygons["radtri_3"].intersection(self.polygons["convring_40"])

            r40t0 = Geovector("ring40_tri0",
                              gs_min_cas=266, gs_max_cas=272,
                              crs_min=325, crs_max=327,
                              poly=ring40_tri0)
            self.geovectors.append(r40t0)
            r40t1 = Geovector("ring40_tri1",
                              gs_min_cas=266, gs_max_cas=272,
                              crs_min=348, crs_max=349,
                              poly=ring40_tri1)
            self.geovectors.append(r40t1)
            r40t2 = Geovector("ring40_tri2",
                              gs_min_cas=266, gs_max_cas=272,
                              crs_min=11, crs_max=12,
                              poly=ring40_tri2)
            self.geovectors.append(r40t2)
            r40t3 = Geovector("ring40_tri3",
                              gs_min_cas=266, gs_max_cas=272,
                              crs_min=33, crs_max=35,
                              poly=ring40_tri3)
            self.geovectors.append(r40t3)

            # For debugging purposes
            # plt.plot(*ring40_tri0.exterior.xy)
            # plt.plot(*ring40_tri1.exterior.xy)
            # plt.plot(*ring40_tri2.exterior.xy)
            # plt.plot(*ring40_tri3.exterior.xy)
            # plt.plot(*ring60_tri0.exterior.xy)
            # plt.plot(*ring60_tri1.exterior.xy)
            # plt.plot(*ring60_tri2.exterior.xy)
            # plt.plot(*ring60_tri3.exterior.xy)
            # plt.plot(*ring80_tri0.exterior.xy)
            # plt.plot(*ring80_tri1.exterior.xy)
            # plt.plot(*ring80_tri2.exterior.xy)
            # plt.plot(*ring80_tri3.exterior.xy)
            # plt.show()

    def create_swarm_zones(self):
        """
        Create
        """

        self.swarm_zones.append(self.polygons["mergecorwedge"])

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
                scnfile.write(geovector.scn_str())

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
            coords = [x for (lat, lon)
                      in self.polygons["experiment_area"].exterior.coords
                      for x in (lat, lon)]
            ring_coords_str = ",".join(str(f"{x:.6f}") for x in coords)
            geofile.write(f"ring,{ring_coords_str}\n")

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
                gs_min, gs_max = geovector.gs_min, geovector.gs_max
                crs_min, crs_max = geovector.crs_min, geovector.crs_max
                coords = geovector.coords
                coord_str = ",".join(str(f"{x:.6f}") for x in coords)
                geovectorfile.write(f"GV{idx+1},{gs_min},{gs_max},"
                                    + f"{crs_min},{crs_max},{coord_str}\n")

    def write_swarmzonefile(self):
        """
        Create csv file containing the swarm zone areas geometric parameters
        to allow generation of figures during post-processing.
        """

        if not self.swarm_zones:
            return

        file_name = (("{}_L{}_W{}_A{}_RESO-{}_swarmzone.csv")
                     .format(self.timestamp,
                             self.corridor_length,
                             self.corridor_width,
                             self.angle,
                             self.resolution_method))
        file_path = os.path.join(self.target_dir, file_name)

        with open(file_path, "w+") as swarmzonefile:
            for idx, swarm_zone in enumerate(self.swarm_zones):
                coords = [x for (lat, lon)
                          in swarm_zone.exterior.coords
                          for x in (lat, lon)]
                coord_str = ",".join(str(f"{x:.6f}") for x in coords)
                swarmzonefile.write(f"SWARMZONE{idx+1},{coord_str}\n")


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
