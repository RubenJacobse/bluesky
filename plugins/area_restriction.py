"""
Restricted Airspace Area Plugin

Uses the AreaRestrictionManager class to keep track of restricted
areas wich are themselves are represented by instances of the
RestrictedAirspaceArea class.

Current implementation is heavily work-in-progress and unstable.

© Ruben Jacobse, 2019
"""

# General imports
from collections.abc import Collection

# Third-party imports
import numpy as np
import shapely.geometry as spgeom
import shapely.ops as spops

# BlueSky imports
import bluesky as bs
from bluesky.tools import areafilter
from bluesky.tools.aero import Rearth
from bluesky.tools.geo import qdrdist
from bluesky.tools.trafficarrays import TrafficArrays, RegisterElementParameters

# Temporary import
from tempgeo import rhumb_azimuth

# Default variable values for numpy arrays
VAR_DEFAULTS = {"float": 0.0, "int": 0, "bool": False, "S": "", "str": "", "object": None}
AREA_AVOIDANCE_CRS_MARGIN = 1 # [deg] Course margin for area avoidance
COMMANDED_CRS_MARGIN = 0.2 # [deg] Used to verify if a commanded heading has been reached
NM_TO_M = 1852. # Conversion factor nautical miles to metres


def init_plugin():
    """Initialize the RAA plugin"""

    # Addtional initilisation code
    areas = AreaRestrictionManager()

    # Configuration parameters
    config = {
        # The name of your plugin
        "plugin_name": "RAA",

        # The type of this plugin. For now, only simulation plugins are possible.
        "plugin_type": "sim",

        # Update interval in seconds. By default, your plugin's update function(s)
        # are called every timestep of the simulation. If your plugin needs less
        # frequent updates provide an update interval.
        "update_interval": 1.0,

        # The update function is called after traffic is updated. Use this if you
        # want to do things as a result of what happens in traffic. If you need to
        # something before traffic is updated please use preupdate.
        "update": areas.update,

        # The preupdate function is called before traffic is updated. Use this
        # function to provide settings that need to be used by traffic in the current
        # timestep. Examples are ASAS, which can give autopilot commands to resolve
        # a conflict.
        "preupdate": areas.preupdate,

        # If your plugin has a state, you will probably need a reset function to
        # clear the state in between simulations.
        "reset": areas.reset,

        # The remove functin is called before the plugin is removed and can be used to
        # clear any references to the plugin.
        "remove": areas.remove,
    }

    stackfunctions = {
        # The command name for your function
        "RAA": [
            # A short usage string. This will be printed if you type HELP <name> in the
            # BlueSky console
            "RAA name, ON/OFF, gsnorth, gseast, [lat1,lon1,lat2,lon2,...]",

            # A list of the argument types your function accepts. For a description of this, see...
            "txt,onoff,spd,spd,[latlon,...]",

            # The name of your function in this plugin
            areas.create_area,

            # A longer help text of your function.
            "Create restricted airspace areas that are to be avoided by all traffic."],
        "DELRAA": [
            # A short usage string. This will be printed if you type HELP <name> in the
            # BlueSky console
            "DELRAA name",

            # A list of the argument types your function accepts. For a description of this, see...
            "txt",

            # The name of your function in this plugin
            areas.delete_area,

            # a longer help text of your function.
            "Delete a given restricted airspace area."],
        "RAACONF": [
            # A short usage string. This will be printed if you type HELP <name> in the
            # BlueSky console
            "RAACONF t_lookahead",

            # A list of the argument types your function accepts. For a description of this, see ...
            "int",

            # The name of your function in this plugin
            areas.set_t_lookahead,

            # a longer help text of your function.
            "Set the lookahead time used for area avoidance in seconds."]
    }

    # init_plugin() should always return these two dicts.
    return config, stackfunctions


class AreaRestrictionManager(TrafficArrays):
    """
    This class implements the avoidance of Restricted Airspace Areas.
    """

    def __init__(self):
        # Initialize TrafficArrays base class
        super().__init__()

        self._ndArrVars = []

        # Register all traffic parameters relevant for this class
        with RegisterElementParameters(self):
            # =================================================================================
            # N-dimensional parameters where each column is an aircraft and each row is an area
            # =================================================================================
            self.rel_gseast = np.array([[]]) # [m/s] East component of relative velocity wrt areas
            self.rel_gsnorth = np.array([[]]) # [m/s] North component of relative velocity wrt areas
            self.rel_gs = np.array([[]]) # [m/s] Magnitude of ac relative velocity wrt areas
            self.brg_left_tangent = np.array([[]]) # [deg] Bearing from ac to leftmost vertex in N-E-D [-180..180]
            self.brg_right_tangent = np.array([[]]) # [deg] Bearing from ac to rightmost vertex in N-E-D [-180..180]
            self.crs_left_tangent = np.array([[]]) # [deg] Compass course from ac to leftmost vertex [0..360]
            self.crs_right_tangent = np.array([[]]) # [deg] Compass course from ac to rightmost vertex [0..360]
            self.is_inside = np.array([[]], dtype = bool) # Stores whether ac is inside area
            self.is_in_conflict = np.array([[]], dtype = bool) # Stores whether ac is in conflict with each area
            self.time_to_intrusion = np.array([[]])  # [s] Time to area intrusion

            # ======================================================
            # Traffic parameters that are 1-dimensional numpy arrays
            # ======================================================

            # Waypoint related
            self.idx_active_wp = np.array([], dtype = int) # index of active waypoint
            self.idx_next_wp = np.array([], dtype = int) # index of next waypoint
            self.crs_to_active_wp = np.array([]) # [deg] Magnetic course to current waypoint
            self.crs_to_next_wp = np.array([]) # [deg] Magnetic course to current waypoint

            # Resolution related
            self.reso_dv_east = np.array([]) # [m/s] Resolution velocity change east component
            self.reso_dv_north = np.array([]) # [m/s] Resolution velocity change north component
            self.is_in_area_reso = np.array([], dtype = bool)
            self.is_in_aircraft_reso = np.array([], dtype = bool)
            self.commanded_crs = np.array([])
            self.commanded_spd = np.array([])

            # Keep track of the avoidance state of each aircraft
            self.is_in_area_conflict_mode = np.array([], dtype = bool)
            self.is_in_aircraft_conflict_mode = np.array([], dtype = bool)
            self.is_in_route_following_mode = np.array([], dtype = bool)

            # =======================
            # Traffic parameter lists
            # =======================
            self.first_conflict_area_idx = [] # Store index of closest conflicting area
            self.current_position = []  # Shapely Point with current aircraft positions
            self.relative_track = []  # Shapely LineString with relative tracks

        # Keep track of all restricted areas in list and by ID (same order)
        self.areas = []
        self.area_ids = []
        self.num_areas = 0
        self.num_traf = 0

        # Default look-ahead-time in seconds, used to detect aircraft-area conflicts
        self.t_lookahead = 300

    def MakeParameterLists(self, keys):
        """
        Override default TrafficArrays.MakeParameterLists() to include
        support for n-dimensional numpy arrays.
        """

        for key in keys:
            # Parameters of type list are added to self._LstVars
            # Parameters of type numpy.ndarray are added to self._ArrVars if they are
            # one-dimensional or to self._ndArrVars if they are multidimensional.
            # Parameters of type TrafficArrays are added to the list of children
            if isinstance(self._Vars[key], list):
                self._LstVars.append(key)
            elif isinstance(self._Vars[key], np.ndarray):
                if np.ndim(self._Vars[key]) == 1:
                    self._ArrVars.append(key)
                elif np.ndim(self._Vars[key]) > 1:
                    self._ndArrVars.append(key)
            elif isinstance(self._Vars[key], TrafficArrays):
                self._Vars[key].reparent(self)

    def create(self, n = 1):
        """
        Append n elements (aircraft) to all lists and arrays.

        Overrides TrafficArrays.create(n) method to allow handling of
        two-dimensional numpy arrays.
        """

        self.num_traf += n

        # Lists (mostly used for strings)
        for var in self._LstVars:
            # Get type
            vartype = None
            lst = self.__dict__.get(var)
            if len(lst) > 0:
                vartype = str(type(lst[0])).split("'")[1]

            if vartype in VAR_DEFAULTS:
                defaultvalue = [VAR_DEFAULTS[vartype]] * n
            else:
                defaultvalue = [""] * n

            self._Vars[var].extend(defaultvalue)

        # One dimensional numpy arrays
        for var in self._ArrVars:

            # Get type without byte length
            fulltype = str(self._Vars[var].dtype)
            vartype = ""
            for c in fulltype:
                if not c.isdigit():
                    vartype = vartype + c

            # Get default value
            if vartype in VAR_DEFAULTS:
                defaultvalue = [VAR_DEFAULTS[vartype]] * n
            else:
                defaultvalue = [0.0] * n

            self._Vars[var] = np.append(self._Vars[var], defaultvalue)

        # Allow two-dimensional numpy arrays in self._ndArrVars
        # Each row can now represent an airspace restriction

        # If no areas exist do nothing (there should be 0 rows if no areas)
        if not self.num_areas:
            return

        for var in self._ndArrVars:  # Numpy array
            fulltype = str(self._Vars[var].dtype)
            vartype = ""
            for c in fulltype:
                if not c.isdigit():
                    vartype = vartype + c

            # Get default value
            if vartype in VAR_DEFAULTS:
                defaultvalue = [VAR_DEFAULTS[vartype]]
            else:
                defaultvalue = [0.0]

            # Create add n columns to existing array
            new_cols = np.full((self.num_areas, n), defaultvalue)
            if not self._Vars[var].size:
                self._Vars[var] = new_cols
            else:
                self._Vars[var] = np.concatenate((self._Vars[var], new_cols), 1)

    def delete(self, idx):
        """
        Delete element (aircraft) idx from all lists and arrays.

        Overrides TrafficArrays.delete(idx) method to allow handling of
        two-dimensional numpy arrays.
        """

        if isinstance(idx, Collection):
            idx = np.sort(idx)
            dec = len(idx)
        else:
            dec = 1

        for var in self._LstVars:
            if isinstance(idx, Collection):
                for i in reversed(idx):
                    del self._Vars[var][i]
            else:
                del self._Vars[var][idx]

        for var in self._ArrVars:
            self._Vars[var] = np.delete(self._Vars[var], idx)

        # Delete entire column idx from v (column = dimension 1)
        for var in self._ndArrVars:
            self._Vars[var] = np.delete(self._Vars[var], idx, 1)

        self.num_traf -= dec

        return True

    def reset(self):
        """ Reset state on simulator reset event. """

        # Delete all traffic parameters
        for var in self._LstVars:
            self._Vars[var] = []

        for var in self._ArrVars:
            self._Vars[var] = np.array([], dtype = self._Vars[var].dtype)

        for var in self._ndArrVars:
            self._Vars[var] = np.array([[]], dtype = self._Vars[var].dtype)

        # Make sure all areas are deleted
        for area in self.areas:
            area.delete()
            self.areas.remove(area) # Probably redundant

        # Reset default values
        self.areas = []
        self.area_ids = []
        self.num_areas = 0
        self.num_traf = 0
        self.t_lookahead = 300

    def remove(self):
        """ Called when plugin is removed. """

        # Remove self from the TrafficArrays tree
        if self._parent:
            self._parent._children.remove(self)

    def preupdate(self):
        """ Update the area positions before traf is updated. """

        # NOTE: Not sure if this should be part of preupdate() or update()
        for area in self.areas:
            area.update_pos(1.0)

    def update(self):
        """ Do area avoidance calculations after traf has been updated. """

        # Cannot do any calculations if no aircraft or areas exist
        if not self.num_traf or not self.num_areas:
            return

        # Find courses to active and next waypoint
        self.calculate_courses_to_waypoints()

        # Identify all aircraft-area conflicts
        self.calculate_positions_and_relative_tracks()
        for area_idx, area in enumerate(self.areas):
            self.find_bearings_tangent_to_area(area_idx, area)
            self.find_ac_inside_and_inconflict(area_idx, area)
            self.calculate_time_to_intrusion(area_idx, area)

        # Calculate resolution vectors for each aircraft's closest area conflict
        self.calculate_area_resolution_vectors()

        # Determine which state each aircraft is in and take appropriate action
        self.determine_aircraft_mode()
        self.apply_mode_based_action()

    def create_area(self, area_id, area_status, gsnorth, gseast, *coords):
        """ Create a new restricted airspace area """

        if area_id in self.area_ids:
            return False, "Error: Airspace restriction {} already exists".format(area_id)

        # Create new RestrictedAirspaceArea instance and add to internal lists
        new_area = RestrictedAirspaceArea(area_id, area_status, gseast, gsnorth, list(coords))

        self.areas.append(new_area)
        self.area_ids.append(area_id)

        for var in self._LstVars:  # Lists (mostly used for strings)

            # Get type
            vartype = None
            lst = self.__dict__.get(var)
            if lst:
                vartype = str(type(lst[0])).split("'")[1]

            if vartype in VAR_DEFAULTS:
                defaultvalue = [VAR_DEFAULTS[vartype]] * bs.traf.ntraf
            else:
                defaultvalue = [""] * bs.traf.ntraf

            if not self._Vars[var]:
                self._Vars[var] = defaultvalue

        # Add row to all numpy arrays
        for var in self._ndArrVars:

            # Get numpy dtype without byte length
            dtype_str = str(self._Vars[var].dtype)
            for dtype in ["int", "float", "bool"]:
                if dtype in dtype_str:
                    vartype = dtype
                    break

            # Get default value
            if vartype in VAR_DEFAULTS:
                defaultvalue = [VAR_DEFAULTS[vartype]]
            else:
                defaultvalue = [0.0]

            # Adds row of defaultvalue to existing array
            newrow = np.full((1, bs.traf.ntraf), defaultvalue)

            if not self.num_areas:
                self._Vars[var] = newrow
            else:
                self._Vars[var] = np.vstack((self._Vars[var], newrow))

        # Increment after adding variable elements
        self.num_areas += 1

        return True, "Restricted Airspace Area {} is initialized".format(area_id)

    def delete_area(self, area_id):
        """ Delete an existing restricted airspace area. """

        if area_id not in self.area_ids:
            return False, "Error: Airspace restriction {} does not exist".format(area_id)
        else:
            # Find index of area
            idx = self.area_ids.index(area_id)

            # Call object delete method before deleting the object
            self.areas[idx].delete()

            # Delete from all lists and arrays
            del self.areas[idx]
            del self.area_ids[idx]
            self.num_areas -= 1

            # Delete row corresponding to area from all numpy arrays
            for var in self._ndArrVars:
                self._Vars[var] = np.delete(self._Vars[var], idx, 0)

            return True, "Sucessfully deleted airspace restriction {}".format(area_id)

    def set_t_lookahead(self, t):
        """ Change the look-ahead-time used for aircraft-area conflict detection. """

        if not isinstance(t, int):
            return False, "Error: Look-ahead-time should be an integer value"
        else:
            self.t_lookahead = t
            return True, "Aircraft-area conflict look-ahead-time set to {} seconds".format(t)

    def calculate_courses_to_waypoints(self):
        """
        For all aircraft calculate the course to the active and next
        waypoint.
        """

        # Find course to active waypoint
        qdr_to_active_wp, _ = qdrdist(bs.traf.lat,
                                      bs.traf.lon,
                                      bs.traf.actwp.lat,
                                      bs.traf.actwp.lon)

        self.crs_to_active_wp = ned2crs(qdr_to_active_wp)

        # Finding the course to the next waypoint is more tricky (if it even exists,
        # otherwise use the active waypoint which should always exist)
        nextwp_lat = np.zeros(np.shape(self.crs_to_next_wp))
        nextwp_lon = np.zeros(np.shape(self.crs_to_next_wp))
        for ac_idx in range(self.num_traf):

            self.idx_active_wp[ac_idx] = bs.traf.ap.route[ac_idx].iactwp
            if bs.traf.ap.route[ac_idx].nwp >= self.idx_active_wp[ac_idx] + 1:
                self.idx_next_wp[ac_idx] = self.idx_active_wp[ac_idx] + 1
            else:
                self.idx_next_wp[ac_idx] = self.idx_active_wp[ac_idx]

            nextwp_lat[ac_idx] = bs.traf.ap.route[ac_idx].wplat[self.idx_next_wp[ac_idx]]
            nextwp_lon[ac_idx] = bs.traf.ap.route[ac_idx].wplon[self.idx_next_wp[ac_idx]]

        qdr_to_next_wp, _ = qdrdist(bs.traf.lat, bs.traf.lon, nextwp_lat, nextwp_lon)
        self.crs_to_next_wp = ned2crs(qdr_to_next_wp)

    def calculate_positions_and_relative_tracks(self):
        """
        Calculate the relevant position parameters used in aircraft-area
        conflict detection.
        """

        # Represent each aircraft position as shapely point
        self.current_position = [spgeom.Point(lon, lat) \
                                    for (lon, lat) in zip(bs.traf.lon, bs.traf.lat)]

        # NOTE: Could this be vectorized instead of looped over all aircraft-area combinations?
        for area_idx, area in enumerate(self.areas):

            self.rel_gseast[area_idx, :] = bs.traf.gseast - area.gseast
            self.rel_gsnorth[area_idx, :] = bs.traf.gsnorth - area.gsnorth

            # Calculate position of each aircraft relative to the area after lookahead time
            future_relative_lon, future_relative_lat = \
                                                calc_future_pos(self.t_lookahead,
                                                                bs.traf.lon,
                                                                bs.traf.lat,
                                                                self.rel_gseast[area_idx, :],
                                                                self.rel_gsnorth[area_idx, :])

            # Create shapely points for future relative position
            future_relative_position = [spgeom.Point(lon, lat) for (lon, lat) \
                                            in zip(future_relative_lon, future_relative_lat)]

            # Use current and future relative position to calculate relative track
            self.relative_track[area_idx] = \
                [spgeom.LineString([curr, fut]) for (curr, fut) \
                    in zip(self.current_position, future_relative_position)]

        # Magnitude of relative velocities
        self.rel_gs = np.sqrt(self.rel_gseast ** 2 + self.rel_gsnorth ** 2)

    def find_bearings_tangent_to_area(self, area_idx, area):
        """ For each aircraft find the tangent bearings to the current area ."""

        # Constant heading requires loxodrome (rhumb line) angle
        self.brg_left_tangent[area_idx, :], self.brg_right_tangent[area_idx, :] \
            = area.calc_rhumb_tangents(bs.traf.ntraf, bs.traf.lon, bs.traf.lat)

        # Convert to course on [0..360] degrees
        self.crs_left_tangent[area_idx, :] = self.brg_left_tangent[area_idx, :] % 360
        self.crs_right_tangent[area_idx, :] = self.brg_right_tangent[area_idx, :] % 360

    def find_ac_inside_and_inconflict(self, area_idx, area):
        """ Check whether each aircraft is inside the area. """

        for ac_idx in range(self.num_traf):
            self.is_inside[area_idx, ac_idx] = \
                self.current_position[ac_idx].within(area.poly)

            self.is_in_conflict[area_idx, ac_idx] = \
                area.ring.intersects(self.relative_track[area_idx][ac_idx])

    def calculate_time_to_intrusion(self, area_idx, area):
        """
        Calculate time-to-intersection [s] for each aircraft-area
        combination, the result can take following values:

        equals -1     : for aircraft not in conflict with the area 0
        equals 0      : for aircraft already inside the area
        higher than 0 : for aircraft that are in conflict
        """

        for ac_idx in range(self.num_traf):
            if self.is_inside[area_idx, ac_idx]:
                t_int = 0   # NOTE: If area avoidance works properly, this should never happen!
            elif self.is_in_conflict[area_idx, ac_idx]:
                # Find intersection points of the relative vector with the area and use the
                # distance to the closest point to calculate time-to-intersection. (We cannot
                # use shapely distance functions because vertex definitions are in degrees).
                intr_points = area.ring.intersection(self.relative_track[area_idx][ac_idx])
                intr_closest = spops.nearest_points(self.current_position[ac_idx], intr_points)[1]
                intr_closest_lat, intr_closest_lon = intr_closest.y, intr_closest.x
                _, intr_dist_nm = qdrdist(bs.traf.lat[ac_idx],
                                          bs.traf.lon[ac_idx],
                                          intr_closest_lat,
                                          intr_closest_lon)
                intr_dist_m = intr_dist_nm * NM_TO_M  # qdrdist returns dist in NM, convert to m
                t_int = intr_dist_m / bs.traf.gs[ac_idx]
            else:
                t_int = -1

            self.time_to_intrusion[area_idx, ac_idx] = t_int

    def calculate_area_resolution_vectors(self):
        """
        Calculate the resolution vectors for all aircraft that are in
        conflict with an area.
        """

        # Ensure delta v vectors are reset each time step
        self.reso_dv_east = np.zeros(np.shape(bs.traf.id))
        self.reso_dv_north = np.zeros(np.shape(bs.traf.id))

        # Loop over all aircraft-area combinations and for each aircraft store the
        # index of the area where the time to intrusion is the smallest positive number.
        for ac_idx, t_int_column in enumerate(self.time_to_intrusion.T):
            for area_idx, t_int in enumerate(t_int_column):
                if not self.first_conflict_area_idx[ac_idx] and t_int > 0:
                    self.first_conflict_area_idx[ac_idx] = area_idx
                elif (self.first_conflict_area_idx[ac_idx]
                      and t_int > 0
                      and t_int < self.first_conflict_area_idx[ac_idx]):
                    self.first_conflict_area_idx[ac_idx] = area_idx

        # Find indices of conflicting aircraft-area pairs
        conflict_pairs = [(ac_idx, area_idx) for ac_idx, area_idx \
                            in enumerate(self.first_conflict_area_idx) if area_idx is not None]

        # Per area, calculate resolution vectors for all aircraft conflicting with that area
        for area_idx in range(self.num_areas):
            ac_indices = [ac for ac, area in conflict_pairs if area == area_idx]

            if ac_indices:
                self.calculate_resolutions_for_single_area(area_idx, ac_indices)

    def calculate_resolutions_for_single_area(self, area_idx, ac_indices):
        """
        Calculate the resolution manoeuvres (heading and speed) for all
        aircraft in conflict with a given area.
        """

        area = self.areas[area_idx]
        ac_gs = bs.traf.gs[ac_indices]

        # Components of unit vectors along left and right VO edges
        u_l_east = np.sin(np.radians(self.brg_left_tangent[area_idx, ac_indices]))
        u_l_north = np.cos(np.radians(self.brg_left_tangent[area_idx, ac_indices]))
        u_r_east = np.sin(np.radians(self.brg_right_tangent[area_idx, ac_indices]))
        u_r_north = np.cos(np.radians(self.brg_right_tangent[area_idx, ac_indices]))

        # For heading-change-only resolution maneuvers, the resolution vector lies on the edge of
        # the Velocity Obstacle (or Collision Cone if area is non-moving). The vector magnitudes
        # are the same as the current aircraft ground speeds (assuming zero wind).
        if area.gs:
            # Find angles between -vrel and left- and right edges of VO using dot
            # product of -vrel and the unit vectors along the VO edges
            beta_l_rad = np.arccos(-1 * (u_l_east * area.gseast + \
                                         u_l_north * area.gsnorth) / (area.gs**2))
            beta_r_rad = np.arccos(-1 * (u_r_east * area.gseast + \
                                         u_r_north * area.gsnorth) / (area.gs**2))

            # Calculate relative resolution velocity component along the VO edges
            v_ul = ac_gs * np.cos(beta_l_rad) + ac_gs \
                    * np.cos(np.arcsin(ac_gs * np.sin(beta_l_rad) / ac_gs))
            v_ur = ac_gs * np.cos(beta_r_rad) + ac_gs \
                    * np.cos(np.arcsin(ac_gs * np.sin(beta_r_rad) / ac_gs))
        else:
            # Resolution velocity magnitudes on left- and right edges of CC
            v_ul = ac_gs
            v_ur = ac_gs

        # Calculate north and east components of left- and right resolution absolute velocities
        # (For a non moving area, area.gseast and area.gsnorth are simply 0, but for a moving area
        # these terms are required to get the absolute resolution velocity of each aircraft.)
        vres_l_east = u_l_east * v_ul - area.gseast
        vres_l_north = u_l_north * v_ul - area.gsnorth
        vres_r_east = u_r_east * v_ur - area.gseast
        vres_r_north = u_r_north * v_ur - area.gsnorth

        # Calculate magnetic tracks of left- and right resolution vectors
        vres_l_crs = (np.degrees(np.arctan2(vres_l_east, vres_l_north)) \
                        - AREA_AVOIDANCE_CRS_MARGIN) % 360
        vres_r_crs = (np.degrees(np.arctan2(vres_r_east, vres_r_north)) \
                        + AREA_AVOIDANCE_CRS_MARGIN) % 360

        # Determine which of the two resolution vectors should be used based on the course to
        # the current active waypoint. We choose the course with the smallest angle difference
        wp_crs = self.crs_to_active_wp[ac_indices]
        new_crs = crs_closest(wp_crs, vres_l_crs, vres_r_crs)

        # Calculate new velocities (currently easiest method to calculate delta v)
        new_v = ac_gs
        new_v_east = np.sin(np.radians(new_crs)) * new_v
        new_v_north = np.cos(np.radians(new_crs)) * new_v

        # Calculate velocity changes
        self.reso_dv_east[ac_indices] = new_v_east - bs.traf.gseast[ac_indices]
        self.reso_dv_north[ac_indices] = new_v_north - bs.traf.gsnorth[ac_indices]

    def determine_aircraft_mode(self):
        """
        Determine which mode each aircraft is in:
            - in area conflict
            - in aircraft conflict
            - in route following
        """

        for ac_idx in range(self.num_traf):
            # Determine which aircraft are in an aircraft-area conflict
            if self.is_in_conflict[:, ac_idx].any():
                self.is_in_area_conflict_mode[ac_idx] = True
            else:
                self.is_in_area_conflict_mode[ac_idx] = False

            # Determine which aircraft are in an aircraft-aircraft conflict
            # NOTE: To be implemented, for now always set to False
            self.is_in_aircraft_conflict_mode[ac_idx] = False

            # Can only be in route following mode if not in either aircraft or area conflict
            if self.is_in_area_conflict_mode[ac_idx] or self.is_in_aircraft_conflict_mode[ac_idx]:
                self.is_in_route_following_mode[ac_idx] = False
            else:
                self.is_in_route_following_mode[ac_idx] = True

    def apply_mode_based_action(self):
        """
        Apply the new velocities for all aircraft that need to perform
        a resolution manoeuver.
        """

        # NOTE: Some of the branches of the if statements inside this loop are redundant, but
        # are still explicitly included to improve readability.
        for ac_idx in range(self.num_traf):
            if self.is_in_area_conflict_mode[ac_idx]:
                if self.is_in_area_reso[ac_idx]:
                    # Aircraft is already in a resolution manoeuver, no action required.
                    pass
                else:
                    # Aircraft is not in a resolution manoeuver, must initiate a new one
                    print("{} new resolution manoeuver initiated".format(bs.traf.id[ac_idx]))

                    new_v_east = bs.traf.gseast[ac_idx] + self.reso_dv_east[ac_idx]
                    new_v_north = bs.traf.gsnorth[ac_idx] + self.reso_dv_north[ac_idx]
                    new_v = np.sqrt(new_v_east ** 2 + new_v_north ** 2)
                    new_crs = np.degrees(np.arctan2(new_v_east, new_v_north)) % 360

                    self.stack_reso_apply(ac_idx, new_crs, new_v)
                    self.is_in_area_reso[ac_idx] = True
                    self.commanded_crs[ac_idx] = new_crs
                    self.commanded_spd[ac_idx] = new_v
            else:
                if self.is_in_area_reso[ac_idx]:
                    # Check if current course is within margin of the resolution commanded course
                    current_crs = ned2crs(bs.traf.trk[ac_idx])
                    delta_crs = abs(current_crs - self.commanded_crs[ac_idx])
                    if delta_crs < COMMANDED_CRS_MARGIN:
                        # Course is within margin, reset resolution variables
                        print("{} resolution manoeuver completed!".format(bs.traf.id[ac_idx]))

                        self.is_in_area_reso[ac_idx] = False
                        self.commanded_crs[ac_idx] = 0
                        self.commanded_spd[ac_idx] = 0
                    else:
                        # Commanded course has not yet been reached, take no action
                        pass
                else:
                    # The aircraft is not in a resolution manoeuver, take no action
                    pass

            if self.is_in_aircraft_conflict_mode[ac_idx] \
                        and not self.is_in_area_conflict_mode[ac_idx]:
                pass

            if self.is_in_route_following_mode[ac_idx]:
                pass

    def stack_reso_apply(self, ac_idx, crs, tas):
        """ Apply all resolution vectors via the BlueSky stack. """

        ac_id = bs.traf.id[ac_idx]
        ac_crs = crs
        ac_alt = bs.traf.alt[ac_idx]
        ac_cas = bs.tools.aero.vtas2cas(tas, ac_alt)

        hdg_cmd = "HDG {},{:.2f}".format(ac_id, ac_crs)
        bs.stack.stack(hdg_cmd)
        # spd_cmd = "SPD {},{}".format(ac_id, ac_cas)
        # bs.stack.stack(spd_cmd)

        # Print command to python console
        # print("{} is {}in area conflict with {}".format(bs.traf.id[ac_idx], \
        #       "" if np.any(self.is_in_conflict[:, ac_idx]) else "not ", \
        #                                                 self.first_conflict_area_idx[ac_idx]))
        # print(hdg_cmd)
        # print(spd_cmd)


class RestrictedAirspaceArea():
    """ Class that represents a single Restricted Airspace Area. """

    def __init__(self, area_id, status, gseast, gsnorth, coords):

        # Store input parameters as attributes
        self.area_id = area_id
        self.status = status
        self.gsnorth = gsnorth
        self.gseast = gseast
        self.gs = np.sqrt(gsnorth**2 + gseast**2)

        # Enforce that the coordinate list defines a valid ring
        coords = self._check_poly(coords)

        # Area coordinates will be stored in four formats:
        #  - self.verts  : numpy array containing [lon, lat] pairs per vertex
        #  - self.ring   : Shapely LinearRing (in lon,lat order) for geometric calculations
        #  - self.poly   : Shapely Polygon (in lon, lat order) for geometric calculations
        #  - self.coords : List of sequential lat,lon pairs for BlueSky functions
        self.verts = self._coords2verts(coords)
        self.ring = spgeom.LinearRing(self.verts)
        self.poly = spgeom.Polygon(self.verts)
        self.coords = coords

        # Numpy array with ground speed vector of each vertex
        self.gs_verts = np.full(np.shape(self.verts), [self.gseast, self.gsnorth])

        # Draw polygon on BlueSky RadarWidget canvas
        self._draw()

    def update_pos(self, dt):
        """
        Update the position of the area (only if its groundspeed is
        nonzero). Recalculates the coordinates and updates the polygon
        position drawn in the BlueSky RadarWidget.
        """

        if self.gsnorth or self.gseast:
            # Calculate new vertex positions after timestep dt
            curr_lon = self.verts[:, 0]
            curr_lat = self.verts[:, 1]
            newlon, newlat = calc_future_pos(dt, curr_lon, curr_lat, self.gseast, self.gsnorth)

            # Update vertex representations in all formats
            self.verts = np.array([newlon, newlat]).T
            self.ring = spgeom.LinearRing(self.verts)
            self.poly = spgeom.Polygon(self.verts)
            self.coords = self._verts2coords(self.verts)

            # Remove current drawing and redraw new position on BlueSky RadarWidget canvas
            # NOTE: Can probably be improved by a lot!
            self._undraw()
            self._draw()

    def delete(self):
        """
        On deletion, remove the drawing of current area from the
        BlueSky RadarWidget canvas.
        """

        self._undraw()

    def _check_poly(self, coords):
        """
        During initialization check that the user specified polygon
        is valid.

        - Vertices shall form a closed ring (first and last vertex are the same)
        - Vertices shall be ordered counterclockwise

        If this is not already the case then create a valid polygon.
        """

        # Make sure the border is a closed ring (first and last coordinate pair should be the same)
        if (coords[0], coords[1]) != (coords[-2], coords[-1]):
            coords = coords + [coords[0], coords[1]]

        # If the coordinates are not ordered in ccw direction, then rearrange the coords
        if not self.is_ccw(coords):
            reordered = []
            for ii in range(0, len(coords) - 1, 2):
                reordered = [coords[ii], coords[ii + 1]] + reordered
            coords = reordered

        return coords

    def _coords2verts(self, coords):
        """
        Convert list with coords in lat,lon order to numpy array of
        vertex pairs in lon,lat order.

        coords = [lat_0, lon_0, ..., lat_n, lon_n] \n
        verts = np.array([[lon_0, lat_0], ..., [lon_n, lat_n]])

        (This is the inverse operation of self._verts2coords).
        """

        verts_latlon = np.reshape(coords, (len(coords) // 2, 2))
        verts_lonlat = np.flip(verts_latlon, 1)

        return verts_lonlat

    def _verts2coords(self, verts):
        """
        Convert numpy array of vertex coordinate pairs in lon,lat order to
        a single list of lat,lon coords.

        verts = np.array([[lon_0, lat_0], ..., [lon_n, lat_n]]) \n
        coords = [lat_0, lon_0, ..., lat_n, lon_n]

        (Essentially the inverse operation of self._coords2verts).
        """

        verts_latlon = np.flip(verts, 1)
        coords_latlon = list(verts_latlon.flatten("C"))

        return coords_latlon

    def _draw(self):
        """ Draw the polygon corresponding to the current area in the
            RadarWidget window. """

        areafilter.defineArea(self.area_id, "POLY", self.coords)

    def _undraw(self):
        """ Remove the polygon corresponding to the current area from
            the RadarWidget window. """

        areafilter.deleteArea(self.area_id)

    # NOTE: Can this be vectorized further?
    def calc_qdr_tangents(self, num_traf, ac_lon, ac_lat):
        """
        For a given aircraft position find left- and rightmost courses
        that are tangent to a given polygon as well as the distance to
        the corresponding vertices.
        """

        # Initialize arrays to store qdrs and distances
        qdr_left = np.zeros(num_traf, dtype = float)
        qdr_right = np.zeros(num_traf, dtype = float)

        # Create array containing [lon, lat] for each vertex
        vertex = np.array(self.ring.coords.xy).T

        # Calculate qdrs for each aircraft
        for ii in range(num_traf):
            ac_pos = [ac_lon[ii], ac_lat[ii]]

            # Start by assuming both tangents touch at polygon vertex with index 0
            idx_left_vert = 0
            idx_right_vert = 0

            # Loop over vertices 1:n-1 and evaluate position of aircraft wrt the edges 
            # to find the indices of the vertices at which the tangents touch the polygon
            #
            # Algorithm from: http://geomalgorithms.com/a15-_tangents.html
            for jj in range(1, len(vertex) - 1):
                edge_prev = self.is_left_of_line(vertex[jj - 1], vertex[jj], ac_pos)
                edge_next = self.is_left_of_line(vertex[jj], vertex[jj + 1], ac_pos)

                if edge_prev <= 0 and edge_next > 0:
                    if not self.is_left_of_line(ac_pos, vertex[jj], vertex[idx_right_vert]) < 0:
                        idx_right_vert = jj
                elif edge_prev > 0 and edge_next <= 0:
                    if not self.is_left_of_line(ac_pos, vertex[jj], vertex[idx_left_vert]) > 0:
                        idx_left_vert = jj

            # Calculate tangent courses from aircraft to left- and rightmost vertices
            qdr_left[ii], _ = qdrdist(ac_pos[1],
                                      ac_pos[0],
                                      vertex[idx_left_vert][1],
                                      vertex[idx_left_vert][0])
            qdr_right[ii], _ = qdrdist(ac_pos[1],
                                       ac_pos[0],
                                       vertex[idx_right_vert][1],
                                       vertex[idx_right_vert][0])

        return qdr_left, qdr_right

    # NOTE: Can this be vectorized further?
    def calc_rhumb_tangents(self, num_traf, ac_lon, ac_lat):
        """
        For a given aircraft position find left- and rightmost courses
        that are tangent to a given polygon as well as the distance to
        the corresponding vertices.
        """

        # Initialize arrays to store loxodrome angles
        lox_angle_left = np.zeros(num_traf, dtype = float)
        lox_angle_right = np.zeros(num_traf, dtype = float)

        # Create array containing [lon, lat] for each vertex
        vertex = np.array(self.ring.coords.xy).T

        # Calculate loxodrome angles for each aircraft
        for ii in range(num_traf):
            ac_pos = [ac_lon[ii], ac_lat[ii]]

            # Start by assuming both tangents touch at polygon vertex with index 0
            idx_left_vert = 0
            idx_right_vert = 0

            # Loop over vertices 1:n-1 and evaluate position of aircraft wrt the edges
            # to find the indices of the vertices at which the tangents touch the polygon.
            #
            # Algorithm from: http://geomalgorithms.com/a15-_tangents.html
            for jj in range(1, len(vertex) - 1):
                edge_prev = self.is_left_of_line(vertex[jj - 1], vertex[jj], ac_pos)
                edge_next = self.is_left_of_line(vertex[jj], vertex[jj + 1], ac_pos)

                if edge_prev <= 0 and edge_next > 0:
                    if not self.is_left_of_line(ac_pos, vertex[jj], vertex[idx_right_vert]) < 0:
                        idx_right_vert = jj
                elif edge_prev > 0 and edge_next <= 0:
                    if not self.is_left_of_line(ac_pos, vertex[jj], vertex[idx_left_vert]) > 0:
                        idx_left_vert = jj

            # Calculate tangent loxodrome angles from aircraft to left- and rightmost vertices
            lox_angle_left[ii] = rhumb_azimuth(ac_pos[1],
                                               ac_pos[0],
                                               vertex[idx_left_vert][1],
                                               vertex[idx_left_vert][0])
            lox_angle_right[ii] = rhumb_azimuth(ac_pos[1],
                                                ac_pos[0],
                                                vertex[idx_right_vert][1],
                                                vertex[idx_right_vert][0])

        return lox_angle_left, lox_angle_right

    @staticmethod
    def is_ccw(coords):
        """
        Check if a list of lat,lon coordinates is defined in
        counterclockwise order.
        """

        dir_sum = 0
        for ii in range(0, len(coords) - 2, 2):  # ii = 0,2,4,...
            edge = (coords[ii + 3] - coords[ii + 1]) * \
                (coords[ii] + coords[ii + 2])
            dir_sum += edge

        return False if dir_sum > 0 else True

    @staticmethod
    def is_left_of_line(line_start, line_end, point):
        """
        Check if point lies to the left of the line through line_start
        to line_end.

        Returns:
            > 0 if point lies on the left side of the line
            = 0 if point lies exactly on the line
            < 0 if point lies on the right side of the line
        """

        return (line_end[0] - line_start[0]) * (point[1] - line_start[1]) \
                    - (point[0] - line_start[0]) * (line_end[1] - line_start[1])


def crs_mid(crs_left, crs_right):
    """
    Find the course that forms the bisector of the angle
    between crs_left and crs_right (in clockwise direction).
    """

    if crs_left < crs_right:
        crs_mid = 0.5 * (crs_left + crs_right)
    elif crs_left > crs_right:
        # North in between crs_l and crs_r
        crs_mid = (crs_left + 0.5 * (360 - crs_left + crs_right)) % 360
    else:
        # Ensure when crs_l,crs_r = 360 then crs_mid = 0
        crs_mid = crs_left % 360

    return crs_mid


def crs_is_between(crs, crs_left, crs_right):
    """
    Check if a given magnetic course crs on [0 .. 360] deg lies
    in between crs_left and crs_right (in clockwise direction).
    """

    is_between = ((crs_left > crs_right) and (crs > crs_left or crs < crs_right)) or \
                    ((crs_left < crs_right) and (crs > crs_left and crs < crs_right))

    return is_between


def calc_future_pos(dt, lon, lat, gseast, gsnorth):
    """
    Calculate future lon and lat vectors after time dt based on
    current position and velocity vectors.
    """

    newlat = lat + np.degrees(dt * gsnorth / Rearth)
    newcoslat = np.cos(np.deg2rad(newlat))
    newlon = lon + np.degrees(dt * gseast / newcoslat / Rearth)

    return newlon, newlat


def enu2crs(enu):
    """
    Convert an array of angles defined in East-North-Up on
    [-180,180] degrees to compass angles on [0,360].
    """

    crs = ((90 - enu) + 360) % 360

    return crs


def ned2crs(ned):
    """
    Convert an array of angles defined in North-East-Down on
    [-180,180] degrees to compass angles on [0,360].
    """

    crs = (ned + 360) % 360

    return crs


def crs_closest(ref_crs, crs_a, crs_b):
    """
    Takes three course vectors ref_rs, crs_a, and crs_b and per element
    returns the value of either crs_a or crs_b depending on which has
    the smallest angle difference with respect to ref_crs.
    """

    # Calculate absolute angle difference between both courses and the reference
    diff_ref_a = np.absolute(np.remainder(ref_crs - crs_a + 180, 360) - 180)
    diff_ref_b = np.absolute(np.remainder(ref_crs - crs_b + 180, 360) - 180)

    # Select the course with the smallest angle difference
    crs = np.where(diff_ref_a < diff_ref_b, crs_a, crs_b)

    return crs
