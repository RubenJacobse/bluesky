""" Restricted Airspace Area Plugin

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

# Default variable values for numpy arrays
VAR_DEFAULTS = {"float": 0.0, "int": 0, "bool": False, "S": "", "str": ""}

# Initialize BlueSky plugin
def init_plugin():
    """Initialize the RAA plugin"""

    # Addtional initilisation code
    areas = AreaRestrictionManager()

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'RAA',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim',

        # Update interval in seconds. By default, your plugin's update function(s)
        # are called every timestep of the simulation. If your plugin needs less
        # frequent updates provide an update interval.
        'update_interval': 1.0,

        # The update function is called after traffic is updated. Use this if you
        # want to do things as a result of what happens in traffic. If you need to
        # something before traffic is updated please use preupdate.
        'update':          areas.update,

        # The preupdate function is called before traffic is updated. Use this
        # function to provide settings that need to be used by traffic in the current
        # timestep. Examples are ASAS, which can give autopilot commands to resolve
        # a conflict.
        'preupdate':       areas.preupdate,

        # If your plugin has a state, you will probably need a reset function to
        # clear the state in between simulations.
        'reset':         areas.reset
    }

    stackfunctions = {
        # The command name for your function
        'RAA': [
            # A short usage string. This will be printed if you type HELP <name> in the BlueSky console
            'RAA name, ON/OFF, gs_north, gs_east, [lat1,lon1,lat2,lon2,...]',

            # A list of the argument types your function accepts. For a description of this, see ...
            'txt,onoff,spd,spd,[latlon,...]',

            # The name of your function in this plugin
            areas.create_area,

            # a longer help text of your function.
            'Create restricted airspace areas that are to be avoided by all traffic.'],
        'DELRAA': [
            # A short usage string. This will be printed if you type HELP <name> in the BlueSky console
            'DELRAA name',

            # A list of the argument types your function accepts. For a description of this, see ...
            'txt',

            # The name of your function in this plugin
            areas.delete_area,

            # a longer help text of your function.
            'Delete a given restricted airspace area.'],
        'RAACONF': [
            # A short usage string. This will be printed if you type HELP <name> in the BlueSky console
            'RAACONF t_lookahead',

            # A list of the argument types your function accepts. For a description of this, see ...
            'int',

            # The name of your function in this plugin
            areas.set_t_lookahead,

            # a longer help text of your function.
            'Set the lookahead time used for area avoidance in seconds.']
    }

    # init_plugin() should always return these two dicts.
    return config, stackfunctions


class AreaRestrictionManager(TrafficArrays):
    """ This class implements the avoidance of Restricted Airspace Areas. """

    def __init__(self):
        # Initialize TrafficArrays base class
        super().__init__()

        # Register all traffic parameters relevant for this class
        with RegisterElementParameters(self):
            # N-dimensional parameters where each column is an aircraft and each row is an area
            self.vrel_east = np.array([[]]) # [m/s] East component of ac relative velocity wrt area
            self.vrel_north = np.array([[]]) # [m/s] North component of ac relative velocity wrt area
            self.vrel = np.array([[]]) # [m/s] Magnitude of ac relative velocity wrt area
            self.brg_l = np.array([[]]) # [deg] Bearing from ac to leftmost vertex in N-E-D [-180..180]
            self.brg_r = np.array([[]]) # [deg] Bearing from ac to rightmost vertex in N-E-D [-180..180]
            self.crs_l = np.array([[]]) # [deg] Compass course from ac to leftmost vertex [0..360]
            self.crs_r = np.array([[]]) # [deg] Compass course from ac to rightmost vertex [0..360]
            self.dist_l = np.array([[]]) # [NM] Distance from ac to leftmost vertex
            self.dist_r = np.array([[]]) # [NM] Distance from ac to rightmost vertex
            self.area_inconf = np.array([[]], dtype = bool) # Stores wheter ac is in conflict with area
            self.area_inside = np.array([[]], dtype = bool) # Stores whether ac is inside area
            self.area_tint = np.array([[]]) # [s] Time to area intrusion

            # NOTE: To be added: support for both 1- and multi-dimensional numpy arrays...
            # self.area_inconf_first = np.array([])  # Store index of closest conflicting area for each aircraft
            self.wp_crs = np.array([[]]) # [deg] Magnetic course to current waypoint

            self.unused = [] # Only used for testing

        # Keep track of all restricted areas in list and by ID (same order)
        self.areaList = []
        self.areaIDList = []
        self.nareas = 0
        self.ntraf = 0

        # Default look-ahead-time in seconds, used to detect aircraft-area conflicts
        self.t_lookahead = 300

    def create(self, n = 1):
        """ Append n elements (aircraft) to all lists and arrays.

            Overrides TrafficArrays.create(n) method to allow
            handling of two-dimensional numpy arrays.
        """

        self.ntraf += n

        # If no areas exist do nothing
        if not self.nareas:
            return

        for v in self._LstVars:  # Lists (mostly used for strings)

            # Get type
            vartype = None
            lst = self.__dict__.get(v)
            if len(lst) > 0:
                vartype = str(type(lst[0])).split("'")[1]

            if vartype in VAR_DEFAULTS:
                defaultvalue = [VAR_DEFAULTS[vartype]] * n
            else:
                defaultvalue = [""] * n

            self._Vars[v].extend(defaultvalue)

        # Allow two-dimensional numpy arrays in _ArrVars
        # Each row can now represent an airspace restriction
        for v in self._ArrVars:  # Numpy array

            # Get numpy dtype without byte length
            dtype_str = str(self._Vars[v].dtype)
            for vartype in ["int", "float", "bool"]:
                if vartype in dtype_str:
                    break

            # Get default value
            if vartype in VAR_DEFAULTS:
                defaultvalue = [VAR_DEFAULTS[vartype]]
            else:
                defaultvalue = [0.0]

            # Create add n columns to existing array
            new_cols = np.full((self.nareas, n), defaultvalue)
            if not self._Vars[v].size:
                self._Vars[v] = new_cols
            else:
                self._Vars[v] = np.concatenate((self._Vars[v], new_cols), 1)

    def delete(self, idx):
        """ Delete element (aircraft) idx from all lists and arrays.

            Overrides TrafficArrays.delete(idx) method to allow
            handling of two-dimensional numpy arrays. """

        if isinstance(idx, Collection):
            idx = np.sort(idx)
            dec = len(idx)
        else:
            dec = 1

        # Delete entire column idx from v (column = dimension 1)
        for v in self._ArrVars:
            self._Vars[v] = np.delete(self._Vars[v], idx, 1)

        if self._LstVars:
            if isinstance(idx, Collection):
                for i in reversed(idx):
                    for v in self._LstVars:
                        del self._Vars[v][i]
            else:
                for v in self._LstVars:
                    del self._Vars[v][idx]

        self.ntraf -= dec

        return True

    def reset(self):
        """ Reset state on simulator reset event. """

        # Clear all variables by deleting the last element
        # until no aircraft remain
        while self.ntraf:
            self.delete(self.ntraf - 1) # Decrements self.ntraf after each deletion

        # Make sure areas are deleted
        for area in self.areaList:
            area.delete()
            self.areaList.remove(area) # Probably redundant

        self.areaList = []
        self.areaIDList = []
        self.nareas = 0
        self.t_lookahead = 300

    def preupdate(self):
        """ Update the area positions before traf is updated. """

        # NOTE: Not sure if this should be part of preupdate() or update()
        for area in self.areaList:
            area.update_pos(1.0)

    def update(self):
        """ Do calculations after traf has been updated. """

        # Might be useful in debugging
        # print("Simulator time is: {} seconds ".format(bs.sim.simt))

        # Cannot do anything if no aircraft or areas exist
        if not self.ntraf or not self.nareas:
            return

        # NOTE: To be removed after numpy array dimension support update
        # Set up numpy array that keeps track of the index of the closest conflicting area for 
        # all aircraft.
        self.area_inconf_first = np.full(self.ntraf, None)

        # Loop over all existing areas
        # NOTE: Could this be vectorized instead of looped over all aircraft-area combinations?
        for idx, area in enumerate(self.areaList):
            # Calculate bearings and distance to the tangent vertices of the area
            self.brg_l[idx, :], self.brg_r[idx, :], self.dist_l[idx, :], self.dist_r[idx, :] = \
                area.calc_tangents(bs.traf.ntraf, bs.traf.lon, bs.traf.lat)

            # Calculate compass courses tangent to current area for each aircraft
            self.crs_l[idx, :] = ned2crs(self.brg_l[idx, :])
            self.crs_r[idx, :] = ned2crs(self.brg_r[idx, :])

            # Calculate velocity components of each aircraft relative to the area
            self.vrel_east[idx, :] = bs.traf.gseast - area.gs_east
            self.vrel_north[idx, :] = bs.traf.gsnorth - area.gs_north
            self.vrel[idx, :] = np.sqrt(self.vrel_east[idx, :]**2 + self.vrel_north[idx, :]**2)

            # Calculate position of each aircraft relative to the area after t_lookahead
            ac_fut_rel_lon, ac_fut_rel_lat = calc_future_pos(self.t_lookahead, bs.traf.lon, bs.traf.lat, \
                                                             self.vrel_east[idx, :], self.vrel_north[idx, :])

            # Create shapely points for current and future relative position
            # and use these to create a shapely LineString with the relative vector
            ac_curr_pos = [spgeom.Point(lon, lat) for (lon, lat) in zip(bs.traf.lon, bs.traf.lat)] # NOTE: Can be moved outside of loop
            ac_fut_rel_pos = [spgeom.Point(lon, lat) for (lon, lat) in zip(ac_fut_rel_lon, ac_fut_rel_lat)]
            ac_rel_vec = [spgeom.LineString([curr, fut]) for (curr, fut) in zip(ac_curr_pos, ac_fut_rel_pos)]

            # Find aircraft-area conflicts and intrusions
            self.area_findconf(idx, area, ac_curr_pos, ac_rel_vec)

        # For each aircraft, perform area avoidance manoeuvre for the closest conflicting area (if any)
        for ac_idx, area_idx in enumerate(self.area_inconf_first):

            # Skip to next aircraft index if no conflict exists for the current aircraft
            if area_idx is None:    # area_idx can take value 0, thus we need to explicitly compare against None
                continue

            # Calculate area avoidance resolution for current aircraft and its conflicting area
            reso_crs, reso_tas = self.calc_reso(ac_idx, area_idx)

            # print("{} first conflict is: {}".format(bs.traf.id[ac_idx], self.areaIDList[area_idx]))

            self.stack_reso_apply(ac_idx, reso_crs, reso_tas)

    def create_area(self, area_id, area_status, gs_north, gs_east, *coords):
        """ Create a new restricted airspace area """

        if area_id in self.areaIDList:
            return False, "Error: Airspace restriction with name {} already exists".format(area_id)

        # Create new RestrictedAirspaceArea instance and add to internal lists
        new_area = RestrictedAirspaceArea(area_id, area_status, gs_east, gs_north, list(coords))

        self.areaList.append(new_area)
        self.areaIDList.append(area_id)

        for v in self._LstVars:  # Lists (mostly used for strings)

            # Get type
            vartype = None
            lst = self.__dict__.get(v)
            if lst:
                vartype = str(type(lst[0])).split("'")[1]

            if vartype in VAR_DEFAULTS:
                defaultvalue = [VAR_DEFAULTS[vartype]] * bs.traf.ntraf
            else:
                defaultvalue = [""] * bs.traf.ntraf

            if not self._Vars[v]:
                self._Vars[v] = defaultvalue

        # Add row to all numpy arrays
        for v in self._ArrVars:

            # Get numpy dtype without byte length
            dtype_str = str(self._Vars[v].dtype)
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

            if not self.nareas:
                self._Vars[v] = newrow
            else:
                self._Vars[v] = np.vstack((self._Vars[v], newrow))

        # Increment after adding variable elements
        self.nareas += 1

        return True, "Restricted Airspace Area {} is initialized".format(area_id)

    def delete_area(self, area_id):
        """ Delete an existing restricted airspace area. """

        if area_id not in self.areaIDList:
            return False, "Error: Airspace restriction with name {} does not exist".format(area_id)
        else:
            # Find index of area
            idx = self.areaIDList.index(area_id)

            # Call object delete method before deleting the object
            self.areaList[idx].delete()

            # Delete from all lists and arrays
            del self.areaList[idx]
            del self.areaIDList[idx]
            self.nareas -= 1

            # Delete row corresponding to area from all numpy arrays
            for v in self._ArrVars:
                self._Vars[v] = np.delete(self._Vars[v], idx, 0)

            return True, "Sucessfully deleted airspace restriction {}".format(area_id)

    def set_t_lookahead(self, t):
        """ Change the look-ahead-time used for aircraft-area conflict detection. """

        if not isinstance(t, int):
            return False, "Error: Look-ahead-time should be an integer value"
        else:
            self.t_lookahead = t
            return True, "Aircraft-area conflict look-ahead-time set to {} seconds".format(t)

    def area_findconf(self, idx, area, ac_curr_pos, ac_rel_vec):
        """" Find all aircraft-area conflicts for RestrictedAispaceArea area
             at row idx. """

        for ac_idx in range(self.ntraf):
            # Use shapely to determine if the aircraft path in relative velocity space
            # with respect to the area crosses the polygon ring.
            self.area_inconf[idx, ac_idx] = area.ring.intersects(ac_rel_vec[ac_idx])

            # Check if the aircraft is inside the area
            self.area_inside[idx, ac_idx] = ac_curr_pos[ac_idx].within(area.poly)

            # Calculate time-to-intersection [s] for aircraft-area combination, takes
            # following values:
            #  -1 : for aircraft not in conflict with the area
            #  0  : for aircraft already inside the area
            #  >0 : for aircraft that are in conflict
            if self.area_inside[idx, ac_idx]:
                t_int = 0   # NOTE: If area avoidance works properly, this should never happen!
            elif self.area_inconf[idx, ac_idx]:
                # Find intersection points of the relative vector with the area and use the
                # distance to the closest point to calculate time-to-intersection. (We cannot
                # use shapely distance functions because vertex definitions are in degrees).
                intr_points = area.ring.intersection(ac_rel_vec[ac_idx])
                intr_closest = spops.nearest_points(ac_curr_pos[ac_idx], intr_points)[1]
                intr_closest_lat, intr_closest_lon = intr_closest.y, intr_closest.x
                _, intr_dist_nm = qdrdist(bs.traf.lat[ac_idx], bs.traf.lon[ac_idx], intr_closest_lat, intr_closest_lon)
                intr_dist_m = intr_dist_nm * 1852 # qdrdist returns dist in NM, convert to m
                t_int = intr_dist_m / bs.traf.gs[ac_idx]
            else:
                t_int = -1
            self.area_tint[idx, ac_idx] = t_int

            # For each aircraft find the conflict that occurs first
            if not self.area_inconf_first[ac_idx] and t_int > 0:
                self.area_inconf_first[ac_idx] = idx
            elif self.area_inconf_first[ac_idx] and t_int > 0 and t_int < self.area_inconf_first[ac_idx]:
                self.area_inconf_first[ac_idx] = idx

            # Print some messages that may be useful in debugging
            dbg_str = "Aircraft {} time to conflict with area {} is: {} seconds"
            t_int_sec = round(self.area_tint[idx, ac_idx])
            # print(dbg_str.format(bs.traf.id[ac_idx], area.area_id, t_int_sec))

    def calc_reso(self, ac_idx, area_idx):
        """ Calculate the resolution manoeuvre (heading and speed)
            for a single aircraft. """

        area = self.areaList[area_idx]
        ac_gs = bs.traf.gs[ac_idx]

        print("{} ground speed: {}".format(bs.traf.id[ac_idx], ac_gs))

        # Components of unit vectors along left and right VO edges
        u_l_east = np.sin(np.radians(self.brg_l[area_idx, ac_idx]))
        u_l_north = np.cos(np.radians(self.brg_l[area_idx, ac_idx]))
        u_r_east = np.sin(np.radians(self.brg_r[area_idx, ac_idx]))
        u_r_north = np.cos(np.radians(self.brg_r[area_idx, ac_idx]))

        # For heading-change-only resolution maneuvers, the resolution vector lies on the edge of
        # the Velocity Obstacle (or Collision Cone if area is non-moving). The vector magnitudes are the
        # same as the current aircraft ground speeds (assuming zero wind).
        if area.gs:
            # Find angles between -vrel and left- and right edges of VO using dot product of -vrel and the 
            # unit vectors along the VO edges
            beta_l_rad = np.arccos(-1 * (u_l_east * area.gs_east + u_l_north * area.gs_north) / (area.gs**2))
            beta_r_rad = np.arccos(-1 * (u_r_east * area.gs_east + u_r_north * area.gs_north) / (area.gs**2))

            # Calculate relative resolution velocity component along the VO edges
            v_ul = ac_gs * np.cos(beta_l_rad) + ac_gs * np.cos(np.arcsin(ac_gs * np.sin(beta_l_rad) / ac_gs)) 
            v_ur = ac_gs * np.cos(beta_r_rad) + ac_gs * np.cos(np.arcsin(ac_gs * np.sin(beta_r_rad) / ac_gs))
        else:
            # Resolution velocity magnitudes on left- and right edges of CC
            v_ul = ac_gs
            v_ur = ac_gs

        # Calculate north and east components of left- and right resolution absolute velocities
        # (For a non moving area, area.gs_east and area.gs_north are simply 0, but for a moving area
        # these terms are required to get the absolute resolution velocity of each aircraft.)
        vres_l_east = u_l_east * v_ul - area.gs_east
        vres_l_north = u_l_north * v_ul - area.gs_north
        vres_r_east = u_r_east * v_ur - area.gs_east
        vres_r_north = u_r_north * v_ur - area.gs_north

        # Calculate magnetic tracks of left- and right resolution vectors
        vres_l_crs = np.degrees(np.arctan2(vres_l_east, vres_l_north)) % 360
        vres_r_crs = np.degrees(np.arctan2(vres_r_east, vres_r_north)) % 360

        # For each aircraft find the track angle to the current active waypoint
        # (NOTE: This assumes that there always is an active waypoint.)
        for ac_idx in range(self.ntraf):
            wp_lat = bs.traf.actwp.lat[ac_idx]
            wp_lon = bs.traf.actwp.lon[ac_idx]
            qdr_wp, _ = qdrdist(bs.traf.lat[ac_idx], bs.traf.lon[ac_idx], wp_lat, wp_lon)
            wp_crs = (360 + qdr_wp) % 360

        # Determine which of the two resolution vectors should be used based on the current
        # active waypoint.
        reso_crs = crs_closest(wp_crs, vres_l_crs, vres_r_crs)

        # Set the true airspeed for the resolution maneuver
        reso_tas = ac_gs

        return reso_crs, reso_tas

    def stack_reso_apply(self, ac_idx, crs, tas):
        """ Apply all resolution vectors via the BlueSky stack. """

        ac_id = bs.traf.id[ac_idx]
        ac_crs = crs
        ac_alt = bs.traf.alt[ac_idx]
        ac_cas = bs.tools.aero.vtas2cas(tas, ac_alt)

        hdg_cmd = "HDG {},{}".format(ac_id, ac_crs)
        bs.stack.stack(hdg_cmd)
        spd_cmd = "SPD {},{}".format(ac_id, ac_cas)
        bs.stack.stack(spd_cmd)

    	# Print command to python console
        print(hdg_cmd)
        print(spd_cmd)

class RestrictedAirspaceArea():
    """ Class that represents a single Restricted Airspace Area. """

    def __init__(self, area_id, status, gs_east, gs_north, coords):

        # Store input parameters as attributes
        self.area_id = area_id
        self.status = status
        self.gs_north = gs_north
        self.gs_east = gs_east
        self.gs = np.sqrt(gs_north**2 + gs_east**2)

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
        self.gs_verts = np.full(np.shape(self.verts), [self.gs_east, self.gs_north])

        # Draw polygon on BlueSky RadarWidget canvas
        self._draw()

    def update_pos(self, dt):
        """ Update the position of the area (only if its groundspeed
            is nonzero). Recalculates the coordinates and updates the
            polygon position drawn in the RadarWidget. """

        if self.gs_north or self.gs_east:

            # Get lon and lat vectors from verts
            curr_lon = self.verts[:, 0]
            curr_lat = self.verts[:, 1]

            # Calculate new lon and lat values for all vertices
            newlon, newlat = calc_future_pos(dt, curr_lon, curr_lat, self.gs_east, self.gs_north)

            # Update vertices using new lat and lon vectors
            self.verts = np.array([newlon, newlat]).T

            # Update the other coordinate representations
            self.ring = spgeom.LinearRing(self.verts)
            self.poly = spgeom.Polygon(self.verts)
            self.coords = self._verts2coords(self.verts)

            # Remove current drawing and redraw new position on BlueSky RadarWidget canvas
            # NOTE: Can probably be improved by a lot!
            self._undraw()
            self._draw()

    def delete(self):
        """ On deletion, remove the drawing of current area from the
            BlueSky RadarWidget canvas. """

        self._undraw()

    def _check_poly(self, coords):
        """ During initialization check that the user specified polygon
            is valid.

            - Vertices shall form a closed ring (first and last vertex are the same)
            - Vertices shall be ordered counterclockwise

            If this is not already the case then create a valid polygon. """

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
        """ Convert list with coords in lat,lon order to numpy array of
            vertex pairs in lon,lat order.

            coords = [lat_0, lon_0, ..., lat_n, lon_n] \n
            verts = np.ndarray([[lon_0, lat_0], ..., [lon_n, lat_n]])

            (Essentially the inverse operation of self._verts2coords). """

        verts_latlon = np.reshape(coords, (len(coords) // 2, 2))
        verts_lonlat = np.flip(verts_latlon, 1)

        return verts_lonlat

    def _verts2coords(self, verts):
        """ Convert numpy array of vertex coordinate pairs in lon,lat order to
            a single list of lat,lon coords.

            verts = np.ndarray([[lon_0, lat_0], ..., [lon_n, lat_n]]) \n
            coords = [lat_0, lon_0, ..., lat_n, lon_n]

            (Essentially the inverse operation of self._coords2verts). """

        verts_latlon = np.flip(verts, 1)
        coords_latlon = list(verts_latlon.flatten("C"))

        return coords_latlon

    def _draw(self):
        """ Draw the polygon corresponding to the current area in the
            RadarWidget window. """

        areafilter.defineArea(self.area_id, 'POLY', self.coords)

    def _undraw(self):
        """ Remove the polygon corresponding to the current area from
            the RadarWidget window. """

        areafilter.deleteArea(self.area_id)

    # NOTE: How can this be vectorized further?
    def calc_tangents(self, ntraf, ac_lon, ac_lat):
        """ For a given aircraft position find left- and rightmost courses
            that are tangent to a given polygon as well as the distance to
            the corresponding vertices. """

        # Initialize arrays to store qdrs and distances
        qdr_l = np.zeros(ntraf, dtype=float)
        qdr_r = np.zeros(ntraf, dtype=float)
        dist_l = np.zeros(ntraf, dtype=float)
        dist_r = np.zeros(ntraf, dtype=float)

        # Create array containing [lon, lat] for each vertex
        vertex = np.array(self.ring.coords.xy).T

        # Calculate qdrs and distances for each aircraft
        for ii in range(ntraf):
            ac_pos = [ac_lon[ii], ac_lat[ii]]

            # Start by assuming both tangents touch at polygon vertex with index 0
            idx_l = 0
            idx_r = 0

            # Loop over vertices 1:n-1 and evaluate position of aircraft wrt the edges to find the
            # indices of the vertices at which the tangents touch the polygon
            #
            # Algorithm from: http://geomalgorithms.com/a15-_tangents.html
            for jj in range(1, len(vertex) - 1):
                eprev = self.is_left(vertex[jj - 1], vertex[jj], ac_pos)
                enext = self.is_left(vertex[jj], vertex[jj + 1], ac_pos)

                if eprev <= 0 and enext > 0:
                    if not self.is_left(ac_pos, vertex[jj], vertex[idx_r]) < 0:
                        idx_r = jj
                elif eprev > 0 and enext <= 0:
                    if not self.is_left(ac_pos, vertex[jj], vertex[idx_l]) > 0:
                        idx_l = jj

            # Calculate tangent courses from aircraft to left- and rightmost vertices
            qdr_l[ii], dist_l[ii] = qdrdist(ac_pos[1], ac_pos[0], vertex[idx_l][1], vertex[idx_l][0])
            qdr_r[ii], dist_r[ii] = qdrdist(ac_pos[1], ac_pos[0], vertex[idx_r][1], vertex[idx_r][0])

        return qdr_l, qdr_r, dist_l, dist_r

    @staticmethod
    def crs_is_between(crs, crs_l, crs_r):
        """ Check if a given magnetic course crs on [0 .. 360] deg
            lies in between crs_l and crl_r (in clockwise direction). """

        if ((crs_l > crs_r) and (crs > crs_l or crs < crs_r)) or \
             ((crs_l < crs_r) and (crs > crs_l and crs < crs_r)):
            return True

        return False

    @staticmethod
    def crs_mid(crs_l, crs_r):
        """ Find the course that forms the bisector of the angle
            between crs_l and crs_r. """

        if crs_l < crs_r:
            crs_mid = 0.5 * (crs_l + crs_r)
        elif crs_l > crs_r:
            crs_mid = (crs_l + 0.5*(360 - crs_l + crs_r)) % 360
        else:
            # Ensure when crs_l,crs_r = 360 then crs_mid = 0
            crs_mid = crs_l % 360

        return crs_mid

    @staticmethod
    def is_left(p_0, p_1, p_2):
        """  Check if point p_2 lies to the left of the line through p_0 and p_1.

            Returns:
                > 0 if p_2 lies on the left side of the line
                = 0 if p_2 lies exactly on the line
                < 0 if p_2 lies on the right side of the line """

        return (p_1[0] - p_0[0]) * (p_2[1] - p_0[1]) - (p_2[0] - p_0[0]) * (p_1[1] - p_0[1])

    @staticmethod
    def is_ccw(coords):
        """ Check if a list of lat,lon coordinates is defined in
            counterclockwise order. """

        dir_sum = 0
        for ii in range(0, len(coords) - 2, 2): # ii = 0,2,4,...
            # edge = (coords[ii + 2] - coords[ii]) * (coords[ii + 1] + coords[ii + 3])
            edge = (coords[ii + 3] - coords[ii + 1]) * (coords[ii] + coords[ii + 2])
            dir_sum += edge

        return False if dir_sum > 0 else True

def calc_future_pos(dt, lon, lat, gs_e, gs_n):
    """ Calculate future lon and lat vectors after time dt based on
        current position and velocity vectors"""

    newlat = lat + np.degrees(dt * gs_n / Rearth)
    newcoslat = np.cos(np.deg2rad(newlat))
    newlon = lon + np.degrees(dt * gs_e / newcoslat / Rearth)

    return newlon, newlat

def enu2crs(enu):
    """ Convert an array of angles defined in East-North-Up on
        [-180,180] degrees to compass angles on [0,360]. """

    crs = ((90 - enu) + 360 ) % 360

    return crs

def ned2crs(ned):
    """ Convert an array of angles defined in North-East-Down on
    [-180,180] degrees to compass angles on [0,360]. """

    crs = (ned + 360 ) % 360

    return crs

def crs_closest(ref, crs_a, crs_b):
    """ Returns the value of either crs_a or crs_b depending on which
        has the smallest angle difference with respect to ref. """

    # Calculate absolute angle difference between both courses and the reference
    crs_diff_a = np.absolute(np.remainder(ref - crs_a + 180, 360) - 180)
    crs_diff_b = np.absolute(np.remainder(ref - crs_b + 180, 360) - 180)

    # Select the course with the smallest angle difference
    crs = np.where(crs_diff_a < crs_diff_b, crs_a, crs_b)

    return crs
