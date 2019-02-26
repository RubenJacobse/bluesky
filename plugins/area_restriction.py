""" Restricted Airspace Area Plugin

    Uses the AreaRestrictions class to represent areas that are restricted to all traffic.

    Current implementation is heavily work-in-progress and unstable.

    © Ruben Jacobse, 2019
"""

# General imports
try:
    from collections.abc import Collection
except ImportError:
    # In python <3.3 collections.abc doesn't exist
    from collections import Collection

import numpy as np
from matplotlib.path import Path

# BlueSky imports
from bluesky import traf
from bluesky.tools import areafilter
from bluesky.tools.aero import Rearth
from bluesky.tools.geo import qdrdist
from bluesky.tools.trafficarrays import TrafficArrays, RegisterElementParameters

# Module level variables
VAR_DEFAULTS = {"float": 0.0, "int": 0, "bool": False, "S": "", "str": ""}

# Initialize BlueSky plugin
def init_plugin():
    """Initialize the SUA plugin"""

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
            'Create restricted airspace areas that are to be avoided by all traffic.']
        ,
        'DELRAA': [
            # A short usage string. This will be printed if you type HELP <name> in the BlueSky console
            'DELRAA name',

            # A list of the argument types your function accepts. For a description of this, see ...
            'txt',

            # The name of your function in this plugin
            areas.delete_area,

            # a longer help text of your function.
            'Delete a given restricted airspace area.']
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
            self.vrel_north = np.array([])   # Relative velocity
            self.vrel_east = np.array([])   # Relative velocity
            self.brg_l = np.array([])   # Bearing from ac to leftmost vertex
            self.brg_r = np.array([])   # Bearing from ac to rightmost vertex
            self.dist_l = np.array([])  # Distance from ac to leftmost vertex
            self.dist_r = np.array([])  # Distance from ac to rightmost vertex
            self.in_conf = np.array([], dtype = bool) # 

        # Keep track of all restricted areas in list and by ID (same order)
        self.areaList = []
        self.areaIDList = []
        self.nareas = 0

        print(np.shape(traf.gseast))

    def create(self, n=1):
        """ Append n elements (aircraft) to all lists and arrays

            Overrides TrafficArrays.create(n) method to allow
            handling of two-dimensional numpy arrays
        """

        # If no areas exist do nothing
        if not self.nareas:
            return

        # Same as in TrafficArrays (_LstVars remain one-dimensional)
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
        """ Delete element (aircraft) idx from all lists and arrays

            Overrides TrafficArrays.delete(idx) method to allow
            handling of two-dimensional numpy arrays
        """

        for child in self._children:
            child.delete(idx)

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

    def reset(self):
        """ Reset state on simulator reset event """
        
        # Call actual reset method defined in TrafficArrays base class
        super().reset()

        # Make sure areas are deleted
        for area in self.areaList:
            area.delete()
            self.areaList.remove(area) # Probably redundant

        self.areaList = []
        self.areaIDList = []
        self.nareas = 0

    def update(self):
        """ Do calculations after traf has been updated. """
        
        # 
        for idx, area in enumerate(self.areaList):
            # Calculate bearings and distance to the tangent vertices
            self.brg_l[idx, :], self.brg_r[idx, :], self.dist_l[idx, :], self.dist_r[idx, :] = area.calc_tangents(traf.ntraf, traf.lat, traf.lon)

            # Calculate area velocity components relative to each aircraft
            self.vrel_east[idx, :], self.vrel_north[idx, :] = area.calc_vrel(traf.gseast, traf.gsnorth)
        
    def preupdate(self):
        """ Update the area positions before traf is updated. """

        # NOTE: Not sure if this should be part of preupdate() or update()
        for area in self.areaList:
            area.update_pos(1.0)

    def create_area(self, area_id, area_status, gs_north, gs_east, *coords):
        """ Create a new restricted airspace area """

        if area_id in self.areaIDList:
            return False, "Error: Airspace restriction with name {} already exists".format(area_id)

        # Create new RestrictedAirspaceArea instance and add to internal lists
        new_area = RestrictedAirspaceArea(area_id, area_status, gs_north, gs_east, list(coords))
        
        self.areaList.append(new_area)
        self.areaIDList.append(area_id)
        self.nareas += 1

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
            newrow = np.full((1, traf.ntraf), defaultvalue)

            if not self.nareas:
                self._Vars[v] = newrow
            else:
                self._Vars[v] = np.vstack((self._Vars[v], newrow))

        return True, "Restricted Airspace Area {} is initialized".format(area_id)

    def delete_area(self, area_id):
        """ Delete an existing restricted airspace area """

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


class RestrictedAirspaceArea():
    """ Class that represents a single Restricted Airspace Area """

    def __init__(self, area_id, status, gs_north, gs_east, coords):

        # Initialize
        self.area_id = area_id
        self.status = status
        self.gs_north = gs_north
        self.gs_east = gs_east

        self.coords = []
        self.border = None

        # Ensure that the coordinates define a valid polygon
        self.coords = self._check_poly(coords)
        
        # Update the border using self.coords
        self._update_border()

        # Draw polygon on RadarWidget canvas
        self._draw()

    def update_pos(self, dt):
        """ Update the position of the area (only if its velocity is
            nonzero). Recalculates the coordinates and updates the
            polygon position drawn in the RadarWidget. """

        if self.gs_north == 0 and self.gs_east == 0:
            pass
        else:
            new_coords = []

            for ii in range(0, len(self.coords), 2):
                newlat = self.coords[ii] + np.degrees(dt * self.gs_north / Rearth)
                newcoslat = np.cos(np.deg2rad(newlat))
                newlon = self.coords[ii + 1] + np.degrees(dt * self.gs_east / newcoslat / Rearth)

                new_coords.extend([newlat, newlon])

            self.coords = new_coords
            self._update_border()

            # Undraw currently drawn object from canvas
            self._undraw()

            # Redraw on canvas with new positions
            self._draw()

    def delete(self):
        # Undraw from canvas
        self._undraw()

    def _check_poly(self, coords):
        """ During initialization check that the user specified polygon
            is valid.

            - Vertices shall form a closed ring (first and last vertex are the same)
            - Vertices shall be ordered counterclockwise

            If this is not already the case then 
        """

        # Make sure the border is a closed ring (first and last coordinate pair should be the same)
        if (coords[0], coords[1]) != (coords[-2], coords[-1]):
            coords = coords + [coords[0], coords[1]]

        # Check if the polygon is ccw, if not, then flip the coordinate order
        dir_sum = 0
        for ii in range(0, len(coords) - 2, 2): # ii = 0,2,4,...
            edge = (coords[ii + 2] - coords[ii]) * (coords[ii + 1] + coords[ii + 3])
            dir_sum += edge
        if dir_sum >= 0:
            reordered = []
            for ii in range(len(coords) - 1, -1, -2):
                reordered.append([coords[ii], coords[ii + 1]])
            coords = reordered

        return coords

    def _update_border(self):
        """ Use current value of self.coords to create a matplotlib Path
            object to represent the border. """

        # Reshape into array of (lat, lon) pairs
        points = np.reshape(self.coords, (len(self.coords) // 2, 2))
        self.border = Path(points, closed = True)

    def _draw(self):
        """ Draw the polygon corresponding to the current area in the
            RadarWidget window. """

        areafilter.defineArea(self.area_id, 'POLY', self.coords)

    def _undraw(self):
        """ Remove the polygon corresponding to the current area from
            the RadarWidget window. """

        areafilter.deleteArea(self.area_id)

    # NOTE: How can this be vectorized further?
    def calc_tangents(self, ntraf, ac_lat, ac_lon):
        """ For a given aircraft position find left- and rightmost courses
            that are tangent to a given polygon. """

        # Initialize arrays to store qdrs and distances
        qdr_l = np.zeros(ntraf, dtype=float)
        qdr_r = np.zeros(ntraf, dtype=float)
        dist_l = np.zeros(ntraf, dtype=float)
        dist_r = np.zeros(ntraf, dtype=float)

        border = self.border.vertices

        # Calculate qdrs and distances for each aircraft
        for ii in range(ntraf):
            ac_pos = [ac_lat[ii], ac_lon[ii]]

            # Start by assuming both tangents touch at polygon vertex with index 0
            idx_l = 0
            idx_r = 0

            # Loop over vertices 1:n-1 and evaluate position of aircraft wrt the edges to find the
            # indices of the vertices at which the tangents touch the polygon
            #
            # Algorithm from: http://geomalgorithms.com/a15-_tangents.html
            for jj in range(1, len(border) - 1):
                eprev = self.is_left(border[jj - 1], border[jj], ac_pos)
                enext = self.is_left(border[jj], border[jj + 1], ac_pos)

                if eprev <= 0 and enext > 0:
                    if not (self.is_left(ac_pos, border[jj], border[idx_r]) < 0):
                        idx_r = jj
                elif eprev > 0 and enext <= 0:
                    if not (self.is_left(ac_pos, border[jj], border[idx_l]) > 0):
                        idx_l = jj

            # Calculate tangent courses from aircraft to left- and rightmost vertices
            qdr_l[ii], dist_l[ii] = qdrdist(ac_pos[0], ac_pos[1], border[idx_l][0], border[idx_l][1])
            qdr_r[ii], dist_r[ii] = qdrdist(ac_pos[0], ac_pos[1], border[idx_r][0], border[idx_r][1])

        return qdr_l, qdr_r, dist_l, dist_r

    def calc_vrel(self, ac_gseast, ac_gsnorth):
        """ Calculate the east and north components of the relative
            velocity of the area with respect to all aircraft. """

        vrel_east = self.gs_east - ac_gseast
        vrel_north = self.gs_north - ac_gsnorth

        return vrel_east, vrel_north

    # Copied from tools/areafilter.py and edited
    @staticmethod
    def is_inside(border, lat, lon):
        """ Takes vectors with lat and lon and returns boolean list per point. """

        points = np.vstack((lat, lon)).T
        inside = border.contains_points(points)

        return inside

    @staticmethod
    def crs_is_between(crs, crs_l, crs_r):
        """ Check if a given course crs lies in between crs_l and crl_r
            (in clockwise direction). """

        if ((crs_l > crs_r) and (crs > crs_l or crs < crs_r)) or ((crs_l < crs_r) and (crs > crs_l and crs < crs_r)):
            return True

        return False

    @staticmethod
    def is_left(p0, p1, p2):
        """  Check if point p2 lies to the left of the line through p0 and p1.

            Returns:
                > 0 if p2 lies on the left side of the line
                = 0 if p2 lies exactly on the line
                < 0 if p2 lies on the right side of the line
        """

        return (p1[1] - p0[1]) * (p2[0] - p0[0]) - (p2[1] - p0[1]) * (p1[0] - p0[0])