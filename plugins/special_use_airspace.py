""" Special Use Airspace Plugin

    Uses the SuaArray class to represent special use airspaces that are restricted to all traffic.
    
    Current implementation is heavily work-in-progress and unstable.

    © Ruben Jacobse, 2019
"""

import numpy as np
from matplotlib.path import Path
from bluesky import traf, stack
from bluesky.tools import areafilter
from bluesky.tools.aero import Rearth
from bluesky.tools.geo import qdrdist


def init_plugin():
    """Initialize the SUA plugin"""

    # Addtional initilisation code
    sua = SuaArray()

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'SUA',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim',

        # Update interval in seconds. By default, your plugin's update function(s)
        # are called every timestep of the simulation. If your plugin needs less
        # frequent updates provide an update interval.
        'update_interval': 1.0,

        # The update function is called after traffic is updated. Use this if you
        # want to do things as a result of what happens in traffic. If you need to
        # something before traffic is updated please use preupdate.
        'update':          sua._count_ac_entering_sua,

        # The preupdate function is called before traffic is updated. Use this
        # function to provide settings that need to be used by traffic in the current
        # timestep. Examples are ASAS, which can give autopilot commands to resolve
        # a conflict.
        'preupdate':       sua._count_ac_in_sua,

        # If your plugin has a state, you will probably need a reset function to
        # clear the state in between simulations.
        'reset':         sua.reset
    }

    stackfunctions = {
        # The command name for your function
        'SUA': [
            # A short usage string. This will be printed if you type HELP <name> in the BlueSky console
            'SUA name, ON/OFF or SUA name, ON/OFF, [lat1,lon1,lat2,lon2,...]',

            # A list of the argument types your function accepts. For a description of this, see ...
            'txt,onoff,[latlon,...]',

            # The name of your function in this plugin
            sua.stack_input,

            # a longer help text of your function.
            'Create special use airspaces that are to be avoided by all traffic.']
    }

    # init_plugin() should always return these two dicts.
    return config, stackfunctions


class SuaArray:
    """ Special Use Airspace, contains all restricted airspaces to be avoided by traffic 

        Public methods:
            stack_input(sua_id, status, *coords):  Processes the input from BlueSky stack
            reset(): Clears the plugin state on sim reset

        Private methods:
            _update_suadict()
            _add_to_dict()
            _remove_from_dict()
            draw_sua()
            undraw_sua()
            _count_ac_in_sua()
            _count_ac_entering_sua()
     """

    # Dictionary object used to store all sua information
    suadict = {}

    def __init__(self):
        """ Initialize SuaArray class object """
        pass

    def reset(self):
        """ Reset SuaArray state between simulations""" 

        # Remove polygons from the gui individually
        for sua_id in self.suadict:
            self.undraw_sua(sua_id)

        # Reset the dictionary at once (cannot remove entries inside loop)
        self.suadict = {}

    def stack_input(self, sua_id, status, *coords):
        """ Process input entered via BlueSky stack command """

        # If variables received from stack input are not in correct format return error message to bluesky console
        if not (isinstance(sua_id, str) and isinstance(status, bool)):
            return False, 'Incorrect arguments!\n\nUse:\nSUA name,ON/OFF or SUA name,ON/OFF,lat1,lon1,lat2,lon2,...'

        # Create SUA polygon border only if coordinates are given
        if not coords:
            border = None
        else:
            # Make sure the border is a valid ring (first and last lat,lon pair should be the same)
            if (coords[0], coords[1]) != (coords[-2], coords[-1]):
                coords = coords + (coords[0], coords[1])

            # Reshape into array of (lat, lon) pairs
            points = np.reshape(coords, (len(coords) // 2, 2))

            # Check if the polygon is ccw, if not, then flip the coordinate order
            dir_sum = 0
            for ii in range(len(points) - 1):
                edge = (points[ii + 1][1] - points[ii][1]) * (points[ii + 1][0] + points[ii][0])
                dir_sum += edge
            if dir_sum >= 0:
                points = points[::-1,:]

            border = Path(points, closed = True)

        # Update the dictionary using the current stack input
        self._update_suadict(sua_id, status, coords, border)

        # If successful, return True and a string that will be displayed in the command window
        return True, sua_id + ' status is: ' + ('ACTIVE' if status else 'INACTIVE')

    def _update_suadict(self, sua_id, status, coords, border):
        """ Update the dictionary containing the SUA definitions 

            Possible actions:
            * If sua_id is not already in suadict then it will be added in full.
            * If sua_id is already in suadict then the status will always be changed using the new input,
              however the coordinates will only be changed if they are not Null and are different from existing coordinates.
        """

        if sua_id not in self.suadict:
            self._add_to_dict(sua_id, status, coords, border)
        else:
            # Always update status with current status input ("ON"-> True; "OFF"-> False)
            self.suadict[sua_id][0] = status

            # Only update coords if different from existing coords and not None.
            if self.suadict[sua_id][1] != coords and coords is not None:
                self.suadict[sua_id][1] = coords
                self.suadict[sua_id][2] = border

                # Remove existing polygon from gui
                self.undraw_sua(sua_id)

                # If the SUA is active ("ON"), draw the polygon in the radarwidget
                if status:
                    self.draw_sua(sua_id, coords)

    def _add_to_dict(self, sua_id, status, coords, border):
        """ Add new SUA definition to the dictionary """

        self.suadict[sua_id] = [status, coords, border]

        # If the SUA is active ("ON"), draw the polygon in the radarwidget
        if status:
            self.draw_sua(sua_id, coords)

    def _remove_from_dict(self, sua_id):
        """ Delete a SUA definition from the dictionary and GUI screen """

        self.undraw_sua(sua_id)
        self.suadict.pop(sua_id)

    def _count_ac_in_sua(self):
        """ Count number of aircraft inside the special use areas """

        for area_id, area in self.suadict.items():
            if area[2]:
                inside = self.is_inside(area[2], traf.lat, traf.lon)
                ids = set(np.array(traf.id)[inside])

                if ids:
                    pass
                    # print("Traffic in Area " + str(area_id) + ": " + str(ids))
        
        self._ac_crs_intersect_sua()

    def _count_ac_entering_sua(self):
        """ Determine which aircraft will be inside the SUA in x minutes time 
        
            NOTE: Deprecated function - does only take into account situation at one specific moment
        """

        look_ahead_time = 300
        newlat, newlon = self.future_ac_pos(look_ahead_time)

        for area_id, area in self.suadict.items():
            if area[2]:
                inside = self.is_inside(area[2], newlat, newlon)
                ids = set(np.array(traf.id)[inside])

                if ids:
                    pass
                    # print(str(look_ahead_time / 60) +
                    #       " min prediction for area " + str(area_id) + ": " + str(ids))

    def _ac_crs_intersect_sua(self):
        """ Calculate which aircraft's course intersects a sua polygon """

        # NOTE: This function probably requires substantial refactoring to reduce computational cost
        
        # Looks dirty because unable to use vectorized traf data for qdrdist functions....
        for ii in range(traf.ntraf):
            ac_id = traf.id[ii]
            ac_lat = traf.lat[ii]
            ac_lon = traf.lon[ii]
            ac_trk = traf.trk[ii]
            ac_gs = traf.gs[ii]
            ac_gs_n = traf.gsnorth[ii]
            ac_gs_e = traf.gseast[ii]

            # List to store all information of all sua's that are intersected
            ac_sua_intersect = []

            # Loop over all sua definitions in the dictionary
            for sua_id, sua in self.suadict.items():

                # Get column vectors containing sua vertex latitudes and longitudes
                border = sua[2]
                
                # Make sure aircraft is not currently inside sua polygon (this causes tangent functions to fail)
                if not self.is_inside(border, ac_lat, ac_lon):
                    tan_l, tan_r, dist_l, dist_r = self._point_poly_tangents([ac_lat, ac_lon], border.vertices)
                    
                    # Convert from -180...180 deg range to 0...360 deg wrt to north
                    trk_l = (360 + tan_l) % 360
                    trk_r = (360 + tan_r) % 360
                    #print("{}, {:.2f} - {} l: {:.2f}, r: {:.2f}".format(ac_id, ac_trk, sua_id, trk_l, trk_r))
                    
                    # Check if current aircraft track lies in between tangents then add polygon to
                    # list of intersected polygons
                    if self.crs_is_between(ac_trk, trk_l, trk_r):

                        # Approximate time to intersection
                        t_intersect = self._time_to_intersect([ac_lat, ac_lon], ac_trk, ac_gs_n, ac_gs_e, border.vertices)
                        print("{} - {} time to intersect: {} s".format(ac_id, sua_id, t_intersect))

                        ac_sua_intersect.append([sua_id, trk_l, trk_r, dist_l, dist_r, t_intersect])

                    # Sort based on distance to tangent vertices such that closest polygon is at top of list
                    # NOTE: only distance to tangent vertices is taken; actual distance to intersection may be much smaller
                    ac_sua_intersect.sort(key=lambda x: x[5])

                    # Waypoint recovery after conflict: Find the next active waypoint
                    # and send the aircraft to that waypoint.
                    iwpid = traf.ap.route[ii].findact(ii)
                    if iwpid != -1:  # To avoid problems if there are no waypoints
                        # Not sure if traf.actwp is the same wp as iwpid
                        wp_lat = traf.actwp.lat[ii]
                        wp_lon = traf.actwp.lon[ii]

                        # Find track to waypoint
                        qdr_wp, _ = qdrdist(ac_lat, ac_lon, wp_lat, wp_lon)
                        trk_wp = (360 + qdr_wp) % 360

                        # If polygon is not in between ac and waypoint then proceed direct to the waypoint
                        # if not self.crs_is_between(trk_wp, trk_l, trk_r) and not ac_sua_intersect:
                        if ac_sua_intersect:
                            if ac_sua_intersect[0][5] < 300:
                                # Avoid via closest angle to waypoint
                                diff_l = abs(((trk_wp - ac_sua_intersect[0][1] + 180) % 360 ) - 180)
                                diff_r = abs(((trk_wp - ac_sua_intersect[0][2] + 180) % 360 ) - 180)

                                if diff_l < diff_r:
                                    crs_sua_avoid = (-1 + ac_sua_intersect[0][1]) % 360
                                else:
                                    crs_sua_avoid = (1 + ac_sua_intersect[0][2]) % 360

                                stack.stack("HDG {},{}".format(ac_id, crs_sua_avoid))
                                print("{} instructed turn to hdg {}".format(ac_id, crs_sua_avoid))

                            else:

                                # Go direct to the waypoint
                                #print("{}, {:.2f} -- l: {:.2f}, r: {:.2f}".format(ac_id, trk_wp, trk_l, trk_r))
                                print("Clear of conflict - turning direct to waypoint")
                                traf.ap.route[ii].direct(ii, traf.ap.route[ii].wpname[iwpid])
                        else:
                            print("Clear of conflict - turning direct to waypoint")
                            traf.ap.route[ii].direct(ii, traf.ap.route[ii].wpname[iwpid])

                else:
                    #input(" Press <ENTER> to continue ")
                    print("Inside polygon")

    def _point_poly_tangents(self, ac_pos, border):
        """ For a given aircraft position find left- and rightmost courses that are tangent to a given polygon """

        # Start by assuming both tangents touch at polygon vertex with index 0
        idx_l = 0
        idx_r = 0

        # Loop over vertices 1:n-1 and evaluate position of aircraft wrt the edges to find the 
        # indices of the vertices at which the tangents touch the polygon
        #
        # Algorithm from: http://geomalgorithms.com/a15-_tangents.html
        for i in range(1, len(border) - 1):
            eprev = self.is_left(border[i - 1], border[i], ac_pos)
            enext = self.is_left(border[i], border[i + 1], ac_pos)

            if eprev <= 0 and enext > 0:
                if not (self.is_left(ac_pos, border[i], border[idx_r]) < 0):
                    idx_r = i
            elif eprev > 0 and enext <= 0:
                if not (self.is_left(ac_pos, border[i], border[idx_l]) > 0):
                    idx_l = i

        # Calculate tangent courses from aircraft to left- and rightmost vertices
        qdr_l, dist_l = qdrdist(ac_pos[0], ac_pos[1], border[idx_l][0], border[idx_l][1])
        qdr_r, dist_r = qdrdist(ac_pos[0], ac_pos[1], border[idx_r][0], border[idx_r][1])

        return qdr_l, qdr_r, dist_l, dist_r

    def _time_to_intersect(self, ac_pos, ac_trk, ac_gs_n, ac_gs_e, border):
        """ Calculate the time to intersection of a polygon edge """

        # Set up list to store time to intersect (concave polygons can have multiple valid intersections)
        t_intersect = []
        
        # Loop over all edges and use only those where the aircraft is to the right of the edge
        for i in range(len(border) - 1):
            if self.is_left(ac_pos, border[i], border [i + 1]) < 0:
                # Calculate tangent courses from aircraft to both vertices on the edge
                qdr_l, _ = qdrdist(ac_pos[0], ac_pos[1], border[i][0], border[i][1])
                qdr_r, _ = qdrdist(ac_pos[0], ac_pos[1], border[i + 1][0], border[i + 1][1])

                trk_l = (360 + qdr_l) % 360
                trk_r = (360 + qdr_r) % 360

                # If the aircraft's current track intersects the edge, calculate the
                # approximate time to intersection
                if self.crs_is_between(ac_trk, trk_l, trk_r):
                    
                    # Aircraft ground speed vector in kts
                    u = [ac_gs_n * 1.94384, ac_gs_e * 1.94384]

                    # Calculate vector from 1st to 2nd vertex in edge
                    qdr_v, d_v = qdrdist(border[i][0], border[i][1], border[i+1][0], border[i+1][1])
                    v = [d_v * np.cos(np.radians(qdr_v)), d_v * np.sin(np.radians(qdr_v))]

                    # Calculate vector from 1st vertex to aircraft position
                    qdr_w, d_w = qdrdist(border[i][0], border[i][1], ac_pos[0], ac_pos[1])
                    w = [d_w * np.cos(np.radians(qdr_w)), d_w * np.sin(np.radians(qdr_w))]

                    # Calculate time to polygon intersect (in seconds)
                    t = ((v[1]*w[0]-v[0]*w[1]) / (v[0]*u[1]-v[1]*u[0])) * 3600

                    t_intersect.append(t)

        return min(t_intersect)

    @staticmethod
    def draw_sua(sua_id, coords):
        """ Draw the polygon corresponding to sua_id in the radarwidget window """

        areafilter.defineArea(sua_id, 'POLY', coords)

    @staticmethod
    def undraw_sua(sua_id):
        """ Remove the polygon corresponding to sua_id from the radarwidget window """

        areafilter.deleteArea(sua_id)

    # Copied from traffic.py and edited
    @staticmethod
    def future_ac_pos(dt):
        """ Predict future (lat, lon) position of all aircraft after time step dt using linear state extrapolation"""

        # Update position
        #newalt = np.where(traf.swaltsel, traf.alt + traf.vs * dt, traf.pilot.alt)  # Uncomment to 
        newlat = traf.lat + np.degrees(dt * traf.gsnorth / Rearth)
        newcoslat = np.cos(np.deg2rad(newlat))
        newlon = traf.lon + np.degrees(dt * traf.gseast / newcoslat / Rearth)

        return newlat, newlon

    # Copied from tools/areafilter.py and edited
    @staticmethod
    def is_inside(border, lat, lon):
        """ Takes vectors with lat and lon and returns boolean list per point """

        points = np.vstack((lat, lon)).T
        inside = border.contains_points(points)

        return inside

    @staticmethod
    def crs_is_between(crs, crs_l, crs_r):
        """ Check if a given course crs lies in between crs_l and crl_r (in clockwise direction) """

        if ((crs_l > crs_r) and (crs > crs_l or crs < crs_r)) or ((crs_l < crs_r) and (crs > crs_l and crs < crs_r)):
            return True
       
        return False

    @staticmethod
    def is_left(p0, p1, p2):
        """  Check if point p2 lies to the left of the line through p0 and p1 
        
            Returns 
                > 0 if p2 lies on the left side of the line
                = 0 if p2 lies exactly on the line
                < 0 if p2 lies on the right side of the line
        """

        return (p1[1] - p0[1]) * (p2[0] - p0[0]) - (p2[1] - p0[1]) * (p1[0] - p0[0])