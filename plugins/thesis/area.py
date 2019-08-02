"""
Module that defines the RestrictedAirspaceArea class in which the 


© Ruben Jacobse, 2019
"""

# Third-party imports
import numpy as np
import shapely.geometry as spgeom

# BlueSky imports
from bluesky.tools import areafilter
from bluesky.tools.geo import qdrdist

# Local imports
from . import geo as tg


class RestrictedAirspaceArea():
    """ Class that represents a single Restricted Airspace Area. """

    def __init__(self, area_id, status, gseast, gsnorth, coords):

        # Store input parameters as attributes
        self.area_id = area_id
        self.status = status
        self.gsnorth = gsnorth
        self.gseast = gseast
        self.gs = np.sqrt(gsnorth ** 2 + gseast ** 2)

        # Enforce that the coordinate list defines a valid ring
        coords = self._check_poly(coords)

        # Area coordinates will be stored in four formats:
        #  - self.verts  : numpy array containing [lon, lat] pairs per vertex
        #  For geometrical calculations:
        #  - self.ring   : Shapely LinearRing (in lon,lat order)
        #  - self.poly   : Shapely Polygon (in lon, lat order)
        # For use in BlueSky functions
        #  - self.coords : List of sequential lat,lon pairs
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

        if not (self.gsnorth or self.gseast):
            return

        # Calculate new vertex positions after timestep dt
        curr_lon = self.verts[:, 0]
        curr_lat = self.verts[:, 1]
        newlon, newlat = tg.calc_future_pos(dt,
                                            curr_lon,
                                            curr_lat,
                                            self.gseast,
                                            self.gsnorth)

        # Update vertex representations in all formats
        self.verts = np.array([newlon, newlat]).T
        self.ring = spgeom.LinearRing(self.verts)
        self.poly = spgeom.Polygon(self.verts)
        self.coords = self._verts2coords(self.verts)

        # Remove current drawing and redraw new position on the BlueSky
        # RadarWidget canvas
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

        # Ensure the border is a closed ring
        if (coords[0], coords[1]) != (coords[-2], coords[-1]):
            coords = coords + [coords[0], coords[1]]

        # Ensure coordinates are defined in ccw direction
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

        coords = [lat_0, lon_0, ..., lat_n, lon_n]
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

        verts = np.array([[lon_0, lat_0], ..., [lon_n, lat_n]])
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
        qdr_left = np.zeros(num_traf, dtype=float)
        qdr_right = np.zeros(num_traf, dtype=float)

        # Create array containing [lon, lat] for each vertex
        vertex = np.array(self.ring.coords.xy).T

        # Calculate qdrs for each aircraft
        for ii in range(num_traf):
            ac_pos = [ac_lon[ii], ac_lat[ii]]

            # Start by assuming both tangents touch at vertex with index 0
            idx_left_vert = 0
            idx_right_vert = 0

            # Loop over vertices 1:n-1 and evaluate position of aircraft wrt
            # the edges to find the indices of the vertices at which the tangents
            # touch the polygon
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
        lox_angle_left = np.zeros(num_traf, dtype=float)
        lox_angle_right = np.zeros(num_traf, dtype=float)

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
            lox_angle_left[ii] = tg.rhumb_azimuth(ac_pos[1],
                                                  ac_pos[0],
                                                  vertex[idx_left_vert][1],
                                                  vertex[idx_left_vert][0])
            lox_angle_right[ii] = tg.rhumb_azimuth(ac_pos[1],
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
