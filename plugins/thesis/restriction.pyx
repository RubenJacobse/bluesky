"""
Module that defines the AreaRestriction class in which the


© Ruben Jacobse, 2019
"""

# Cython imports
import cython
import numpy as np
cimport numpy as np
from libc.math cimport sin, cos, pi, atan2, sqrt

# Third-party imports
import shapely.geometry as spgeom

# BlueSky imports
from bluesky.tools import areafilter, geo

# Typedefs for numpy array
DTYPE = np.double
ctypedef np.double_t DTYPE_t


cdef class AreaRestriction:
    """ Class that represents a single Restricted Airspace Area. """

    cdef public object area_id, status, verts, ring, poly, coords

    def __init__(self, area_id, status, coords):

        # Store input parameters as attributes
        self.area_id = area_id
        self.status = status

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

        # Draw polygon on BlueSky RadarWidget canvas
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

    @cython.cdivision(True)
    @cython.boundscheck(False)
    @cython.wraparound(False)
    cpdef calc_tangents(self,
                        Py_ssize_t num_traf,
                        DTYPE_t[:] ac_lon,
                        DTYPE_t[:] ac_lat):
        """
        For a given aircraft position find left- and rightmost courses
        that are tangent to a given polygon as well as the distance to
        the corresponding vertices.
        """

        # Initialize arrays to store qdrs and distances
        cdef np.ndarray[DTYPE_t, ndim=1] qdr_left = np.empty(num_traf, dtype=DTYPE)
        cdef np.ndarray[DTYPE_t, ndim=1] qdr_right = np.empty(num_traf, dtype=DTYPE)

        # Create view containing [lon, lat] for each vertex
        cdef DTYPE_t[:, :] vertex = self.verts

        # Declare variables used inside loop body
        cdef DTYPE_t edge_prev, edge_next, avg_lat, cos_avg_lat, delta_lat, delta_lon
        cdef Py_ssize_t idx_left_vert, idx_right_vert

        # Calculate qdrs for each aircraft
        cdef Py_ssize_t ii, jj # loop variables
        for ii in range(num_traf):
            # Start by assuming both tangents touch at vertex with index 0
            idx_left_vert = 0
            idx_right_vert = 0

            # Loop over vertices 1:n-1 and evaluate position of aircraft wrt
            # the edges to find the indices of the vertices at which the
            # tangents touch the polygon
            #
            # Algorithm from: http://geomalgorithms.com/a15-_tangents.html
            for jj in range(1, vertex.shape[0] - 1):
                edge_prev = _is_left_of_line(vertex[jj - 1, 0],
                                             vertex[jj - 1, 1],
                                             vertex[jj, 0],
                                             vertex[jj, 1],
                                             ac_lon[ii],
                                             ac_lat[ii])
                edge_next = _is_left_of_line(vertex[jj, 0],
                                             vertex[jj, 1],
                                             vertex[jj + 1, 0],
                                             vertex[jj + 1, 1],
                                             ac_lon[ii],
                                             ac_lat[ii])

                if edge_prev <= 0 and edge_next > 0:
                    if not _is_left_of_line(ac_lon[ii],
                                            ac_lat[ii],
                                            vertex[jj, 0],
                                            vertex[jj, 1],
                                            vertex[idx_right_vert, 0],
                                            vertex[idx_right_vert, 1]) < 0:
                        idx_right_vert = jj
                elif edge_prev > 0 and edge_next <= 0:
                    if not _is_left_of_line(ac_lon[ii],
                                            ac_lat[ii],
                                            vertex[jj, 0],
                                            vertex[jj, 1],
                                            vertex[idx_left_vert, 0],
                                            vertex[idx_left_vert, 1]) > 0:
                        idx_left_vert = jj

            # Calculate approximate tangents from aircraft to left- and
            # rightmost vertices using equirectangular earth approximation
            qdr_left[ii] = kwikqdr(ac_lat[ii],
                                   ac_lon[ii],
                                   vertex[idx_left_vert][1],
                                   vertex[idx_left_vert][0])
            qdr_right[ii] = kwikqdr(ac_lat[ii],
                                    ac_lon[ii],
                                    vertex[idx_right_vert][1],
                                    vertex[idx_right_vert][0])

        return qdr_left, qdr_right

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

        return _is_left_of_line(line_start[0], line_start[1],
                                line_end[0], line_end[1],
                                point[0], point[1])


cdef DTYPE_t _is_left_of_line(DTYPE_t line_start_x,
                              DTYPE_t line_start_y,
                              DTYPE_t line_end_x,
                              DTYPE_t line_end_y,
                              DTYPE_t point_x,
                              DTYPE_t point_y):
    """
    Check if point lies to the left of the line through line_start
    to line_end.
    Returns:
        > 0 if point lies on the left side of the line
        = 0 if point lies exactly on the line
        < 0 if point lies on the right side of the line
    """
    return ((line_end_x - line_start_x) * (point_y - line_start_y)
            - (point_x - line_start_x) * (line_end_y - line_start_y))


@cython.cdivision(True)
cdef DTYPE_t kwikqdr(DTYPE_t lata,
                     DTYPE_t lona,
                     DTYPE_t latb,
                     DTYPE_t lonb):
    """Gives quick and dirty qdr[deg]
       from lat/lon. (note: does not work well close to poles)"""

    cdef DTYPE_t re, dlat, dlon, cavelat, dangle, qdr

    re      = 6371000.0  # radius earth [m]
    dlat    = (latb - lata) * pi / 180
    dlon    = (lonb - lona) * pi / 180
    cavelat = cos(((lata + latb) * pi / 180) * 0.5)

    dangle  = sqrt(dlat * dlat + dlon * dlon * cavelat * cavelat)
    # dist    = re * dangle / nm

    qdr = atan2(dlon * cavelat, dlat) * 180 / pi
    qdr = qdr % 360.0

    return qdr
