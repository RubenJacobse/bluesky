"""
    Based on 2016 NASA paper by Narkawicz and Hagen:
    "Algorithms for Collision Detection Between a Point and a Moving Polygon,
    with Applications to Aircraft Weather Avoidance"

    Functions and variable names correspond to notation used in paper as much as possible.
    © Ruben Jacobse, 2019
"""

import numpy as np

# General quadratic functions and evaluations
def quad(a, b, c, t):
    """ Evaluate univariate quadratic in t:

        quad = at² + bt + c """

    return a * t**2 + b*t + c

def quad_min_le_D(a, b, c, D):
    """ Determine if quadratic takes value less than D on the
        interval [0, 1] """

    if quad(a, b, c, 0) < D or quad(a, b, c, 1) < D:
        return True
    elif a > 0 and b <= 0 and -b <= 2*a and b**2 - 4*a*(c - D) > 0:
        return True

    return False

def quad2D(a, b, c, d, e, f, r, t):
    """ Evaluate bivariate quadratic in r and t:

        quad2D = ar² + bt² + crt + dr + et + f """

    return a * r**2 + b * t**2 + c*r*t + d*r + e*t + f

# Theory behind this method is not explained; not sure how it works
def quad_min_box_le_D(a, b, c, d, e, f, D):
    """ Determine if bivariate quadratic takes value less than D
        for r and t on interval [0,1] """

    if quad_min_le_D(a, c + d, quad(b, e, f, 1), D):
        return True
    elif quad_min_le_D(a, d, f, D):
        return True
    elif quad_min_le_D(b, c + e, quad(a, d, f, 1), D):
        return True
    elif quad_min_le_D(b, e, f, D):
        return True
    else:
        disc = c**2 - 4*a*b
        mx = 2*b*d - c*e
        my = 2*a*e - c*d

        # Test conditions that must all pass:
        cond1 = 0 <= mx * disc <= disc**2
        cond2 = 0 <= my * disc <= disc**2
        cond3 = quad2D(a, b, c, d * disc, e * disc, f * disc**2, mx, my) < (D * disc**2)

        if cond1 and cond2 and cond3:
            return True

    # Only reached when none of the criteria are met
    return False

# Geometric functions
def near_edge(p_i, p_j, s, BUFF):
    """ Check whether point s lies within a distance BUFF from the
        line between point_i and point_j """

    if np.linalg.norm(s - p_i)**2 < BUFF**2 or np.linalg.norm(s - p_j)**2 < BUFF**2:
        return True
    elif not np.array_equal(p_i, p_j):
        a = np.dot(p_j - p_i, p_j - p_i)
        b = 2 * np.dot(p_i - s, p_j - p_i)
        c = np.dot(p_i - s, p_i - s)

        if quad_min_le_D(a, b, c, BUFF**2):
            return True

    return False

def num_cross(P, s):
    """ Count the number of times the infinite ray originating
        at s in positive y direction crosses the edges of P. """

    # Keep track of number of times the ray crosses the edges of the polygon
    crosscount = 0

    # Requires P to have __len__ and be two-dimensional
    # Assumes P = np.array([[x0, y0], [x1, y1], ..., [xn, yn]])
    for ii in range(len(P) - 1):
        p_i = P[ii]
        p_j = P[ii + 1]
        p_ix = p_i[0]
        p_iy = p_i[1]
        p_jx = p_j[0]
        p_jy = p_j[1]
        s_x = s[0]

        # Calculate vector perpendicular to (p_j-p_i)
        # If p = [px, py] then p_perp = [-py, px] is its right perpendicular
        p_i_perp = np.array([-p_iy, p_ix])
        p_j_perp = np.array([-p_jy, p_jx])
        p_ji_perp = np.subtract(p_j_perp, p_i_perp)

        if p_ix >= s_x and p_jx >= s_x:
            continue
        if p_ix < s_x and p_jx < s_x:
            continue
        if p_ix == p_jx:
            continue
        # Note: definition given in Narkawicz & Hagen paper is possibly wrong.... (by factor -1)
        if -1 * (p_jx - p_ix) * (np.dot(s - p_i, p_ji_perp)) >= 0:
            crosscount += 1
        else:
            continue

    return crosscount

def definitely_outside(P, s, BUFF):
    """ Check that s is not inside P or less than distance
        BUFF from one of the edges of P """

    for idx in range(len(P) - 1):
        p_i = P[idx]
        p_j = P[idx + 1]

        if near_edge(p_i, p_j, s, BUFF):
            return False

    if num_cross(P, s) % 2 == 0:
        return True

    # If none of the above conditions are met s is inside P
    return False

def segments_close(c, d, e, f, BUFF):
    """ Check if two segments between points c-d and e-f are within
        distance BUFF from each other. """

    if near_edge(e, f, c, BUFF) or near_edge(e, f, d , BUFF):
        return True
    elif near_edge(c, d, e, BUFF) or near_edge(c, d, f, BUFF):
        return True
    else:
        q = c - e
        u = d - c
        w = f - e
        if quad_min_box_le_D(np.dot(u, u), np.dot(w, w), -2*np.dot(u, w), 2*np.dot(q, u), -2*np.dot(q, w), np.dot(q, q), BUFF**2):
            return True

    # If none of the above conditions are met the segments are not close
    return False

def detect_same_2D(T, s, v, P, V, BUFF):
    """ Detect if conflict exists between 2D point s moving at velocity
        v and polygon P moving with vertex velocities V in the time
        interval [0,T] seconds within margin BUFF.

        s = np.array([sx, sy])
        v = np.array([vx, vy])
        P = np.array([[x0, y0], [x1, y1], ..., [xn, yn]])
        V = np.array([[v0, v0], [v1, v1], ..., [vn, vn]]) """

    # If s inside P or s+T*v inside P+T*V return true
    if not definitely_outside(P, s, BUFF):
        return True
    if not definitely_outside(P + T*V, s + T*v, BUFF):
        return True

    # Check for crossing of edges
    for idx in range(len(P) - 1):
        p_i = P[idx]
        p_j = P[idx + 1]
        v_i = V[idx]

        if segments_close(s, s + T*(v - v_i), p_i, p_j, BUFF):
            return True

    # If none of the above criteria are met s does not conflict with P in
    # time interval [0,T]
    return False
