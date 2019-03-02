import pytest
import pdb

import numpy as np

from .. import area_mathfuncs as amf

FLT_DIFF = 0.01 # Acceptable difference to assert "equality" of float variables
POLY_BUFF = 0.1 # Buffer for tests with lines and polygons

def assert_flt(arg1, arg2):
    """ Assert that two floating point numbers differ less than FLT_DIFF. """

    assert abs(arg1 - arg2) < FLT_DIFF

def test_quad():
    """ Test function that evaluates a univariate quadratic function. """

    assert amf.quad(1, 2, 3, 0) == 3
    assert amf.quad(0, 0, 0, 0) == 0
    assert amf.quad(1, 1, 1, 1) == 3
    assert amf.quad(-1, 1, -1, 1) == -1

    assert_flt(amf.quad(1, 2, 3, 1.001), 6.0)
    assert_flt(amf.quad(1, 2, 3, -1.001), 2.0)

def test_quad_min_le_D():
    """ Test function that determines whether quad() takes value less than D
        anywhere on interval [0,1] """

    assert not amf.quad_min_le_D(1, 0, 1, 0)
    assert not amf.quad_min_le_D(1, 0, 1, 1)
    assert amf.quad_min_le_D(-1, 0, 1, 1)
    assert not amf.quad_min_le_D(1, 0, 1, 0)
    assert amf.quad_min_le_D(1, 0, 1, 1.1)

def test_quad2D():
    """ Test the function that evaluates a bivariate quadratic function. """

    assert_flt(amf.quad2D(0, 0, 0, 0, 0, 0, 1.2, 1.3), 0.0)
    assert_flt(amf.quad2D(1, 1, 1, 1, 1, 1, 0, 0), 1.0)
    assert_flt(amf.quad2D(1, 1, 1, 1, 1, 1, 1, 1), 6.0)
    assert_flt(amf.quad2D(-1, -1, -1, -1, -1, -1, -1, -1), -2.0)

def test_quad_min_box_le_D():
    """ Tests the function that determines whether a bivariate quadratic
        takes a value less than D on the interval [0,1]x[0,1] """

    assert amf.quad_min_box_le_D(1, 1, 0, 0, 0, 0, 0.001)
    assert not amf.quad_min_box_le_D(1, 1, 0, 0, 0, 0, -0.1)

def test_near_edge():
    """ Test the function that determines whether a point lies within
        a distance POLY_BUFF from one an edge (line piece between two
        points). """

    # Test using triangle (0,0), (0,1), (1,0)
    p_0 = np.array([0, 0])
    p_1 = np.array([0, 1])
    p_2 = np.array([1, 0])

    # Test points to be used
    s_0 = np.array([0, 0])
    s_1 = np.array([0, 1])
    s_2 = np.array([0.01, 0.01])
    s_3 = np.array([-0.01 , -0.01])
    s_4 = np.array([1, 1])
    s_5 = np.array([0.5, 0.5])
    s_6 = np.array([0.6, 0.6])

    # Line piece p0-p1
    assert amf.near_edge(p_0, p_1, s_0, POLY_BUFF)
    assert amf.near_edge(p_0, p_1, s_1, POLY_BUFF)
    assert amf.near_edge(p_0, p_1, s_2, POLY_BUFF)
    assert amf.near_edge(p_0, p_1, s_3, POLY_BUFF)
    assert not amf.near_edge(p_0, p_1, s_4, POLY_BUFF)
    assert not amf.near_edge(p_0, p_1, s_5, POLY_BUFF)
    assert not amf.near_edge(p_0, p_1, s_6, POLY_BUFF)

    # Line piece p0-p2
    assert amf.near_edge(p_0, p_2, s_0, POLY_BUFF)
    assert not amf.near_edge(p_0, p_2, s_1, POLY_BUFF)
    assert amf.near_edge(p_0, p_2, s_2, POLY_BUFF)
    assert amf.near_edge(p_0, p_2, s_3, POLY_BUFF)
    assert not amf.near_edge(p_0, p_2, s_4, POLY_BUFF)
    assert not amf.near_edge(p_0, p_2, s_5, POLY_BUFF)
    assert not amf.near_edge(p_0, p_2, s_6, POLY_BUFF)

    # Line piece p1-p2
    assert not amf.near_edge(p_1, p_2, s_0, POLY_BUFF)
    assert amf.near_edge(p_1, p_2, s_1, POLY_BUFF)
    assert not amf.near_edge(p_1, p_2, s_2, POLY_BUFF)
    assert not amf.near_edge(p_1, p_2, s_3, POLY_BUFF)
    assert not amf.near_edge(p_1, p_2, s_4, POLY_BUFF)
    assert amf.near_edge(p_1, p_2, s_5, POLY_BUFF)
    assert not amf.near_edge(p_1, p_2, s_6, POLY_BUFF)

def test_num_cross():
    """ Test the function that calculates the number of times an infinite
        ray originating at a point crosses the edges of a polygon. """

    # P: Simple triangle
    # Vertices (0,0), (1,0), (0,1)
    p_0 = np.array([0, 0])
    p_1 = np.array([1, 0])
    p_2 = np.array([0, 1])

    # Polygons in cw and ccw direction
    P_0 = np.array([p_0, p_1, p_2, p_0])
    P_1 = np.array([p_1, p_2, p_0, p_1])
    P_2 = np.array([p_2, p_0, p_1, p_2])
    P_3 = np.array([p_0, p_2, p_1, p_0])
    P_4 = np.array([p_1, p_0, p_2, p_1])
    P_5 = np.array([p_2, p_1, p_0, p_2])

    # Test points s at which rays start
    s_0 = np.array([1, 1])
    s_1 = np.array([0.5, -1])
    s_2 = np.array([-1, -1])
    s_3 = np.array([2, -1])
    s_4 = np.array([0.25, 0.25])

    pList = [P_0, P_1, P_2, P_3, P_4, P_5]
    sList = [s_0, s_1, s_2, s_3, s_4]
    crossList = [0, 2, 0, 0, 1]

    # Test all combinations P,s
    for poly in pList:
        for idx, s in enumerate(sList):
            assert amf.num_cross(poly, s) == crossList[idx]

    ###############################################################

    # Q: More complex polygon (non-convex)
    # Vertices (-1,-1), (1,-1), (-0.5,0.5), (2,1), (-1.5,3), (-2,1)
    q_0 = np.array([-1, -1])
    q_1 = np.array([1, -2])
    q_2 = np.array([-0.5, 0.5])
    q_3 = np.array([2, 1])
    q_4 = np.array([-1.5, 3])
    q_5 = np.array([-2, 1])

    # Polygons in cw and ccw direction
    Q_0 = np.array([q_0, q_1, q_2, q_3, q_4, q_5, q_0])
    Q_1 = np.array([q_1, q_2, q_3, q_4, q_5, q_0, q_1])
    Q_2 = np.array([q_2, q_3, q_4, q_5, q_0, q_1, q_2])
    Q_3 = np.array([q_3, q_4, q_5, q_0, q_1, q_2, q_3])
    Q_4 = np.array([q_4, q_5, q_0, q_1, q_2, q_3, q_4])
    Q_5 = np.array([q_5, q_4, q_3, q_2, q_1, q_0, q_5])
    Q_6 = np.array([q_3, q_2, q_1, q_0, q_5, q_4, q_3])
    Q_7 = np.array([q_0, q_5, q_4, q_3, q_2, q_1, q_0])

    # Test points t at which rays start
    t_0 = np.array([1.5, -2.5])
    t_1 = np.array([0.5, -2.5])
    t_2 = np.array([0, -1])
    t_3 = np.array([-1.75, -1.5])
    t_4 = np.array([1, -1])
    t_5 = np.array([3, 0])

    qList = [Q_0, Q_1, Q_2, Q_3, Q_4, Q_5, Q_6, Q_7]
    tList = [t_0, t_1, t_2, t_3, t_4, t_5]
    crossList = [2, 4, 3, 2, 2, 0]

    # Test all combinations Q,t
    for poly in qList:
        for idx, t in enumerate(tList):
            assert amf.num_cross(poly, t) == crossList[idx]

def test_definitely_outside():
    """ Test function that checks if a point is at least a distance
        POLY_BUFF outside a polygon """

    # P: Simple triangle
    # Vertices (0,0), (1,0), (0,1)
    p_0 = np.array([0, 0])
    p_1 = np.array([1, 0])
    p_2 = np.array([0, 1])

    # Polygons in cw and ccw direction
    P_0 = np.array([p_0, p_1, p_2, p_0])

    # Points to test
    s_0 = np.array([-0.01, -0.01])
    s_1 = np.array([-0.1, -0.1])
    s_2 = np.array([0.01, 0.01])
    s_3 = np.array([0.01, 1.01])
    s_4 = np.array([1.01, 0.01])

    assert not amf.definitely_outside(P_0, s_0, POLY_BUFF)
    assert amf.definitely_outside(P_0, s_1, POLY_BUFF)
    assert not amf.definitely_outside(P_0, s_2, POLY_BUFF)
    assert not amf.definitely_outside(P_0, s_3, POLY_BUFF)
    assert not amf.definitely_outside(P_0, s_4, POLY_BUFF)

def test_segments_close():
    """ Test the function that determines whether two line pieces
        are separated by less than distance POLY_BUFF. """

    s_10 = np.array([0, 0])
    s_11 = np.array([1, 0])
    s_20 = np.array([0, 0.5])
    s_21 = np.array([1, 0.5])
    s_30 = np.array([0, 0.099])
    s_31 = np.array([1, 0.099])
    s_40 = np.array([-1, -1])
    s_41 = np.array([1, 1])
    s_50 = np.array([-1, 1])
    s_51 = np.array([1, -1])

    assert not amf.segments_close(s_10, s_11, s_20, s_21, POLY_BUFF)
    assert not amf.segments_close(s_20, s_21, s_10, s_11, POLY_BUFF)
    assert amf.segments_close(s_10, s_11, s_30, s_31, POLY_BUFF)
    assert amf.segments_close(s_40, s_41, s_10, s_11, POLY_BUFF)
    assert amf.segments_close(s_40, s_41, s_20, s_21, POLY_BUFF)
    assert amf.segments_close(s_40, s_41, s_30, s_31, POLY_BUFF)
    assert amf.segments_close(s_50, s_51, s_10, s_11, POLY_BUFF)
    assert not amf.segments_close(s_50, s_51, s_20, s_21, POLY_BUFF)
    assert amf.segments_close(s_50, s_51, s_30, s_31, POLY_BUFF)
    assert amf.segments_close(s_40, s_41, s_50, s_51, POLY_BUFF)
    assert amf.segments_close(s_40, s_41, s_50, s_51, POLY_BUFF)

def test_detect_same_2D():
    """ Test function that determines whether a point and polygon that
        are both moving are in conflict. """

    s_0 = np.array([0, 0.5])
    vs_00 = np.array([0.1, 0])
    vs_01 = np.array([0, 0])
    vs_02 = np.array([0, 0.1])

    s_1 = np.array([1.5, 0.5])
    vs_1 = np.array([0, -0.5])

    P = np.array([[1, 0], [2, 0], [2, 1], [1, 1], [1, 0]])

    vp_0 = [0, 0]
    vp_1 = [-0.1, 0]
    vp_2 = [-0.1, 0.1]
    V_0 = np.full(np.shape(P), vp_0)
    V_1 = np.full(np.shape(P), vp_1)
    V_2 = np.full(np.shape(P), vp_2)

    # Both point and polygon not moving
    assert not amf.detect_same_2D(0, s_0, vs_01, P, V_0, POLY_BUFF)
    assert not amf.detect_same_2D(10, s_0, vs_01, P, V_0, POLY_BUFF)
    # Only point is moving
    assert not amf.detect_same_2D(0, s_0, vs_00, P, V_0, POLY_BUFF)
    assert not amf.detect_same_2D(8.99, s_0, vs_00, P, V_0, POLY_BUFF)
    assert amf.detect_same_2D(9.01, s_0, vs_00, P, V_0, POLY_BUFF)  # Within buffer distance
    assert amf.detect_same_2D(10.01, s_0, vs_00, P, V_0, POLY_BUFF)
    assert amf.detect_same_2D(19.99, s_0, vs_00, P, V_0, POLY_BUFF)
    assert amf.detect_same_2D(21.00, s_0, vs_00, P, V_0, POLY_BUFF)
    assert amf.detect_same_2D(21.01, s_0, vs_00, P, V_0, POLY_BUFF)
    # Only polygon is moving
    assert not amf.detect_same_2D(0, s_0, vs_01, P, V_1, POLY_BUFF)
    assert not amf.detect_same_2D(8.99, s_0, vs_01, P, V_1, POLY_BUFF)
    assert amf.detect_same_2D(9.01, s_0, vs_01, P, V_1, POLY_BUFF)  # Within buffer distance
    assert amf.detect_same_2D(10.01, s_0, vs_01, P, V_1, POLY_BUFF)
    assert amf.detect_same_2D(19.99, s_0, vs_01, P, V_1, POLY_BUFF)
    assert amf.detect_same_2D(21.00, s_0, vs_01, P, V_1, POLY_BUFF)
    assert amf.detect_same_2D(21.01, s_0, vs_01, P, V_1, POLY_BUFF)
    # Point and polygon both moving
    assert not amf.detect_same_2D(0, s_0, vs_02, P, V_2, POLY_BUFF)
    assert not amf.detect_same_2D(8.99, s_0, vs_02, P, V_2, POLY_BUFF)
    assert amf.detect_same_2D(9.01, s_0, vs_02, P, V_2, POLY_BUFF)  # Within buffer distance
    assert amf.detect_same_2D(10.01, s_0, vs_02, P, V_2, POLY_BUFF)
    assert amf.detect_same_2D(19.99, s_0, vs_02, P, V_2, POLY_BUFF)
    assert amf.detect_same_2D(21.00, s_0, vs_02, P, V_2, POLY_BUFF)
    assert amf.detect_same_2D(21.01, s_0, vs_02, P, V_2, POLY_BUFF)
