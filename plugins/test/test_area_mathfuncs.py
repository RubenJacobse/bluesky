import pytest
import pdb

import numpy as np

from .. import area_mathfuncs as amf

FLT_DIFF = 0.01
POLY_BUFF = 0.1

def assert_flt(arg1, arg2):
    """ Assert that two floating point numbers differ less than FLT_DIFF """

    assert abs(arg1 - arg2) < FLT_DIFF

def test_quad():
    """ Test univariate quadratic function """

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
    pass

def test_quad_min_box_le_D():
    pass

def test_near_edge():
    # Test using triangle (0,0), (0,1), (1,0)
    p_0 = np.array([0, 0])
    p_1 = np.array([0, 1])
    p_2 = np.array([1, 0])

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

    #################################################################

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
