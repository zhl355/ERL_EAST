#!/usr/bin/env python3
"""
Euclidean Distance Between a Point and The ICE-CREAM CONE Described in Omur's Paper

---

UCSD ERL Y. Yi, v1.0

---

Consisting an Algorithm Calculating Point to Triangle Distance, with The Triangle Define by THREE VERTICES,
This Algorithm also Supports Point to Line Distance,  Give TWO IDENTICAL POINTS When Specifying the Triangle Will
Resulted in Calculating Point to Point Distance

When the Radius of THE CIRCLE is INCORRECTLY Defined, The Algorithm Will Return POINT TO LINE Distance,
with THE LINE Defined by CURRENT POSITION AND GOAL POINT
"""

import numpy as np


def point2triangle_distance(tri, points):

    # Input Check
    size = np.shape(tri)
    if size == (3, 2) or size == (2, 3):
        if size == (3, 2):
            vertices_a = tri[0, :]
            vertices_b = tri[1, :]
            vertices_c = tri[2, :]
        else:
            vertices_a = tri[:, 0]
            vertices_b = tri[:, 1]
            vertices_c = tri[:, 2]
        if np.shape(points)[0] != 2:
            vertices_d = points.T
        else:
            vertices_d = points

        # Interior Check via Area Method
        area_abc = tri_area(vertices_a, vertices_b, np.array([[vertices_c[0]], [vertices_c[1]]]))
        area_abd = tri_area(vertices_a, vertices_b, vertices_d)
        area_adc = tri_area(vertices_a, vertices_c, vertices_d)
        area_dbc = tri_area(vertices_b, vertices_c, vertices_d)

        delta_area = abs(area_abc - (area_abd + area_adc + area_dbc))
        delta_index = delta_area > 1e-5

        # Point to Line Distance Calculation
        point2line_ab = line2point_dist(vertices_a, vertices_b, vertices_d)
        point2line_ac = line2point_dist(vertices_a, vertices_c, vertices_d)
        point2line_bc = line2point_dist(vertices_b, vertices_c, vertices_d)

        pt2tri = np.minimum(point2line_ab, point2line_ac, point2line_bc)

        if area_abc >= 1e-2:
            dist = pt2tri * delta_index
        else:
            dist = pt2tri

        return dist


def tri_area(p_1, p_2, p_var):

    point_a = p_1
    point_b = p_2
    point_c = p_var

    # Calculating Triangle Area
    temp_1 = (point_b[1] + point_a[1]) * (point_a[0] - point_b[0])
    temp_2 = (point_a[1] + point_c[1, :]) * (point_c[0, :] - point_a[0])
    temp_3 = (point_b[1] + point_c[1, :]) * (point_c[0, :] - point_b[0])
    area = temp_1 + temp_2 - temp_3
    area = area/2

    return np.abs(area)


def line2point_dist(p_1, p_2, p_tar):

    # Input Dimension Check
    if np.shape(p_1) != (2, 1):
        point_a = np.array([[p_1[0]], [p_1[1]]])
    else:
        point_a = p_1

    if np.shape(p_2) != (2, 1):
        point_b = np.array([[p_2[0]], [p_2[1]]])
    else:
        point_b = p_2

    if np.shape(p_tar)[0] != 2:
        print('Check P_tar Dimension')
    else:
        point_c = p_tar

    # Calculating Point to Line Distance
    line_length = (point_b - point_a).T @ (point_b - point_a)
    if line_length == 0:
        p2l_dist = np.sqrt(np.diag((point_a - point_c).T @ (point_a - point_c)))
    else:
        m = point_c - point_a
        n = point_b - point_a
        k = (n.T @ m) / (n.T @ n)
        k = np.minimum(k, 1)
        k = np.maximum(k, 0)

        p2l_dist = m - n @ k
        p2l_dist = np.sqrt(np.diag( p2l_dist.T @ p2l_dist ))

    return p2l_dist


def triangle_def(position_current, position_goal, rad_circle):
    # Input Dimension Check
    if np.shape(position_current) != (2, 1):
        print('Check Current Position Dimension')
    else:
        point_a = position_current

    if np.shape(position_goal) != (2, 1):
        print('Check Goal Position Dimension')
    else:
        point_b = position_goal

    if rad_circle is None:
        print('Check Circle Radius Existence')
    else:
        length_bc = float(rad_circle)



    # TODO YYZ, for all np.sqrt, make sure operands are non-negative!
    line_ab = point_b - point_a
    length_ab = np.sqrt(line_ab.T @ line_ab)
    length_ac_square = line_ab.T @ line_ab - length_bc ** 2

    if length_ab <= 1e-2:
        # print('Reach Goal')
        # triangle_vertices = np.array([(point_b[0], point_b[1]), (point_b[0], point_b[1]), (point_b[0], point_b[1])])
        triangle_vertices = np.column_stack((point_b, point_b, point_b))
    else:
        if length_ac_square <= 1e-5:
            # print('Check Circle Radius Magnitude')
            # triangle_vertices = np.array([(point_a[0], point_a[1]), (point_b[0], point_b[1]), (point_b[0], point_b[1])])
            triangle_vertices = np.column_stack((point_a, point_b, point_b))
        else:
            length_ac = np.sqrt(length_ac_square)
            length_am = (length_ac ** 2 - length_bc ** 2 + length_ab ** 2) / (2 * length_ab)

            # length_cm = np.sqrt(length_ac ** 2 - length_am ** 2)
            tmp = length_ac ** 2 - length_am ** 2
            length_cm = np.sqrt(max(tmp, 0.0))

            point_m = point_a + (length_am / length_ab) * (point_b - point_a)
            point_c1_x = point_m[0] + (length_cm / length_ab) * (point_b[1] - point_a[1])
            point_c2_x = point_m[0] - (length_cm / length_ab) * (point_b[1] - point_a[1])
            point_c1_y = point_m[1] - (length_cm / length_ab) * (point_b[0] - point_a[0])
            point_c2_y = point_m[1] + (length_cm / length_ab) * (point_b[0] - point_a[0])
            point_c1 = np.row_stack((point_c1_x, point_c1_y))
            point_c2 = np.row_stack((point_c2_x, point_c2_y))
            triangle_vertices = np.column_stack((point_a, point_c1, point_c2))

    return triangle_vertices


def point2cone_distance(position_current, position_goal, rad_circle, obs_point):

    # Input Dimension Check
    if np.shape(obs_point)[0] == np.shape(obs_point)[1]:
        print('Be Careful About Dimension')
    elif np.shape(obs_point)[0] != 2:
        obs_point = obs_point.T

    # Calculate Point to Circle Distance
    point2circle_distance = np.sqrt(np.diag((position_goal - obs_point).T @ (position_goal - obs_point))) - rad_circle
    point2circle_distance = np.maximum(point2circle_distance, 0)

    # Calculate Triangle
    tri_vertices = triangle_def(position_current, position_goal, rad_circle)
    pt2tri = point2triangle_distance(tri_vertices, obs_point)

    pt2cone_dist = np.minimum(pt2tri, point2circle_distance)

    # print('Dist = ', dist)
    # print('Triangle = ', tri_vertices)
    return pt2cone_dist

# ------------------- Debug Lines ----------------------

# obs = np.hstack((np.array([[5], [0]]), np.array([[1], [0.1]]), np.array([[5.5], [0]]), np.array([[0], [-0.5]])))

# Verified With AUTOCAD
# point2cone_distance(np.array([[3.4], [2.2]]), np.array([[8.3], [2.36]]), np.array([2.9]), obs)

# dist = point2cone_distance(np.array([[0], [0]]), np.array([[4], [0]]), np.array([5]), obs)
# print('Dist = ', dist)
