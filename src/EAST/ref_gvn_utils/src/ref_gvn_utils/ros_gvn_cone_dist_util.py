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
import numpy.linalg as npla


def point2triangle_distance(tri, points):
    """
    Point to triangular distance calculator in 2d

    Interfaces:
        Input:
            tri: triangular vertices, (2,3)
            points: collection of 2d points, (2, # of points)
        Output:
            point_to_triangular_dist: distance to triangular defined by the provided vertices: (# of points,)
    """
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

        # distances to single triangle
        pt2tri = np.minimum(point2line_ab, point2line_ac, point2line_bc)

        if area_abc >= 1e-2:
            point_to_triangular_dist = pt2tri * delta_index
        else:
            point_to_triangular_dist = pt2tri

        return point_to_triangular_dist


def tri_area(p_1, p_2, p_var):
    """
    This function calculated the area of a triangular give three vertices

    Interfaces:
        Input:
          triangular vertices, p_1, p_2, p_var, (2)
        Output:
          area of the triangular defined by three vertices as inputs, scalar
    """
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


def line2point_dist(p_1, p_2, p_tar, eps_len=1e-6):
    """
    This function calculated point to line segment distance

    Interfaces:
        Input:
            p_1 : segment start (2, 1)
            p_2 : segment end (2, 1)
            p_tar : point_cloud (2, num_pts)
            eps_len : threshold for 2 point min distance
        Output:
            p2l_dist : point_cloud to segments distance (n,)
    """
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
        point_c = p_tar.T
    else:
        point_c = p_tar

    # Calculating Point to Line Distance
    line_length = npla.norm(point_b - point_a)  # this result in a scalar
    
    # do not compare float with 0
    if line_length < eps_len:
        p2l_dist = npla.norm(point_c - point_a, axis=0) 
    else:
        m = point_c - point_a
        n = point_b - point_a
        k = np.matmul(n.T, m) / np.sum(n * n)
        k[k < 0] = 0
        k[k > 1] = 1

        # vector difference
        p2l = m - np.matmul(n, k)
        # distance calculation
        p2l_dist = npla.norm(p2l, axis=0)

    return p2l_dist


def triangle_def(position_current, position_goal, rad_circle):
    """
    This function finds out the three vertices of a triangular, given one point p1, serve as one of its vertices,
    and a circle centered at another point p2, with radius r, then the triangular is defined by the three vertices
    p1, v1, v2 with (p1-v1) perp (v1-p2) and (p1-v2) perp (v2-p2).

    Interfaces:
        Input:
            position_current : top point, (2, 1)
            position_goal : center of circle, (2, 1)
            rad_circle : circle radius, scalar
        Output:
            triangle_vertices : vertices of triangular, (2,3)
    """
    # Input Dimension Check
    if np.shape(position_current) != (2, 1):
        point_a = np.array([[position_current[0]], [position_current[1]]])
    else:
        point_a = position_current

    if np.shape(position_goal) != (2, 1):
        point_b = np.array([[position_goal[0]], [position_goal[1]]])
    else:
        point_b = position_goal

    if rad_circle is None:
        length_bc = 0.0
    else:
        length_bc = float(rad_circle)

    line_ab = point_b - point_a

    length_ab = npla.norm(line_ab)

    length_ac_square = length_ab ** 2 - length_bc ** 2

    if length_ab <= 1e-2:
        # point case
        triangle_vertices = np.column_stack((point_b, point_b, point_b))
    else:
        if length_ac_square <= 1e-5:
            # line case
            triangle_vertices = np.column_stack((point_a, point_b, point_b))
        else:
            length_ac = np.sqrt(length_ac_square)
            # point m is defined as the intersection between line a,b and line c1,c2
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
    """
    This function calculated point to ice-cream cone distance

    Interfaces:
        Input:
            position_current : top point, (2, 1)
            position_goal : center of circle, (2, 1)
            rad_circle : circle radius, scalar
            obs_point : points, (2, # of points)
        Output:
            p2l_dist : point_cloud to segments distance (n,)
    """
    # Input Dimension Check
    if np.shape(obs_point)[0] == np.shape(obs_point)[1]:
        print('Only two points being queried, be Careful About Dimension')
    elif np.shape(obs_point)[0] != 2:
        obs_point = obs_point.T
    
    # check if inside circle
    x = position_goal - obs_point  # 2 * N
    point2circle_distance = np.sqrt(np.sum(x * x, axis=0)) - rad_circle

    # point to circle distance
    point2circle_distance = np.maximum(point2circle_distance, 0)

    # calculate point to triangle distance
    tri_vertices = triangle_def(position_current, position_goal, rad_circle)
    pt2tri = point2triangle_distance(tri_vertices, obs_point)

    # take minimum and get point to cone distance
    pt2cone_dist = np.minimum(pt2tri, point2circle_distance)

    return pt2cone_dist


# ------------------- Debug Lines ----------------------
if __name__ == "__main__":

    obs = np.hstack((np.array([[5], [0]]), np.array([[1], [0.1]]), np.array([[5.5], [0]]), np.array([[0], [-0.5]])))
    
    # Verified With AUTOCAD
    point2cone_distance(np.array([[3.4], [2.2]]), np.array([[8.3], [2.36]]), np.array([2.9]), obs)
    
    dist = point2cone_distance(np.array([[0], [0]]), np.array([[4], [0]]), np.array([5]), obs)
    print('Dist = ', dist)
