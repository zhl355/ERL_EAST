#!/usr/bin/env python3
"""
Distance Under Q Norm Between a Point and The ICE-CREAM CONE Described in Omur's Paper

How to solve Quartic equation:
https://maa.org/sites/default/files/pdf/upload_library/22/Ford/auckly29.pdf

The algorithm above involves solving a cubic equation, which solution could be found here:
https://mathworld.wolfram.com/CubicFormula.html

Quadratic programming with ball constraint:
https://people.clas.ufl.edu/hager/files/sphere.pdf

These formulas involve evaluating the square root of a potential negative number, be careful about the imaginary part of
the solution.

---

UCSD ERL Y. Yi, v1.1

---

"""

import cvxpy as cp
import numpy as np
import numpy.linalg as npla

from ref_gvn_utils.ros_gvn_cone_dist_util import triangle_def


def point2triangle_q_distance(Q, tri, points):
    """
    Point to triangular distance evaluated in Q norm in 2D

    Interfaces:
        Input:
            Q : matrix defining the Q norm in 2D, (2,2)
            tri : triangular vertices, (3,2)
            points : collection of 2d points, (2, # of points)
        Output:
            dist : distance to triangular defined by the provided vertices: (# of points)
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
        point2line_ab = line2point_q_dist(Q, vertices_a, vertices_b, vertices_d)
        point2line_ac = line2point_q_dist(Q, vertices_a, vertices_c, vertices_d)
        point2line_bc = line2point_q_dist(Q, vertices_b, vertices_c, vertices_d)

        # point_cloud distances to single triangle
        pt2tri = np.minimum(point2line_ab, point2line_ac, point2line_bc)

        if area_abc >= 1e-2:
            dist = pt2tri * delta_index
        else:
            dist = pt2tri

        return dist


def line2point_q_dist(Q, p_1, p_2, p_tar, eps_len=1e-6):
    """
    This function calculated point to line segment distance in Q norm

    Interfaces:
        Input:
            Q : matrix defining the Q norm in 2D, (2,2)
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
        point_c = p_tar.T
    else:
        point_c = p_tar

    # Calculating Point to Line Distance
    line_length = npla.norm(point_b - point_a)  # this result in a scalar

    # do not compare float with 0
    if line_length < eps_len:
        p2l_dist = np.sqrt((Q @ (point_c - point_a) * (point_c - point_a)).sum(0))
    else:
        m = point_c - point_a
        n = point_b - point_a
        k = n.T @ Q @ m / (n.T @ Q @ n)
        k[k < 0] = 0
        k[k > 1] = 1

        p2l = m - np.matmul(n, k)
        p2l_dist = np.sqrt((Q @ p2l * p2l).sum(0))

    return p2l_dist


def generate_q_matrix(v):
    """
    This function generates the Q matrix for Q norm calculation

    Interfaces:
        Input:
            v : vector defining the semi-major axis of the ellipsoid induced by Q
        Output:
            Q : matrix defining the Q norm in 2D, (2,2)
    """
    if np.linalg.norm(v) != 0:
        Q = 9 * np.eye(2) + (1 - 9) * (v @ v.T) / np.linalg.norm(v) ** 2
    else:
        Q = np.eye(2)

    return Q


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
    area = area / 2.0

    return np.abs(area)


def solve_qubic_equation(coeff):
    """
    This function solves cubic equation given coefficients

    Interfaces:
        Input:
            coeff : coefficients of the cubic equation, (# of equations to solve, 3)
        Output:
            sol_real : real solution, (# of equations to solve, )
            full_sol : completer solution, first one always the real solution, (# of equations to solve, 3)
    """
    if np.sum(coeff[:, 0] != np.ones((coeff.shape[0], 1))) != 0:
        coeff = coeff / np.kron(np.ones((1, 4)), coeff[:, 0:1])

    a_2 = coeff[:, 1:2]
    a_1 = coeff[:, 2:3]
    a_0 = coeff[:, 3:]

    Q = (3 * a_1 - a_2 ** 2) / 9.0
    R = (9 * a_2 * a_1 - 27 * a_0 - 2 * a_2 ** 3) / 54.0

    D = Q ** 3 + R ** 2
    D_sqrt = np.emath.sqrt(D)

    idx = np.abs(np.imag(D_sqrt)) == 0

    S_cu = R + D_sqrt
    T_cu = R - D_sqrt

    S_real = np.cbrt(np.real(S_cu * idx))
    T_real = np.cbrt(np.real(T_cu * idx))

    S_cplx = (S_cu * ~idx) ** (1 / 3)
    T_cplx = (T_cu * ~idx) ** (1 / 3)

    S = S_real + S_cplx
    T = T_real + T_cplx

    sol_real = -a_2 / 3 + S + T
    sol_2 = -a_2 / 3 - (S + T) / 2 + 1 / 2 * np.sqrt(3) * 1j * (S - T)
    sol_3 = -a_2 / 3 - (S + T) / 2 - 1 / 2 * np.sqrt(3) * 1j * (S - T)

    full_sol = np.hstack((sol_real, sol_2, sol_3))

    return sol_real, full_sol


def solve_quadratic_program_ball_constraint(Q, cir_cen, cir_rad, obs):
    """
    Point to circle distance measured in Q norm, formulated as a QP

    Interfaces:
    Input:
        Q : matrix defines the Q norm, (2, 2)
        cir_cen : center of the circle, (2, 1)
        cir_rad : radius of circle, scalar
        obs : points, (# of points, 2)
    Output:
        triangle_vertices : vertices of triangular, (2,3)
    """
    p_hat = obs.T - cir_cen
    A = Q
    b = A @ p_hat
    r = cir_rad

    S, U = np.linalg.eig(A)
    flag_mr = False

    idx_inside_circle = npla.norm(p_hat, axis=0) <= r

    if S[0] > S[1]:
        lambda_1 = S[1]
        lambda_2 = S[0]
        beta_1 = U[:, 1:].T @ b
        beta_2 = U[:, 0:1].T @ b
        U_ord = np.hstack((U[:, 1:], U[:, 0:1]))
    elif S[0] < S[1]:
        lambda_1 = S[0]
        lambda_2 = S[1]
        beta_1 = U[:, 0:1].T @ b
        beta_2 = U[:, 1:].T @ b
        U_ord = U
    else:
        flag_mr = True

    if ~flag_mr:
        c = np.zeros((2, np.shape(obs)[0]))
        idx_degen_1 = np.abs(beta_1) <= 1e-4
        idx_degen_2 = (beta_2 ** 2 / (lambda_2 - lambda_1) ** 2) <= r ** 2
        idx_degen = idx_degen_1 & idx_degen_2

        if np.sum(idx_degen) != 0:
            c1_degen = r ** 2 - beta_2[idx_degen] ** 2 / (lambda_2 - lambda_1) ** 2
            c2_degen = beta_2[idx_degen] / (lambda_2 - lambda_1)
            c[:, idx_degen[0, :]] = np.vstack((c1_degen, c2_degen))

        temp_a2 = (r ** 2 * (lambda_1 + lambda_2) ** 2 * np.ones((np.sum(~idx_degen), 1))
                   + 2 * r ** 2 * lambda_1 * lambda_2 * np.ones((np.sum(~idx_degen), 1))
                   - beta_1[0:1, ~idx_degen[0, :]].T ** 2 - beta_2[0:1, ~idx_degen[0, :]].T ** 2)

        temp_a1 = (2 * r ** 2 * (lambda_1 ** 2 * lambda_2 + lambda_1 * lambda_2 ** 2) * np.ones((np.sum(~idx_degen), 1))
                   - 2 * beta_1[0:1, ~idx_degen[0, :]].T ** 2 * lambda_2
                   - 2 * beta_2[0:1, ~idx_degen[0, :]].T ** 2 * lambda_1)

        temp_a0 = (r ** 2 * lambda_1 ** 2 * lambda_2 ** 2 * np.ones((np.sum(~idx_degen), 1))
                   - beta_1[0:1, ~idx_degen[0, :]].T ** 2 * lambda_2 ** 2
                   - beta_2[0:1, ~idx_degen[0, :]].T ** 2 * lambda_1 ** 2)

        quartic_coeff = np.hstack((r ** 2 * np.ones((np.sum(~idx_degen), 1)),
                                   2 * r ** 2 * (lambda_1 + lambda_2) * np.ones((np.sum(~idx_degen), 1)),
                                   temp_a2, temp_a1, temp_a0))

        mu_candi_non_degenerated = solve_quartic_equation(quartic_coeff)

        idx_mu_real = np.abs(np.imag(mu_candi_non_degenerated)) <= 1e-5

        idx_real_check = np.sum(idx_mu_real, 1) == 0

        if np.sum(idx_real_check) != 0:
            raise Exception("NO REAL SOLUTION FOUND")

        mu_nondegenerated = np.amax(np.real(mu_candi_non_degenerated * idx_mu_real), 1)
        idx_cond_check = (mu_nondegenerated + lambda_1) <= 0

        if np.sum(idx_cond_check) != 0:
            raise Exception("NO VALID SOLUTION FOUND")

        c1_ndegen = beta_1[~idx_degen] / (lambda_1 + mu_nondegenerated.T)
        c2_ndegen = beta_2[~idx_degen] / (lambda_2 + mu_nondegenerated.T)

        c[:, ~idx_degen[0, :]] = np.vstack((c1_ndegen, c2_ndegen))

        x = U_ord @ c + cir_cen

        cost = np.sqrt(((Q @ (obs.T - x)) * (obs.T - x)).sum(0))

        cost[idx_inside_circle.flatten()] = 0
        x[:, idx_inside_circle.flatten()] = obs[idx_inside_circle.flatten(), :].T

    else:
        x = p_hat / npla.norm(p_hat, axis=0).T * cir_rad + cir_cen
        cost = np.sqrt(((Q @ (obs.T - x)) * (obs.T - x)).sum(0))

    return x, cost


def sqrt_my(x_in):
    """
    This function gives the same square root calculation result as matlab
    """
    idx_x = np.imag(x_in) < 0

    shade_x = -np.ones(np.shape(x_in)) * idx_x + np.ones(np.shape(x_in)) * ~idx_x

    res = np.emath.sqrt(x_in) * shade_x

    return res


def solve_quartic_equation(coeff):
    """
    This function solves quartic equation given coefficients

    Interfaces:
        Input:
            coeff : coefficients of the cubic equation, (# of equations to solve, 4)
        Output:
            sol : completer solution, first one always the real solution, (# of equations to solve, 4)
    """
    if np.sum(coeff[:, 0] != np.ones((coeff.shape[0], 1))) != 0:
        coeff = coeff / np.kron(np.ones((1, 5)), coeff[:, 0:1])

    a = coeff[:, 1:2]
    b = coeff[:, 2:3]
    c = coeff[:, 3:4]
    d = coeff[:, 4:]

    p = b - 3 * a ** 2 / 8
    q = c - a * b / 2 + a ** 3 / 8
    r = d - a * c / 4 + a ** 2 * b / 16 - 3 * a ** 4 / 256

    lamb, cubsol_full = solve_qubic_equation(np.hstack((np.ones(np.shape(a)), 2 * p, p ** 2 - 4 * r, - q ** 2)))

    lambda_idx = np.abs(lamb) <= 1e-2
    lamb = lamb * ~lambda_idx + cubsol_full[:, 1:2] * lambda_idx
    lambda_idx = np.abs(lamb) <= 1e-2
    lamb = lamb * ~lambda_idx + cubsol_full[:, 2:] * lambda_idx

    in_sqrt_1 = lamb - 2 * (p + lamb - q / sqrt_my(lamb))
    in_sqrt_2 = lamb - 2 * (p + lamb + q / sqrt_my(lamb))

    sol_1 = (-sqrt_my(lamb) + sqrt_my(in_sqrt_1)) / 2 - a / 4
    sol_2 = (-sqrt_my(lamb) - sqrt_my(in_sqrt_1)) / 2 - a / 4
    sol_3 = (sqrt_my(lamb) + sqrt_my(in_sqrt_2)) / 2 - a / 4
    sol_4 = (sqrt_my(lamb) - sqrt_my(in_sqrt_2)) / 2 - a / 4

    sol = np.hstack((sol_1, sol_2, sol_3, sol_4))

    return sol


def solve_quartic_cvx(Q, cir_cen, cir_rad, obs):
    """
    Debug function, solving a quartic equation using cvxpy
    """
    x_var = cp.Variable((2, 1))
    cost_var = cp.Minimize(cp.quad_form((x_var - obs), Q))
    constraints = [cp.norm(x_var - cir_cen) <= cir_rad]
    prob = cp.Problem(cost_var, constraints)
    result = prob.solve()

    return x_var.value, cost_var.value


def quadratic_program_linear_constraints_cvx(Q, pt1, pt2, obs):
    """
    Solve a qp with linear constraints using cvxpy
    """
    pt1 = pt1.reshape((2, 1))
    pt2 = pt2.reshape((2, 1))
    k_var = cp.Variable((1, 1))
    cost_var = cp.Minimize(cp.quad_form((pt1 + k_var * (pt2 - pt1) - obs), Q))
    constraints = [0 <= k_var, k_var <= 1]
    prob = cp.Problem(cost_var, constraints)
    result = prob.solve()

    return k_var.value, cost_var.value


def cvx_point2triangle_q_distance(Q, tri, points):
    """
    Calculate the point to triangular distance under Q norm using cvxpy
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

        point2line_ab = np.zeros((1, np.shape(points)[1]))
        point2line_ac = np.zeros((1, np.shape(points)[1]))
        point2line_bc = np.zeros((1, np.shape(points)[1]))

        # Point to Line Distance Calculation
        for i in range(np.shape(points)[1] - 1):
            _, point2line_ab[0, i] = quadratic_program_linear_constraints_cvx(Q, vertices_a, vertices_b,
                                                                              vertices_d[:, i:i + 1])
            _, point2line_ac[0, i] = quadratic_program_linear_constraints_cvx(Q, vertices_a, vertices_c,
                                                                              vertices_d[:, i:i + 1])
            _, point2line_bc[0, i] = quadratic_program_linear_constraints_cvx(Q, vertices_b, vertices_c,
                                                                              vertices_d[:, i:i + 1])
        _, point2line_ab[0, -1] = quadratic_program_linear_constraints_cvx(Q, vertices_a, vertices_b,
                                                                           vertices_d[:, -1:])
        _, point2line_ac[0, -1] = quadratic_program_linear_constraints_cvx(Q, vertices_a, vertices_c,
                                                                           vertices_d[:, -1:])
        _, point2line_bc[0, -1] = quadratic_program_linear_constraints_cvx(Q, vertices_b, vertices_c,
                                                                           vertices_d[:, -1:])

        # pointclouds distances to single triangle
        pt2tri = np.minimum(point2line_ab, point2line_ac, point2line_bc)

        if area_abc >= 1e-2:
            dist = pt2tri * delta_index
        else:
            dist = pt2tri

        return dist


def cvx_tri_check(v, position_current, position_goal, rad_circle, obs_point):
    """
    Verify the calculation results of closed-form point to triangular distance using the results obtained by cvxpy
    """
    # Input Dimension Check
    if np.shape(obs_point)[0] == np.shape(obs_point)[1]:
        print('Be Careful About Dimension')
    elif np.shape(obs_point)[0] != 2:
        obs_point = obs_point.T

    # Define Q
    Q = generate_q_matrix(v)

    # Calculate Triangle
    tri_vertices = triangle_def(position_current, position_goal, rad_circle)
    pt2tri = point2triangle_q_distance(Q, tri_vertices, obs_point)
    cvx_pt2tri = cvx_point2triangle_q_distance(Q, tri_vertices, obs_point)

    err = sum(npla.norm((pt2tri - np.sqrt(cvx_pt2tri)), axis=0))
    flag_tri = err <= 1e-2

    return flag_tri


def point2cone_q_distance(v, position_current, position_goal, rad_circle, obs_point):
    """
    This function calculated point to ice-cream cone distance in Q norm

    Interfaces:
        Input:
            v : vector defining the semi-major axis of the ellipsoid induced by Q
            position_current : top point, (2, 1)
            position_goal : center of circle, (2, 1)
            rad_circle : circle radius, scalar
            obs_point : points, (2, # of points)
        Output:
            pt2cone_dist : point_cloud to segments distance (n)
            Q : Q matrix used to calculate Q norm
    """
    # Input Dimension Check
    if np.shape(obs_point)[0] == np.shape(obs_point)[1]:
        print('Be Careful About Dimension')
    elif np.shape(obs_point)[0] != 2:
        obs_point = obs_point.T

    # Define Q
    Q = generate_q_matrix(v)

    # Calculate Point to Circle Distance
    if rad_circle >= 1e-2:
        _, point2circle_distance = solve_quadratic_program_ball_constraint(Q, position_goal, rad_circle, obs_point.T)
    else:
        point2circle_distance = np.sqrt((Q @ (obs_point - position_goal) * (obs_point - position_goal)).sum(0))

    # Calculate Triangle
    tri_vertices = triangle_def(position_current, position_goal, rad_circle)
    pt2tri = point2triangle_q_distance(Q, tri_vertices, obs_point)
    pt2cone_dist = np.minimum(pt2tri, point2circle_distance)

    return pt2cone_dist, Q


if __name__ == "__main__":
    """
    Debug main function, verify closed-form results using cvxpy
    """
    flag = True
    count = 1

    while flag:

        x_cvx = np.zeros((2, 20))
        cost_cvx = np.zeros((1, 20))

        v = np.random.randn(2, 1)
        rbt = np.random.randn(2, 1)
        cir_cen = rbt
        cir_rad = 0

        obs = np.random.randn(20, 2)

        Q = generate_q_matrix(v)

        for i in range(19):
            x_cvx[:, i:i + 1], cost_cvx[0, i:i + 1] = solve_quartic_cvx(Q, cir_cen, cir_rad, obs[i:i + 1, :].T)
        x_cvx[:, 19:], cost_cvx[0, 19:] = solve_quartic_cvx(Q, cir_cen, cir_rad, obs[19:, :].T)

        if cir_rad >= 1e-2:
            x_cl, cost_cl = solve_quadratic_program_ball_constraint(Q, cir_cen, cir_rad, obs)
        else:
            cost_cl = np.sqrt((Q @ (obs - cir_cen.T).T * (obs - cir_cen.T).T).sum(0))
            x_cl = np.kron(cir_cen, np.ones((1, np.shape(obs)[0])))

        cone_q_dist = point2cone_q_distance(v, rbt, cir_cen, cir_rad, obs)

        delta_x = x_cl - x_cvx
        delta_cost = cost_cl - np.sqrt(cost_cvx)

        flag_1 = np.linalg.norm(delta_cost) <= 1e-2
        flag_2 = np.linalg.norm(delta_x) <= 1e-2
        flag_3 = cvx_tri_check(v, rbt, cir_cen, cir_rad, obs)

        flag = flag_1 & flag_2 & flag_3

        print(count)

        count = count + 1
