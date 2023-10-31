#!/usr/bin/env python3
"""
Backend of active reference governor control law. Formulate CBFs.

Author: Zhuolin Niu @ ERL

"""
import cvxpy as cp
import numpy as np

import rospy


class RefGvnActiveGoal:
    """
    Optimization module used for
        1) computed CBF values of surrounding moving obstacles
        2) formulated QCQP on constraints from CBFs and deltaE
        3) solved QCQP for optimized local projected goal (lpg_star)
    """

    def __init__(self, mo_list, gamma, kg, r_robot=0.33):
        """
        INPUT
        mo_list          list of moving obstacle information, including:
            p_mo        2d position [px, py] (m)
            v_mo        linear velocity [vx, vy] (m/s)
            r_mo        moving obstacle set radius
        gamma            CBF class-K function parameter
        kg               governor control gain
        r_robot          robot circumscribed radius (default 0.33m for Jackal)
        """

        self.mo_n = len(mo_list)  # number of moving obstacles
        self.Ao = np.repeat(np.array([np.eye(2)]), self.mo_n, axis=0)  # CBF normalization matrix

        self.r_mo = np.zeros((self.mo_n, 1, 1))
        self.p_mo = np.zeros((self.mo_n, 2, 1))
        self.v_mo = np.zeros((self.mo_n, 2, 1))

        self.gamma = gamma
        self.kg = kg
        self.r_robot = r_robot

        self.db_idx = []  # index list of driving backwards obstacles
        self.ho_log = np.empty(1) # safety value container

        self.update_movobs_info(mo_list)
        self.setup_cp_problem()
        self.qp_infeasible = False  # QP status flag
        pass

    def update_movobs_info(self, mo_list):
        """
        load current moving obstacle information
        """
        self.r_mo[:, :, 0] = mo_list[:, 4:]
        self.p_mo[:, :, 0] = mo_list[:, 0:2]
        self.v_mo[:, :, 0] = mo_list[:, 2:4]

        self.R_mo = self.r_mo + self.r_robot
        self.Ao[:, 0:1, 0:1] = (1 / self.R_mo ** 2)
        self.Ao[:, 1:, 1:] = (1 / self.R_mo ** 2)
        return

    def form_cbf(self, x, g, ho_safe_limit=-0.2):
        """
        INPUT
        x               4d robot state (2d position, 2d velocity) [zx, zy, wx, wy]
        g               2d governor state (2d position) [gx, gy]
        ho_safe_limit   lowest CBF value to determine safe status as TRUE

        CBF equation:
            ho = (g-z)^T Ao (g-z) - (g-p)^T Ao (g-p) - 1 -2||g-p||/R_mo
            ho_dot = (partial_h/partial_g)*ug+(partial_h/partial_z)*w+(partial_h/partial_p)*v
        """
        z, w = x[0:2], x[2:]

        p = self.p_mo
        v = self.v_mo

        d_gz = np.linalg.norm(g - z) # ||g-z||
        R_mo = self.R_mo
        gamma = self.gamma

        # augmented (higher dimension) arrays for matrix computation
        g_high_dim = np.repeat(np.array([np.array([g]).T]), self.mo_n, axis=0)
        z_high_dim = np.repeat(np.array([np.array([z]).T]), self.mo_n, axis=0)
        w_high_dim = np.repeat(np.array([np.array([w]).T]), self.mo_n, axis=0)

        # compute ho_dot values for all moving obstacles
        Ao = self.Ao
        if d_gz < 1e-3:
            bo = 0.0
            delta_gz = 0.0
        else:
            bo = 1.0 / (R_mo * d_gz)
            delta_gz = (2.0 * d_gz) / R_mo

        partial_d_partial_g = bo * (g_high_dim - z_high_dim)
        partial_d_partial_z = -partial_d_partial_g

        partial_ho_partial_g = 2 * Ao @ (g_high_dim - p) + 2 * Ao @ (z_high_dim - g_high_dim) - partial_d_partial_g
        partial_ho_partial_z = 2 * Ao @ (g_high_dim - z_high_dim) - partial_d_partial_z
        partial_ho_partial_p = -2 * Ao @ (g_high_dim - p)

        d_gp_sq = np.transpose(g_high_dim - p, (0, 2, 1)) @ (g_high_dim - p) # ||g-p||^2
        d_gp = np.sqrt(d_gp_sq)

        # compute ho values to select the least safe moving obstacle
        ho = (np.transpose(g_high_dim - p, (0, 2, 1)) @ Ao @ (g_high_dim - p)
              - np.transpose(g_high_dim - z_high_dim, (0, 2, 1)) @ Ao @ (g_high_dim - z_high_dim) - 1 - delta_gz)
        minh_ind = np.argmin(ho[:, 0, 0])

        # compute h_dot of the least safe moving obstacle
        # ho_dot = ho_dot_g*ug+ho_dot_f
        ho_dot_g = partial_ho_partial_g[minh_ind].T
        ho_dot_f = partial_ho_partial_z[minh_ind].T @ w_high_dim[minh_ind] + partial_ho_partial_p[minh_ind].T @ v[minh_ind]
        self.ho_dot_f = ho_dot_f[0, 0]
        self.ho_dot_g = ho_dot_g[0]

        # get CBF value from the least safe moving obstacle
        min_ho = ho[minh_ind, 0, 0]

        # compute classK
        self.classk_func_g = gamma * (min_ho ** 2)

        # get safety status
        g_safe = min_ho >= ho_safe_limit

        # store CBF value
        self.min_ho = min_ho
        self.ho_log = np.hstack((self.ho_log, min_ho))

        # update states and geometry distances of the system
        self.g = g
        self.z2d = z
        self.p = p
        self.r_gz = d_gz + self.r_robot # radius of robot-governor set
        self.d_gp = d_gp

        return g_safe

    # noinspection PyAttributeOutsideInit
    def setup_cp_problem(self):
        """
        initially setup QCQP problem in CVXPY

        QCQP formulation:
            min ||lpg2d_star - lpg2d||^2
            s.t. ho_dot + classK >=0, ||lpg2d_star-g||^2<=||lpg2d-g||^2
        
        lpg2d - 2d local projected goal
        """

        x = cp.Variable(2)
        P_mat = np.eye(2)
        P = cp.Parameter((2, 2), value=P_mat)
        q = cp.Parameter(2)
        l = cp.Parameter()
        A = cp.Parameter((2))
        m = cp.Parameter()

        objective = cp.Minimize(cp.norm(x - P @ q))
        constraints = [l <= A @ x, cp.norm(x) <= m]
        prob = cp.Problem(objective, constraints)

        self.x = x
        self.q = q
        self.l = l
        self.A = A
        self.m = m
        self.prob = prob

        return prob

    def update_cp_problem(self, lpg2d):
        """
        update parameters in CVXPY problem
        
        INPUT
        lpg2d       nominal 2d local projected goal computed from deltaE
        """
        gvn2d = self.g[0:2]  # 2d governor position
        ug2d = -1.0 * self.kg * (gvn2d - lpg2d)  # 2d governor control input

        # update CVXPY parameters 
        self.q.value = ug2d
        self.l.value = -self.classk_func_g - self.ho_dot_f
        self.A.value = self.ho_dot_g
        self.m.value = np.linalg.norm(ug2d)

        # solve QCQP
        self.prob.solve()
        ug2d_star = self.x.value

        # for infeasible problems, raise warning and qp_infeasible flag
        if ug2d_star is None:
            self.qp_infeasible = True
            rospy.logwarn_throttle(0.5, "Moving obstacle avoidance Failed! Infeasible conditions")
            lpg2d_star = gvn2d + ug2d / self.kg

        # output valid solution
        else:
            self.qp_infeasible = False
            lpg2d_star = gvn2d + ug2d_star / self.kg
        return lpg2d_star
