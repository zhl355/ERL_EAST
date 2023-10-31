#!/usr/bin/env python3
"""
Safe path-following controller based on reference governor

Interfaces:
    Input:
      nonlinear robot states (z), SE(2)
      strictly feasible path (r), 2d
      dist from governor to observed obstacle space, i.e., d_I/Q(g,O)
    Output:
      governor state (g) SE(2)

      # for visualization purpose
        local projected goal (lpg2d), 2d
        local safe zone (LS) ball/ellipse characteristics

"""

import numpy as np
import numpy.linalg as npla

import rospy
from ref_gvn_core.geometry import ball_and_path
from ref_gvn_core.ref_gvn_se2_utils import rk4_update, integrator
from ref_gvn_utils.geometry_utils import wrap_angle_pmp
from sensor_msgs.msg import Joy


class RefGvnSE2Core:
    """
    Reference Governor Core Module. Responsible for:
        1) dynamic safety margin (dsm) update
        2) local projected goal (lpg) update
        3) governor state update (g)
    Update governor sates:
        NORMAL
        IDLE
        GOAL_REACHED
        INIT_FAILED
        WARN
        HALT
    """

    # Running status table, higher number better status
    IDLE = 0
    NORMAL = 50
    GOAL_REACHED = 100
    INIT_FAILED = -10
    WARN = -50
    HALT = -100

    def __init__(
            self,
            z0,  # init robot state, in SE(2), represented in (3,)
            dist,  # init distance from governor to obstacle space at beginning
            nav_path0,  # init nav path2d, represented in (num waypoints, 2)
            goal2d,  # goal 2d location
            kg=1.0,  # governor controller gain, scalar
            dt=0.02,  # discretization time / integration step used in Runge Kutta Integration, scalar
            dsm_fatal_lb=0.005,  # dsm fatal lower bound, scalar
            gvn2goal_th=0.01,  # governor to goal positional threshold in meter, scalar
            path_dim=2,  # path waypoint dimension, (2,)
    ):
        """
        Reference governor object init. Load parameters.
        """

        # -------------------- system information init --------------------------
        # robot state
        self.z = z0

        # governor state init
        self.g = z0

        # local projected goal init
        self.gbar = z0

        # desired heading initialization
        self.theta_dsr = z0[2]

        # euclidean dist from current governor position to final goal (gvn2d, goal2d)
        self.g2G = 0.0

        # dimensional of path
        self._nPath = path_dim

        # dsm fatal lower bound
        self.deltaE = self._calculate_dsm(dist)

        # final navigation goal
        self._goal2d = goal2d

        # time
        self.tidx = 0
        self.curr_time = self.tidx * dt
        self.rate = int(1.0 / dt)
        self._dt = dt

        # time step
        self.kg = kg

        # governor to goal positional threshold in meter
        self._gvn2G_th = gvn2goal_th

        # dsm fatal lower bound, scalar
        self._deltaE_lb = dsm_fatal_lb

        # other parameters
        self.lpg2d_radius = 0
        self.lpg_status = 0

        # ---------------------- cached large containers  ------------------------
        self.path_raw = nav_path0
        # Store previous lpg2d and gvn2d inferred theta
        self.theta_dsr_previous = 0

        # ---------------------------- local flag -------------------------------
        self.gvn_goal_reached_flag = False
        self.plan_fail = True
        self.human_override = False

        # ------------------- perform init checklist ---------------------------
        # Init status to normal for a full check
        self.gvn_status = RefGvnSE2Core.NORMAL
        # Perform status check
        self.ref_gvn_core_status_check()

        # strict safety metric at init
        if self.gvn_status >= RefGvnSE2Core.IDLE:
            rospy.logwarn("[RefGvnSE2Core] ref_gvn init safely")
        else:
            err_msg = "Init robot state z0 = %s is unsafe" % z0
            err_msg += "| deltaE0 = %.2f" % self.deltaE
            self.gvn_status = RefGvnSE2Core.INIT_FAILED
            rospy.logerr_throttle(0.5, err_msg)

        # for local projected goal
        self.lpg2d = self.get_lpg2d_ball(nav_path0)

        # for ball, center and radius, # for ellipse, major and minor semi axis + orientation
        self.ls_params = {'center': self.g[0:self._nPath], 'radius': self.lpg2d_radius}
        self.lf_params = {'center': self.g[0:self._nPath], 'radius': dist}

        # ------------------- ROS Subscribers  --------------------
        self.human_sub = rospy.Subscriber("/bluetooth_teleop/joy", Joy, self.human_override_callback, queue_size=1)

    def _get_theta_from_two_endpts(self, ptA, ptB): # noqa
        """
        Get theta from two points A-->B by taking atan2, to get desired angle
        from horizontal line measured counter_clockwise
        Input:
            @ptA: starting 2d point
            @ptB: ending 2d point
        Output:
            @theta: orientation inferred from ptA--> ptB
        """
        s1, s2 = ptA
        e1, e2 = ptB
        # If norm(ptA - ptB) >= 0.1m, then calculate theta normally,
        # otherwise, use the last theta when norm(ptA - ptB) >= 0.1m
        if npla.norm(ptA - ptB) >= 0.1:
            theta = np.arctan2(e2 - s2, e1 - s1)
            self.theta_dsr_previous = theta
        else:
            theta = self.theta_dsr_previous
        theta = wrap_angle_pmp(theta)
        return theta

    @staticmethod
    def _calculate_dsm(dist):
        """
        Calculate deltaE based on safety distance metric

        Input:
            dist: raw distance to obstacle, scalar
        Output:
            deltaE: discounted safety metric, scalar
        """

        # you could put any class K function here, e.g., piecewise linear function
        deltaE = dist
        return deltaE

    def human_override_callback(self, msg):
        """
        Human override status set function, when the L1 button on the joystick is pressed, it will set
        the human_override flag to true.
        If the human_override flag is true, ref_gvn_status will be set to IDLE, but it will not send an
        emergency stop signal.
        """
        if msg.buttons[4] == 1:
            self.human_override = True
        else:
            self.human_override = False

    def update_gvn_state(self):
        """
        Update governor system. Currently, it is a first_order linear system
        gdot = -k_g (g - gbar)
        Input:
            @gbar: desired state of governor, (3)
        Output/Update:
            @g: Governor state g update, (3)
        """
        # governor dynamics update
        g_dot_xy = -1.0 * self.kg * (self.g[0:2] - self.gbar[0:2])
        g_dot_theta = -1.0 * self.kg * wrap_angle_pmp(self.g[2] - self.gbar[2])
        g_dot = np.array([g_dot_xy[0], g_dot_xy[1], g_dot_theta])

        # limit local projected goal based on error in heading angle between current robot heading and governor heading
        err_angle = wrap_angle_pmp(self.g[2] - self.z[2])
        err_angle_norm = np.abs(err_angle)

        if err_angle_norm >= np.pi / 4.0:
            gvn_projection_range_limit = 0.1
        else:
            # hard corded projection limit, change this according to environment, indoor 1.0, 
            # simulation or outdoor 4.0
            gvn_projection_range_limit = 4.0

        # update governor state, this hack is added to reduce control inputs
        if npla.norm(self.z[0:2] - self.g[0:2]) <= gvn_projection_range_limit:
            g_new = rk4_update(self._dt, integrator, self.curr_time, self.g, g_dot)
        else:
            g_new = self.g
            rospy.logdebug_throttle(0.5, "[ref_gvn_core | gvn too far > gvn_projection_range_limit]")

        self.g = g_new

    def check_gvn_goal_reached(self):
        """
        Check whether goal is reached for governor
        Output/Update:
            @gvn_goal_reached_flag: governor goal reach flag, bool
            @gvn_status: reference governor running status
        """
        gvn2d = self.g[0:self._nPath]
        # Euclidean distance from governor position to goal position
        self.g2G = npla.norm(gvn2d - self._goal2d)
        if self.g2G < self._gvn2G_th and not self.gvn_goal_reached_flag:
            print("\n------------ Governor Reached Goal Region! -----------\n")
            self.gvn_goal_reached_flag = True
            self.gvn_status = RefGvnSE2Core.GOAL_REACHED

    def path_validation(self):
        """
        Check if path is valid, path is deemed invalid if less than 2 waypoints are given, and if there are
        exactly two waypoints, then check if them are identical, if they are identical, then the plan_fail
        status will be triggered.
        """
        self.plan_fail = True
        # Test if path dimension valid
        if len(self.path_raw) < 2:
            rospy.logerr_throttle(1.0, "path len is too short, >=2 waypoints is required!")
        # Test if planning failed
        elif np.size(self.path_raw) == 4:
            # if start loc equals to goal loc (hacked in A* planning for fail case. But this is not correct.)
            # sometimes A* success, with same start and goal. But has waypoint in path.
            if np.all(self.path_raw[0, :] == self.path_raw[1, :]):
                self.plan_fail = True
            else:
                self.plan_fail = False
        else:
            self.plan_fail = False

    def ref_gvn_core_status_check(self):
        r"""
        Run status change for reference governor, no status change outside this function. Run status table:

        ---
        input status -- condition --> target status
        ---

        GOAL_REACHED -- any condition --> GOAL_REACHED

                  | -- safety violation --> HALT
                 | -- safety critical --> WARN
        NORMAL -- -- plan fail --> IDLE
                 \ -- goal reach --> GOAL_REACHED
                  \ -- otherwise --> NORMAL

        IDLE -- any condition --> IDLE

               | -- safety no longer critical --> NORMAL
        WARN -- -- safety violation --> HALT
               \ -- otherwise --> WARN
        """
        # human override, status chane to IDLE
        if self.human_override:
            self.gvn_status = RefGvnSE2Core.IDLE
            rospy.logwarn_throttle(0.5, "[ref_gvn_core] ---- HUMAN OVERRIDE | DRIVE SAFE ----")
            return

        # GOAL_REACHED Case, no action take
        if self.gvn_status == RefGvnSE2Core.GOAL_REACHED:
            self.gvn_status = RefGvnSE2Core.GOAL_REACHED
            rospy.logdebug_throttle(0.5, "[ref_gvn_core] ------------ GOAL REACHED ------------")

        # NORMAL Case, run status table
        elif self.gvn_status == RefGvnSE2Core.NORMAL:
            # HALT Case, violation of safety metric
            if self.deltaE <= 0:
                self.gvn_status = RefGvnSE2Core.HALT
                rospy.logwarn("[deltaE] = [%.2f]" % self.deltaE)
                rospy.logwarn_throttle(0.5, "[ref_gvn_core] NORMAL ==========> HALT")
                rospy.logerr_throttle(0.5, "Robot is in danger deltaE <= 0")

            # WARN Case, safety metric in danger
            if self.deltaE <= self._deltaE_lb:
                self.gvn_status = RefGvnSE2Core.WARN
                rospy.logwarn("Warning deltaE <= %.2f, set gvn_status = WARN" % self._deltaE_lb)
                rospy.logwarn("[deltaE] = [%.2f]" % self.deltaE)
                rospy.logwarn_throttle(0.5, "[ref_gvn_core] NORMAL ==========> WARN")
                return

            # If no safety violation, check path to set flag: self.plan_fail
            self.path_validation()

            # IDLE Case
            if self.plan_fail:
                self.gvn_status = RefGvnSE2Core.IDLE
                rospy.logerr("Warning Planning Failed, set gvn_status = IDLE")
                rospy.logerr_throttle(0.5, "[ref_gvn_core] NORMAL ==========> IDLE")
                return

            # If planning not fail, check if reach goal to set flag: self.gvn_goal_reached_flag
            self.check_gvn_goal_reached()

            # GOAL REACHED Case
            if self.gvn_goal_reached_flag:
                # Regular Case -> Gvn Goal Reached
                self.gvn_status = RefGvnSE2Core.GOAL_REACHED
                rospy.loginfo("Governor Reach Goal")
                rospy.loginfo_throttle(0.5, "[ref_gvn_core] NORMAL ==========> GOAL_REACHED")
            # NORMAL Case
            else:
                # Regular Case && Gvn Goal not Reached
                self.gvn_status = RefGvnSE2Core.NORMAL
                rospy.logdebug_throttle(0.5, "[ref_gvn_core] ------------ NORMAL ------------")

        # IDLE Case, no action take, keep staying at IDLE
        elif self.gvn_status == RefGvnSE2Core.IDLE:
            self.gvn_status = RefGvnSE2Core.IDLE
            rospy.logwarn_throttle(0.5, "[ref_gvn_core] ------------ IDLE ------------")

        # WARN Case, no action take
        elif self.gvn_status == RefGvnSE2Core.WARN:
            # WARN Case --> NORMAL
            if self.deltaE > self._deltaE_lb:
                self.gvn_status = RefGvnSE2Core.NORMAL
                rospy.logdebug("Now deltaE (%.2f) > delta_lb (%.2f), set gvn_status = NORMAL" % (self.deltaE, self._deltaE_lb))
                rospy.logwarn_throttle(0.5, "[ref_gvn_core] WARN ==========> NORMAL")
                return
            elif self.deltaE <= 0:
                # WARN Case --> HALT
                self.gvn_status = RefGvnSE2Core.HALT
                rospy.logwarn("Warning deltaE (%.2f) <= delta_lb (%.2f), set gvn_status = HALT" % (self.deltaE, self._deltaE_lb))
                rospy.logwarn_throttle(0.5, "[ref_gvn_core] WARN >>>>>>>>>>> HALT")
                return

            rospy.logwarn_throttle(0.5, "[ref_gvn_core] ------------ WARN ------------")

        # HALT Case, log error
        else:
            rospy.logerr_throttle(0.5, 'Safety Violation.')

    def get_lpg2d_ball(self, nav_path2d):
        """
        Compute local projected goal gbar in 2d i.e., furthest intersection
        between local safe zone L and reference path r.
        Input:
            nav_path2d: external global strictly feasible path in 2d (xx, 2)
        Output:
            lpg2d: local projected goal gbar in 2d
        """
        gvn2d = self.g[0:self._nPath]

        # the tiny number (1e-6) is for ball_and_path function to prevent zero radius ball
        self.lpg2d_radius = max(self.deltaE - 0.015, 1e-6)
        if self.lpg2d_radius > 1e-3:
            status, x_star, _ = ball_and_path(x=gvn2d, r=self.lpg2d_radius, path=nav_path2d)
            self.lpg_status = status
            # 0 means successful
            if status == 0:
                lpg2d = x_star
            else:
                # if failed, bring lpg to gvn location
                lpg2d = gvn2d
                rospy.logwarn_throttle(0.5, "[ref_gvn_node | get_lpg2d_ball] Failed to find intersection")
                pass
        else:
            lpg2d = gvn2d

        return lpg2d

    def get_lpgSE2(self, nav_path2d):
        """
        Compute local projected goal, i.e., gbar in SE(2)
        steps:
            a) find positional part using lpg2d, find_lpg2d_ball
            b) find desired theta_dsr by (gvn2d, lpg2d) two endpoints
            c) lpgSE2 = (lpg2d, theta_dsr)
        Input:
            @nav_path2d: external global strictly feasible path in 2d (xx, 2)
        Output:
            @lpgSE2: local projected goal i.e., gbar in SE2
        """
        lpg2d = self.get_lpg2d_ball(nav_path2d)
        gvn2d = self.g[0:self._nPath]

        self.lpg2d = lpg2d
        self.theta_dsr = self._get_theta_from_two_endpts(gvn2d, lpg2d)
        lpgSE2 = np.hstack((lpg2d, self.theta_dsr))
        return lpgSE2

    def update(self, dist, z, r, debug=True):
        """
        Update reference governor state g and desired governor goal gbar based on reference governor running status:
        NORMAL: update normally
        IDLE: governor state g = desired governor goal gbar = robot state z,
        GOAL_REACHED: governor state g = [[final_goal, (2,)],[inferred final heading theta_dsr, scalar]]
        WARN: desired governor state gbar = governor state g
        HALT: no action
        Input:
            @dist: distance metric for internal safety metric computation
            @z: robot states, (3)
            @r: reference path, (num_pts,2)
            debug: debug message print switch, bool
        Output/Update:
            @gvn_status: reference governor running status
            @self.gbar: local projected goal, (3)
        """
        # load parameters from input args
        self.deltaE = self._calculate_dsm(dist)
        self.path_raw = r
        self.z = z

        # perform status check
        self.ref_gvn_core_status_check()

        # operations based on gvn_status, rough sort ---------------------------
        # status better than IDLE
        if self.gvn_status >= RefGvnSE2Core.IDLE:
            # IDLE Case, set gbar to current g
            if self.gvn_status == RefGvnSE2Core.IDLE:
                self.g = z
                self.gbar = self.g
            # NORMAL Case, update governor state
            elif self.gvn_status == RefGvnSE2Core.NORMAL:
                self.gbar = self.get_lpgSE2(r)
                self.update_gvn_state()
                self.tidx += 1
                self.curr_time = self._dt * self.tidx
            # GOAL_REACH Case, set gbar to final goal
            else:
                self.gbar = np.hstack((self._goal2d, self.theta_dsr))
        else:
            if self.gvn_status == RefGvnSE2Core.WARN:
                self.gbar = self.g
            else:
                rospy.logerr_throttle(0.5, 'System in danger during update.')

        # update local safe zone property
        self.ls_params['center'] = self.g[0:self._nPath]
        self.ls_params['radius'] = self.lpg2d_radius

        # update local free space property
        self.lf_params['center'] = self.g[0:self._nPath]
        self.lf_params['radius'] = dist

        if debug and self.gvn_status >= RefGvnSE2Core.IDLE:
            print("[ITER %4d | %6.2f sec] g2G = %.2f " % (self.tidx, self.curr_time, self.g2G), end="")
            print(" g =    [%.2f, %.2f, %.2f (deg)]" % (self.g[0], self.g[1], np.rad2deg(self.g[2])), end="")
            print(" gbar = [%.2f, %.2f, %.2f (deg)]" % (self.gbar[0], self.gbar[1], np.rad2deg(self.gbar[2])), end="")
