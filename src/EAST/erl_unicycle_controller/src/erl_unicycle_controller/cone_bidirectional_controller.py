#!/usr/bin/env python3
import rospy
import numpy as np
from ref_gvn_utils.geometry_utils import transform_goal2d_w2b


# bi-directional cone controller for unicycle-like robot positional stabilization
#   (angle convergence is not guaranteed)
# https://arxiv.org/pdf/2209.12648.pdf


class ConeBDController:
    """ 
    Modified cone controller bi-direction includes backward motion. Using this to compute 
    velocity control signal given current and desired robot states.
    *Not tracking goal heading*
    """

    # Running status table, higher number better status
    NORMAL = 0
    GOAL_LOC_REACHED = 1

    def __init__(self, ctrl_params):
        """
        Init cone bi-directional controller
        Input: 
            @ctrl_params: controller design parameters
        """
        if ctrl_params is not None:
            # controller design parameters
            self.kv = ctrl_params["kv"]
            self.kw = ctrl_params["kw"]
        else:
            print("[ConeBDController] use default params")
            self.kv = 0.5
            self.kw = 1.5

        self.warning_msg = None
        self.status = ConeBDController.NORMAL
        self._goal_pose_reached_announced = False

    def generate_control(self, z, z_dsr,
                         eps_dist=0.1,
                         eps_dist_reset=0.1,
                         sddm_boost=None,
                         debug=True):
        """
        Generate velocity control signal (v, omega) given current robot states and desired robot states.
        Input:
            @z: current robot states (x, y, theta)
            @z_dsr: desired robot states (x*, y*, theta*)
            @eps_dist: goal region tolerance (meter)
            @eps_dist_reset: hysteresis reset mechanism for dist
        Output:
            @cmd_v: raw linear velocity command (m/sec)
            @cmd_w: raw angular velocity command (rad/sec)
        """
        # init control inputs to be zero
        cmd_v = 0.0
        cmd_w = 0.0

        if not self._goal_pose_reached_announced:
            rospy.logdebug_throttle(0.5,
                                    "[cone controller bi-direction] current status = --------------------------- %s" % self.status)
            if self.status == ConeBDController.GOAL_LOC_REACHED:
                rospy.logwarn("[cone controller bi-direction] GOAL REACHED !!")
                self._goal_pose_reached_announced = True

        e = z_dsr[0:2] - z[0:2]  # positional vector from current position to goal position
        err_dist_norm = np.linalg.norm(e)

        msg = None
        new_status = self.status

        # ----------------------- Finite State Machine -----------------------
        # hysteresis state jump to combat against noise

        # start with loc_reached status
        if self.status == ConeBDController.GOAL_LOC_REACHED:
            if err_dist_norm > eps_dist_reset:
                new_status = ConeBDController.NORMAL
                msg = "[cone controller bi-direction] status [down] [loc --> normal] triggered by [dist] err"
                msg += ": dist err > eps_dist_reset (%.3f > %.3f)" % (err_dist_norm, eps_dist_reset)
                rospy.logwarn_throttle(0.5, msg)
            # distance error <= eps_dist_reset
            else:
                # remain at LOC_REACHED
                pass

        # start with normal status
        if self.status == ConeBDController.NORMAL:
            if err_dist_norm <= eps_dist:
                new_status = ConeBDController.GOAL_LOC_REACHED
                msg = "[cone controller bi-direction] status [ up ] [normal --> loc]"
                msg += ": dist err <= eps_dist (%.3f > %.3f)" % (err_dist_norm, eps_dist)
                rospy.logwarn_throttle(0.5, msg)
            # distance error > eps_dist 
            else:
                # remain at NORMAL
                pass

        # -------------------- applied control strategy by status --------------
        # stay static
        if new_status == ConeBDController.GOAL_LOC_REACHED:
            cmd_v = 0.0
            cmd_w = 0.0

        # cone bi-directional controller
        if new_status == ConeBDController.NORMAL:
            rospy.logdebug_throttle(0.5, "[llc  = bi-direction cone controller] active")

            # ------------------ normal case  ----------------
            z_dsr_prime = transform_goal2d_w2b(z, z_dsr[0:2])
            e_prime = z_dsr_prime[0]
            e_phi = np.arctan(z_dsr_prime[1] / z_dsr_prime[0])

            cmd_v = self.kv * e_prime
            cmd_w = self.kw * e_phi

            if debug:
                rospy.logdebug_throttle(0.5, "input z = [%.2f, %.2f, %.2f]" % (z[0], z[1], z[2]))
                rospy.logdebug_throttle(0.5, "input z_dsr = [%.2f, %.2f, %.2f]" % (z_dsr[0], z_dsr[1], z_dsr[2]))
                rospy.logdebug_throttle(0.5, "pos. error e = [%.2f, %.2f]" % (e[0], e[1]))
                rospy.logdebug_throttle(0.5, "error states [e_prime, e_phi(deg)] = [%.2f, %.2f]" % (
                    e_prime, np.rad2deg(e_phi)))

        self.status = new_status

        if not self._goal_pose_reached_announced:
            rospy.logdebug_throttle(0.5, "[err_dist_norm,  cmd_v] = [%.2f, %.2f]" % (err_dist_norm, cmd_v))
            rospy.logdebug_throttle(0.5, "after func call status = --------------------------- %s" % self.status)

        self.warning_msg = msg

        return cmd_v, cmd_w
