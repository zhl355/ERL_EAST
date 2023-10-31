#!/usr/bin/env python3
import numpy as np

import rospy
from ref_gvn_utils.geometry_utils import wrap_angle_pmp


# Cone controller for unicycle-like robot positional stabilization
#   (angle convergence is not guaranteed)
# https://arxiv.org/pdf/2209.12648.pdf


class ConeController:
    """ 
    Cone controller from Omur's technical report. Using this to compute 
    velocity control signal given current and desired robot states.
    """

    # Running status table, higher number better status
    NORMAL = 0
    GOAL_LOC_REACHED = 1
    GOAL_POSE_REACHED = 2

    def __init__(self, ctrl_params):
        """
        Init cone controller
        Input: 
            @ctrl_params: controller design parameters
        """
        if ctrl_params is not None:
            # controller design parameters
            self.kv = ctrl_params["kv"]
            self.kw = ctrl_params["kw"]
        else:
            print("[ConeController] use default params")
            self.kv = 0.5
            self.kw = 1.5

        self.warning_msg = None
        self.status = ConeController.NORMAL
        self.goal_pose_reached_announced = False

    def generate_control(self, z, z_dsr,
                         eps_dist=0.1,
                         eps_dist_reset=0.3,
                         eps_angle=0.05,
                         eps_angle_reset=0.2,
                         sddm_boost=1.0,
                         debug=False):
        """
        Generate velocity control signal (v, omega) given current robot states and desired robot states.
        Input:
            @z: current robot states (x, y, theta)
            @z_dsr: desired robot states (x*, y*, theta*)
            @eps_dist: goal region tolerance (meter)
            @eps_angle: goal pose angle tolerance in rad
            @eps_dist_reset: hysteresis reset mechanism for dist
            @eps_angle_reset: hysteresis reset mechanism for angle
        Output:
            @cmd_v: raw linear velocity command (m/sec)
            @cmd_w: raw angular velocity command (rad/sec)
        """

        if not self.goal_pose_reached_announced:
            rospy.logdebug_throttle(0.5, "[cone controller] current status = --------------------------- %s" % self.status)
            if self.status == ConeController.GOAL_POSE_REACHED:
                rospy.logdebug("[cone controller] GOAL POSE REACHED !!")
                self.goal_pose_reached_announced = True

        e = z_dsr[0:2] - z[0:2]  # positional vector from current position to goal position
        err_dist_norm = np.linalg.norm(e)
        err_angle = wrap_angle_pmp(z_dsr[2] - z[2])
        err_angle_norm = np.abs(err_angle)

        msg = None
        new_status = self.status
        # ----------------------- Finite State Machine -----------------------
        # hysteresis state jump to combat against noise

        # start with pose_reached status
        if self.status == ConeController.GOAL_POSE_REACHED:
            if err_dist_norm > eps_dist_reset:
                new_status = ConeController.NORMAL
                msg = "[cone controller] status [down] [pose --> normal] triggered by [dist] err"
                msg += ": dist err > eps_dist_reset (%.3f > %.3f)" % (err_dist_norm, eps_dist_reset)
                rospy.logdebug_throttle(0.5, msg)
                self.goal_pose_reached_announced = False
            # distance error <= eps_dist_reset 
            else:
                if err_angle_norm > eps_angle_reset:
                    new_status = ConeController.GOAL_LOC_REACHED
                    msg = "[cone controller] status [down] [pose --> loc] triggered by [angle] err"
                    msg += ": angle err > eps_angle_reset (%.3f > %.3f)" % (err_angle_norm, eps_angle_reset)
                    rospy.logdebug_throttle(0.5, msg)
                    self.goal_pose_reached_announced = False
                # distance error <= eps_dist_reset, angle error <= eps_angle_reset 
                else:
                    # remain at POSE_REACHED
                    pass

        # start with loc_reached status
        if self.status == ConeController.GOAL_LOC_REACHED:
            if err_dist_norm > eps_dist_reset:
                new_status = ConeController.NORMAL
                msg = "[cone controller] status [down] [loc --> normal] triggered by [dist] err"
                msg += ": dist err > eps_dist_reset (%.3f > %.3f)" % (err_dist_norm, eps_dist_reset)
                rospy.logdebug_throttle(0.5, msg)
            # distance error <= eps_dist_reset 
            else:
                if err_angle_norm <= eps_angle:
                    new_status = ConeController.GOAL_POSE_REACHED
                    msg = "[cone controller] status [ up ] [loc --> pose]"
                    msg += ": |angle err| <= eps_angle (%.3f < %.3f)" % (err_angle_norm, eps_angle)
                    rospy.logdebug_throttle(0.5, msg)
                # distance error <= eps_dist_reset, angle error > eps_angle
                else:
                    # remain at LOC_REACHED
                    pass

        # start with normal status
        if self.status == ConeController.NORMAL:
            if err_dist_norm <= eps_dist:
                new_status = ConeController.GOAL_LOC_REACHED
                msg = "[cone controller] status [ up ] [normal --> loc]"
                msg += ": dist err <= eps_dist (%.3f > %.3f)" % (err_dist_norm, eps_dist)
                rospy.logdebug_throttle(0.5, msg)
            # distance error > eps_dist 
            else:
                # remain at NORMAL
                pass

        # -------------------- applied control strategy by status --------------
        # stay static
        cmd_v = 0.0
        cmd_w = 0.0

        if new_status == ConeController.GOAL_POSE_REACHED:
            pass  # do nothing
        # turn in place
        angular_velocity_sf = 0.3  # angular velocity scale factor, applied when close to goal, prevent turn-in-place drifting.
        if new_status == ConeController.GOAL_LOC_REACHED:
            cmd_v = 0.0
            # when close to goal, slow turn, prevent turn-in-place induced position drifting
            cmd_w = angular_velocity_sf * self.kw * err_angle
            rospy.logdebug_throttle(0.5, "[llc = proportional controller] self.kw = %.2f, err_angle = %.2f, cmd_w = %.2f" % (self.kw, err_angle, cmd_w))
        # cone controller
        if new_status == ConeController.NORMAL:
            rospy.logdebug_throttle(0.5, "[llc  = cone controller] active")
            # ------------------ normal case  ----------------
            theta = z[2]
            u1 = np.array([np.cos(theta), np.sin(theta)])  # heading direction
            u2 = np.array([-np.sin(theta), np.cos(theta)])  # R_ccw(pi/2) * u1

            e_proj_u1 = float(np.inner(u1, e))
            e_proj_u2 = float(np.inner(u2, e))

            cmd_v = sddm_boost * self.kv * max(0.0, e_proj_u1)

            # when close to goal, the angle error is sensitive due to atan2 discontinuity, apply angular velocity scaling
            if np.abs(e_proj_u1) < 2.0 * eps_dist:
                cmd_w = angular_velocity_sf * self.kw * np.arctan2(e_proj_u2, e_proj_u1)
                rospy.logdebug_throttle(0.5, "[cone controller] close too goal")
            else:
                cmd_w = self.kw * np.arctan2(e_proj_u2, e_proj_u1)

            if debug:
                print("input z = [%.2f, %.2f, %.2f]" % (z[0], z[1], z[2]))
                print("input z_dsr = [%.2f, %.2f, %.2f]" % (z_dsr[0], z_dsr[1], z_dsr[2]))
                print("pos. error e = [%.2f, %.2f]" % (e[0], e[1]))
                print("u1 = [%.2f, %.2f]" % (u1[0], u1[1]))
                print("u2 = [%.2f, %.2f]" % (u2[0], u2[1]))
                print("[e_proj_u1, e_proj_u2]  = [%.2f, %.2f]" % (e_proj_u1, e_proj_u2))

        self.status = new_status

        if not self.goal_pose_reached_announced:
            rospy.logdebug_throttle(0.5, "[err_dist_norm,  cmd_v] = [%.2f, %.2f]" % (err_dist_norm, cmd_v))
            rospy.logdebug_throttle(0.5, "[err_angle_norm, cmd_w] = [%.2f, %.2f]" % (err_angle_norm, cmd_w))
            rospy.logdebug_throttle(0.5, "after func call status = --------------------------- %s" % self.status)

        self.warning_msg = msg

        return cmd_v, cmd_w
