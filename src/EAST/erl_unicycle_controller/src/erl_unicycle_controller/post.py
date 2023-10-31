#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D
from erl_msgs.msg import ConeControllerDebug
from ref_gvn_utils.geometry_utils import wrap_angle_pmp
import numpy as np


def clip(x, x_min, x_max):
    """
    clip x in [x_min, xmax]
    """
    if x < x_min:
        x = x_min
    if x > x_max:
        x = x_max
    return x


class UnicycleControlPostprocess:
    """ 
    Unicycle Controller Post-process Module. Responsible for:
        1) create downstream publisher interface 
        2) optional state transformation, e.g., linear->nonlinear, polar->cartesian
        3) clip control with repsect to hardware limits
    """

    def __init__(self, ctrl_limits=None):

        # ------------ init ros interface -------------
        # publish velocity command (body twist) for mobile platform hardware / simulated dynamics
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.debug_pub = rospy.Publisher('~debug', ConeControllerDebug, queue_size=1)
        self._body_twist = Twist()
        rospy.loginfo("[unicycle controller post-processor initialized!]")

        if ctrl_limits is not None:
            self.ctrl_limits = ctrl_limits

    def send_cmd(self, v_dsr, w_dsr, clip_ctrl=False, debug=False):
        """
        Publish low level command velocity to hardware or simulated dynamics
        For unicycle-like robot, desired linear and angular velocity
        """
        if debug:
            rospy.logwarn_throttle(0.5, "Input body twist (v_dsr, omega_dsr) [%.2f, %.2f]" % (v_dsr, w_dsr))

        if clip_ctrl and self.ctrl_limits is not None:
            v_dsr = clip(v_dsr, self.ctrl_limits['v_min'], self.ctrl_limits['v_max'])
            w_dsr = clip(w_dsr, self.ctrl_limits['w_min'], self.ctrl_limits['w_max'])

        if debug:
            rospy.logdebug_throttle(0.5, "Output body twist (v_dsr, omega_dsr) [%.2f, %.2f]" % (v_dsr, w_dsr))

        self._body_twist.linear.x = v_dsr
        self._body_twist.angular.z = w_dsr
        self.cmd_vel_pub.publish(self._body_twist)

        return

    def pub_debug(self, z, z_dsr, sddm_boost, status, ctrl_limits=None):
        """
        Publish debug information of low level controller. In particular, 
        cone controller detailed intermediate results.
        """
        msg = ConeControllerDebug()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.z = Pose2D(x=z[0], y=z[1], theta=z[2])
        msg.z_dsr = Pose2D(x=z_dsr[0], y=z_dsr[1], theta=z_dsr[2])
        e_pose2d_raw = z_dsr - z
        e = e_pose2d_raw[0:2]
        msg.e_pose2d_raw = Pose2D(x=e_pose2d_raw[0], y=e_pose2d_raw[1], theta=e_pose2d_raw[2])
        msg.err_angle = wrap_angle_pmp(e_pose2d_raw[2])
        msg.err_angle_norm = np.abs(msg.err_angle)
        msg.err_angle_deg = np.rad2deg(msg.err_angle)
        msg.err_dist = np.linalg.norm(e)


        theta = z[2]
        u1 = np.array([np.cos(theta), np.sin(theta)]) # heading direction
        u2 = np.array([-np.sin(theta), np.cos(theta)]) # R_ccw(pi/2) * u1

        msg.e_proj_u1 = np.inner(u1, e)
        msg.e_proj_u2 = np.inner(u2, e)

        # If dist error is too small, ignore heading error
        if np.linalg.norm(e) >= 1e-3:
            msg.err_heading = np.arctan2(msg.e_proj_u2, msg.e_proj_u1)
        else:
            msg.err_heading = 0

        # Local controller status
        msg.cone_controller_status = status

        # add new control limits
        if ctrl_limits is not None:
            msg.v_min = ctrl_limits['v_min']
            msg.v_max = ctrl_limits['v_max']
            msg.w_min = ctrl_limits['w_min']
            msg.w_max = ctrl_limits['w_max']

        # cone controller linear control gain boost from directional metric.
        msg.sddm_boost = sddm_boost
        msg.err_heading_norm = np.abs(msg.err_heading)

        self.debug_pub.publish(msg)
