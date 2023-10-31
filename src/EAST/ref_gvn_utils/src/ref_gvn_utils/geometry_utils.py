#!/usr/bin/env python3

"""
Put geometry utility functions. 
"""

import numpy as np


def get_theta_from_two_pts(ptA, ptB, debug=False):
    dy = ptB[1] - ptA[1]
    dx = ptB[0] - ptA[0]
    theta = np.arctan2(dy, dx)
    if debug:
        print(ptA)
        print(ptB)
        print("[dy, dx, theta] = [%.2f, %.2f, %.2f]" % (dy, dx, theta))
    return theta


def wrap_angle_pmp(angle_vec):
    """
    npla.normalize angle in radian to [-pi, pi)
    angle_vec: angle description in radian
    """
    angle_vec = (angle_vec + np.pi) % (2 * np.pi) - np.pi
    return angle_vec


def transform_goal2d_w2b(bf_pose, wf_pt):
    """
    Transform goal from world frame to body frame.
    input: bf_pose - body frame origin pose in world frame (1d array [x, y, theta])
           wf_pt - 2d goal position in world frame need to be converted to body frame coordinates (1d array [xw, yw])

    Output: 2d position in body frame (1d array [xb, yb])
    """
    theta = bf_pose[2]
    wRb = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    bf_pt = (wRb.T) @ (wf_pt - bf_pose[0:2])
    return bf_pt
