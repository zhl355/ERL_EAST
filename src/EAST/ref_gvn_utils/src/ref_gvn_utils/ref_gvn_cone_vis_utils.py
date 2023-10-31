#!/usr/bin/env python3
import numpy as np

from geometry_msgs.msg import Point
from ref_gvn_utils.geometry_utils import get_theta_from_two_pts, wrap_angle_pmp
from ref_gvn_utils.ros_marker_utils import create_actor2d


def get_iccone_params(rbt_pose, gvn_loc2):
    """
    Compute radius of ice cream ball from current robot pose and goal robot pose;
    generate coordinates of two tangent 2d-points for ice cream cone in robot Body Frame.
    Input: 1D-array contains position & orientation information: [x, y, phi]
    """

    rbt_loc2, gvn_loc2 = rbt_pose[0:2], gvn_loc2
    rbt_yaw = rbt_pose[2]

    tracking_error = np.linalg.norm(rbt_loc2 - gvn_loc2)
    heading_error_wf = get_theta_from_two_pts(rbt_loc2, gvn_loc2)
    heading_error_bf = wrap_angle_pmp(heading_error_wf - rbt_yaw)

    if np.abs(heading_error_bf) >= np.pi / 2.0:
        icball_radius = tracking_error
    else:
        icball_radius = tracking_error * np.sin(heading_error_bf)

    # compute coordinates for 2 tangent points (tan_pt1, tan_pt2)
    tan_pt1_x = tracking_error * np.cos(heading_error_bf)
    tan_pt2_x = tan_pt1_x * np.cos(2 * heading_error_bf)
    tan_pt2_y = tan_pt1_x * np.sin(2 * heading_error_bf)

    # TODO YYZ naming are bad, use longer but clear name
    if np.abs(heading_error_bf) >= np.pi / 2.0:
        tpt3d1 = Point(0, 0.0, 0.0)
        tpt3d2 = Point(0, 0, 0.0)
    else:
        tpt3d1 = Point(tan_pt1_x, 0.0, 0.0)
        tpt3d2 = Point(tan_pt2_x, tan_pt2_y, 0.0)
    z_pos3d = Point(0.0, 0.0, 0.0)

    return icball_radius, tpt3d1, tpt3d2, z_pos3d


def create_icball_marker(z, zg):
    """
    Create marker for predicted reachable set centered at goal position (ice cream ball).
    Green ball
    """

    radius, _, _, _ = get_iccone_params(z, zg)
    icball_actor = create_actor2d(
        id=101,
        type=3,
        loc2d=zg[0:2],
        disp_h=0.1,
        color_rgba=[0.0, 0.4, 0.0, 0.5],
        scale_xyz=[2 * radius, 2 * radius, 0.1],
    )
    icball_actor.action = icball_actor.ADD
    return icball_actor


def create_iccone_marker(z, zg):
    """
    Create marker for predicted reachable set initialed at current position (ice cream cone).
    Green line-strip
    """

    iccone_actor = create_actor2d(
        id=111,
        type=4,
        loc2d=z[0:2],
        disp_h=0.1,
        color_rgba=[0.0, 0.4, 0.0, 0.5],
        scale_xyz=[0.1, 0.0, 0.0],
        yaw=z[2]
    )

    _, tpt3d1, tpt3d2, z_pos3d = get_iccone_params(z, zg)
    iccone_actor.points = [tpt3d1, z_pos3d, tpt3d2]
    iccone_actor.action = iccone_actor.ADD
    return iccone_actor
