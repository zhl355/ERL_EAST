#!/usr/bin/env python3

"""
ROS marker utility functions customized for reference governor. 
"""

from geometry_msgs.msg import Point
from ref_gvn_utils.ros_marker_utils import create_actor2d, create_cylinder2d_marker
from ref_gvn_utils.ros_marker_utils import create_point2d_marker, create_arrow2d_marker


def create_start_marker(loc2d):
    """
    Create marker for start 2d point. 
    Red cube
    """
    start_actor = create_point2d_marker(loc2d=loc2d, id=0, color_rgba=[1.0, 0.0, 0.0, 1.0])
    start_actor.ns = "ref_gvn_planning"
    return start_actor


def create_goal_marker(loc2d):
    """
    Create marker for goal 2d point. 
    Green cube
    """
    goal_actor = create_point2d_marker(loc2d=loc2d, id=1, color_rgba=[0.0, 1.0, 0.0, 1.0])
    goal_actor.ns = "ref_gvn_planning"
    return goal_actor


def create_gvn2d_marker(g0):
    """
    Create marker for governor 2d point.
    Small blue ball (using cylinder type 3)
    """
    gvn2d_actor = create_actor2d(
        id=10,
        type=3,
        loc2d=g0[0:2],
        disp_h=0.0,
        color_rgba=[0.0, 0.0, 1.0, 1.0],  # blue
        scale_xyz=[0.2, 0.2, 0.1],  # diameter x, diameter y, height
    )
    gvn2d_actor.ns = "ref_gvn_core"
    return gvn2d_actor


def create_lpg2d_marker(gbar0):
    """
    Create marker for local projected goal 2d point. 
    Tiny red ball (using cylinder, type 3)
    """
    lpg2d_actor = create_actor2d(
        id=11,
        type=3,
        loc2d=gbar0[0:2],
        disp_h=0.2,
        color_rgba=[1.0, 0.0, 0.0, 1.0],  # red
        scale_xyz=[0.1, 0.1, 0.1],  # diameter x, diameter y, height
    )
    lpg2d_actor.ns = "ref_gvn_core"
    return lpg2d_actor


def create_ls_marker(ball_center, ball_radius):
    """
    Create marker for local safe zone (ball)
    Yellow ball (using cylinder, type 3)
    """
    ls_actor = create_actor2d(
        id=12,
        type=3,
        loc2d=ball_center,
        disp_h=-0.1,
        color_rgba=[1.0, 1.0, 0.0, 0.5],  # yellow transparent ball
        scale_xyz=[ball_radius * 2.0, ball_radius * 2.0, 0.1],  # diameter x, diameter y, height
    )
    ls_actor.ns = "ref_gvn_core"
    return ls_actor


def create_lf_marker(ball_center, ball_radius):
    """
    Create marker for local free space (ball)
    Gray ball (using cylinder, type 3)
    """
    lf_actor = create_actor2d(
        id=13,
        type=3,
        loc2d=ball_center,
        disp_h=-0.2,
        color_rgba=[1.0, 1.0, 1.0, 0.3],
        scale_xyz=[ball_radius * 2.0, ball_radius * 2.0, 0.05],  # diameter x, diameter y, height
    )
    lf_actor.ns = "ref_gvn_core"
    return lf_actor


def create_lpg2d_star_marker(lpg2d_star):
    """
    Create marker for optimized local projected goal 2d point. 
    Tiny red cross (using line_list, type 5)
    """

    point0 = Point(-0.1, 0.0, 0.0)
    point1 = Point(+0.1, 0.0, 0.0)
    point2 = Point(0.0, +0.1, 0.0)
    point3 = Point(0.0, -0.1, 0.0)

    cross_points = [point0, point1, point2, point3]

    lpg2d_star_actor = create_actor2d(
        id=11,
        type=5,
        loc2d=lpg2d_star,
        disp_h=1.0,
        color_rgba=[0.0, 1.0, 0.0, 1.0],  # green
        scale_xyz=[0.04, 0.01, 0.01],  # line width
    )
    lpg2d_star_actor.ns = "actlpg_core"
    lpg2d_star_actor.points = cross_points
    return lpg2d_star_actor


def create_gvn_dir_marker(gvn2d, lpg2d):
    """
    Create marker displaying gvn direction. (gvn2d-->lpg2d)
    purple arrow (using cylinder, type 3)
    """
    gvn_dir_actor = create_arrow2d_marker(start2d=gvn2d, goal2d=lpg2d, id=14)
    gvn_dir_actor.ns = "ref_gvn_core"
    return gvn_dir_actor


def create_lpg2d_star_marker(lpg2d_star):
    """
    Create marker for optimized local projected goal 2d point.
    Tiny red cross (using line_list, type 5)
    """

    point0 = Point(-0.1, 0.0, 0.0)
    point1 = Point(+0.1, 0.0, 0.0)
    point2 = Point(0.0, +0.1, 0.0)
    point3 = Point(0.0, -0.1, 0.0)

    cross_points = [point0, point1, point2, point3]

    lpg2d_star_actor = create_actor2d(
        id=11,
        type=5,
        loc2d=lpg2d_star,
        disp_h=1.0,
        color_rgba=[0.0, 1.0, 0.0, 1.0],  # green
        scale_xyz=[0.04, 0.01, 0.01],  # line width
    )
    lpg2d_star_actor.ns = "actlpg_core"
    lpg2d_star_actor.points = cross_points
    return lpg2d_star_actor


def create_robot_set_marker(g, d_gz):
    """
    Create marker displaying robot trajectory Euclidean Ball
    (for computing active local projected goal)
    """
    r = max(0.01, d_gz)
    robot_ball_actor = create_cylinder2d_marker(id=12, loc2d=g, c='purple', alpha=0.4, r=r, ns='actlpg_core')
    return robot_ball_actor
