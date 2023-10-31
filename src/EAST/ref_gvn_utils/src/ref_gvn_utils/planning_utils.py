#!/usr/bin/env python3

"""
Put planning utility functions. 
"""



import numpy as np
from ref_gvn_utils.geometry_utils import get_theta_from_two_pts
import rospy

def path_simplify(planning_path, debug=False):
    """
    Convert path message to 2d numpy array (num_pts, 2). During this process, 
    simplified path, merging segments within line. for example, if 10 waypoints 
    (pt1, pt2, pt3, ...pt10) are within in a straight line, convert them to (pt1, pt10)
    Note that a path can contain multiple lines. This compression is loseless.
    """

    # check if path has more than 2 poses
    path = planning_path
    nav_path = []

    if len(path) < 2:
        rospy.logwarn_throttle(5.0, "[Preprocess Warning, path has less than 2 poses]")
        return nav_path

    # initialize two pointers for scanning the whole path
    max_idx = len(path) - 1
    nav_path = [path[0]]
    s1 = 0
    s2 = 1
    last_angle = get_theta_from_two_pts(path[0], path[1])

    while s2 <= max_idx:
        seg_start = path[s1]
        seg_end = path[s2]
        angle = get_theta_from_two_pts(seg_start, seg_end)

        if np.abs(angle - last_angle) < 1e-3:
            s2 += 1
        else:
            if debug:
                print("angle jump %.2f to %.2f add node %d" % (last_angle, angle, s2 - 1), end='')
            nav_path.append(path[s2 - 1])
            last_angle = get_theta_from_two_pts(path[s2 - 1], path[s2])  # be cautious about last angle update
            s1 = s2
            s2 += 1
            if debug:
                print("\t\tscan %d -> %d next" % (s1, s2))

    # add last node
    nav_path.append(seg_end)
    # rospy.logdebug_throttle(0.5, "Simplified path from %d points to %d points" % (len(planning_path), len(nav_path)))
    nav_path = np.round(np.array(nav_path), 2)
    return nav_path

    return path
