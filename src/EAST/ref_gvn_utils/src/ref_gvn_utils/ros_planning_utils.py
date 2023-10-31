#!/usr/bin/env python3

"""
Put ROS message utility functions. 
"""



import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def create_path_msg_from_waypoints(waypoints, frame_id="map", debug=False):
    """
    Create a ros path message from waypoints. 
    Input: 
        waypoints: numpy array (num_pts, dim[2/3])
    Return:
        path: ROS path message
    """
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = rospy.Time.now()

    # force waypoint to be numpy 2d array 
    waypoints = np.reshape(waypoints, (-1, waypoints.shape[-1]))

    for pt in waypoints:
        pose_stp = PoseStamped()
        pose_stp.pose.position.x = pt[0]
        pose_stp.pose.position.y = pt[1]
        # correct init zero orientation for quaternion
        pose_stp.pose.orientation.w = 1.0
        path.poses.append(pose_stp)
    if debug:
        print("debug create_path_from_waypoints")
        print(path)

    return path
