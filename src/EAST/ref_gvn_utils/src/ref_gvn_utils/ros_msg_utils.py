#!/usr/bin/env python3

"""
Put ROS message utility functions. 
"""

import numpy as np

from geometry_msgs.msg import Pose2D, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def pose_to_pose2d(pose):
    """ Create pose2d (x, y, theta) msg from pose msg. 
    """
    quaternion_sxyz = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    (_, _, yaw) = euler_from_quaternion(quaternion_sxyz)
    pose2d = Pose2D()
    pose2d.x = pose.position.x
    pose2d.y = pose.position.y
    pose2d.theta = yaw
    return pose2d


def odom_to_pose2d(msg_odom):
    """ Extract (x, y, theta) from odometry msg and converted it to pose2d.
    """
    return pose_to_pose2d(msg_odom.pose.pose)


def pose2d_to_pose(pose2d_msg, fix_pos_z=0.01):
    """ Create pose_msg from pose2d
    """
    pose_msg = Pose()
    pose_msg.position.x = pose2d_msg.x
    pose_msg.position.y = pose2d_msg.y
    pose_msg.position.z = fix_pos_z

    q = quaternion_from_euler(0.0, 0.0, pose2d_msg.theta)
    pose_msg.orientation.x = q[0]
    pose_msg.orientation.y = q[1]
    pose_msg.orientation.z = q[2]
    pose_msg.orientation.w = q[3]
    return pose_msg


def point3d_np_to_pose(point3d_np):
    """ Create pose_msg from pose2d
    """
    pose_msg = Pose()
    pose_msg.position.x = point3d_np[0]
    pose_msg.position.y = point3d_np[1]
    pose_msg.position.z = point3d_np[2]

    q = quaternion_from_euler(0.0, 0.0, 0.0)
    pose_msg.orientation.x = q[0]
    pose_msg.orientation.y = q[1]
    pose_msg.orientation.z = q[2]
    pose_msg.orientation.w = q[3]

    return pose_msg


def pose_to_position_np(pose_msg):
    """ Create position vector as np array from pose message.
    """
    point3d_np = np.zeros(3)
    point3d_np[0] = pose_msg.position.x
    point3d_np[1] = pose_msg.position.y
    point3d_np[2] = pose_msg.position.z
    return point3d_np


def pose_stamped_to_pose2d(pose_stamped):
    """ Extract (x, y, theta) from odometry msg and converted it to pose2d. 
    """
    return pose_to_pose2d(pose_stamped.pose)


def odom_to_pose2d(msg_odom):
    """ Extract (x, y, theta) from odometry msg and converted it to pose2d. 
    """
    return pose_to_pose2d(msg_odom.pose.pose)


def get_loc2d_from_pose(pose):
    """ 
    Extract location information from pose msg and create a numpy object
    http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html
    """
    return np.array([pose.position.x, pose.position.y])
