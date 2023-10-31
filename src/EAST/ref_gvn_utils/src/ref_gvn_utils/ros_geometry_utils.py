#!/usr/bin/env python3

"""
Put ROS geometry utility functions. 
"""



import numpy as np
import rospy
from geometry_msgs.msg import Pose, Pose2D
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def get_theta_from_two_poses(poseA, poseB, debug=False):
    dy = poseB.position.y - poseA.position.y
    dx = poseB.position.x - poseA.position.x
    theta = np.arctan2(dy, dx)
    if debug:
        print(poseA)
        print(poseB)
        print("[dy, dx, theta] = [%.2f, %.2f, %.2f]" % (dy, dx, theta))
    return theta
