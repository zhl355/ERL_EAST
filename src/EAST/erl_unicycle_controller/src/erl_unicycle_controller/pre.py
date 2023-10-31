#!/usr/bin/env python3
""" Low level velocity controller for unicycle-like robot.

Interfaces:
    Input:
            desired robot states (z*) from ref_gvn node z* = zg
            current robot states (z) from odometry
    Output:
            velocity command for mobile platform or simulated dynamics

"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from ref_gvn_utils.ros_msg_utils import odom_to_pose2d
from tf.transformations import euler_from_quaternion

from erl_msgs.msg import RefGvnMsg


class UnicycleControlPreprocess:
    """ 
    Unicycle Control Preprocess Module. Responsible for:
        1) take ros odom message generate robot states z (e.g., unicycle states z = (x, y, theta))
        2) optional state transformation, e.g., nonlinear->linear, cartesian-> polar for low level controller 
    """

    # Running status table, higher number better status

    def __init__(self):

        # ------------------- init ros interface --------------------
        # params from launch file
        _setpoint_topic = rospy.get_param('~setpoint_topic')
        _odom_topic = rospy.get_param('~odom_topic')
        _gvn_msg_topic = rospy.get_param('~gvn_msg_topic', [])

        # subscribers
        self._odom_sub = rospy.Subscriber(_odom_topic, Odometry, self.odom_callback)
        self._setpoint_sub = rospy.Subscriber(_setpoint_topic, Pose2D, self.setpoint_callback)

        # draw information from custom topic (optional)
        if not _gvn_msg_topic:
            pass
        else:
            self._gvn_msg_sub = rospy.Subscriber(_gvn_msg_topic, RefGvnMsg, self.gvn_msg_callback)

        # transform robot state (odom) to Pose2d and publish
        self._ctrl_odom_pose2d_pub = rospy.Publisher('~ctrl_pose2d', Pose2D, queue_size=1)

        # pub message 
        self.ctrl_pose2d = None

        # upstream status variables
        self._upstream_connection = 0
        self._upstream_connection_ready = False
        self._upstream_data_ready = False
        self._gvn_msg_received = False

        # ------------------- upstream data container --------------------
        # containers for converted message in numpy
        self.np_z = None  # robot states from odom
        self.np_z_dsr = None  # desired robot states from high level controller

        # ------------------- Init Upstream --------------------
        self.init_upstream()
        rospy.loginfo("[Unicycle Controller Preprocessor Created!]  \n")

        # compute directional metric boost factor beta =  dQcO/dIcO
        self.sddm_boost = 1

    def _check_upstream_connections(self, upstream_connection=2):
        """ check whether subscribers' uplink connections are established """

        self._upstream_connection = \
            self._odom_sub.get_num_connections() + \
            self._setpoint_sub.get_num_connections()

        if self._upstream_connection < upstream_connection:
            # we need to wait states, setpoint ready
            rospy.loginfo('[unicycle controller] waiting upstream connections [%d / %d]:', self._upstream_connection, upstream_connection)

            # odom
            if self._odom_sub.get_num_connections() < 1:
                rospy.loginfo_throttle(1.0, "[unicycle controller] waiting odom...")

            # setpoint
            if self._setpoint_sub.get_num_connections() < 1:
                rospy.loginfo_throttle(1.0, "[unicycle controller] waiting setpoint...")
        else:
            self._upstream_connection_ready = True
            rospy.loginfo("\n[unicycle controller] %d upstream connections established !\n", upstream_connection)

    def _check_upstream_data(self):
        """ check whether upstream data container are loaded/initialized correctly"""
        status = True

        # robot state z
        if self.np_z is None:
            status = False
            rospy.loginfo_throttle(1.0, "[unicycle controller] waiting zvec init...")

        # desired robot state z* (setpoint), for ref_gvn high level controller z* = zg
        if self.np_z_dsr is None:
            status = False
            rospy.loginfo_throttle(1.0, "[unicycle controller] waiting zvec_dsr init...")

        if status:
            self._upstream_data_ready = True
            rospy.loginfo_once("\n[unicycle controller] all %d upstream data initialized !\n" % self._upstream_connection)

    def init_upstream(self):
        """ 
        Init upstream of unicycle controller.
            1. check upstream connection
            2. check upstream message and initialize downstream data containers
        """
        while (not self._upstream_connection_ready) and (not rospy.is_shutdown()):
            self._check_upstream_connections()
            rospy.sleep(0.1)  # avoid inquery too fast
            rospy.loginfo_throttle(1.0, "[unicycle controller] waiting upstream connections...")
        rospy.loginfo("upstream [connection] is ready, check upstream [data]...")

        while (not self._upstream_data_ready) and (not rospy.is_shutdown()):
            self._check_upstream_data()
            rospy.sleep(0.1)  # avoid inquery too fast

        rospy.loginfo_once("[unicycle controller] upstream [data] is ready!")
        rospy.loginfo("[unicycle controller] Upstream Init Done!")

    def odom_callback(self, msg_odom):
        """
        This function obtain odometry information, and convert robot heading from quaternion to euler angle
        """
        rospy.logdebug("[unicycle controller pre] Received odometry!")
        pose = msg_odom.pose.pose
        quaternion_sxyz = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quaternion_sxyz)
        self.np_z = np.array([pose.position.x, pose.position.y, yaw])

        # echo odom as pose2d
        self.ctrl_pose2d = odom_to_pose2d(msg_odom)
        self._ctrl_odom_pose2d_pub.publish(self.ctrl_pose2d)
        rospy.logdebug_once("ctrl odom received, converted and published.")
        return

    def setpoint_callback(self, msg_pose2d):
        """
        This function obtain desired goal in se(2)
        """
        rospy.logdebug("Received setpoint!")
        self.np_z_dsr = np.array([msg_pose2d.x, msg_pose2d.y, msg_pose2d.theta])
        return

    def gvn_msg_callback(self, msg: RefGvnMsg):
        """
        If RefGvnMsg has been provided, then calculate sddm boost using dQcO and dIcO from that message
        """
        rospy.logdebug("Received ref gvn msg!")
        self._gvn_msg_received = True

        # map resolution
        dist_eps = 0.1

        # prevent divide by close zero number
        if msg.dIcO > dist_eps:
            self.sddm_boost = msg.dQcO / msg.dIcO
        else:
            self.sddm_boost = 1.0

        rospy.logdebug_throttle(0.5, "[cone controller] sddm boost = %.2f" % self.sddm_boost)
        return
