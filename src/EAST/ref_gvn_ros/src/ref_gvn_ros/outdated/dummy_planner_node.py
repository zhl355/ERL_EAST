#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

from ref_gvn_utils.ros_planning_utils import create_path_msg_from_waypoints

jackal_race_waypoints = np.array([
    [0, 0],
    [6, 6],
    [6, 8],
    [0, 8],
    [-8, 2],
    [-8, -1],
    [-3, -1],
], dtype=np.float)


class DummyPlannerGenerator:
    def __init__(self, config_dict=None):
        """ Init dummy planner class.

            This node subscribes:
                rviz 2D nav goal (/move_base_simple/goal)
            Publish:
                hard-coded simple straight line path message
        """
        # loading external config parameters
        self._config_dict = config_dict
        _odom_topic = rospy.get_param('~odom_topic')
        self._odom_sub = rospy.Subscriber(_odom_topic, Odometry, self.odom_callback, queue_size=1)
        _gvn_restart_topic = rospy.get_param('~gvn_restart_topic')

        # ------------------- subscribers ----------------------
        self.rviz_Nav2D_button_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.button_callback)

        # ------------------- publishers ---------------------- 
        # rviz goal 
        self.path_pub = rospy.Publisher('/path', Path, queue_size=1)
        # self.dummy_path = self.create_two_point_dummy_path()
        self.dummy_path = create_path_msg_from_waypoints(waypoints=jackal_race_waypoints)
        # Governor Restart Request
        self.gvn_restart_pub = rospy.Publisher(_gvn_restart_topic, Bool, queue_size=1)
        # -------------------- other status -------------------------
        self._start_planning = False
        self.button_count = 0
        self.rbt_loc_xy = None

    def create_two_point_dummy_path(self):
        """
        Create a two-point dummy path message. 
        """
        path = Path()
        path.header.frame_id = "map"

        pose_stamped_start = PoseStamped()
        pose_stamped_start.pose.position.x = 0.0
        pose_stamped_start.pose.position.y = 0.0
        pose_stamped_start.pose.orientation.w = 1.0

        pose_stamped_end = PoseStamped()
        pose_stamped_end.pose.position.x = 3.0
        pose_stamped_end.pose.position.y = 3.0
        pose_stamped_end.pose.orientation.w = 1.0

        path.poses.append(pose_stamped_start)
        path.poses.append(pose_stamped_end)
        return path

    def button_callback(self, msg):
        """
        Trigger dummy planner.
        """
        self._start_planning = True
        rospy.loginfo("[dummy planner] start planning")
        if self.button_count == 1:
            self.replan()
            self.gvn_restart_pub.publish(True)
            rospy.loginfo_throttle(1.0, "[dummy planner] Replan Path")

        self.button_count = self.button_count + 1

    def odom_callback(self, odom_msg):
        rospy.logwarn_once("[ref_gvn] Received odometry!")
        pose = odom_msg.pose.pose
        self.rbt_loc_xy = np.array([pose.position.x, pose.position.y])
        return

    def publish_dummy_path(self):
        """
        Publish cached goal if goal received. 
        """
        if self._start_planning:
            self.path_pub.publish(self.dummy_path)

    def replan(self):
        self.dummy_path = None
        temp_path = np.array([
            [0, 0],
            [6, 6],
            [6, 8],
            [0, 8],
            [-8, 2],
        ], dtype=np.float)
        new_waypoints = np.row_stack((self.rbt_loc_xy, temp_path))
        self.dummy_path = create_path_msg_from_waypoints(waypoints=new_waypoints)
        self.path_pub.publish(self.dummy_path)


if __name__ == '__main__':

    try:
        rospy.init_node('dummy_path')
        rospy.loginfo("bring up [dummy_path] node!\n")

        # loading parameters
        pub_path_freq = rospy.get_param("~pub_path_freq", 50.0)
        dummy_path_gen = DummyPlannerGenerator()
        rate = rospy.Rate(pub_path_freq)

        while not rospy.is_shutdown():
            # publish path at certain freq
            dummy_path_gen.publish_dummy_path()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr("[dummy path] node init failed.")
        rospy.signal_shutdown('[dummy path] node init fail')
        pass
