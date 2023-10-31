#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

from ref_gvn_utils.ros_msg_utils import pose_stamped_to_pose2d
from ref_gvn_utils.ros_gvn_marker_utils import create_gvn2d_marker


class DummyGoalGenerator:
    def __init__(self, config_dict=None):
        """ Init dummy goal class.

            This controller subscribes:
                rviz 2D nav goal (/move_base_simple/goal)
            Publish:
                 desired robot states (z*)
                 marker to inidate goal location
        """
        # loading external config parameters
        self._config_dict = config_dict
        # u_shape_path = np.array([[0, 0, 0],
        #                          [5.25, 0, 0],
        #                          [5.25, -10, -np.pi/2],
        #                          [0, -10, -np.pi]])
	

        self.np_path = np.load('/home/yiyz/jackal_ws/src/debugging_ros_bag/scripts/mid_path.npy')

        # ------------------- subscribers ----------------------
        self.dummy_goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        self.gvn_mk = create_gvn2d_marker(np.zeros(2))  # marker goal2d

        # ------------------- publishers ----------------------
        # rviz goal
        self.dummy_goal_pub = rospy.Publisher('/ref_gvn/local_goal', Pose2D, queue_size=1)
        self.dummy_path_pub = rospy.Publisher('/path', Path, queue_size=1)

        # marker
        self.marker_pub = rospy.Publisher('~goal_marker', Marker, queue_size=1)
        rospy.logwarn("DummyGoalGenerator Inited!")

        # -------------------- other status -------------------------
        self._goal_received = False

    def rviz_goal_callback(self, msg):
        """
        Update local goal by chaning rviz 2D nav mannually.
        """
        self._goal_received = True
        # receive goal (odometry) from rviz button nav2d. The goal is at map frame.
        pose2d = Pose2D()
        path = Path()

        # dummy goal
        pose2d.x = 0
        pose2d.y = -10
        pose2d.theta = -np.pi
        info_msg = "[dummy goal] update local goal = [%.2f, %.2f, %.2f]" % (pose2d.x, pose2d.y, pose2d.theta)
        rospy.loginfo(info_msg)

        # cache local goal states
        # self.local_goal = pose2d

        # publish new local goal and update marker
        self.dummy_goal_pub.publish(pose2d)
        self.gvn_mk.pose.position.x = pose2d.x
        self.gvn_mk.pose.position.y = pose2d.y
        self.marker_pub.publish(self.gvn_mk)

        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()
        way_points = []

        # assign way points for path
        for point in self.np_path:
            way_point = PoseStamped()
            way_point.pose.position.x = point[0]
            way_point.pose.position.y = point[1]
            way_point.pose.position.z = 0
            way_points.append(way_point)

        path.poses = way_points
        self.dummy_path = path

        self.dummy_path_pub.publish(self.dummy_path)

    def publish_local_goal(self):
        """
        Publish cached goal if goal received.
        """
        if self._goal_received:
            # self.dummy_goal_pub.publish(self.local_goal)

            self.dummy_path.header.stamp = rospy.Time.now()
            self.dummy_path_pub.publish(self.dummy_path)


if __name__ == '__main__':

    try:
        rospy.init_node('dummy_path')
        rospy.loginfo("bring up [dummy_path] node!\n")
        rospy.logwarn("start publishing dummy path by clicking rviz [2D Nav Goal] button in [map] frame!")

        # loading parameters
        pub_goal_freq = rospy.get_param("~pub_path_freq", 50.0)

        dummy_goal_gen = DummyGoalGenerator()

        rate = rospy.Rate(pub_goal_freq)

        while not rospy.is_shutdown():
            # publish local goal at certain freq
            dummy_goal_gen.publish_local_goal()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr("dummy path node init failed.")
        rospy.signal_shutdown('dummy path node init fail')
        pass
