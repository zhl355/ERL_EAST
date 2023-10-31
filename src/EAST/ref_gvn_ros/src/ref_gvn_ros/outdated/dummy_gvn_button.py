#!/usr/bin/env python3
"""
Take Rivz 2D Nav Goal and publish as governor.
"""
import numpy as np 
import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


from ref_gvn_utils.ros_msg_utils import pose_stamped_to_pose2d
from ref_gvn_utils.ros_gvn_marker_utils import create_gvn2d_marker


class DummyGvnGenerator:
    def __init__(self, config_dict=None):
        """ Init dummy governor class. 

            This controller subscribes:
                rviz 2D nav goal (/move_base_simple/goal)
            Publish:
                 gvn as pose2d message
                 marker to inidate gvn location
        """
        # loading external config parameters
        self._config_dict = config_dict

        # ------------------- subscribers ----------------------
        self.dummy_goal_sub = rospy.Subscriber('/move_base_simple/goal',PoseStamped, self.rviz_button_callback)
        self.gvn_mk = create_gvn2d_marker(np.zeros(2)) # marker goal2d

        # ------------------- publishers ---------------------- 
        # rviz goal 
        self.dummy_gvn_pub = rospy.Publisher('/gvn_state', Pose2D, queue_size=1)

        # marker
        self.marker_pub = rospy.Publisher('~goal_marker', Marker, queue_size=1)
        rospy.logwarn("DummyGoalGenerator Inited!")

        # -------------------- other status -------------------------
        self._goal_received = False
        self._gvn_pose2d = Pose2D()

    def rviz_button_callback(self, msg):
        """
        Update gvn by chaning rviz 2D nav mannually. 
        """
        self._goal_received = True
        # receive goal (odometry) from rviz button nav2d. The goal is at map frame.
        pose2d = pose_stamped_to_pose2d(msg)
        info_msg = "[dummy gvn] update gvn = [%.2f, %.2f, %.2f] in map frame" %(pose2d.x, pose2d.y, pose2d.theta)
        rospy.logerr(info_msg)

        # cache gvn states 
        self._gvn_pose2d = pose2d

        # publish gvn and its marker
        self.gvn_mk.pose.position.x = pose2d.x
        self.gvn_mk.pose.position.y = pose2d.y
        
        self.dummy_gvn_pub.publish(pose2d)
        self.marker_pub.publish(self.gvn_mk)
    
    def publish_gvn(self):
        """
        Publish cached goal if goal received. 
        """
        if self._goal_received:
            self.dummy_gvn_pub.publish(self._gvn_pose2d)

if __name__ == '__main__':

    try:
        rospy.init_node('dummy_gvn')
        rospy.loginfo("bring up [dummy_gvn] node!\n")
        rospy.logwarn("dummy gvn update by rviz [2D Nav Goal] button in [map] frame!")

        # loading parameters
        pub_freq = rospy.get_param("~pub_freq", 50.0)
        gvn_gen = DummyGvnGenerator()
        rate = rospy.Rate(pub_freq)

        while not rospy.is_shutdown():
            # publish gvn at certain freq
            gvn_gen.publish_gvn()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr("[dummy_gvn] node init failed.")
        rospy.signal_shutdown('[dummy_gvn] node init fail')
        pass
