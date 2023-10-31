#!/usr/bin/env python3

import numpy as np 
import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler

from ref_gvn_utils.ros_msg_utils import pose_to_pose2d, odom_to_pose2d
from ref_gvn_utils.ros_gvn_marker_utils import create_gvn2d_marker
from ref_gvn_utils.ros_marker_utils import create_arrow_from_pose

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
        # -------------------- Constants -------------------------
        if self._config_dict["goal_snap"]:
            self._goal_snap = True
            self._snap_res_x = self._config_dict['snap_res_x']
            self._snap_res_y = self._config_dict['snap_res_y']
            self._snap_res_theta = self._config_dict['snap_res_theta']
            snap_res_msg = "snap res [%.2f, %.2f, %.2f]" % (self._snap_res_x, self._snap_res_y, self._snap_res_theta)
            rospy.logwarn("[DummyGoalGenerator] goal_snap is enabled " + snap_res_msg)
        else:
            self._goal_snap = False

        # ------------------- subscribers ----------------------
        # subscribe different call back by flag [goal_snap]
        if not self._goal_snap:
            self.dummy_goal_sub = rospy.Subscriber('/move_base_simple/goal',PoseStamped, self.rviz_goal_callback)
            self.gvn_mk = create_gvn2d_marker(np.zeros(2)) # marker goal2d
        else:
            self.dummy_goal_sub = rospy.Subscriber('/move_base_simple/goal',PoseStamped, self.rviz_simple_goal_callback)
            self.gvn_snapped_mk = create_arrow_from_pose(Pose2D())

        # ------------------- publishers ---------------------- 
        # rviz goal 
        self.dummy_goal_pub = rospy.Publisher('/ref_gvn/local_goal', Pose2D, queue_size=1)

        # marker
        self.marker_pub = rospy.Publisher('~goal_marker', Marker, queue_size=1)
        rospy.logwarn("DummyGoalGenerator Inited!")

        self.local_goal = Pose2D()

        # ------------------- class status ---------------------- 
        self._goal_received = False

    def rviz_goal_callback(self, msg):
        """
        Update local goal by chaning rviz 2D nav mannually. 
        """
        # receive goal from rviz button nav2d. The goal is at map frame.
        self._goal_received = True       
        self.local_goal = odom_to_pose2d(msg)

        info_msg = "update local goal = [%.2f, %.2f, %.2f]" %(self.local_goal.x, self.local_goal.y, self.local_goal.theta)
        rospy.logwarn(info_msg)

        # update marker
        self.gvn_mk.pose.position.x = self.local_goal.x
        self.gvn_mk.pose.position.y = self.local_goal.y

        # publish messages
        self.dummy_goal_pub.publish(self.local_goal)
        self.marker_pub.publish(self.gvn_mk)
        
    def rviz_simple_goal_callback(self, msg):
        """
        Update local goal by clicking rviz [2D Nav Goal] mannually. 
        This funciton will snap the goal to grid with xy_resolution determined 
        by params [snap_res_x, snap_res_y].
        
        Orientation is determined by two ways:
        Close to origin, set theta to res_theta defined grids
        Away from origin, set theta by arctan2(snapped_goal_y, snapped_goal_x)
        """
        # receive goal from rviz button nav2d. The goal is at map frame.
        self._goal_received = True
        raw_pose2d = pose_to_pose2d(msg.pose)
        rospy.loginfo("\ngoal received [%.2f, %.2f, %.2f]" % (raw_pose2d.x, raw_pose2d.y, np.rad2deg(raw_pose2d.theta)))
        # goal snap to 2d grid, theta determined automatically by atan2(dy, dx)
        self.local_goal.x = np.round(raw_pose2d.x / self._snap_res_x) * self._snap_res_x
        self.local_goal.y = np.round(raw_pose2d.y / self._snap_res_y) * self._snap_res_y
        self.local_goal.theta = np.round(raw_pose2d.theta / self._snap_res_theta) * self._snap_res_theta
        
        # if np.abs(raw_pose2d.x) < snap_res_x and np.abs(raw_pose2d.y) < snap_res_y:
        #     self.local_goal.theta = np.round(raw_pose2d.theta / self._snap_res_theta) * self._snap_res_theta
        #     rospy.loginfo("close to origin, theta from snapped theta")
        # else:
        #     self.local_goal.theta = np.arctan2(self.local_goal.y, self.local_goal.x)
        #     rospy.loginfo("away from origin, theta from snapped location")
            
        
        theta_deg = np.rad2deg(self.local_goal.theta)
        info_msg = "update snapped local goal = [%.2f, %.2f, %.2f (deg)]" %(self.local_goal.x, self.local_goal.y, theta_deg)
        rospy.logwarn(info_msg)

        self.gvn_snapped_mk.pose.position.x = self.local_goal.x 
        self.gvn_snapped_mk.pose.position.y = self.local_goal.y

        q = quaternion_from_euler(0.0, 0.0, self.local_goal.theta)
        self.gvn_snapped_mk.pose.orientation.x = q[0]
        self.gvn_snapped_mk.pose.orientation.y = q[1]
        self.gvn_snapped_mk.pose.orientation.z = q[2]
        self.gvn_snapped_mk.pose.orientation.w = q[3]

        # publish messages
        self.dummy_goal_pub.publish(self.local_goal)
        self.marker_pub.publish(self.gvn_snapped_mk)

    def publish_local_goal(self):
        """
        Publish cached goal if goal received. 
        """
        if self._goal_received:
            self.dummy_goal_pub.publish(self.local_goal)

if __name__ == '__main__':

    try:
        rospy.init_node('dummy_local_goal')
        rospy.loginfo("bring up [dummy_local_goal] node!\n")
        rospy.logwarn("dummy goal update by rviz [2D Nav Goal] button in [map] frame!")

        # loading parameters
        pub_goal_freq = rospy.get_param("~pub_goal_freq", 50.0)
        goal_snap = rospy.get_param("~goal_snap", False)
        snap_res_x = rospy.get_param("~snap_res_x", 0.5)
        snap_res_y = rospy.get_param("~snap_res_y", 0.5)
        snap_res_theta = np.deg2rad(rospy.get_param("~snap_res_theta_deg", 45.0))
        

        config_dict = {
            'goal_snap': goal_snap,
            'snap_res_x': snap_res_x,
            'snap_res_y': snap_res_y,
            'snap_res_theta': snap_res_theta,
        }

        dummy_goal_gen = DummyGoalGenerator(config_dict)

        rate = rospy.Rate(pub_goal_freq)

        while not rospy.is_shutdown():
            # publish local goal at certain freq
            dummy_goal_gen.publish_local_goal()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr("dummy local goal node init failed.")
        rospy.signal_shutdown('dummy local goal node init fail')
        pass
