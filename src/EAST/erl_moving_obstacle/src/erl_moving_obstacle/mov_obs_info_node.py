#!/usr/bin/env python3

import numpy as np
import rospy

from erl_msgs.msg import MovObsInfo, MovObsInfoArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

from ref_gvn_utils.ros_marker_utils import create_pose_arrow_marker, create_cylinder2d_marker
from ref_gvn_utils.ros_msg_utils import odom_to_pose2d
from erl_moving_obstacle.virtual_movobs import MovObsStates


# noinspection PyShadowingNames
class MovObsWrapper:

    def __init__(self, virtual=False, virtual_list=None):
        """
        This node 
        subscribes:
            odom from (moving object detection)
            (when running "nonVirtual" mode)
        publishs:
            moving obstacle info [2d-position, 2d-velocity, radius]=[pi, vi, ri]
        """
        if virtual_list is None:
            virtual_list = []
        self.mo_array_msg = MovObsInfoArray()
        self.mo1_msg = MovObsInfo()
        self.mo2_msg = MovObsInfo()
        self.mo_array = [self.mo1_msg, self.mo2_msg]
        self.mo1_ready = False
        self.mo2_ready = False

        if not virtual:
            # params of subscribed topics from launch file
            mo1_subject_name = rospy.get_param("~mo1_subject_name")
            mo2_subject_name = rospy.get_param("~mo2_subject_name")
            # generate Vicon odom topic name
            mo1_odom_topic = "/vicon/" + mo1_subject_name + "/odom"
            mo2_odom_topic = "/vicon/" + mo2_subject_name + "/odom"
            # subscribers
            self.mo1_odom_sub = rospy.Subscriber(mo1_odom_topic, Odometry, self.mo1_odom_callback)
            self.mo2_odom_sub = rospy.Subscriber(mo2_odom_topic, Odometry, self.mo2_odom_callback)
            print(mo1_odom_topic, mo2_odom_topic)
        else:
            self.virtual_odom(virtual_list)

        # publishers
        self.marker_pub = rospy.Publisher("/mov_obs_markers", MarkerArray, queue_size=1)
        self.moinfo_pub = rospy.Publisher("/mov_obs_info_array", MovObsInfoArray, queue_size=1)

        # params of moving obstacle radius form launch file
        self.r_mo1 = rospy.get_param("~mo1_radius")
        self.r_mo2 = rospy.get_param("~mo2_radius")
        self.mo1_msg.r_mo = self.r_mo1
        self.mo2_msg.r_mo = self.r_mo2

        self.virtual = virtual
        pass

    def mo1_odom_callback(self, mo1_msg_odom):
        mo1_pose = odom_to_pose2d(mo1_msg_odom)
        self.mo1_msg.p_mo.x = mo1_pose.x
        self.mo1_msg.p_mo.y = mo1_pose.y
        self.mo1_msg.p_mo.theta = mo1_pose.theta
        self.pose_mo1 = np.array([mo1_pose.x, mo1_pose.y, mo1_pose.theta])

        mo1_twist = mo1_msg_odom.twist.twist
        self.mo1_msg.v_mo.x = mo1_twist.linear.x
        self.mo1_msg.v_mo.y = mo1_twist.linear.y

        self.mo1_ready = True
        rospy.logwarn_once("[moving obstacle 1 info] Received odometry!")
        pass

    def mo2_odom_callback(self, mo2_msg_odom):
        mo2_pose = odom_to_pose2d(mo2_msg_odom)
        self.mo2_msg.p_mo.x = mo2_pose.x
        self.mo2_msg.p_mo.y = mo2_pose.y
        self.mo2_msg.p_mo.theta = mo2_pose.theta
        self.pose_mo2 = np.array([mo2_pose.x, mo2_pose.y, mo2_pose.theta])

        mo2_twist = mo2_msg_odom.twist.twist
        self.mo2_msg.v_mo.x = mo2_twist.linear.x
        self.mo2_msg.v_mo.y = mo2_twist.linear.y

        self.mo2_ready = True
        rospy.logwarn_once("[moving obstacle 2 info] Received odometry!")
        pass

    def virtual_odom(self, mo_state_list):
        """
        generate MovObsInfo msgs from virtual moving obstacle info list for publishers
        """
        mo1_state, mo2_state = mo_state_list[0], mo_state_list[1]

        self.mo1_msg.p_mo.x = mo1_state[0]
        self.mo1_msg.p_mo.y = mo1_state[1]
        self.mo1_msg.p_mo.theta = mo1_state[2]
        self.mo1_msg.v_mo.x = mo1_state[3]
        self.mo1_msg.v_mo.y = mo1_state[4]

        self.mo2_msg.p_mo.x = mo2_state[0]
        self.mo2_msg.p_mo.y = mo2_state[1]
        self.mo2_msg.p_mo.theta = mo2_state[2]
        self.mo2_msg.v_mo.x = mo2_state[3]
        self.mo2_msg.v_mo.y = mo2_state[4]

        self.pose_mo1 = mo1_state[0:3]
        self.pose_mo2 = mo2_state[0:3]

        self.mo1_ready = True
        self.mo2_ready = True
        rospy.logwarn_once("[virtual moving obstacle] Running!")

        pass

    def publish_mo_info(self, mo_state_list=None):
        """
        publish moving obstacle info
        """
        if mo_state_list is None:
            mo_state_list = []
        if self.virtual:
            self.virtual_odom(mo_state_list)

        self.mo_array_msg.movobsinfos = self.mo_array
        self.moinfo_pub.publish(self.mo_array_msg)
        return

    def init_marker(self, arrow_dl=0.5):
        """
        Initial markers to display moving obstacle set and pose in Rviz
        """
        self.arrow_l1 = self.r_mo1 + arrow_dl
        self.arrow_l2 = self.r_mo2 + arrow_dl
        p_mo1, p_mo2 = self.pose_mo1[0:2], self.pose_mo2[0:2]
        theta_mo1, theta_mo2 = self.pose_mo1[2], self.pose_mo2[2]
        pg_mo1 = p_mo1 + np.array([self.arrow_l1 * np.cos(theta_mo1), self.arrow_l1 * np.sin(theta_mo1)])
        pg_mo2 = p_mo2 + np.array([self.arrow_l2 * np.cos(theta_mo2), self.arrow_l2 * np.sin(theta_mo2)])
        r_mo1, r_mo2 = self.r_mo1, self.r_mo2

        self.mk_mo1set = create_cylinder2d_marker(id=121, disp_h=0.1, loc2d=p_mo1,
                                                  c='dimgray', alpha=0.8, r=r_mo1, ns='mo1_set')
        self.mk_mo1pose = create_pose_arrow_marker(id=122, disp_h=0.2, start2d=p_mo1, goal2d=pg_mo1, c='dimgray')

        self.mk_mo2set = create_cylinder2d_marker(id=131, disp_h=0.1, loc2d=p_mo2,
                                                  c='dimgray', alpha=0.8, r=r_mo2, ns='mo2_set')
        self.mk_mo2pose = create_pose_arrow_marker(id=132, disp_h=0.2, start2d=p_mo2, goal2d=pg_mo2, c='dimgray')

        self.mo_markers = [self.mk_mo1set, self.mk_mo1pose, self.mk_mo2set, self.mk_mo2pose]
        self.marker_pub.publish(self.mo_markers)
        return

    def update_marker(self):
        """
        Update non-static markers in Rviz
        """
        p_mo1, p_mo2 = self.pose_mo1[0:2], self.pose_mo2[0:2]
        theta_mo1, theta_mo2 = self.pose_mo1[2], self.pose_mo2[2]
        pg_mo1 = p_mo1 + np.array([self.arrow_l1 * np.cos(theta_mo1), self.arrow_l1 * np.sin(theta_mo1)])
        pg_mo2 = p_mo2 + np.array([self.arrow_l2 * np.cos(theta_mo2), self.arrow_l2 * np.sin(theta_mo2)])

        self.mk_mo1set.pose.position.x = p_mo1[0]
        self.mk_mo1set.pose.position.y = p_mo1[1]
        self.mk_mo1pose.points[0] = Point(p_mo1[0], p_mo1[1], 0.0)
        self.mk_mo1pose.points[1] = Point(pg_mo1[0], pg_mo1[1], 0.0)

        self.mk_mo2set.pose.position.x = p_mo2[0]
        self.mk_mo2set.pose.position.y = p_mo2[1]
        self.mk_mo2pose.points[0] = Point(p_mo2[0], p_mo2[1], 0.0)
        self.mk_mo2pose.points[1] = Point(pg_mo2[0], pg_mo2[1], 0.0)

        self.marker_pub.publish(self.mo_markers)
        return


if __name__ == '__main__':

    try:
        rospy.init_node("mov_obs_info")
        rospy.logwarn("bring up [mov_obs_info] node!\n")

        VirtualMode = rospy.get_param("~virtual", False)
        Marker_Init = False
        rate = rospy.Rate(100.0)

        if VirtualMode:
            # define moving obstacle start & goal positon, linear velocity
            t_idx = 0
            dt = 0.01
            t = t_idx * dt
            # load params from launch file
            p_start_mo1_x = rospy.get_param("~mo1_pstart_x")
            p_start_mo1_y = rospy.get_param("~mo1_pstart_y")
            p_start_mo2_x = rospy.get_param("~mo2_pstart_x")
            p_start_mo2_y = rospy.get_param("~mo2_pstart_y")
            p_goal_mo1_x = rospy.get_param("~mo1_pgoal_x")
            p_goal_mo1_y = rospy.get_param("~mo1_pgoal_y")
            p_goal_mo2_x = rospy.get_param("~mo2_pgoal_x")
            p_goal_mo2_y = rospy.get_param("~mo2_pgoal_y")

            p_start_mo1 = np.array([p_start_mo1_x, p_start_mo1_y])
            p_goal_mo1 = np.array([p_goal_mo1_x, p_goal_mo1_y])
            v_mo1 = rospy.get_param("~mo1_v")

            p_start_mo2 = np.array([p_start_mo2_x, p_start_mo2_y])
            p_goal_mo2 = np.array([p_goal_mo2_x, p_goal_mo2_y])
            v_mo2 = rospy.get_param("~mo2_v")

            mo1_info = [p_start_mo1, p_goal_mo1, v_mo1]
            mo2_info = [p_start_mo2, p_goal_mo2, v_mo2]
            MO1 = MovObsStates(mo1_info)
            MO2 = MovObsStates(mo2_info)

            mo1_state = MO1.get_moving_obstacle_state(t)
            mo2_state = MO2.get_moving_obstacle_state(t)
            mo_state_list = [mo1_state, mo2_state]
            mov_obs = MovObsWrapper(VirtualMode, mo_state_list)

        else:
            mov_obs = MovObsWrapper(VirtualMode)
            mov_obs.publish_mo_info()

        while not rospy.is_shutdown():
            if mov_obs.mo1_ready and mov_obs.mo2_ready and (not Marker_Init):
                rospy.logwarn("[mov_obs_info] ready!\n")
                mov_obs.init_marker()
                Marker_Init = True

            if VirtualMode:
                # publish virtual moving obstacle states
                t = t_idx * dt
                mo1_state = MO1.get_moving_obstacle_state(t)
                mo2_state = MO2.get_moving_obstacle_state(t)
                mo_state_list = [mo1_state, mo2_state]
                mov_obs.publish_mo_info(mo_state_list)
                t_idx += 1
            else:
                # print( "publish captured moving obstacle states")
                mov_obs.publish_mo_info()

            if Marker_Init:
                mov_obs.update_marker()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr("[mov_obs_info] node init failed.")
        rospy.signal_shutdown("[mov_obs_info] node init fail")
        pass
