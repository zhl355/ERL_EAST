#!/usr/bin/env python3

"""
ROS utility function for saving path message. 
"""

import numpy as np
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import pickle as pkl
import os 
import time


class ClickSaver(object):
    """ 
    Class for OccupancyGrid in ROS.
    """

    def __init__(self):
        """ 
        Init base class. 
        """
        # load parameters needed for base class --------------------------------
        self.path_topic = rospy.get_param("~path_topic", "/path")
        self._output_filename = rospy.get_param("~output_filename", "debug_path") + ".npz"

        rospy.logwarn_once("ClickSave %s to %s" % (self.path_topic, self._output_filename))

        # subscribers ----------------------------------------------------------
        # path sub, this sub trigger init process
        self._path_msg_sub = rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)
        
        # Nav button sub
        self._button_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.click_save_callback)
                
        # publisher ------------------------------------------------------------
        
        # cached large containers ----------------------------------------------
        self._np_path = None
        

        rospy.logwarn_once("[ClickSaver init started...]")
        self._first_msg_received = False
        self._new_request = False
        self.base_init()

        rospy.logwarn_once("[ClickSaver init finished!]")

    def base_init(self):
        """
        Init ClickSaver class.
        """
        # wait callback to trigger init process
        while not self._first_msg_received:
            rospy.logwarn_once("waiting first path msg to trigger ClickSaver init...")

            
    def click_save_callback(self, msg):
        """
        Click rviz 2D Nav Button to save grid msg to pickle file. 
        msg: ROS subscriber interface compliance.
        """
        rospy.logwarn("[ClickSaver] Button clicked")
        if not self._first_msg_received:
            rospy.logwarn("[ClickSaver] path has not been received!")
            return 
        
        self._new_request = True


    def path_callback(self, msg:Path):
        """ 
        Path msg callback. Init or update data container.
        """
        self._first_msg_received = True

        path_dim = 2
        rospy.loginfo_throttle(5, "[click & save] received path")
        planning_path = np.zeros((len(msg.poses), path_dim))
        for idx, pose in enumerate(msg.poses):
            planning_path[idx, 0] = pose.pose.position.x
            planning_path[idx, 1] = pose.pose.position.y

        # assume path accuracy multiple of 1 cm, round to 2 precision
        self._np_path = np.round(np.array(planning_path), 2)
        
        if self._new_request:
            fd_base = os.path.dirname(os.path.realpath(__file__))
            log_fd = fd_base + "/../../tmp_data/"
            if not os.path.exists(log_fd):
                os.makedirs(log_fd)
            full_path = os.path.join(log_fd, self._output_filename)
            rospy.logwarn("[click & save | saving msg to %s]" % full_path)
            print('save path from [%.2f %.2f] to [%.2f, %.2f]' % (
                self._np_path[0, 0], self._np_path[0, 1], 
                self._np_path[-1, 0], self._np_path[-1, 1]))
            np.savez(full_path, path=self._np_path)
            rospy.logwarn("[click & save | msg saved!")
            self._new_request = False
            

if __name__ == '__main__':

    try:
        rospy.init_node('path saver')
        rospy.loginfo("[test click and save] start node!\n")


        path_saver = ClickSaver()
        rate = rospy.Rate(50.0)

        while not rospy.is_shutdown():
            rospy.logwarn_once("Click 2D Nav Button to save path to npz file")
            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.logerr("[ClickSavePath] node init failed.")
        rospy.signal_shutdown('[ClickSavePath] node init fail')
        pass
