#!/usr/bin/env python3
"""
A simple pose2d publisher. Debug purpose
"""
import numpy as np 
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose2D



class DummyGvn:
    def __init__(self, config_dict=None):
        """ Init dummy gvn class.
            Publish:
                hard-coded pose2d message
        """
        # ------------------- publishers ---------------------- 
        self.gvn_pub = rospy.Publisher('/gvn_state', Pose2D, queue_size=1)
        self._gvn = Pose2D()
        self._gvn.x = 1
    
    def publish_gvn_state(self):
        """
        Publish cached goal if goal received. 
        """
        self.gvn_pub.publish(self._gvn)

if __name__ == '__main__':

    try:
        rospy.init_node('dummy_gvn')
        rospy.logwarn("bring up [dummy_gvn] node!\n")

        # loading parameters
        rate = rospy.Rate(50.0)
        dummy_gvn = DummyGvn()

        while not rospy.is_shutdown():
            # publish path at certain freq
            dummy_gvn.publish_gvn_state()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr("[dummy_gvn] node init failed.")
        rospy.signal_shutdown('[dummy_gvn] node init fail')
        pass
