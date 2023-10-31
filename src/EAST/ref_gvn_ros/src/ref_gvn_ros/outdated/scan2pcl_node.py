#!/usr/bin/env python3

import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection

class Laser2PCL:
    def __init__(self, config_dict=None):
        """ Init class.
            Transform laser 2d scan to pointcloud2
        """
        # loading external config parameters
        _scan_topic = rospy.get_param('~scan_topic')

        # ------------------- subscribers ----------------------
        # scan subscriber
        self._scan_sub = rospy.Subscriber(_scan_topic, LaserScan, self.scan_callback, queue_size=1)
        self._laser_projector = LaserProjection()

        # ------------------- publishers ---------------------- 
        # publist 2d point cloud
        self._pcl_pub = rospy.Publisher("/pcl", PointCloud2, queue_size=1)

        # -------------------- other status -------------------------

    def scan_callback(self, scan_msg):
        rospy.logdebug("Received LaserScan!")
        cloud_out = self._laser_projector.projectLaser(scan_msg)
        self._pcl_pub.publish(cloud_out)
        return

if __name__ == '__main__':

    try:
        rospy.init_node('laser2pcl')
        rospy.logwarn("bring up [laser2pcl] node!\n")
        scan2pcl = Laser2PCL()
        rate = rospy.Rate(50.0)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("[laser2pcl] node init failed.")
        rospy.signal_shutdown('[laser2pcl] node init fail')
        pass
