#!/usr/bin/env python3

import numpy as np
import numpy.linalg as npla
import rospy
from geometry_msgs.msg import Pose2D, PointStamped
from nav_msgs.msg import Odometry
from ref_gvn_utils.ros_grid_map_utils import OccupancyGridMapPy

from erl_msgs.msg import RefGvnDist


class RefGvnSensing:
    def __init__(self, config_dict=None):
        """ Init class.

            This node subscribes:
                2d Occupancy Grid Map
                robot odometry 
                gvn pose2d
            Publish:
                distance info: d_Q(g, O), d_I(r, O)
        """
        # loading external config parameters
        self._config_dict = config_dict

        # load parameters 
        # ------------------- subscribers ----------------------
        # grid map sub 
        self.ogm = OccupancyGridMapPy(topic=config_dict['map_topic'])
        self.grid_map_sub = self.ogm._sub

        # robot state sub
        self.rbt_state_sub = rospy.Subscriber(self._config_dict['odom_topic'], Odometry, self.rbt_state_callback)

        # governor sub
        self.gvn_state_sub = rospy.Subscriber(self._config_dict['gvn_topic'], Pose2D, self.gvn_state_callback)

        # ------------------- publishers ---------------------- 
        # Ref Gvn dist information
        self.sense_pub = rospy.Publisher('~dist', RefGvnDist, queue_size=1)

        # show closest point d(g, O) \approx d(g, obstacle_grids),
        self.closest_pt_dg2occ_pub = rospy.Publisher('~closest_pt_dg2occ', PointStamped, queue_size=1)
        # show closest point d(r, O) \approx d(r, obstacle_grids),
        self.closest_pt_dr2occ_pub = rospy.Publisher('~closest_pt_dr2occ', PointStamped, queue_size=1)

        # -------------------- other status -------------------------
        self._closest_pt_to_rbt_ready = False
        self._closest_pt_to_gvn_ready = False

        # ------------------- Init Upstream --------------------
        self.init_containers()
        rospy.logwarn("[ref_gvn_sense] Init Done! -------------------- \n")

    def init_containers(self):
        """
        Init up/downstream containers.
        """
        # -------------------  down container --------------------
        self._ref_gvn_dist = RefGvnDist()

        # two closest points
        self._closest_pt_to_gvn = PointStamped()
        self._closest_pt_to_rbt = PointStamped()

        # hard corded for simplicity
        self._closest_pt_to_gvn.header.frame_id = "map"
        self._closest_pt_to_rbt.header.frame_id = "map"

    def gvn_state_callback(self, pose2d_msg):
        """
        Process governor state messages. Need transform pose2d_msg (in map frame) to lidar frame
        """
        rospy.logwarn_once("[ref_gvn_sense] get first governor pose2d message")
        # get closet pt to gvn 
        gvn_loc2d = np.array([pose2d_msg.x, pose2d_msg.y])

        if self.ogm.map_init_finished:
            obs_star_to_gvn = self.ogm.find_closest_obstacle_to_pt(pt_loc=gvn_loc2d)
            self._closest_pt_to_gvn.point.x = obs_star_to_gvn[0]
            self._closest_pt_to_gvn.point.y = obs_star_to_gvn[1]
            rospy.logwarn_throttle(1.0, "obs_star to gvn = [%.2f %.2f]" % (obs_star_to_gvn[0], obs_star_to_gvn[1]))
            self._ref_gvn_dist.dIgO = npla.norm(gvn_loc2d - obs_star_to_gvn)
            self.closest_pt_dg2occ_pub.publish(self._closest_pt_to_gvn)

        self._closest_pt_to_gvn_ready = True

    def rbt_state_callback(self, odom_msg):
        """
        Process robot odom messages. 
        """
        rospy.logwarn_once("[ref_gvn_sense] Get first robot odometry message")

        rbt_pos = odom_msg.pose.pose.position
        # get closet pt to gvn 
        rbt_loc2d = np.array([rbt_pos.x, rbt_pos.y])

        if self.ogm.map_init_finished:
            obs_star_to_rbt = self.ogm.find_closest_obstacle_to_pt(pt_loc=rbt_loc2d)
            self._closest_pt_to_rbt.point.x = obs_star_to_rbt[0]
            self._closest_pt_to_rbt.point.y = obs_star_to_rbt[1]
            rospy.logwarn_throttle(1.0, "obs_star to rbt = [%.2f %.2f]" % (obs_star_to_rbt[0], obs_star_to_rbt[1]))
            self._ref_gvn_dist.dIrO = npla.norm(rbt_loc2d - obs_star_to_rbt)
            self.closest_pt_dr2occ_pub.publish(self._closest_pt_to_rbt)

        self._closest_pt_to_rbt_ready = True

    def publish_sense_info(self):
        """
        Publish sensing information to ref governor node. 
        Currently, only have distance info.
        """
        if self._closest_pt_to_rbt_ready and self._closest_pt_to_gvn_ready:
            # distance info
            self.sense_pub.publish(self._ref_gvn_dist)

            rospy.logwarn_once("[ref_gvn_sense] publish msg")


if __name__ == '__main__':

    try:
        rospy.init_node('Ref Gvn Sensing')
        rospy.loginfo("[ref_gvn_sense] start node!\n")

        # loading parameters
        sensing_config = {
            "gvn_topic": rospy.get_param("~gvn_topic", "/gvn_state"),
            "odom_topic": rospy.get_param("~odom_topic", "/odom"),
            "map_topic": rospy.get_param("~map_topic", "/map"),
            "pub_freq": rospy.get_param("~pub_freq", 50.0),
        }

        rospy.logwarn_once("[ref_gvn_sense] config ------start-----------")
        print(sensing_config)
        rospy.logwarn_once("[ref_gvn_sense] config---------end---------")

        ref_gvn_sens = RefGvnSensing(config_dict=sensing_config)
        rate = rospy.Rate(sensing_config["pub_freq"])

        while not rospy.is_shutdown():
            # publish path at certain freq
            ref_gvn_sens.publish_sense_info()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr("[ref_gvn_sense] node init failed.")
        rospy.signal_shutdown('[ref_gvn_sense] node init fail')
        pass
