#!/usr/bin/env python3
""" 
Reference Governor Cone Distance ROS Wrapper.
"""

import rospy
from ref_gvn_utils.ref_gvn_cone_dist import RefGvnConeDist
from ref_gvn_utils.ref_gvn_bdcone_dist import RefGvnBDConeDist

if __name__ == '__main__':

    try:
        rospy.init_node('RefGvnConeDist')
        rospy.loginfo("[RefGvnConeDist] start node!\n")

        # loading parameters
        conedist_config = {
            "gvn_topic": rospy.get_param("~gvn_topic", "/gvn_state"),
            "odom_topic": rospy.get_param("~odom_topic", "/odom"),
            "dist_topic": rospy.get_param("~dist_topic", "/ref_gvn_dist"),
            "map_type": rospy.get_param("~map_type", "cost_map"),
            "costmap_topic": rospy.get_param("~costmap_topic", "/costmap"),
            "use_costmap": rospy.get_param("~use_costmap", True),
            "cost_map_unknown": rospy.get_param("~costmap_cost_unknown"),
            "cost_map_free_lb": rospy.get_param("~costmap_free_lb"),
            "cost_map_free_ub": rospy.get_param("~costmap_free_ub"),
            "cost_map_obstacle_lb": rospy.get_param("~costmap_obstacle_lb"),
            "cost_map_obstacle_ub": rospy.get_param("~costmap_obstacle_ub"),
            "pub_freq": rospy.get_param("~pub_freq"),
            "radius_offset": rospy.get_param("~radius_offset"),
            "bi_direction": rospy.get_param("~bi_direction", False),
            "act_lpg": rospy.get_param("~act_lpg", False),
        }

        rospy.logwarn_once("[RefGvnConeDist] config ------start--------------------------")
        print(conedist_config)
        rospy.logwarn_once("[RefGvnConeDist] config---------end---------")

        bi_direction = conedist_config["bi_direction"]
        if bi_direction:
            cone_dist = RefGvnBDConeDist(config_dict=conedist_config)
        else:
            cone_dist = RefGvnConeDist(config_dict=conedist_config)
        rate = rospy.Rate(conedist_config["pub_freq"])

        while not rospy.is_shutdown():
            cone_dist.publish_cone_info()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr("[RefGvnConeDist] node init failed.")
        rospy.signal_shutdown('[RefGvnConeDist] node init fail')
        pass
