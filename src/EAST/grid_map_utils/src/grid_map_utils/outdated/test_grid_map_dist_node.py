#!/usr/bin/env python3

import rospy
from grid_map_utils.grid_map_dist import GridMapDist


if __name__ == '__main__':

    try:
        rospy.init_node('Test GridMapDist')
        rospy.loginfo("[Test GridMapDist] start node!\n")

        # loading parameters
        gridmap_config = {}
        gridmap_config["map_topic"] = rospy.get_param("~map_topic", "/map")
        gridmap_config["click_test"] = rospy.get_param("~click_test", "False")
        gridmap_config["map_type"] = rospy.get_param("~map_type")
                        
        rospy.logwarn_once("[Test GridMapDist] config ------start--------------------------")
        print(gridmap_config)
        rospy.logwarn_once("[Test GridMapDist] config---------end---------")

        test = GridMapDist(config_dict=gridmap_config)
        rate = rospy.Rate(50.0)

        while not rospy.is_shutdown():
            rospy.logwarn_once("Click 2D Nav Button to Test")
            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.logerr("[Test GridMapDist] node init failed.")
        rospy.signal_shutdown('[Test GridMapDist] node init fail')
        pass
