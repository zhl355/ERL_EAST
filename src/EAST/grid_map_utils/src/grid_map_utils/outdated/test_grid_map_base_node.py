#!/usr/bin/env python3

import rospy
from grid_map_utils.grid_map_base import GridMapBase

if __name__ == '__main__':

    try:
        rospy.init_node('Test GridMapBase click and save')
        rospy.loginfo("[test click and save] start node!\n")

        # loading parameters
        gridmap_config = {}
        gridmap_config["map_topic"] = rospy.get_param("~map_topic", "/map")
        gridmap_config["click_save_grid_map"] = rospy.get_param("~click_save_grid_map", "False")
                        
        rospy.logwarn_once("[ClickSaveGridMap] config ------start-----------")
        print(gridmap_config)
        rospy.logwarn_once("[ClickSaveGridMap] config---------end---------")

        test = GridMapBase(config_dict=gridmap_config)
        rate = rospy.Rate(50.0)

        while not rospy.is_shutdown():
            rospy.logwarn_once("Click 2D Nav Button to save map to pkl file")
            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.logerr("[ClickSaveGridMap] node init failed.")
        rospy.signal_shutdown('[ClickSaveGridMap] node init fail')
        pass
