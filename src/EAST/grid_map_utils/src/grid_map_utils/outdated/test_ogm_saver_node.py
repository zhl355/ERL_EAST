#!/usr/bin/env python3

import rospy
from grid_map_utils.ogm_saver import OccupancyMsgSaver

if __name__ == '__main__':

    try:
        rospy.init_node('Test GridMapBase click and save')
        rospy.loginfo("[test click and save] start node!\n")

        # loading parameters
        ogm_saver_config = {}
        ogm_saver_config["ogm_topic"] = rospy.get_param("~ogm_topic")
                        
        rospy.logwarn_once("[ClickSaveGridMap] config ------start-----------")
        print(ogm_saver_config)
        rospy.logwarn_once("[ClickSaveGridMap] config---------end---------")

        ogm_saver = OccupancyMsgSaver(config_dict=ogm_saver_config)
        rate = rospy.Rate(50.0)

        while not rospy.is_shutdown():
            rospy.logwarn_once("Click 2D Nav Button to save map to pkl file")
            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.logerr("[ClickSaveGridMap] node init failed.")
        rospy.signal_shutdown('[ClickSaveGridMap] node init fail')
        pass
