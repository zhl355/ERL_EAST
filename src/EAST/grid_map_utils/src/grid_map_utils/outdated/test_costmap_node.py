#!/usr/bin/env python3


import rospy
from grid_map_utils.costmap import Costmap

if __name__ == '__main__':

    try:
        rospy.init_node('Test Costmap')
        rospy.loginfo("[costmap] start node!\n")

        # loading parameters
        costmap_config = {}
        costmap_config["map_topic"] = rospy.get_param("~map_topic", "/map")
        costmap_config["pub_freq"] = rospy.get_param("~pub_freq", 50.0)

        # grid map config
        costmap_config["click_save_grid_map"] = rospy.get_param("~click_save_grid_map", False)

        # costmap parameters
        costmap_config["map_type"] = rospy.get_param("~map_type")
        costmap_config["rbt_type"] = rospy.get_param("~rbt_type")

        # do not use default value, it makes debug hard
        costmap_config["costmap_gamma"] = rospy.get_param("~costmap_gamma")
        costmap_config["costmap_cost_unknown"] = rospy.get_param("~costmap_cost_unknown")
        costmap_config["costmap_cost_lethal"] = rospy.get_param("~costmap_cost_lethal")
        costmap_config["costmap_cost_inscribed"] = rospy.get_param("~costmap_cost_inscribed")
        costmap_config["costmap_planning_cutoff_cost"] = rospy.get_param("~costmap_planning_cutoff_cost")
        costmap_config["check_gvn_on_unknown"] = rospy.get_param("~check_gvn_on_unknown", False)

        # whether to round up critical distance threshold to map resolution in cost computation.
        costmap_config["costmap_round_up_to_map_res"] = rospy.get_param("~costmap_round_up_to_map_res", False)
        
        
        rospy.logwarn_once("[costmap] config ------start-----------")
        print(costmap_config)
        rospy.logwarn_once("[costmap] config---------end---------")

        costmap_gen = Costmap(config_dict=costmap_config)
        rate = rospy.Rate(costmap_config["pub_freq"])

        while not rospy.is_shutdown():
            # costmap_gen.publish()
            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.logerr("[costmap] node init failed.")
        rospy.signal_shutdown('[costmap] node init fail')
        pass
