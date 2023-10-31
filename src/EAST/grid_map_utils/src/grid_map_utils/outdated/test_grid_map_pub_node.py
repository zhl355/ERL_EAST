#!/usr/bin/env python3

import os
import rospy

from grid_map_utils.grid_map_publisher import GridMapPub

def get_filepath(map_name='fah'):
    fd_base = os.path.dirname(os.path.realpath(__file__))
    log_fd = fd_base + "/../../data/"
    if not os.path.exists(log_fd):
        os.makedirs(log_fd)
    filepath = os.path.join(log_fd, map_name + '.pkl')
    return filepath


if __name__ == '__main__':


    try:
        rospy.init_node('Test GridMapPub')

        # loading parameters
        map_gen_config = {}
        map_gen_config["map_name"] = rospy.get_param("~map_name", "fah")
        map_gen_config["use_dummy_odom"] = rospy.get_param("~use_dummy_odom", "False")
        test_map_filepath = get_filepath(map_name=map_gen_config["map_name"])
        map_gen_config["test_map_filepath"] = test_map_filepath
     
        rospy.logwarn_once("[GridMapPub] config ------start-----------")
        print(map_gen_config)

        print('Publish map %s' % map_gen_config["map_name"])
        rospy.logwarn_once("[GridMapPub] config---------end---------")


        grid_map_gen = GridMapPub(config_dict=map_gen_config)
        rate = rospy.Rate(20.0)

        while not rospy.is_shutdown():
            rospy.logwarn_once("Publish saved map %s" % test_map_filepath)
            grid_map_gen.publish()
            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.logerr("[GridMapPub] node init failed.")
        rospy.signal_shutdown('[GridMapPub] node init fail')
        pass
