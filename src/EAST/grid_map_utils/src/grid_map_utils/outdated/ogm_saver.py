#!/usr/bin/env python3

"""
ROS utility function for saving external grid map message. 
    1. Load msg meta info and data to class fields. 
    2. update class container from latest occupancy grid message.
"""



import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import pickle as pkl
import os 
import time

class OccupancyMsgSaverError(Exception):
    """ User Defined Exceptions for OccupancyMsgSaver Class.
    """

    def __init__(self, *args):
        if args:
            self.msg = args[0]
        else:
            self.msg = ""

    def __str__(self):
        if self.msg:
            return "OccupancyMsgSaver exception: {0}".format(self.msg)
        else:
            return "OccupancyMsgSaver exception"

class OccupancyMsgSaver(object):
    """ 
    Class for OccupancyGrid in ROS.
    """

    def __init__(self, config_dict):
        """ 
        Init base class. 
        """
        # load parameters needed for base class --------------------------------
        self._load_params(config_dict)

        # class status ---------------------------------------------------------
        self._first_map_msg_received = False
        # class init finished flag
        self.grid_map_init_finished = False

        # subscribers ----------------------------------------------------------
        # this sub trigger init process
        self._grid_msg_sub = rospy.Subscriber(self._map_in, OccupancyGrid, self.grid_map_callback, queue_size=1)
        
        # publisher ------------------------------------------------------------
        
        # cached large containers ----------------------------------------------
        self._map_header = None
        self._map_data = None
        self._map2d_raw = None

        # this base class init ----------------------------------------------
        rospy.logwarn_once("[OccupancyMsgSaver init started...]")
        self.base_init()

        self._button_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.click_save_callback)

        rospy.logwarn_once("[OccupancyMsgSaver init finished!]")

    def base_init(self):
        """
        Init OccupancyMsgSaver class.
        """
        # wait grid map callback to trigger grid map init process
        while not self.grid_map_init_finished:
            rospy.logwarn_once("waiting first grid map msg to trigger OccupancyMsgSaver init...")
            time.sleep(0.1)
            rospy.sleep(0.1)
        
    def _load_params(self, config_dict):
        """
        Load parameters from external node wrapper. 
        """
        self.config_dict = config_dict
        # get required params from config dictionary ---------------------------
        if "ogm_topic" in config_dict:
            self._map_in = config_dict["ogm_topic"]
        else:
            raise OccupancyMsgSaverError("[OccupancyMsgSaver | required params 'ogm_topic' missing ]")
        
        self._click_save_grid_map = True
        self._output_filename = 'debug_ogm_msg.pkl'
                        
    def _set_meta_info(self, map_meta):
        """ 
        Init map meta info.
        http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/MapMetaData.html
        """
        # OccupancyGrid starts on lower left corner
        # The map data, in row-major order, starting with (0,0). 
        self._map_info = map_meta
        self._map_height = map_meta.height
        self._map_width = map_meta.width

        # in meter
        self._map_res = map_meta.resolution
        self._map_origin_x = map_meta.origin.position.x
        self._map_origin_y = map_meta.origin.position.y

    def grid_map_init(self, msg):
        """
        Init occupancy grid map. Fill in meta information and map data container.
        """
        if not self.grid_map_init_finished:
            self._dim = 2
            self._set_meta_info(msg.info)
            self._grid_res = np.round(self._map_res, 3) # assuming finest map resolution 1mm
            self._grid_origin = np.array([self._map_origin_x, self._map_origin_y])
            self._reference_frame = msg.header.frame_id   
            rospy.logwarn_once("[OccupancyMsgSaver | grid map init finished]")
            self.grid_map_init_finished = True
            
    def click_save_callback(self, msg):
        """
        Click rviz 2D Nav Button to save grid msg to pickle file. 
        msg: ROS subscriber interface compliance.
        """
        rospy.logwarn("[OccupancyMsgSaver] Button clicked")
        if not self._first_map_msg_received:
            rospy.logwarn("[OccupancyMsgSaver] map has not been received!")
            return 
        
        # map received case 
        fd_base = os.path.dirname(os.path.realpath(__file__))
        log_fd = fd_base + "/../../tmp_data/"
        if not os.path.exists(log_fd):
            os.makedirs(log_fd)
        pkl_full_path = os.path.join(log_fd, self._output_filename)
        rospy.logwarn("[click & save | save grid msg to %s]" % pkl_full_path)
        res_log = {
            "map_data": self._map_data,
            "map_metainfo": self._map_info,
            "map_resolution": self._map_res,
            "map_origin_x": self._map_origin_x,
            "map_origin_y": self._map_origin_y,
            "map_height": self._map_height,
            "map_width": self._map_width,
            "map_frame": self._reference_frame,
        }

        with open(pkl_full_path, "wb") as f: 
            pkl.dump(res_log, f)
        
        rospy.logwarn("[click & save | map saved!")

    def grid_map_callback(self, msg):
        """ 
        Grid map callback. Init or update data container.
        """
        self._map_header = msg.header
        self._map_data = np.array(msg.data, dtype=np.int8)
        self._map2d_raw = self._map_data.reshape(msg.info.height, msg.info.width)
        self._first_map_msg_received = True

        if not self.grid_map_init_finished:
            self.grid_map_init(msg)
            

