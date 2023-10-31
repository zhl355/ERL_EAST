#!/usr/bin/env python3

"""
ROS utility function for grid map message. 
    1. Load msg meta info and data to class fields. 
    2. update class container from the latest occupancy grid message.
"""
from typing import Union, Any

import numpy as np
from numpy import ndarray

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import pickle as pkl
import os 
import time


class GridMapBaseError(Exception):
    """ User Defined Exceptions for GridMapBase Class.
    """

    def __init__(self, *args):
        if args:
            self.msg = args[0]
        else:
            self.msg = ""

    def __str__(self):
        if self.msg:
            return "GridMapBase exception: {0}".format(self.msg)
        else:
            return "GridMapBase exception"


class GridMapBase(object):
    """ 
    Class for OccupancyGrid in ROS.
    """
    _reference_frame: str
    _grid_origin: ndarray
    _grid_res: float
    _dim: int

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
        rospy.logwarn_once("[GridMapBase init started...]")
        self.base_init()

        # optional subscribers -------------------------------------------------
        if self._click_save_grid_map:
            self._button_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.click_save_callback)
        # optional publisher ---------------------------------------------------

        rospy.logwarn_once("[GridMapBase init finished!]")        

    def base_init(self):
        """
        Init GridMapBase class.
        """
        # wait grid map callback to trigger grid map init process
        while not self.grid_map_init_finished:
            rospy.logwarn_once("waiting first grid map msg to trigger GripMapBase init...")
            time.sleep(0.1)
            rospy.sleep(0.1)
        
    def _load_params(self, config_dict):
        """
        Load parameters from external node wrapper. 
        """
        self.config_dict = config_dict
        # get required params from config dictionary ---------------------------
        if "map_topic" in config_dict:
            self._map_in = config_dict["map_topic"]
        else:
            raise GridMapBaseError("[GridMapBase | required params missing ]")
        
        # set up optional params -----------------------------------------------
        if "click_save_grid_map" in config_dict:
            self._click_save_grid_map = config_dict['click_save_grid_map']
        else:
            self._click_save_grid_map = False

        if "debug_output_filename" in config_dict:
            self._debug_output_filename = config_dict['debug_output_filename']
        else:
            self._debug_output_filename = 'debug_grid_msg.pkl'
            
    def _set_meta_info(self, map_meta):
        """ 
        Init map meta info.
        https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/MapMetaData.html
        """
        # OccupancyGrid starts on lower left corner
        # The map data, in row-major order, starting with (0,0). 
        self._map_info = map_meta
        self._map_height = map_meta.height
        self._map_width = map_meta.width

        if self._map_height % 2 == 0 or self._map_width % 2 == 0:
            raise GridMapBaseError("GridMap size in each dimension must be odd number for erl A* planner")

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
            self._grid_res = np.round(self._map_res, 3)  # assuming finest map resolution 1mm
            self._grid_origin = np.array([self._map_origin_x, self._map_origin_y])
            self._reference_frame = msg.header.frame_id   
            rospy.logwarn_once("[GridMapBase | grid map init finished]")
            self.grid_map_init_finished = True
            
    def click_save_callback(self):
        """
        Click rviz 2D Nav Button to save grid msg to pickle file. 
        msg: ROS subscriber interface compliance.
        """
        rospy.logwarn("[GridMapBase] Button clicked")
        if not self._first_map_msg_received:
            rospy.logwarn("[GridMapBase] map has not been received!")
            return 
        
        # map received case 
        fd_base = os.path.dirname(os.path.realpath(__file__))
        log_fd = fd_base + "/../../tmp_data/"
        if not os.path.exists(log_fd):
            os.makedirs(log_fd)
        pkl_full_path = os.path.join(log_fd, self._debug_output_filename)
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
