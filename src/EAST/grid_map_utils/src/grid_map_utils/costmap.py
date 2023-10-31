#!/usr/bin/env python3

"""
ROS utility function custom costmap. 
"""

import os
import numpy as np

import cv2
import pickle as pkl

import rospy
from nav_msgs.msg import OccupancyGrid

from grid_map_utils.grid_map_dist import GridMapDist
from geometry_msgs.msg import PoseStamped
import time


class CostmapError(Exception):
    """ User Defined Exceptions for CostmapError Class.
    """

    def __init__(self, *args):
        if args:
            self.msg = args[0]
        else:
            self.msg = ""

    def __str__(self):
        if self.msg:
            return "CostmapError exception: {0}".format(self.msg)
        else:
            return "CostmapError exception"


class Costmap(GridMapDist):
    """ 
    Class for Costmap Utility Functions.
    """

    def __init__(self, config_dict):
        """ 
        Init class for costmap. 
        """
        # super class init
        super(Costmap, self).__init__(config_dict)

        # load parameters needed for costmap utils -----------------------------
        self.load_costmap_params(config_dict)
        
        # class status ---------------------------------------------------------
        self.costmap_init_finished = False
        self.costmap_msg_ready = False

        # subscribers ----------------------------------------------------------
        self._costmap_trigger_sub = rospy.Subscriber(self._map_in, OccupancyGrid, self.grid_msg_callback, queue_size=2)
        # publisher ------------------------------------------------------------
        self.costmap_pub = rospy.Publisher('/costmap', OccupancyGrid, queue_size=1)

        # cached large containers ----------------------------------------------
        self._costmap_header = None
        self._costmap_data = None
        self._distmap2d = None 
        self._costmap_msg = None
        self.previous_costmap_msg_published_time = time.time()
        
        # this derived class init ----------------------------------------------
        rospy.logwarn_once("[Costmap | costmap init start...]")
        self.costmap_init()

        # optional subscribers 
        if self._click_save_costmap:
            self._button_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.click_save_costmap_callback)
        # optional publisher 

        rospy.logwarn_once("[Costmap | costmap init finished!]")

    def load_costmap_classification_critical_values(self):
        """
        Load critical values for costmap classification from config dict. 
        """
        try:
            self._costmap_gamma = self.config_dict['costmap_gamma']
            self._costmap_cost_unknown = self.config_dict['costmap_cost_unknown']
            self._costmap_cost_lethal = self.config_dict['costmap_cost_lethal']
            self._costmap_cost_inscribed = self.config_dict['costmap_cost_inscribed']
            self._costmap_planning_cutoff_cost = self.config_dict['costmap_planning_cutoff_cost']
        except KeyError:
            raise CostmapError("You need to define custom critical values for costmap")

    def load_costmap_params(self, config_dict):
        """
        Load parameters relates to costmap computation.
        """
        self.config_dict = config_dict
        # for class debug purpose, click 2D Nav Button to save latest costmap to ./tmp_data
        if "click_save_costmap" in self.config_dict:
            self._click_save_costmap = self.config_dict['click_save_costmap']
        else:
            self._click_save_costmap = False
        
        # load critical value for grid cell type classification, unknown, free, obstacle
        self.load_costmap_classification_critical_values()
        # init values for costmap computation
        self.get_robot_params_for_costmap_computation()

    def set_costmap_meta_info(self, copy_grid_meta_info=True):
        """
        Set costmap meta info. 
        In future, costmap meta_info can differ from grid map. 
        i.e., different size and resolution. 
        """
        if copy_grid_meta_info:
            self._costmap_info = self._map_info
            self._costmap_height = self._map_height
            self._costmap_width = self._map_width
            if self._costmap_height % 2 == 0 or self._costmap_height % 2 == 0:
                raise CostmapError("Costmap size in each dimension must be odd number for erl A* planner")

            # in meter
            self._costmap_res = self._map_res 
            self._costmap_origin_x = self._map_origin_x
            self._costmap_origin_y = self._map_origin_y
        else:
            raise CostmapError("Not Implemented Local Costmap")
    
    def get_robot_params_for_costmap_computation(self):
        """
        For each robot, costmap critical values must be specified in advance. 
        """
        try:
            self.rbt_type = self.config_dict['rbt_type']
        except KeyError:
            raise CostmapError("You need to specify robot type")
        
        # default round costmap computation safely to map resolution
        if "costmap_round_up_to_map_res" in self.config_dict:
            self._costmap_round_up_to_map_res = self.config_dict['costmap_round_up_to_map_res']
        else:
            self._costmap_round_up_to_map_res = False

        # default round costmap computation safely to map resolution
        if "planning_allowed_in_unknown" in self.config_dict:
            self._planning_allowed_in_unknown = self.config_dict['planning_allowed_in_unknown']
        else:
            self._planning_allowed_in_unknown = True

        rospy.logwarn_once("planning in unknown space is allowed %s" % self._planning_allowed_in_unknown)

        if self.rbt_type == "Jackal01":
            # Jackal Constants
            # https://www.notion.so/ERL-Jackal-Robot-Manuals-9eee0ff9b908439cbadc94fbb6f5cee8
            JACKAL_INSCRIBED_RADIUS = 0.430 / 2
            # JACKAL_CIRCUMSCRIBED_RADIUS = 0.508 / 2        
            map_res = self._map_res        
            # radii non-pixelated
            radius_inscribed = JACKAL_INSCRIBED_RADIUS
        else:
            return CostmapError("costmap for robot type %s has not been designed" % self.rbt_type)
            
        # round up critical distances to multiple of map resolution
        if self._costmap_round_up_to_map_res:
            self._costmap_radius_r0 = map_res
            self._costmap_radius_r1 = self.round_dist_up(radius_inscribed)
        else:
            self._costmap_radius_r0 = map_res / 2.0
            self._costmap_radius_r1 = radius_inscribed

    def costmap_init(self):
        """
        Init process for costmap utils.
        """
        # wait obstacle detected in grid map
        while not self.dist_init_finished:
            rospy.logwarn_throttle(1.0, "[costmap] waiting map init...")
            time.sleep(0.1)
            rospy.sleep(0.1)

        # set costmap meta info
        self.set_costmap_meta_info()
        
        # init costmap header and reference frame
        self._costmap_header = self._map_header
        self._costmap_reference_frame = self._map_header.frame_id
        rospy.logwarn_once("[costmap | costmap_init ready]")
        self.costmap_init_finished = True

    def click_save_costmap_callback(self, msg): # noqa
        """
        Click rviz 2D Nav Button to save costmap 
        """
        rospy.logwarn("[Costmap] Button clicked")

        if not self.costmap_msg_ready:
            rospy.logwarn_once("[costmap | click_save_costmap_callback] waiting costmap msg init...")
            return 
        
        # else case
        fd_base = os.path.dirname(os.path.realpath(__file__))
        log_fd = fd_base + "/../../tmp_data/"
        if not os.path.exists(log_fd):
            os.makedirs(log_fd)
        pkl_full_path = os.path.join(log_fd, 'debug_costmap' + ".pkl")
        rospy.logwarn("[click & save | save costmap map to %s]" % pkl_full_path)

        res_log = {
            "map_data": self._map_data,
            "costmap_data": self._costmap_data,
            "map_metainfo": self._costmap_info,
            "map_resolution": self._costmap_res,
            "map_origin_x": self._costmap_origin_x,
            "map_origin_y": self._costmap_origin_y,
            "map_height": self._costmap_height,
            "map_width": self._costmap_width,
            "costmap_gamma": self._costmap_gamma,
            "costmap_cost_lethal": self._costmap_cost_lethal,
            "costmap_cost_inscribed": self._costmap_cost_inscribed,
            "robot_type": self.rbt_type,
        }

        with open(pkl_full_path, "wb") as f: 
            pkl.dump(res_log, f)
        
        rospy.logwarn("[click & save | costmap saved!")        

    def round_dist_up(self, d):
        """ Round distance up map resolution. 
        """
        return (np.floor(d / self._map_res) + 1) * self._map_res
    
    def compute_distance_map(self):
        """
        Compute distance field to obstacle. 
        """
        if not self._planning_allowed_in_unknown:
            # notify end_user that we are allowed planning in unknown area
            err_msg = "Only support planning_allowed_in_unknown=True"
            rospy.signal_shutdown(err_msg)
            raise CostmapError(err_msg)
        
        if self._map_type == "hector_map":
            obs_lb = self._grid_map_obstacle_lb
            obs_ub = self._grid_map_obstacle_ub
        else:
            err_msg = "Only support map_type='hector_map'"
            rospy.signal_shutdown(err_msg)
            raise CostmapError("Unknown input map type %s" % self._map_type)

        # shallow copy do not modify this 
        raw = self._map2d_raw
        threshold = np.ones(raw.shape, dtype=np.uint8) * 255
        # set costmap, obstacle cell value as 0
        threshold[(raw >= obs_lb) & (raw <= obs_ub)] = 0
        # distanceTransform will compute pixel distance to closest zero-value pixels
        distTransform = cv2.distanceTransform(threshold, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        self._distmap2d = distTransform * self._map_res
        
    def compute_costmap1d(self):
        """ 
        Use hard corded costmap design to transform distance map to costmap.
        """

        # get distance map 2d first
        self.compute_distance_map()
        distmap1d = self._distmap2d.flatten()

        r0 = self._costmap_radius_r0
        r1 = self._costmap_radius_r1
        # compute cost according to distance map
        costmap1d = self._costmap_cost_inscribed * np.exp(-self._costmap_gamma * (distmap1d - r1))
        
        # overwrite region less or equal to r1
        costmap1d[distmap1d <= r0] = self._costmap_cost_lethal
        costmap1d[(distmap1d <= r1) & (distmap1d > r0)] = self._costmap_cost_inscribed

        # handle unknown cost separately if cost_unknown not equal 0
        if self._costmap_cost_unknown != 0:
            costmap1d[self._map_data == self._grid_map_unknown] = self._costmap_cost_unknown

        self._costmap_data = np.array(costmap1d, dtype=np.int8)
        self._costmap1d = costmap1d
        self.costmap_msg_ready = True
    
    def grid_msg_callback(self, msg):
        """
        Use grid msg to trigger costmap computation.
        """
        if self.costmap_init_finished:
            self._costmap_header = msg.header
            self.compute_costmap1d()
            self.costmap_msg_ready = True
            rospy.logwarn_once('[costmap | costmap msg ready]')
            self._costmap_msg = OccupancyGrid()
            self._costmap_msg.info = self._costmap_info
            self._costmap_msg.header.frame_id = self._costmap_reference_frame
            self._costmap_msg.data = self._costmap_data
            self.costmap_pub.publish(self._costmap_msg)
