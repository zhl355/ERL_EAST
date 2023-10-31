#!/usr/bin/env python3

"""
ROS utility function for grid map message. 
    1. Load msg meta info and data to class fields. 
    2. update class container from the latest occupancy grid message.
"""


import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import pickle as pkl
import os
from ref_gvn_utils.ros_gvn_marker_utils import create_start_marker
from visualization_msgs.msg import Marker


class GridMapPubError(Exception):
    """
    User Defined Exceptions for GridMapPub Class.
    """

    def __init__(self, *args):
        if args:
            self.msg = args[0]
        else:
            self.msg = ""

    def __str__(self):
        if self.msg:
            return "GridMapPub exception: {0}".format(self.msg)
        else:
            return "GridMapPub exception"


class GridMapPub:
    """ 
    Class for Publishing save map message. 
    """

    def __init__(self, config_dict):
        """ 
        Init base class. 
          @config_dict: config dictionary
        """
        # load parameters needed for base class --------------------------------
        map_path = config_dict['test_map_filepath']
        self.use_dummy_odom = config_dict['use_dummy_odom']
        # class status ---------------------------------------------------------
        # class init finished flag
        self.grid_map_loaded = False
        self.grid_map_msg_ready = False
        
        # subscribers ----------------------------------------------------------
        
        # publisher ------------------------------------------------------------
        self.grid_map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)

        # cached large containers ----------------------------------------------
        self._map_data = None
        self.grid_msg = None
        self._map_metainfo = None
        self._filename = None
        self._res_dict = None
        self._map_frame = None

        rospy.logwarn_once("[GridMapPub init started...]")
        # this base class init -------------------------------------------------
        self.grid_map_init(filepath=map_path)
        # optional subscribers -------------------------------------------------

        # optional publisher ---------------------------------------------------
        if self.use_dummy_odom:
            self.dummy_odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
            self.dummy_start_mk_pub = rospy.Publisher('~dummy_start', Marker, queue_size=1)
            self.odom_msg = Odometry()
            self.odom_msg.header.frame_id = "/map"

            loc_2d = [-1.0, 0.0]
            self.odom_msg.pose.pose.orientation.w = 1.0
            self.odom_msg.pose.pose.position.x = loc_2d[0]
            self.odom_msg.pose.pose.position.y = loc_2d[1]

            # start marker
            self.start_mk = create_start_marker(loc2d=loc_2d)

        rospy.logwarn_once("[GridMapPub init finished!]")        

    def load_map(self, filepath):
        """
        Load map from pickle file.
        """
        self._filename = filepath
        # unpack log file
        if os.path.isfile(self._filename):
            with open(self._filename, "rb") as f:
                res_dict = pkl.load(f, encoding='latin1')
        else:
            err_msg = str(self._filename) + " not exists!"
            raise GridMapPubError(err_msg)

        self._res_dict = res_dict
        self._map_data = res_dict['map_data']
        self._map_metainfo = res_dict['map_metainfo']

        if 'map_frame' in res_dict:
            self._map_frame = res_dict['map_frame']
        else:
            self._map_frame = "map"

        rospy.logwarn_once("[GridMapPub | grid map loaded]")
        self.grid_map_loaded = True

    def grid_map_init(self, filepath):
        """
        Init occupancy grid map. Fill in meta information and map data container.
        """
        # load map 
        self.load_map(filepath)

        # create message
        self.grid_msg = OccupancyGrid()
        self.grid_msg.data = self._map_data
        self.grid_msg.info = self._map_metainfo
        self.grid_msg.header.frame_id = self._map_frame
        self.grid_map_msg_ready = True

    def publish(self):
        """ 
        Grid map callback. Init or update data container.
        """
        if not self.grid_map_msg_ready:
            rospy.logwarn_throttle(1.0, "[GridMapPub | waiting first message...]")
        else:
            # update message header
            self.grid_msg.header = Header()
            self.grid_msg.header.frame_id = self._map_frame
           
            self.grid_map_pub.publish(self.grid_msg)
            
            if self.use_dummy_odom:
                self.dummy_odom_pub.publish(self.odom_msg)
                self.dummy_start_mk_pub.publish(self.start_mk)

            rospy.logwarn_once("[GridMapPub | grid map published!]")
