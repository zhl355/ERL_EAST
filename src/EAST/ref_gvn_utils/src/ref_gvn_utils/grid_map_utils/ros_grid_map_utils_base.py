#!/usr/bin/env python3

"""
ROS utility function for grid map message.
"""



import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid


class GridMapUtilsError(Exception):
    """ User Defined Exceptions for GridMapUtils Class.
    """

    def __init__(self, *args):
        if args:
            self.msg = args[0]
        else:
            self.msg = ""

    def __str__(self):
        if self.msg:
            return "GridMapUtils exception: {0}".format(self.msg)
        else:
            return "GridMapUtils exception"


class GridMapUtils:
    """ 
    Class for OccupancyGrid in ROS.
    """

    def __init__(self, config_dict):
        """ Init class. 
        """

        self._load_params(config_dict)

        # subscribers ----------------------------------------------------------
        self._sub = rospy.Subscriber(self._map_in, OccupancyGrid, self.grid_map_callback, queue_size=1)
        
        # publisher ------------------------------------------------------------
        
        # cached large containers ----------------------------------------------
        self._map_data = None
        self._map2d_raw = None
        self._obstacle_array_coords = None

        # class status ---------------------------------------------------------
        self.map_init_finished = False
        self.obstacle_found = False
        self.surrounding_obstacle_found = False
        self._first_map_msg_received = False
        self._first_time_obstacle_detected = False
        self._classification_value_inited = False
        
    def _load_params(self, config_dict):
        """
        Load parameters from external node wrapper. 
        """
        self.config = config_dict
        #------------ get params from config dictionary -------
        self._map_in = config_dict['grid_map_topic']
        self._map_type = config_dict['map_type']

        self.init_classification_values()
    
    def init_classification_values(self):
        """
        Load critical values for grid type classification from config dict. 
        # For example, hector map generate occupancy grid map define as follows:
        # The map data, in row-major order, starting with (0,0).  
        # Occupancy probabilities are in the range [0, 100].  Unknown is -1.
        # int8[] data
        # we can defind critical values as follows:
        # self._hector_slam_raw_data_mapping = {'unknown': -1, 'free_ub': 10, 'obstacle_lb': 80}
        """
        if self._map_type == 'hector_map':
            self._grid_map_unknown = -1
            self._grid_map_free_lb = 0
            self._grid_map_free_ub = 10
            self._grid_map_obstacle_lb = 80
            self._grid_map_obstacle_ub = 100
        else:
            try:
                self._grid_map_unknown = self.config['grid_map_unknown']
                self._grid_map_free_lb = self.config['grid_map_free_lb']
                self._grid_map_free_ub = self.config['grid_map_free_ub']
                self._grid_map_obstacle_lb = self.config['grid_map_obstacle_lb']
                self._grid_map_obstacle_ub = self.config['grid_map_obstacle_ub']
            except KeyError:
                raise("You need to define custom bounds for uknown, free_lb, free_ub, obs_lb, obs_ub")

    def _get_meta_info(self, map_meta):
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

    def map_init(self, og_msg):
        """
        Init occupancy grid map. 
        """
        if not self._first_map_msg_received:
            self._dim = 2
            self._get_meta_info(og_msg.info)
            self._map_header = og_msg.header
            self._grid_res = np.round(self._map_res, 3) # assuming finest map resolution 1mm
            self._grid_origin = np.array([self._map_origin_x, self._map_origin_y])
            self._reference_frame = og_msg.header.frame_id
            self._map_data = og_msg.data
            self._map2d_raw = np.array(og_msg.data, dtype=np.int8).reshape(og_msg.info.height, og_msg.info.width)
            self._first_map_msg_received = True

        while not self._first_time_obstacle_detected:
            self.find_obstacle_arr_coordinates()
            if self.obstacle_found:
                self._first_time_obstacle_detected = True
                break
            else:
                rospy.logwarn_once("[grid utils | waiting first obstacle detection]")
                rospy.sleep(0.1)

        if self._first_time_obstacle_detected:
            self.map_init_finished = True
            rospy.logwarn_once("[grid_utils| map_init ready]")

    def find_obstacle_arr_coordinates(self):
        """
        Find obstacle coordinates in numpy array reprenetation. 
        Update obstacle detection flag.
        """
        coords = np.where((self._map2d_raw >= self._grid_map_obstacle_lb) & (self._map2d_raw <= self._grid_map_obstacle_ub))
        if len(coords[0]) == 0:
            self.obstacle_found = False
        else:
            self.obstacle_found = True
        self._obstacle_array_coords = coords

        return coords

    def grid_map_callback(self, msg):
        """ 
        Grid map callback. Init or update data.
        """
        if not self.map_init_finished:
            self.map_init(msg)
        else:
            self._map_header = msg.header
            self._map_data = np.array(msg.data, dtype=np.int8)
            self._map2d_raw = self._map_data.reshape(msg.info.height, msg.info.width)

    def meter2cell(self, loc, debug=False):
        """ Convert environment from meter to cell
        Input:
            loc:     nd vector containing loc coordinates in world frame (in meters)
            grid_min: nd vector containing the lower left corner of the grid (in meters)\n"
            grid_res: nd vector containing the grid resolution (in meters)\n"
        Output:
            loc_cell: nd vector containg index of cell grid index goes
            from lower left to upper right
        """
        grid_min = self._grid_origin
        grid_res = self._grid_res
        loc = np.array(loc)
        diff = (loc - grid_min) / grid_res
        loc_cell = diff.astype(int)
        if debug:
            print("loc at [%.2f %.2f], loc cell at [%d, %d]" % (loc[0], loc[1], loc_cell[0], loc_cell[1]))
        return loc_cell

    def cell2meter(self, loc_cell, debug=False):
        """
        Input:
            cell_loc: nd vector containing the cell index (from lower left to upper right)
            mesh_origin: nd vector containing the cell origin (in meters)"
            grid_res: nd vector containing the grid resolution (in meters)\n"
        Output:
            loc: nd vector containing loc coordinates in world frame (in meters)
            from lower left to upper right
        """
        grid_res = self._grid_res
        loc = np.array(loc_cell) * grid_res + self._grid_origin

        if debug:
            print("loc cell at [%d,  %d], loc at [%.2f, %.2f]" % (loc_cell[0], loc_cell[1], loc[0], loc[1]))
        return loc

    def find_closest_obstacle_cell_to_pt(self, pt_loc, debug=True):
        """
        Find closest obstacle cell with respect to pt_loc.
        pt_loc: 2d location in meter.
        """
        
        if not self.map_init_finished:
            raise GridMapUtilsError("[grid utils | find_closest_obstacle_cell_to_pt] func call should after map init")
        
        obstacle_coords = self.find_obstacle_arr_coordinates()
        
        if not self.obstacle_found:
            rospy.logwarn("no obstacle found?")
            rospy.logwarn("raw data unique values %s" % np.unique(self._map2d_raw))
            rospy.logwarn("obstacle bounds [lb, ub] = [%d, %d]" \
                          % (self._grid_map_obstacle_lb, self._grid_map_obstacle_ub))
            raise GridMapUtilsError("[grid utils | find_closest_obstacle_cell_to_pt] no obstacle in map after init")

        pt_loc_cell = self.meter2cell(loc=pt_loc, debug=False)
        # be caution about coordinates difference in matrix / array vs costmap/cellmap
        obstacle_loc_cell = np.vstack((obstacle_coords[1], obstacle_coords[0])).T
        distances = np.linalg.norm(obstacle_loc_cell - pt_loc_cell, axis=1)
        min_dist_idx = np.argmin(distances)
        closest_obstacle_cell_coord = obstacle_loc_cell[min_dist_idx]

        return closest_obstacle_cell_coord

    def find_all_surrounding_obstacles(self, pt_loc, radius, debug=False):
        """
        Find all surrounding obstacle cells with respect to pt_loc 
        pt_loc: 2d location in meter.
        """
        self.surrounding_obstacle_found = False
        surrounding_obstacle_cell_coords = None 
        
        if not self.map_init_finished:
            raise GridMapUtilsError("[grid utils | find_all_surrounding_obstacles] func call should after map init")

        obstacle_coords = self.find_obstacle_arr_coordinates()
        # map has no obstacle
        if not self.obstacle_found:
            raise GridMapUtilsError("[grid utils | find_all_surrounding_obstacles] no obstacle in map after init")

        # map has obstacle
        radius_cell = np.ceil(radius / self._grid_res)
        pt_loc_cell = self.meter2cell(loc=pt_loc, debug=False)
        # be caution about coordinates difference in matrix / array vs costmap/cellmap
        obstacle_loc_cell = np.vstack((obstacle_coords[1], obstacle_coords[0])).T
        distances = np.linalg.norm(obstacle_loc_cell - pt_loc_cell, axis=1)
        surrouding_obstacle_idx = np.argwhere(distances <= radius_cell)
        surrounding_obstacle_cell_coords = obstacle_loc_cell[surrouding_obstacle_idx]

        # no surrounding obstacle found
        if np.size(surrounding_obstacle_cell_coords) == 0:
            err_msg = 'map has obstacle, but not within radius = %.2f' % radius
            rospy.logerr(err_msg)
            raise GridMapUtilsError(err_msg)
        else:
            self.surrounding_obstacle_found = True 

        return surrounding_obstacle_cell_coords

    def find_closest_obstacle_to_pt(self, pt_loc):
        """
        Find closest obstacle loc with respect to pt_loc
        """
        obs_star_cell = self.find_closest_obstacle_cell_to_pt(pt_loc=pt_loc)
        obs_star = self.cell2meter(loc_cell=obs_star_cell)

        return obs_star

    def find_surrounding_obstacle_to_pt(self, pt_loc, radius):
        """
        Find surrounding obstacle loc with respect to pt_loc
        """

        obs_surr_cell = self.find_all_surrounding_obstacles(pt_loc=pt_loc, radius=radius)
        obs_surr = self.cell2meter(loc_cell=obs_surr_cell)

        return obs_surr
