#!/usr/bin/env python3

"""
ROS utility function for computing distance to obstacles in grid map.
"""

import time
import numpy as np
import rospy
from grid_map_utils.grid_map_base import GridMapBase
from geometry_msgs.msg import PoseStamped

from erl_msgs.msg import RefGvnMsg
from std_msgs.msg import Bool


class GridMapDistError(Exception):
    """
    User Defined Exceptions for GridMapDist Class.
    """

    def __init__(self, *args):
        if args:
            self.msg = args[0]
        else:
            self.msg = ""

    def __str__(self):
        if self.msg:
            return "GridMapDist exception: {0}".format(self.msg)
        else:
            return "GridMapDist exception"


class GridMapDist(GridMapBase):
    """ 
    Class for OccupancyGrid in ROS.
    """

    def __init__(self, config_dict):
        """ Init class. 
        """
        # super class init
        super(GridMapDist, self).__init__(config_dict)

        # load parameters needed for distance utils ----------------------------
        self.load_dist_params(config_dict)

        # class status ---------------------------------------------------------
        self._obstacle_found = False
        self._closest_obstacle_found = False
        self._surrounding_obstacle_found = False
        self._first_time_obstacle_detected = False
        # class init finished flag
        self.dist_init_finished = False
        # class debug
        self._click_test = False
        # class config flag
        self._check_gvn_on_unknown = False

        # cached large containers ----------------------------------------------
        self._obstacle_array_coords = None
        self.config_dict = None
        self._map_type = None

        # this derived class init ----------------------------------------------
        rospy.logwarn_once("[GridMapDist init started...]")
        self.dist_init()

        # optional subscribers 
        if self._click_test:
            self._button_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.click_test_callback)

        # subscriber to trigger gvn location check, checking gvn is on unknown map grid or not
        if self._check_gvn_on_unknown:
            self._ref_gvn_status_sub = rospy.Subscriber('/ref_gvn_status', RefGvnMsg,
                                                        self.gvn_located_on_unknown_status_check_callback)
            self.gvn_on_unknown_flag = False

        # optional publisher
        # publish Bool message indicating current gvn location is on unknown map grid
        if self._check_gvn_on_unknown:
            self._gvn_located_on_unknown_status_pub = rospy.Publisher('/gvn_located_on_unknown_status', Bool,
                                                                      queue_size=1)
            # Important, we need to publish False immediately, otherwise when this feature is disabled, down stream nodes will not update gvn normally.
            self._gvn_located_on_unknown_status_pub.publish(False)

        rospy.logwarn_once("[GridMapDist init finished!]")

    def load_grid_map_classification_critical_values(self):
        """
        Load critical values for grid type classification from config dict. 
        # For example, hector map generate occupancy grid map define as follows:
        # The map data, in row-major order, starting with (0,0).  
        # Occupancy probabilities are in the range [0, 100].  Unknown is -1.
        # int8[] data
        # we can define critical values as follows:
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
                self._grid_map_unknown = self.config_dict['grid_map_unknown']
                self._grid_map_free_lb = self.config_dict['grid_map_free_lb']
                self._grid_map_free_ub = self.config_dict['grid_map_free_ub']
                self._grid_map_obstacle_lb = self.config_dict['grid_map_obstacle_lb']
                self._grid_map_obstacle_ub = self.config_dict['grid_map_obstacle_ub']
            except KeyError:
                raise GridMapDistError(
                    "You need to define custom bounds for unknown, free_lb, free_ub, obs_lb, obs_ub")

    def load_dist_params(self, config_dict):
        """
        Load parameters relates to distance computation.
        """
        self.config_dict = config_dict

        if "map_type" in self.config_dict:
            self._map_type = self.config_dict['map_type']
        else:
            rospy.logwarn("map type is unspecified!")
            self._map_type = None

        # for class debug purpose
        if "click_test" in self.config_dict:
            self._click_test = self.config_dict['click_test']

        # for class debug purpose checking whether gvn located on unknown type cell
        if "check_gvn_on_unknown" in self.config_dict:
            self._check_gvn_on_unknown = self.config_dict['check_gvn_on_unknown']

        self.load_grid_map_classification_critical_values()

    def dist_init(self):
        """
        Init process for distance utils.
        """
        # wait grid map msg callback to trigger GridMapBase init process
        while not self.grid_map_init_finished:
            rospy.logwarn_throttle(1.0, "waiting grid map init...")
            time.sleep(0.1)
            rospy.sleep(0.1)

        # our map should contains obstacles
        while not self._first_time_obstacle_detected:
            self.find_obstacle_arr_coordinates()
            if self._obstacle_found:
                self._first_time_obstacle_detected = True
                break
            else:
                rospy.logwarn_throttle(1.0, "[grid_map_dist | waiting first obstacle to be detected]")
                time.sleep(0.1)
                rospy.sleep(0.1)

        self.dist_init_finished = True
        rospy.logwarn_once("[grid_map_dist| dist_init ready]")

    def meter2cell(self, loc, debug=False):
        """ 
        Convert environment from meter to cell
        Input:
            loc:     nd vector containing loc coordinates in world frame (in meters)
            grid_min: nd vector containing the lower left corner of the grid (in meters)\n"
            grid_res: nd vector containing the grid resolution (in meters)\n"
        Output:
            loc_cell: nd vector containing index of cell grid index goes
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

    def find_obstacle_arr_coordinates(self):
        """
        Find obstacle coordinates in numpy array representation.
        Update obstacle detection flag.
        """
        coords = np.where(
            (self._map2d_raw >= self._grid_map_obstacle_lb) & (self._map2d_raw <= self._grid_map_obstacle_ub))
        if len(coords[0]) == 0:
            self._obstacle_found = False
        else:
            self._obstacle_found = True
        self._obstacle_array_coords = coords

        return coords

    def find_closest_obstacle_cell_to_pt(self, pt_loc, debug=True):
        """
        Find the closest obstacle cell with respect to pt_loc.
        pt_loc: 2d location in meter.
        """
        self._closest_obstacle_found = False
        if not self.dist_init_finished:
            raise GridMapDistError(
                "[grid_map_dist | find_closest_obstacle_cell_to_pt] func call should after map init")

        obstacle_coords = self.find_obstacle_arr_coordinates()

        if not self._obstacle_found:
            if debug:
                rospy.logwarn("no obstacle found?")
                rospy.logwarn("raw data unique values %s" % np.unique(self._map2d_raw))
                rospy.logwarn("obstacle bounds [lb, ub] = [%d, %d]"
                              % (self._grid_map_obstacle_lb, self._grid_map_obstacle_ub))
            raise GridMapDistError("[grid_map_dist | find_closest_obstacle_cell_to_pt] no obstacle in map after init")

        pt_loc_cell = self.meter2cell(loc=pt_loc, debug=False)
        # be caution about coordinates difference in matrix / array vs costmap/cellmap
        obstacle_loc_cell = np.vstack((obstacle_coords[1], obstacle_coords[0])).T
        distances = np.linalg.norm(obstacle_loc_cell - pt_loc_cell, axis=1)
        min_dist_idx = np.argmin(distances)
        closest_obstacle_cell_coord = obstacle_loc_cell[min_dist_idx]

        # set this at last of func call
        self._closest_obstacle_found = True

        return closest_obstacle_cell_coord

    def find_closest_obstacle_to_pt(self, pt_loc):
        """
        Find the closest obstacle loc (in meter) with respect to pt_loc
        """
        obs_star_cell = self.find_closest_obstacle_cell_to_pt(pt_loc=pt_loc)
        # this case never should happen
        if not self._closest_obstacle_found:
            raise GridMapDistError("[grid_map_dist | closest obstacle not found!]")

        obs_star = self.cell2meter(loc_cell=obs_star_cell)

        return obs_star

    def find_all_surrounding_obstacles_cell(self, pt_loc, radius):
        """
        Find all surrounding obstacle cells with respect to pt_loc 
        pt_loc: 2d location in meter.
        """
        self._surrounding_obstacle_found = False
        surrounding_obstacle_cell_coords = None # noqa

        # you should not see this error been thrown, change your code if you see this
        if not self.dist_init_finished:
            raise GridMapDistError("[grid_map_dist | find_all_surrounding_obstacles] func call should after map init")

        # unexpected behavior in daily usage, map suddenly detects no obstacle
        obstacle_coords = self.find_obstacle_arr_coordinates()
        if not self._obstacle_found:
            raise GridMapDistError("[grid_map_dist | find_all_surrounding_obstacles] no obstacle in map after init")

        # map has obstacle, normal case
        search_radius = np.ceil(radius / self._grid_res)
        pt_loc_cell = self.meter2cell(loc=pt_loc, debug=False)
        obstacle_loc_cell = np.vstack((obstacle_coords[1], obstacle_coords[0])).T
        distances = np.linalg.norm(obstacle_loc_cell - pt_loc_cell, axis=1)

        # filtering cells within certain radius
        surrounding_obstacle_idx = np.argwhere(distances <= search_radius)
        surrounding_obstacle_cell_coords = obstacle_loc_cell[surrounding_obstacle_idx]

        # no surrounding obstacle found, this can happen due to improper parameter (search radius) setup
        if np.size(surrounding_obstacle_cell_coords) == 0:
            err_msg = 'map has obstacle, but not within radius = %.2f' % radius
            err_msg += " consider increase search radius"
            rospy.logwarn_throttle(0.1, err_msg)
        else:
            self._surrounding_obstacle_found = True

        return surrounding_obstacle_cell_coords

    def find_surrounding_obstacle_to_pt(self, pt_loc, radius):
        """
        Find surrounding obstacle loc with respect to pt_loc
        """
        all_surrounding_obs_cell = self.find_all_surrounding_obstacles_cell(pt_loc=pt_loc, radius=radius)
        # this case can happen due to in-proper search radius
        if not self._surrounding_obstacle_found:
            rospy.logwarn_throttle(0.1, "[grid_map_dist | surrounding obstacle not found!]")

        all_surrounding_obs_locations = self.cell2meter(loc_cell=all_surrounding_obs_cell)

        return all_surrounding_obs_locations

    def click_test_callback(self, msg):
        """
        Click rviz 2D Nav Button to test 
            1. find the closest obstacle
            2. find surrounding obstacle within specified radius
        """
        rospy.logwarn("[GridMapDist] Button clicked")

        if not self.dist_init_finished:
            rospy.logwarn("[GridMapDist] waiting map dist init...")
            return

            # use button generated pose as probe location
        x = msg.pose.position.x
        y = msg.pose.position.y
        rospy.logwarn("[GridMapDist] check position [%.2f, %.2f]" % (x, y))
        pt_loc = np.array([x, y])
        # test closet obstacle utils
        obs_star = self.find_closest_obstacle_to_pt(pt_loc=pt_loc)
        if obs_star is not None:
            rospy.logwarn("closest obstacle at [%.2f, %.2f]" % (obs_star[0], obs_star[1]))
        else:
            rospy.logerr("[GridMapDist] obstacle not found in map")
        # test surrounding obstacle utils
        all_surrounding_obs = self.find_surrounding_obstacle_to_pt(pt_loc=pt_loc, radius=1.0)
        surrounding_obstacle_count = len(all_surrounding_obs)
        rospy.logwarn("all surrounding obstacle num = %d " % surrounding_obstacle_count)

    def gvn_located_on_unknown_status_check_callback(self, msg):
        """
        Check whether governor is inside grid map or not.
        """
        if self.grid_map_init_finished:
            gvn_pos = np.array([msg.gvn_pose2D.x, msg.gvn_pose2D.y])
            gvn_loc_cell = self.meter2cell(loc=gvn_pos, debug=False)
            gvn_cell_value = self._map2d_raw[gvn_loc_cell[1], gvn_loc_cell[0]]
            # check whether gvn is on cell characterized as unknown by mapping package (hector -1)
            if gvn_cell_value == self._grid_map_unknown:
                self.gvn_on_unknown_flag = True
            else:
                self.gvn_on_unknown_flag = False
        else:
            self.gvn_on_unknown_flag = True

        self._gvn_located_on_unknown_status_pub.publish(self.gvn_on_unknown_flag)
