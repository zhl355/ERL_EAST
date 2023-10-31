#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, PolygonStamped, Point32
from grid_map_utils.grid_map_dist import GridMapDist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from ref_gvn_utils.geometry_utils import wrap_angle_pmp
import numpy.linalg as npla
from ref_gvn_utils.ros_gvn_cone_dist_util import point2cone_distance, triangle_def
from ref_gvn_utils.ros_gvn_cone_q_dist_util import point2cone_q_distance
from erl_msgs.msg import RefGvnDist, RefGvnMsg
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header
from ref_gvn_utils.ref_gvn_cone_vis_utils import create_icball_marker, create_iccone_marker, get_iccone_params
from ref_gvn_utils.ros_marker_utils import create_ellipsoid_marker
import time


class ConeDistError(Exception):
    """ User Defined Exceptions for RefGvnConeDist Class.
    """

    def __init__(self, *args):
        if args:
            self.msg = args[0]
        else:
            self.msg = ""

    def __str__(self):
        if self.msg:
            return "ConeDistError exception: {0}".format(self.msg)
        else:
            return "ConeDistError exception"


class RefGvnConeDist(GridMapDist):
    """
    Class for Costmap Utility Functions.
    """

    def __init__(self, config_dict):
        """
        Init class for costmap.
        Need to load parameters first and then init super class
        """
        # load parameters needed for cone distance calculation -----------------
        self.load_cone_dist_params(config_dict)

        # super class init -----------------------------------------------------
        super(RefGvnConeDist, self).__init__(self._rgc_config_dict)

        # class status ---------------------------------------------------------
        self.ref_gvn_cone_dist_init_finished = False
        self.gvn_dist_ready = False
        self.gvn_loc_ready = False
        self.rbt_pos_ready = False

        # subscribers ----------------------------------------------------------
        self.rbt_state_sub = rospy.Subscriber(config_dict['odom_topic'], Odometry, self.rbt_state_callback)
        self.gvn_state_sub = rospy.Subscriber(config_dict['gvn_topic'], RefGvnMsg, self.gvn_state_callback)

        # publisher ------------------------------------------------------------
        # Distance Publisher
        self.cone_pub = rospy.Publisher(config_dict['dist_topic'], RefGvnDist, queue_size=1)
        # Closest Obstacles to Cone as PointCloud under Euclidean Norm
        self.pcl2_pub = rospy.Publisher('~obstacles', PointCloud2, queue_size=1)
        self.pcl2_q_pub = rospy.Publisher('~q_obstacles', PointCloud2, queue_size=1)
        # Cone Visualization as MarkerArray
        self.cone_vis_pub = rospy.Publisher("~vis_cone", MarkerArray, queue_size=1)
        # Polygon Cone, NOT STABLE, USE WITH CAUTION
        self.poly_cone_vis_pub = rospy.Publisher("~vis_poly_cone", PolygonStamped, queue_size=1)

        # cached large containers ----------------------------------------------
        self.gvn_loc2d = None
        self.rbt_pose = None
        self.ref_gvn_dist = None

        # this derived class init ----------------------------------------------
        rospy.logwarn_once("[RefGvnConeDist init started...]")
        self.cone_dist_init()
        rospy.logwarn_once("[RefGvnConeDist init finished!]")

    def check_cone_dist_required_params(self, config_dict):
        if 'odom_topic' not in config_dict or 'gvn_topic' not in config_dict or 'dist_topic' not in config_dict:
            raise ConeDistError("Missing Odom, Governor or Dist Topic")
        
        if 'radius_offset' not in config_dict:
            raise ConeDistError("Local obstacle search radius offset missing")

    def load_cone_dist_params(self, config_dict):
        """
        Load parameters relates to cone distance computation.
        """
        # check required parameter
        self.check_cone_dist_required_params(config_dict)

        # reference governor cone dist config dictionary
        self._rgc_config_dict = {}
        self._rgc_config_dict['radius_offset'] = config_dict['radius_offset']

        if 'use_costmap' in config_dict:
            if config_dict['use_costmap']:
                try:
                    self._rgc_config_dict['map_topic'] = config_dict['costmap_topic']
                    self._rgc_config_dict['grid_map_unknown'] = config_dict['cost_map_unknown']
                    self._rgc_config_dict['grid_map_free_lb'] = config_dict['cost_map_free_lb']
                    self._rgc_config_dict['grid_map_free_ub'] = config_dict['cost_map_free_ub']
                    self._rgc_config_dict['grid_map_obstacle_lb'] = config_dict['cost_map_obstacle_lb']
                    self._rgc_config_dict['grid_map_obstacle_ub'] = config_dict['cost_map_obstacle_ub']
                    self._rgc_config_dict['map_type'] = 'cost_map'
                except KeyError:
                    raise ConeDistError("Missing Parameters Using Costmap")
                self._rgc_config_dict["map_topic"] = config_dict['costmap_topic']
        else:
            try:
                self._rgc_config_dict['map_topic'] = config_dict['map_topic']
                self._rgc_config_dict['map_type'] = config_dict['map_type']
            except KeyError:
                raise ConeDistError("Missing Parameters Using Gridmap")

    def cone_dist_init(self):
        """
        Init process for cone distance.
        """
        # wait obstacle detected in grid map
        while not self.dist_init_finished:
            rospy.logwarn_throttle(1.0, "[conedist] waiting map init...")
            time.sleep(0.1)
            rospy.sleep(0.1)

        # wait robot states, trigger by rbt_state_callback()
        while not self.rbt_pos_ready:
            rospy.logwarn_throttle(1.0, "[conedist] waiting robot state init...")
            time.sleep(0.2)
            rospy.sleep(0.2)

        # prevent deadlock at init process
        if not self.gvn_loc_ready:
            rospy.logwarn_throttle(1.0, "[conedist] init gvn2d use rbt2d...")
            try:
                self.gvn_loc2d = self.rbt_pose[0:2]
                self.gvn_loc_ready = True
            except TypeError:
                rospy.logfatal("self.gvn_loc2d = self.rbt_pose[0:2], rbt_pose is not ready?!")
        
        # init marker when robot and governor locations are known
        if self.gvn_loc_ready and self.rbt_pos_ready:
            self.init_markers()
        else:
            raise ConeDistError("[conedist] Init logic error!")
        
        # downstream container
        self.ref_gvn_dist = RefGvnDist()

        # PointCloud2 Init -----------------------------------------------------
        self.pcl2_fields = [PointField('x', 0, PointField.FLOAT32, 1),
                            PointField('y', 4, PointField.FLOAT32, 1),
                            PointField('z', 8, PointField.FLOAT32, 1)]

        self.pcl2_header = Header()
        self.pcl2_header.frame_id = "map"
        self.pcl2_header.stamp = rospy.Time.now()

        self.pcl2_q_fields = [PointField('x', 0, PointField.FLOAT32, 1),
                              PointField('y', 4, PointField.FLOAT32, 1),
                              PointField('z', 8, PointField.FLOAT32, 1)]

        self.pcl2_q_header = Header()
        self.pcl2_q_header.frame_id = "map"
        self.pcl2_q_header.stamp = rospy.Time.now()

        rospy.logwarn_once("[RefGvnConeDist | cone_dist_init ready]")
        self.ref_gvn_cone_dist_init_finished = True

    def init_markers(self):
        """
        Create a bunch of marker for rviz visualization of lower level controller.
        """
        # Cone Visualization Init ----------------------------------------------
        self.cone_array = MarkerArray()
        self.cone_vertices = PolygonStamped()
        self.cone_vertices.header = Header()
        self.cone_vertices.header.frame_id = "map"
        self.cone_vertices.header.stamp = rospy.Time.now()

        # create predicted reachable set (ice-cream cone) display
        self.mk_icball = create_icball_marker(self.rbt_pose, self.gvn_loc2d)
        self.mk_iccone = create_iccone_marker(self.rbt_pose, self.gvn_loc2d)
        self.mk_ellipsoid = create_ellipsoid_marker(self.rbt_pose[0:2], self.rbt_pose[2], 2, 2)

        # assemble markers in makerArray
        self.cone_array.markers = [self.mk_icball, self.mk_iccone, self.mk_ellipsoid]

        # ------------ publish marker array---------------
        self.cone_vis_pub.publish(self.cone_array)

        rospy.loginfo("ICE CREAM CONE INIT SUCCESSFUL!")

    def gvn_state_callback(self, ref_gvn_msg):
        """
        Process governor state messages. Need transfrom pose2d_msg (in map frame) to lidar frame
        """
        rospy.logdebug_once("[ref_gvn_cone_map_node] get first governor pose2d message")
        self.gvn_loc2d = np.array([ref_gvn_msg.gvn_pose2D.x, ref_gvn_msg.gvn_pose2D.y])
        self.gvn_loc_ready = True

    def rbt_state_callback(self, odom_msg):
        """
        Process robot odom messages.
        """
        rospy.logwarn_once("[ref_gvn_cone_map_node] get first robot odometry message")

        rbt_loc2 = odom_msg.pose.pose.position
        rbt_quaternion_sxyz = [odom_msg.pose.pose.orientation.x,
                               odom_msg.pose.pose.orientation.y,
                               odom_msg.pose.pose.orientation.z,
                               odom_msg.pose.pose.orientation.w]

        (_, _, rbt_yaw) = euler_from_quaternion(rbt_quaternion_sxyz)
        self.rbt_pose = np.array([rbt_loc2.x, rbt_loc2.y, rbt_yaw])
        if not self.rbt_pos_ready:
            time.sleep(0.1)
            rospy.logwarn_once("prevent ros init error")
        self.rbt_pos_ready = True


    def calculate_obs2cone_dist(self):
        """
        Calculate obstacle point to reachable cone distance
        """
        # Obtain robot 2d position and heading
        rbt_loc2d = self.rbt_pose[0:2]
        rbt_yaw = self.rbt_pose[2]

        # Obtain heading error
        gvn_rbt_dy = self.gvn_loc2d[1] - rbt_loc2d[1]
        gvn_rbt_dx = self.gvn_loc2d[0] - rbt_loc2d[0]
        gvn_rbt_theta = np.arctan2(gvn_rbt_dy, gvn_rbt_dx)
        gvn_rbt_head_err = wrap_angle_pmp(rbt_yaw - gvn_rbt_theta)

        # Get surrounding obstacles
        radius = npla.norm(rbt_loc2d - self.gvn_loc2d) + self._rgc_config_dict['radius_offset']
        obs_surr_temp = self.find_surrounding_obstacle_to_pt(pt_loc=self.gvn_loc2d, radius=radius)
        while not self._surrounding_obstacle_found:
            radius = radius + 1
            obs_surr_temp = self.find_surrounding_obstacle_to_pt(pt_loc=self.gvn_loc2d, radius=radius)
            if radius >= 10:
                ConeDistError('Unable to Find Obstacle Within 10 Meters')

        obs_surr_to_gvn = np.zeros([np.shape(obs_surr_temp)[0], 2])
        obs_surr_to_gvn[:, 0] = obs_surr_temp[:, 0, 0]
        obs_surr_to_gvn[:, 1] = obs_surr_temp[:, 0, 1]

        # Obstacle to cone distance calculation
        if np.abs(gvn_rbt_head_err) >= (np.pi / 2):
            cone_rad = npla.norm(rbt_loc2d - self.gvn_loc2d)
        else:
            cone_rad = np.abs(np.sin(gvn_rbt_head_err) * npla.norm(rbt_loc2d - self.gvn_loc2d))

        pt2cone_dist = point2cone_distance(rbt_loc2d[:, np.newaxis], self.gvn_loc2d[:, np.newaxis], cone_rad, obs_surr_to_gvn)
        v = np.array([[np.cos(self.rbt_pose[2])], [np.sin(self.rbt_pose[2])]])
        pt2cone_q_dist, Q = point2cone_q_distance(v, rbt_loc2d[:, np.newaxis], self.gvn_loc2d[:, np.newaxis], cone_rad, obs_surr_to_gvn)

        # Obtain The Closest Distance and Corresponding Obstacle
        closest_pt2cone_dist = np.amin(pt2cone_dist)
        closest_pt2cone_q_dist = np.amin(pt2cone_q_dist)
        cone_pt_idx = pt2cone_dist == closest_pt2cone_dist
        cone_q_pt_idx = pt2cone_q_dist == closest_pt2cone_q_dist
        closest_pt2cone_point = obs_surr_to_gvn[cone_pt_idx]
        closest_pt2cone_q_point = obs_surr_to_gvn[cone_q_pt_idx]
        closest_pt2cone_xyz = np.column_stack((closest_pt2cone_point, np.zeros(np.shape(closest_pt2cone_point)[0])))
        closest_pt2cone_q_xyz = np.column_stack((closest_pt2cone_q_point, np.zeros(np.shape(closest_pt2cone_q_point)[0])))

        # Ellipsoid Visualization in Standard Form
        eig_val_Q, _ = npla.eig(Q)
        self.ellip_axes_a = closest_pt2cone_q_dist
        self.ellip_axes_b = closest_pt2cone_q_dist / 3

        # Closest Obstacle Visualization
        self.pcl2_header.stamp = rospy.Time.now()
        self.pcl2_q_header.stamp = rospy.Time.now()
        self.closest_pt2cone_pcl2 = point_cloud2.create_cloud(self.pcl2_header, self.pcl2_fields, closest_pt2cone_xyz)
        self.closest_pt2cone_q_pcl2 = point_cloud2.create_cloud(self.pcl2_q_header, self.pcl2_q_fields, closest_pt2cone_q_xyz)

        # Publish Cone Visualization
        tri_vertices = triangle_def(rbt_loc2d[:, np.newaxis], self.gvn_loc2d[:, np.newaxis], cone_rad)
        self.cone_vertices.polygon.points = [
            Point32(x=tri_vertices[0, 0], y=tri_vertices[1, 0], z=0),
            Point32(x=tri_vertices[0, 1], y=tri_vertices[1, 1], z=0),
            Point32(x=tri_vertices[0, 2], y=tri_vertices[1, 2], z=0)]
        self.cone_vertices.header.stamp = rospy.Time.now()

        # Distance Loading
        obs_star_to_rbt = self.find_closest_obstacle_to_pt(pt_loc=rbt_loc2d)
        obs_star_to_gvn = self.find_closest_obstacle_to_pt(pt_loc=self.gvn_loc2d)
        self.ref_gvn_dist.dIrO = npla.norm(rbt_loc2d - obs_star_to_rbt)
        self.ref_gvn_dist.dIgO = npla.norm(self.gvn_loc2d - obs_star_to_gvn)
        self.ref_gvn_dist.dIcO = closest_pt2cone_dist
        self.ref_gvn_dist.dQcO = closest_pt2cone_q_dist

        # Print
        rospy.loginfo_throttle(1.0, "gvn cone dist = %.2f" % closest_pt2cone_dist)
        self.gvn_dist_ready = True


    def update_moving_markers(self, z, z_dsr):
        """
        Update markers along with robot:solve_quar
        [ice cream cone predicted reachable set]
            a. update ice cream ball center with current robot goal position
            b. update ice cream ball radius with robot current heading
            c. update ice cream cone vertices with robot current pose
        """
        # ------------update ice cream ball markers------------
        # get new params
        r_upd, t1p_upd, t2p_upd, zpt_upd = get_iccone_params(z, z_dsr)
        q_upd = quaternion_from_euler(0.0, 0.0, z[2])

        # update ball marker
        self.mk_icball.pose.position.x = z_dsr[0]
        self.mk_icball.pose.position.y = z_dsr[1]
        self.mk_icball.scale.x = 2 * r_upd
        self.mk_icball.scale.y = 2 * r_upd

        # update cone marker
        self.mk_iccone.pose.position.x = z[0]
        self.mk_iccone.pose.position.y = z[1]
        self.mk_iccone.pose.orientation.x = q_upd[0]
        self.mk_iccone.pose.orientation.y = q_upd[1]
        self.mk_iccone.pose.orientation.z = q_upd[2]
        self.mk_iccone.pose.orientation.w = q_upd[3]
        self.mk_iccone.points = [t1p_upd, zpt_upd, t2p_upd]

        # update ellipsoid marker
        self.mk_ellipsoid.pose.position.x = z[0]
        self.mk_ellipsoid.pose.position.y = z[1]
        self.mk_ellipsoid.scale.x = 2 * self.ellip_axes_a
        self.mk_ellipsoid.scale.y = 2 * self.ellip_axes_b
        self.mk_ellipsoid.pose.orientation.x = q_upd[0]
        self.mk_ellipsoid.pose.orientation.y = q_upd[1]
        self.mk_ellipsoid.pose.orientation.z = q_upd[2]
        self.mk_ellipsoid.pose.orientation.w = q_upd[3]

        self.cone_array.markers = [self.mk_icball, self.mk_iccone, self.mk_ellipsoid]

        rospy.loginfo_throttle(1.0, "[ref_gvn_cone_map_node] ice-cream cone updated")
        # ---------------- re-publish marker array ----------------
        self.cone_vis_pub.publish(self.cone_array)

    def publish_cone_info(self):
        """
        Publish sensing information to ref governor node.
        """
        if self.ref_gvn_cone_dist_init_finished:
            self.calculate_obs2cone_dist()

        if self.gvn_dist_ready:
            # distance info
            self.cone_pub.publish(self.ref_gvn_dist)
            # visualization info
            self.poly_cone_vis_pub.publish(self.cone_vertices)
            self.pcl2_pub.publish(self.closest_pt2cone_pcl2)
            self.pcl2_q_pub.publish(self.closest_pt2cone_q_pcl2)
            self.update_moving_markers(self.rbt_pose, self.gvn_loc2d)
