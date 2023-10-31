#!/usr/bin/env python3

import rospy
import numpy as np
import numpy.linalg as npla
import ros_numpy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose2D, PoseStamped, PointStamped
from erl_msgs.msg import RefGvnDist
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs
from ref_gvn_utils.ros_msg_utils import pose_to_position_np


class RefGvnSensing:
    def __init__(self, config_dict=None):
        """ Init class.

            This node subscribes:
                3D pointcloud2 message
            Publish:
                distance info: d_Q(g, O), d_I(r, O)
        """
        # loading external config parameters
        self._config_dict = config_dict

        # load parameters 
        # pointcloud height clip level, note that 0 height is mesaured at 
        # ouster_lidar frame, approximately 0.247 m above ground 
        # so the ground level is about -0.247 m
        self._pcl_z_max = self._config_dict['pcl_z_max']
        self._pcl_z_min = self._config_dict['pcl_z_min']
        self._lidar_frame = self._config_dict['lidar_frame']
        self._localmap_frame = self._config_dict['map_frame']
        self._gvn_frame = self._config_dict['gvn_frame']

        # ------------------- subscribers ----------------------
        # pointcould subscriber
        self.pcl2_sub = rospy.Subscriber(self._config_dict['pcl_topic'], PointCloud2, self.pcl_callback)

        # governor subscriber
        self.gvn_state_sub = rospy.Subscriber(self._config_dict['gvn_topic'], Pose2D, self.gvn_state_callback)

        # robot state subscriber to init system properly without cyclic dependency 
        self.rbt_state_sub = rospy.Subscriber(self._config_dict['odom_topic'], Odometry, self.rbt_state_callback)

        # ------------------- publishers ---------------------- 
        # Ref Gvn dist information
        self.sense_pub = rospy.Publisher('~dist', RefGvnDist, queue_size=1)

        self.rbt3d_lidar_frame_pub = rospy.Publisher('~rbt3d_lidar_frame', PoseStamped, queue_size=1)
        self.gvn3d_lidar_frame_pub = rospy.Publisher('~gvn3d_lidar_frame', PoseStamped, queue_size=1)

        # show closest point d(g, O) \approx d(g, lidar_pts), g and lidar pts should in same frame 

        self.closest_pt_dg2pcl_pub = rospy.Publisher('~closest_pt_dg2pcl', PointStamped, queue_size=1)
        self.closest_pt_dr2pcl_pub = rospy.Publisher('~closest_pt_dr2pcl', PointStamped, queue_size=1)
        
        # -------------------  container --------------------
        # cached received message 
        self._rbt_odom = Odometry()
        self._gvn_pose2d = Pose2D()

        self._gvn_posestamped = PoseStamped()
        self._rbt_posestamped = PoseStamped()

        # debug transformed pt visualization
        self._rbt_ps_in_lidar_frame = PoseStamped()
        self._gvn_ps_in_lidar_frame = PoseStamped()

        # debug closest pt visualization
        self._pt_dg2pcl_ps_in_lidar_frame = PointStamped()
        self._pt_dr2pcl_ps_in_lidar_frame = PointStamped()


        # containers for converted message in numpy
        self._np_rbt3d_in_lidar_frame = np.zeros(3)
        self._np_gvn3d_in_lidar_frame = np.zeros(3) 

        # pub message 
        self._ref_gvn_dist = RefGvnDist()

        # -------------------- other status -------------------------
        self._first_gvn_received = False
        self._first_rbt_received = False

        self._first_transformation_done = False
        self._first_pub_msg_ready = False

        # cached transform
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # gvn frame update
        self._tf_buffer_gvn = tf2_ros.Buffer()
        self._tf_listener_gvn = tf2_ros.TransformListener(self._tf_buffer_gvn)

        
        # ------------------- Init Upstream --------------------
        self._class_init_done = True
        rospy.logwarn("[ref_gvn_sense] Init Done! -------------------- \n")

    
    def transform_point_in_map_frame_into_lidar_frame(self, debug=True):
        """ get points in lidar frame
        """
        
        self._transform_localmap2lidar = self._tf_buffer.lookup_transform(self._lidar_frame, self._localmap_frame, rospy.Time())
        self._transform_gvn_frame2lidar = self._tf_buffer_gvn.lookup_transform(self._lidar_frame, self._gvn_frame, rospy.Time())
        
        rbt_pose = self._rbt_odom.pose.pose
        self._rbt_posestamped.header = self._rbt_odom.header

        # borrow headers and overwrite frame_id later
        self._gvn_posestamped.header = self._rbt_odom.header

        self._pt_dg2pcl_ps_in_lidar_frame.header = self._rbt_odom.header
        self._pt_dr2pcl_ps_in_lidar_frame.header = self._rbt_odom.header

        self._pt_dg2pcl_ps_in_lidar_frame.header.frame_id = self._lidar_frame
        self._pt_dr2pcl_ps_in_lidar_frame.header.frame_id = self._lidar_frame
        

        self._rbt_posestamped.pose = rbt_pose
        # use robot pose if gvn pose is not received yet
        if self._first_gvn_received:
            self._gvn_posestamped.pose.position.x = self._gvn_pose2d.x
            self._gvn_posestamped.pose.position.y = self._gvn_pose2d.y
            self._gvn_posestamped.header.frame_id = self._gvn_frame
        else:
            self._gvn_posestamped.pose.position.x = rbt_pose.position.x
            self._gvn_posestamped.pose.position.y = rbt_pose.position.y
            # self._gvn_pss.header.frame_id = self._rbt_odom.header.frame_id
        
        # fill gvn_ps fields (exccept x, y position) with corresponding rbt fields
        self._gvn_posestamped.pose.position.z =  rbt_pose.position.z
        self._gvn_posestamped.pose.orientation =  rbt_pose.orientation

        try:
            # need take poseStamped
            self._rbt_ps_in_lidar_frame = tf2_geometry_msgs.do_transform_pose(self._rbt_posestamped, self._transform_localmap2lidar)

            if self._first_gvn_received:
                self._gvn_ps_in_lidar_frame = tf2_geometry_msgs.do_transform_pose(self._gvn_posestamped, self._transform_gvn_frame2lidar)
            else:
                self._gvn_ps_in_lidar_frame = tf2_geometry_msgs.do_transform_pose(self._gvn_posestamped, self._transform_localmap2lidar)

        except AttributeError as err:
            err_msg = err.__repr__()
            rospy.logwarn_throttle(5.0, '[ref_gvn_sense] node failed due to TF error %s' % err_msg)

        self._np_rbt3d_in_lidar_frame = pose_to_position_np(self._rbt_ps_in_lidar_frame.pose)
        self._np_gvn3d_in_lidar_frame = pose_to_position_np(self._gvn_ps_in_lidar_frame.pose)

        # overwrite height as 0.0 in lidar frame, so only x,y coordinates is used 
        self._np_rbt3d_in_lidar_frame[2] = 0.0
        self._np_gvn3d_in_lidar_frame[2] = 0.0
        

        self._first_transformation_done = True

        if debug:
            rbt_position = self._rbt_odom.pose.pose.position
            gvn_position = self._gvn_posestamped.pose.position

            msg = "\n\n\n[ref_gvn_sense] before transformation, coords in local map frame ---------------"
            msg += "\nrbt3d = [%.2f, %.2f, %.2f]" % (rbt_position.x, rbt_position.y, rbt_position.z)
            msg += "\ngvn3d = [%.2f, %.2f, %.2f]" % (gvn_position.x, gvn_position.y, gvn_position.z)

            # rospy.logwarn_throttle(1.0, "[ref_gvn_sense] before transformation in map frames ---------------")
            # rospy.logwarn_throttle(1.0, "rbt3d = [%.2f, %.2f, %.2f]" % (rbt_position.x, rbt_position.y, rbt_position.z))
            # rospy.logwarn_throttle(1.0, "gvn3d = [%.2f, %.2f, %.2f]" % (gvn_position.x, gvn_position.y, gvn_position.z))

            # rospy.logwarn_throttle(1.0, "[ref_gvn_sense] after transformation in lidar frames")

            rbt3d_np = self._np_rbt3d_in_lidar_frame
            gvn3d_np = self._np_gvn3d_in_lidar_frame

            msg += "\n[ref_gvn_sense] after transformation in lidar frames, coords in lidar frame "
            msg += "\nrbt3d = [%.2f, %.2f, %.2f]" % (rbt3d_np[0], rbt3d_np[1], rbt3d_np[2])
            msg += "\ngvn3d = [%.2f, %.2f, %.2f]" % (gvn3d_np[0], gvn3d_np[1], gvn3d_np[2])

            # rospy.logwarn_throttle(1.0, "rbt3d = [%.2f, %.2f, %.2f]" % (rbt3d_np[0], rbt3d_np[1], rbt3d_np[2]))
            # rospy.logwarn_throttle(1.0, "gvn3d = [%.2f, %.2f, %.2f]" % (gvn3d_np[0], gvn3d_np[1], gvn3d_np[2]))
            # rospy.logwarn_throttle(1.0, "")
            
            rospy.logwarn_throttle(1.0, msg)
            self.gvn3d_lidar_frame_pub.publish(self._gvn_ps_in_lidar_frame)
            self.rbt3d_lidar_frame_pub.publish(self._rbt_ps_in_lidar_frame)

    def compute_ref_gvn_dist(self, pcl_filtered, dim=2, debug=True):
        """
        Compute ref gvn distance message. 
        TODOZHL, Q = I now, Q psd later
        dim: 2/3 for distance computation
        """
        if self._first_transformation_done:
            filtered_points = np.zeros((pcl_filtered.shape[0], 3))
            filtered_points[:,0] = pcl_filtered['x']
            filtered_points[:,1] = pcl_filtered['y']
            filtered_points[:,2] = pcl_filtered['z'] # for dispaly we still need this

            # dist(gvn3d, pcl3d) in Euclidean norm
            distances_dIgO = npla.norm(self._np_gvn3d_in_lidar_frame[0:dim] - filtered_points[:, 0:dim], axis=1)
            min_idx_dIgO = np.argmin(distances_dIgO)
            dIgO = distances_dIgO[min_idx_dIgO]
            pt_dg2pcl = filtered_points[min_idx_dIgO]
            self._pt_dg2pcl_ps_in_lidar_frame.point.x = pt_dg2pcl[0]
            self._pt_dg2pcl_ps_in_lidar_frame.point.y = pt_dg2pcl[1]
            self._pt_dg2pcl_ps_in_lidar_frame.point.z = pt_dg2pcl[2]

            # dist(rbt3d, pcl3d) in Euclidean norm TODOZHL debug purpose, disable later or use as emergency stop
            # dIrO = np.min(npla.norm(self._np_rbt3d_in_lidar_frame[0:dim] - filtered_points[0:dim], axis=1))

            distances_dIrO = npla.norm(self._np_rbt3d_in_lidar_frame[0:dim] - filtered_points[:, 0:dim], axis=1)
            min_idx_dIrO = np.argmin(distances_dIrO)
            dIrO = distances_dIrO[min_idx_dIrO]
            pt_dr2pcl = filtered_points[min_idx_dIrO]
            self._pt_dr2pcl_ps_in_lidar_frame.point.x = pt_dr2pcl[0]
            self._pt_dr2pcl_ps_in_lidar_frame.point.y = pt_dr2pcl[1]
            self._pt_dr2pcl_ps_in_lidar_frame.point.z = pt_dr2pcl[2]
            
            dist_msg = "[cpt_closest_pt] compute dist" + "filtered_points %d" % filtered_points.shape[0]
            rospy.logdebug_throttle(1.0, dist_msg)

            if debug:
                msg = "\n[cpt_closest_pt] in lidar frame ---------------"
                msg += "\npt_dg2pcl = [%.2f, %.2f, %.2f]" % (pt_dg2pcl[0], pt_dg2pcl[1], pt_dg2pcl[2])
                msg += "\npt_dr2pcl = [%.2f, %.2f, %.2f]" % (pt_dr2pcl[0], pt_dr2pcl[1], pt_dr2pcl[2])
            
            # prepare message
            self._ref_gvn_dist.dIgO = dIgO
            self._ref_gvn_dist.dIrO = dIrO
            self._ref_gvn_dist.dQgO = dIgO
            self._first_pub_msg_ready = True
        else:
            rospy.logwarn_once("[ref_gvn_sense] waiting first transformation")

    def pcl_callback(self, pcl2_msg):
        """
        Process pointcloud2 messages. 
        """
        rospy.logdebug_throttle(1.0, "[ref_gvn_sense] Get PointCloud2 message!!!!!!!!!!!")
        pcl_data = ros_numpy.numpify(pcl2_msg)
        # message filtering # height filtering
        pcl_filtered = pcl_data[(pcl_data['z'] > self._pcl_z_min) & (pcl_data['z'] < self._pcl_z_max)]

        if pcl_data.shape[0] != pcl_filtered.shape[0]:
            filter_msg = "(raw_pcl num, filtered_pcl num) = (%d, %d)" % (pcl_data.shape[0], pcl_filtered.shape[0])
            rospy.logdebug_throttle(1.0, "[ref_gvn_sense pcl callback" + filter_msg)

        self.compute_ref_gvn_dist(pcl_filtered=pcl_filtered)

    def gvn_state_callback(self, pose2d_msg):
        """
        Process governor state messages. Need transform pose2d_msg (in map frame) to lidar frame
        """
        rospy.logwarn_once("[ref_gvn_sense] get first governor state message")
        self._first_gvn_received = True
        self._gvn_pose2d = pose2d_msg
        
    def rbt_state_callback(self, odom_msg):
        """
        Process robot odom messages. 
        """
        rospy.logwarn_once("[ref_gvn_sense] Get robot state message")
        self._first_rbt_received = True
        self._rbt_odom = odom_msg
        self.transform_point_in_map_frame_into_lidar_frame()

    def publish_sense_info(self):
        """
        Publish sensing information to ref governor node. 
        Currenty, only have distance info.
        """
        if self._first_pub_msg_ready:
            rospy.logwarn_throttle(1.0, "[ref_gvn_sense] publish msg")
            self.sense_pub.publish(self._ref_gvn_dist)
            # closest points
            self.closest_pt_dg2pcl_pub.publish(self._pt_dg2pcl_ps_in_lidar_frame)
            self.closest_pt_dr2pcl_pub.publish(self._pt_dr2pcl_ps_in_lidar_frame)

if __name__ == '__main__':

    try:
        rospy.init_node('Ref Gvn Sensing')
        rospy.loginfo("[ref_gvn_sense] start node!\n")

        # loading parameters
        sensing_config = {}

        
        sensing_config["pcl_topic"] = rospy.get_param("~pcl_topic", "/pcl")
        sensing_config["gvn_topic"] = rospy.get_param("~gvn_topic", "/gvn_state")
        sensing_config["odom_topic"] = rospy.get_param("~odom_topic", "/odom")

        sensing_config["pcl_z_max"] = rospy.get_param("~pcl_z_max", 1.0)
        sensing_config["pcl_z_min"] = rospy.get_param("~pcl_z_min", -0.1)

        sensing_config["ref_gvn_z"] = rospy.get_param("~ref_gvn_z", -0.1)
        sensing_config["pub_freq"] = rospy.get_param("~pub_freq", 50.0)


        sensing_config["lidar_frame"] = rospy.get_param("~lidar_frame", "front_laser")
        sensing_config["map_frame"] = rospy.get_param("~map_frame", "map_link")

        sensing_config["gvn_frame"] = rospy.get_param("~gvn_frame", "map_link")

        
        rospy.logwarn_once("[ref_gvn_sense] config ---------------")
        print(sensing_config)
        rospy.logwarn_once("[ref_gvn_sense] config------------------")


        # sensing_config["pcl_range_max"] = rospy.get_param("~pcl_range_max", 90.0)
        # sensing_config["pcl_range_min"] = rospy.get_param("~pcl_range_min", 0.5)


        ref_gvn_sens = RefGvnSensing(config_dict=sensing_config)
        rate = rospy.Rate(sensing_config["pub_freq"])

        while not rospy.is_shutdown():
            # publish path at certain freq
            ref_gvn_sens.publish_sense_info()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr("[ref_gvn_sense] node init failed.")
        rospy.signal_shutdown('[ref_gvn_sense] node init fail')
        pass
