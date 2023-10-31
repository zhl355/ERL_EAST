#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


from tf.transformations import euler_from_quaternion, quaternion_from_euler
from ref_gvn_utils.planning_utils import path_simplify


class RefGvnPreprocess:
    """ 
    Reference Governor Preprocess Module. Responsible for:
        1) taken ros 2d path from A* and simplified it (merge straight line) into numpy array path r (num_pts, 2)
        2) optional state transformation, e.g., nonlinear->linear, cartesian-> polar 
        3) taken ros odom message generate robot states z (e.g., unicycle states z = (x, y, theta))
    """

    # Running status table, higher number better status

    def __init__(self):

        # params from launch file
        _scan_topic = rospy.get_param('~scan_topic')
        _path_topic = rospy.get_param('~path_topic')
        _odom_topic = rospy.get_param('~odom_topic')

        self._publish_simple_path = rospy.get_param('~display_nav_path')
        
        rospy.logwarn_once('_scan_topic: [%s]' % _scan_topic)

        # subscribers
        self._path_sub = rospy.Subscriber(_path_topic, Path, self.path_callback)
        self._odom_sub = rospy.Subscriber(_odom_topic, Odometry, self.odom_callback)
        self._scan_sub = rospy.Subscriber(_scan_topic, LaserScan, self.scan_callback, queue_size=1)
        
        # publishers for display simple path (navigation path r)

        if self._publish_simple_path:
            self._nav_path_pub = rospy.Publisher("~nav_path", Path, queue_size=1)

        # upstream status variables
        self._upstream_connection = 0
        self._upstream_connection_ready = False
        self._upstream_data_ready = False

        # ------------------- upstream data numpy container --------------------

        # containers for converted message in numpy
        self._np_path = None
        self._np_dist_info = None
        self._np_z = None

        # ------------------- Init Upstream --------------------
        self.init_upstream()
        rospy.loginfo("[Ref Gvn Preprocessor Created!]  \n")

    def _check_upstream_connections(self, upstream_connection=3):
        """ check whether subscribers' uplink connections are established """

        self._upstream_connection = \
            self._path_sub.get_num_connections() + \
            self._odom_sub.get_num_connections() + \
            self._scan_sub.get_num_connections()

        if self._upstream_connection < upstream_connection:
            # we need to wait path, states and lidar all ready
            rospy.loginfo('[Ref Gvn] waiting upstream connections [%d / %d]:', self._upstream_connection, upstream_connection)

            # path
            if self._path_sub.get_num_connections() < 1:
                rospy.loginfo_throttle(1.0, "[Ref Gvn] waiting path...")

            # odom
            if self._odom_sub.get_num_connections() < 1:
                rospy.loginfo_throttle(1.0, "[Ref Gvn] waiting odom...")

            # scan
            if self._scan_sub.get_num_connections() < 1:
                rospy.loginfo_throttle(1.0, "[Ref Gvn] waiting scan...")
        else:
            self._upstream_connection_ready = True
            rospy.loginfo("\n[Ref Gvn] %d upstream connections established !\n", upstream_connection)

    def path_callback(self, path_msg):
        """
        Convert path message to 2d numpy array (num_pts, 2). During this process, 
        simplified path, merging segments within line. for example, if 10 waypoints 
        (pt1, pt2, pt3, ...pt10) are within in a straight line, convert them to (pt1, pt10)
        Note that a path can contain multiple lines. This compression is loseless.
        """
        path_dim = 2
        path_rounding_precision = 2
        rospy.logdebug("Received path!")
        planning_path = np.zeros((len(path_msg.poses), path_dim))
        for idx, pose in enumerate(path_msg.poses):
            planning_path[idx, 0] = pose.pose.position.x
            planning_path[idx, 1] = pose.pose.position.y

        # assuem path accuracy multiple of 1 cm, round to 2 precision
        planning_path.round(path_rounding_precision)
        self._np_path = path_simplify(planning_path)


        # verify simple path in rviz
        if self._publish_simple_path:
            sp_msg = Path()
            sp_msg.header = path_msg.header
            for waypoint in self._np_path:
                # create pose from waypoints
                pose = PoseStamped()
                pose.pose.position.x = waypoint[0]
                pose.pose.position.y = waypoint[1]
                pose.pose.position.z = 0.0
                sp_msg.poses.append(pose)

            self._nav_path_pub.publish(sp_msg)
        return

    def odom_callback(self, msg_odom):
        rospy.logdebug("Received odometry!")
        pose = msg_odom.pose.pose
        quaternion_sxyz = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quaternion_sxyz)
        self._np_z = np.array([pose.position.x, pose.position.y, yaw])
        return

    def scan_callback(self, scan_msg):
        rospy.logdebug("Received LaserScan!")
        # TODOZHL frame update lidar_base_to_obstacle
        # some lidar driver return out_of_range as nan
        scan_ranges = np.array(scan_msg.ranges)
        scan_ranges_filtered = scan_ranges[np.isfinite(scan_ranges)]
        dIlO_sq = np.min(scan_ranges_filtered) ** 2
        #TODOZHL
        rospy.logdebug_throttle(0.5, "d_I(lidar, obstacle)_sq = %.2f" % (dIlO_sq))
        self._np_dist_info = np.array([dIlO_sq])
        return

    def _check_upstream_data(self):
        """ check whether upstream data container are loaded/initialized correctly"""
        # navigation path r
        status = True
        if self._np_path is None:
            rospy.loginfo_throttle(1.0, "[Ref Gvn] waiting nav_path init...")
            status = False

        # robot state z
        if self._np_z is None:
            status = False
            rospy.loginfo_throttle(1.0, "[Ref Gvn] waiting zvec init...")

        # distance info dQgO_sq
        if self._np_dist_info is None:
            status = False
            rospy.loginfo_throttle(1.0, "[Ref Gvn] waiting distance info init...")

        if status:
            # self._check_upstream_data = True
            self._upstream_data_ready = True
            rospy.loginfo_once("\n[Ref Gvn] all %d upstream data initialized !\n" % self._upstream_connection)

    def init_upstream(self):
        """ 
        Init upstream of ref_gvn adaptive tracker. 
            1. check upstream connection
            2. check upstream message and initialize downstream data containers
                for ref gvn core (nav_path r, robot state z, dist info dQgO_sq)
        """
        while (not self._upstream_connection_ready) and (not rospy.is_shutdown()):
            self._check_upstream_connections()
            rospy.sleep(0.1)  # avoid inquery too fast
            rospy.logdebug_throttle(1.0, "waiting upstream connections...")
        rospy.loginfo("[Ref Gvn] upstream [connection] is ready, check upstream [data]...")

        while (not self._upstream_data_ready) and (not rospy.is_shutdown()):
            self._check_upstream_data()
            rospy.sleep(0.1)  # avoid inquery too fast

        rospy.loginfo_throttle(1.0, "[Ref Gvn] upstream [data] is ready!")

        rospy.loginfo("[Ref Gvn] Upstream Init Done!")
