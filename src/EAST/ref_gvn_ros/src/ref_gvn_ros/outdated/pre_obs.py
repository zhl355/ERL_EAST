#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from erl_msgs.msg import RefGvnDist
from std_msgs.msg import Bool


from tf.transformations import euler_from_quaternion
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
        _dist_topic = rospy.get_param('~dist_topic')
        _path_topic = rospy.get_param('~path_topic')
        _odom_topic = rospy.get_param('~odom_topic')

        self._publish_simple_path = rospy.get_param('~display_nav_path')
        
        rospy.logwarn_once('[ref_gvn_pre] dist_topic: [%s]' % _dist_topic)

        # subscribers
        self._path_sub = rospy.Subscriber(_path_topic, Path, self.path_callback, queue_size=1)
        self._odom_sub = rospy.Subscriber(_odom_topic, Odometry, self.odom_callback, queue_size=1)
        self._dist_sub = rospy.Subscriber(_dist_topic, RefGvnDist, self.dist_callback, queue_size=1)
        
        self._gvn_restart_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.button_callback)
        self._gvn_located_on_unknown_status_sub = rospy.Subscriber('/gvn_located_on_unknown_status', Bool, self.gvn_located_on_unknown_status_callback)
        self._estop_sub = rospy.Subscriber("/e_stop", Bool, self.estop_callback, queue_size=1)

        # publishers for display simple path (navigation path r)
        if self._publish_simple_path:
            self._nav_path_pub = rospy.Publisher("~nav_path", Path, queue_size=1)

        # upstream status variables
        self._upstream_connection = 0
        self._upstream_connection_ready = False
        self._upstream_data_ready = False
        self.gvn_restart_received = False
        self.gvn_located_on_unknown_flag = False
        self.localization_fail = False

        # ------------------- upstream data numpy container --------------------

        # containers for converted message in numpy
        self.np_path = None
        self.np_dist = None
        self.np_z = None
        self._prev_rbt_loc = np.array([0, 0, 0])
        self.e_stop = True

        # cached previous robot location
        self._dist_msg = None

        self.path_received_time = rospy.Time.now()
        self.gvn_restart_received_time = rospy.Time.now()

        # ------------------- Init Upstream --------------------
        self.init_upstream()
        rospy.loginfo("[Ref Gvn Preprocessor Created!]  \n")

    def _check_upstream_connections(self, upstream_connection=3):
        """ 
        Check whether all subscribers' uplink connections are established successfully.
        """

        self._upstream_connection = \
            self._path_sub.get_num_connections() + \
            self._odom_sub.get_num_connections() + \
            self._dist_sub.get_num_connections()

        if self._upstream_connection < upstream_connection:
            # we need to wait path, states and lidar all ready
            rospy.logwarn_throttle(1.0, '[ref_gvn_pre] waiting upstream connections [%d / %d]:', self._upstream_connection, upstream_connection)

            # path
            if self._path_sub.get_num_connections() < 1:
                rospy.loginfo_throttle(1.0, "[ref_gvn_pre] waiting path...")

            # odom
            if self._odom_sub.get_num_connections() < 1:
                rospy.loginfo_throttle(1.0, "[ref_gvn_pre] waiting odom...")

            # dist
            if self._dist_sub.get_num_connections() < 1:
                rospy.loginfo_throttle(1.0, "[ref_gvn_pre] waiting dist...")
        else:
            self._upstream_connection_ready = True
            rospy.loginfo("\n[ref_gvn_pre] %d upstream connections established !\n", upstream_connection)

    def path_callback(self, path_msg):
        """
        Convert path message to 2d numpy array (num_pts, 2). During this process, 
        simplified path, merging segments within line. for example, if 10 waypoints 
        (pt1, pt2, pt3, ...pt10) are within in a straight line, convert them to (pt1, pt10)
        Note that a path can contain multiple lines. This compression is lossless.
        """
        self.path_received_time = path_msg.header.stamp
        path_dim = 2
        path_rounding_precision = 2
        rospy.logdebug_throttle(0.5, "[ref_gvn_pre] received path")
        planning_path = np.zeros((len(path_msg.poses), path_dim))
        for idx, pose in enumerate(path_msg.poses):
            planning_path[idx, 0] = pose.pose.position.x
            planning_path[idx, 1] = pose.pose.position.y

        # assume path accuracy multiple of 1 cm, round to 2 precision
        planning_path.round(path_rounding_precision)
        self.np_path = path_simplify(planning_path)

        # verify simple path in rviz
        if self._publish_simple_path:
            sp_msg = Path()
            sp_msg.header = path_msg.header
            for waypoint in self.np_path:
                # create pose from waypoints
                pose = PoseStamped()
                pose.pose.position.x = waypoint[0]
                pose.pose.position.y = waypoint[1]
                pose.pose.position.z = 0.0
                sp_msg.poses.append(pose)

            self._nav_path_pub.publish(sp_msg)
        return

    def button_callback(self, msg):
        """
        Trigger dummy planner.
        """
        self.gvn_restart_received = True
        self.gvn_restart_received_time = msg.header.stamp

    def odom_callback(self, odom_msg):
        """
        1. get robot location from odom message. 
        2. monitor localization status, report de-localization error in debug message

        """
        rospy.logwarn_once("[ref_gvn_pre] Received odometry!")
        pose = odom_msg.pose.pose
        quaternion_sxyz = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quaternion_sxyz)
        self.np_z = np.array([pose.position.x, pose.position.y, yaw])
        self.check_de_localization()
        return

    def dist_callback(self, dist_msg):
        """
        Cache ref gvn distance message.
        """
        rospy.logwarn_once("[ref_gvn_pre] received dist")
        self._dist_msg = dist_msg
        self.np_dist = np.array([dist_msg.dQcO, dist_msg.dIcO, dist_msg.dIgO, dist_msg.dIrO])
        return

    def estop_callback(self, estop_msg):
        """
        Cache estop message.
        """
        self.e_stop = estop_msg.data

    def _check_upstream_data(self):
        """ 
        Check whether upstream data container are loaded/initialized correctly.
        """
        # navigation path r
        status = True
        if self.np_path is None:
            status = False
            rospy.loginfo_throttle(1.0, "[ref_gvn_pre] waiting nav_path init...")

        # robot state z
        if self.np_z is None:
            status = False
            rospy.loginfo_throttle(1.0, "[ref_gvn_pre] waiting zvec init...")

        # distance info dQgO_sq
        if self.np_dist is None:
            status = False
            rospy.loginfo_throttle(1.0, "[ref_gvn_pre] waiting dist init...")

        if status:
            self._upstream_data_ready = True
            rospy.loginfo_once("\n[ref_gvn_pre] all %d upstream data initialized !\n" % self._upstream_connection)

        if self.np_z is not None and self.np_dist is not None and self.np_path is None:
            # remind user to click 2D Nav Button to set goal to start reference governor
            rospy.logwarn_throttle(1.0, "[ref_gvn_pre] ------------ Click [Rviz 2D Nav Button] to Start ------------")

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
        rospy.loginfo("[ref_gvn_pre] upstream [connection] is ready, check upstream [data]...")

        while (not self._upstream_data_ready) and (not rospy.is_shutdown()):
            self._check_upstream_data()
            rospy.sleep(0.1)  # avoid inquery too fast

        rospy.loginfo_throttle(1.0, "[ref_gvn_pre] upstream [data] is ready!")

        rospy.loginfo("[ref_gvn_pre] Upstream Init Done!")

    def gvn_located_on_unknown_status_callback(self, msg):
        """
        Set gvn_located_on_unknown_flag according upstream msg.
        """
        self.gvn_located_on_unknown_flag = msg.data

    def check_de_localization(self):
        """
        This function checks whether de-localization occurs.
        If the robot location different too much between in the direction perpendicular to the robot heading,
        then a de-localization is considered to be occurred.
        """
        current_rbt_loc = self.np_z[0:2]
        previous_rbt_loc = self._prev_rbt_loc[0:2]
        rbt_heading = self.np_z[2]
        dist_perp = np.abs((current_rbt_loc - previous_rbt_loc) @ np.array([[-np.sin(rbt_heading)], [np.cos(rbt_heading)]]))

        if dist_perp <= 0.1 and not self.localization_fail:
            self._prev_rbt_loc = self.np_z
        else:
            self.localization_fail = True
            self.np_z = self._prev_rbt_loc
