#!/usr/bin/env python3
""" 
ActiveGoal Reference Governor ROS Wrapper.
"""
from typing import List

import numpy as np
import rospy
from erl_msgs.msg import ActRefGvnMsg, EmergencyStopMsg, ActRefGvnDebug
from geometry_msgs.msg import Point, Pose2D
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker

from ref_gvn_ros.pre import RefGvnPreprocess
from ref_gvn_core.actlpg_core import ActRefGvnSE2Core
from ref_gvn_ros.post import RefGvnPostprocess
from ref_gvn_utils.ros_gvn_marker_utils import create_start_marker, create_goal_marker
from ref_gvn_utils.ros_gvn_marker_utils import create_gvn2d_marker, create_lpg2d_marker, create_lpg2d_star_marker
from ref_gvn_utils.ros_gvn_marker_utils import create_ls_marker
from ref_gvn_utils.ros_gvn_marker_utils import create_gvn_dir_marker, create_robot_set_marker


class ActRefGvnWrapper:
    markers: List[Marker]
    mk_ls: Marker
    mk_lpg2d: Marker
    mk_gvn2d: Marker
    mk_goal: Marker
    mk_start: Marker
    mk_gvn_dir: Marker

    def __init__(self, config_dict):
        """ Init ActRefGvnWrapper class.

            This node subscribes (via preprocessor):
                odom from (localization)
                path from (simplified path from A star 2D planner)
                dist_vec [dgO, drg, drO] from perception
                odom velocity from gazebo p3d (for sim)/ control cmd from controller (for robot)

                goal from RViz 2D nav button
                emergency stop from PS4 remote controller

            Publish:
                vis_marker_array
                actref_gvn_status
                ref_gvn_emergency
                actref_gvn_debug
        """
        self.pre = None
        self.core = None
        self.post = None

        # loading external config parameters
        self._config_dict = config_dict

        # -------------------- Constants -------------------------
        self._nPath = 2
        self._dt = 0.02

        # ------------------- ROS Interfaces  --------------------
        self.marker_pub = rospy.Publisher("~vis_marker_array", MarkerArray, queue_size=1)
        self.gvn_status_pub = rospy.Publisher("/actlpg_gvn_status", ActRefGvnMsg, queue_size=1)
        self.emergency_stop_pub = rospy.Publisher("/ref_gvn_emergency", EmergencyStopMsg, queue_size=1)
        self.gvn_debug_pub = rospy.Publisher("/actlpg_gvn_debug", ActRefGvnDebug, queue_size=1)

        # ------------------- Init Modules  --------------------
        self.init_preprocessor()
        self.init_core()
        self.init_postprocessor()
        self.init_markers()

        # set numpy array console print precision = 2
        np.set_printoptions(formatter={'float': '{: 0.2f}'.format})

        rospy.loginfo("REFERENCE GOVERNOR ROS NODE INIT SUCCESSFUL!")

        # ------------------- Cached Property --------------------
        self._last_gvn = None
        self._base_marker = None  # constant created once
        self.start2d = None
        self.goal2d = None
        self.gvn_restart = False
        self.lpg_no_intersection = False
        self.localization_fail = False

    def get_2d_loc(self, xvec):
        """
        Get 2d location from robot states, or governor states
        """
        return xvec[0:self._nPath]

    def init_preprocessor(self):
        """
        Init preprocessor of ref_gvn.
        """
        self.pre = RefGvnPreprocess()

    def init_core(self):
        """
        Init ref_gvn core object. The core class need g0, z0, dist0, goal_loc. 
        More details in ref_core class internal init function.
        """
        z0 = self.pre.np_z
        dist0 = self.pre.np_dist[0]  # dist_info from upstream
        nav_path0 = self.pre.np_path
        mo_list = self.pre.mo_list
        goal2d = nav_path0[-1]  # last waypoint as goal_loc2d
        self._dt = 1.0 / self._config_dict["ctrl_freq"]
        self._kg = self._config_dict["kg"]
        self._gamma = self._config_dict["gamma"]

        self.core = ActRefGvnSE2Core(
            z0=z0,
            dist=dist0,
            nav_path0=nav_path0,
            mo_list=mo_list,
            goal2d=goal2d,
            dt=self._dt,
            kg=self._kg,
            gamma=self._gamma
        )

        # check if path dimension is compatible with ref_gvn
        if self._nPath != self.core._nPath:
            raise ValueError("self._nPath = %d != ref_gvn._nPath = %d" % (self._nPath, self.core._nPath))

        # fill cached container
        self.start2d = self.get_2d_loc(z0)
        self.goal2d = goal2d

    def init_postprocessor(self):
        """
        Init preprocessor of ref_gvn.
        """
        self.post = RefGvnPostprocess()

    def init_markers(self):
        """ 
        Create a bunch of marker for rviz visualization of ref_gvn.
        """
        # ------------ create markers for each object ---------------
        # for start and goal 
        self.mk_start = create_start_marker(self.start2d)
        self.mk_goal = create_goal_marker(self.goal2d)

        # for governor and local projected goal location 
        gvn2d = self.get_2d_loc(self.core.g)
        lpg2d = self.get_2d_loc(self.core.gbar)

        self.mk_gvn2d = create_gvn2d_marker(gvn2d)
        self.mk_lpg2d = create_lpg2d_marker(lpg2d)

        # for active local projected goal location & robot-governor set Ball
        lpg2d_star = self.get_2d_loc(self.core.gbar_star)
        self.mk_lpg2d_star = create_lpg2d_star_marker(lpg2d_star)
        r_gz = self.core.r_gz
        self.mk_robot_ball = create_robot_set_marker(gvn2d, r_gz)

        # create local safe zone and local free space display 
        self.mk_ls = create_ls_marker(gvn2d, self.core.ls_params['radius'])

        # arrows gvn2d --> lpg2d
        self.mk_gvn_dir = create_gvn_dir_marker(gvn2d, lpg2d)

        # assemble markers in makerArray
        self.markers = [self.mk_start, self.mk_goal]
        self.markers += [self.mk_gvn2d, self.mk_lpg2d, self.mk_lpg2d_star, self.mk_robot_ball,
                         self.mk_ls, self.mk_gvn_dir]

        # ------------ publish marker array---------------
        self.marker_pub.publish(self.markers)

    def show_debug_info(self, yaw_idx=2):
        """ 
        Display loop debug info.
        """

        gbar = self.core.gbar
        gbar_star = self.core.gbar_star
        g = self.core.g
        tidx = self.core.tidx
        curr_time = self.core.curr_time
        g2G = self.core.g2G
        deltaE = self.core.deltaE

        rospy.loginfo_throttle(0.5, "[ITER %4d | %6.2f sec] g2G = %.2f " % (tidx, curr_time, g2G))
        rospy.loginfo_throttle(0.5, " g =    [%.2f, %.2f, %.2f (deg)]" % (g[0], g[1], np.rad2deg(g[yaw_idx])))
        rospy.loginfo_throttle(0.5, " gbar = [%.2f, %.2f, %.2f (deg)]" % (gbar[0], gbar[1], np.rad2deg(gbar[yaw_idx])))
        rospy.loginfo_throttle(0.5, " gbar_star = [%.2f, %.2f, %.2f (deg)]" % (
            gbar_star[0], gbar_star[1], np.rad2deg(gbar_star[yaw_idx])))

    def update_moving_markers(self):
        """
        Update non-static markers.
        https://wiki.ros.org/rviz/DisplayTypes/Marker#Sphere_.28SPHERE.3D2.29
        
        Notes: Be careful on scale definition for different type of markers.
        """

        gvn2d = self.get_2d_loc(self.core.g)
        lpg2d = self.core.gbar[0:2]
        lpg2d_star = self.get_2d_loc(self.core.gbar_star)
        r_gz = self.core.r_gz

        # update mk_gvn2d
        self.mk_gvn2d.pose.position.x = gvn2d[0]
        self.mk_gvn2d.pose.position.y = gvn2d[1]

        # update mk_lpg2d
        self.mk_lpg2d.pose.position.x = lpg2d[0]
        self.mk_lpg2d.pose.position.y = lpg2d[1]

        # update mk_lpg2d_star
        self.mk_lpg2d_star.pose.position.x = lpg2d_star[0]
        self.mk_lpg2d_star.pose.position.y = lpg2d_star[1]

        # update mk_robot_ball
        self.mk_robot_ball.pose.position.x = gvn2d[0]
        self.mk_robot_ball.pose.position.y = gvn2d[1]
        self.mk_robot_ball.scale.x = max(0.01, 2 * r_gz)
        self.mk_robot_ball.scale.y = max(0.01, 2 * r_gz)

        # update mk_ls
        self.mk_ls.pose.position.x = gvn2d[0]
        self.mk_ls.pose.position.y = gvn2d[1]
        self.mk_ls.scale.x = 2.0 * self.core.ls_params['radius']
        self.mk_ls.scale.y = 2.0 * self.core.ls_params['radius']

        # update mk_gvn_arrow
        self.mk_gvn_dir.points[0] = Point(gvn2d[0], gvn2d[1], 0.0)
        self.mk_gvn_dir.points[1] = Point(lpg2d[0], lpg2d[1], 0.0)

        # ---------------- re-publish marker array ----------------
        self.marker_pub.publish(self.markers)

    def publish_gvn_state(self):
        """
        Publish governor states.
        """
        # Init Msg Header
        msg = ActRefGvnMsg()
        msg.header = Header()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()

        debug_msg = ActRefGvnDebug()
        debug_msg.header = Header()
        debug_msg.header.frame_id = "map"
        debug_msg.header.stamp = rospy.Time.now()

        # set False status to all status in ActRefGvnDebug
        debug_msg.plan_fail = 0.0
        debug_msg.localization_fail = 0.0
        debug_msg.lpg_no_intersect = 0.0
        debug_msg.safety_violation = 0.0
        debug_msg.qp_infeasible = 0.0
        debug_msg.mo_safety_violation = 0.0

        # perform debug check
        if self.core.plan_fail:
            debug_msg.plan_fail = 0.5

        if self.pre.localization_fail:
            debug_msg.localization_fail = 0.7
            self.localization_fail = True
        else:
            self.localization_fail = False

        if self.core.lpg_status != 0:
            debug_msg.lpg_no_intersect = 0.6
            self.lpg_no_intersection = True
        else:
            self.lpg_no_intersection = False

        if self.core.gvn_status <= -50:
            debug_msg.safety_violation = 0.8

        if self.core.qp_infeasible:
            debug_msg.qp_infeasible = 0.9

        if not self.core.MOsafe:
            debug_msg.mo_safety_violation = 1.0

        self.gvn_debug_pub.publish(debug_msg)

        # Robot Pose2D
        msg.rbt_pose2D = Pose2D(x=self.pre.np_z[0], y=self.pre.np_z[1], theta=self.pre.np_z[2])

        # Governor Dist
        msg.dIcO = self.pre.np_dist[1]
        msg.dIgO = self.pre.np_dist[2]
        msg.dIrO = self.pre.np_dist[3]

        # Governor Pose2D
        msg.gvn_pose2D = Pose2D(x=self.core.g[0], y=self.core.g[1], theta=self.core.g[2])

        # Local Projected goal
        msg.lpg_pose2D = Pose2D(x=self.core.gbar[0], y=self.core.gbar[1], theta=self.core.theta_dsr)

        # Active Local Projected goal
        msg.lpgstar_pose2D = Pose2D(x=self.core.gbar_star[0], y=self.core.gbar_star[1], theta=self.core.theta_dsr)

        # min CBF value w.r.t. moving obstacles
        msg.min_ho = self.core.min_ho

        # Governor Delta E
        msg.deltaE = self.core.deltaE

        # Governor Status
        msg.gvn_status = self.core.gvn_status

        # Publish Msg
        self.gvn_status_pub.publish(msg)

        # For Controller
        emergency_msg = EmergencyStopMsg()
        emergency_msg.header.stamp = rospy.Time.now()
        if self.core.gvn_status < -50:
            emergency_msg.error_msg.data = "DANGER"
            emergency_msg.error_code.data = 250
        elif self.core.gvn_status == -50:
            emergency_msg.error_msg.data = "WARN"
            emergency_msg.error_code.data = 100
        elif self.core.gvn_status == -10:
            emergency_msg.error_msg.data = "INIT FAIL"
            emergency_msg.error_code.data = 50
        elif self.core.gvn_status == 0:
            if not self.core.human_override:
                emergency_msg.error_msg.data = "IDLE"
                emergency_msg.error_code.data = 10
            else:
                emergency_msg.error_msg.data = "OVERRIDE"
                emergency_msg.error_code.data = 0
        else:
            emergency_msg.error_msg.data = "NORMAL"
            emergency_msg.error_code.data = 0

        self.emergency_stop_pub.publish(emergency_msg)

    def gvn_restart_check(self):
        """ 
        In main loop, check if we need to restart governor.
        """
        if not self.pre.gvn_restart_received:
            return
        rospy.logwarn_throttle(1.0, "[ref_gvn_node] >>>>>>>> restart received <<<<<<<<")
        goal_updated = False
        # make sure new path is received after gvn restart requested
        ros_time_gap = self.pre.path_received_time - self.pre.gvn_restart_received_time
        # wait approximately 0.5 seconds
        if self.core.gvn_status >= self.core.IDLE:
            if ros_time_gap.secs > 1:
                goal_updated = True
                if np.size(self.pre.np_path) != 0:
                    self.goal2d = self.pre.np_path[-1]
                    self.mk_goal = create_goal_marker(self.goal2d)
                    self.core._goal2d = self.goal2d
                else:
                    self.mk_goal = create_goal_marker(self.pre.np_z)
                    self.core._goal2d = self.pre.np_z

                self.markers[1] = self.mk_goal
                # if task finished (100), or gvn is IDLE (1) you can approve ref gvn restart request
                if self.core.gvn_status == self.core.GOAL_REACHED or self.core.gvn_status == self.core.IDLE:
                    self.gvn_restart = True
                    # this request is completed
                    self.pre.gvn_restart_received = False
                    rospy.logwarn("[ref_gvn_node] >>>>>>>> restart approved <<<<<<<<")

                # when gvn is running at NORMAL (50), only update goal, deny restart request
                if goal_updated and self.core.gvn_status == self.core.NORMAL:
                    self.pre.gvn_restart_received = False
                    rospy.logwarn("[ref_gvn_node] >>>>>>>> restart denied <<<<<<<<")
        else:
            self.pre.gvn_restart_received = False
            rospy.logwarn("[ref_gvn_node] >>>>>>>> restart denied <<<<<<<<")

    def update(self):
        """
        Update reference as follows:
            1. collect latest data from preprocessor (callback automatically)
            2. execute update loop using core
                    update(self, dist, z, r, v, mo_list, debug=False):
            3. sending new desired robot states to downstream
        """
        # load robot state (position & linear velocity)
        z = self.pre.np_z
        v = self.pre.np_v

        # here to choose between Euclidean/Qudratic distance 
        dIcO = self.pre.np_dist[1]
        dist = dIcO  # Euclidean distance

        # extract path from pre-processor
        r = self.pre.np_path

        # load moving obstacle info list from pre-processor
        mo_list = self.pre.mo_list

        # Check if Ref_Gvn Restart Needed
        self.gvn_restart_check()

        # if localization fail, then restart immediately
        if self.localization_fail:
            rospy.logerr_throttle(1.0, "de-localization detected, please restart experiment manually")

        # gvn restart program
        if self.gvn_restart:
            self.core = None
            self._last_gvn = None
            self.start2d = None
            self.goal2d = None
            self.init_core()
            self.init_markers()
            self.gvn_restart = False
            rospy.logwarn_throttle(0.5, "[ref_gvn_node] ------------ RESTARTED ------------")
        else:
            gvn_state_msg = "[%.2f, %.2f %.2f(deg)]" % (self.core.g[0], self.core.g[1], np.rad2deg(self.core.g[2]))

            rospy.logdebug_throttle(0.5, "[ref_gvn_node] ------------  RUNNING  ------------")
            rospy.logdebug_throttle(0.5, "gvn_status = %d, g2G = %.3f" % (self.core.gvn_status, self.core.g2G))
            rospy.logdebug_throttle(0.5, "gvn_state = %s" % gvn_state_msg)

            # rospy.loginfo_throttle(0.5, "upstream")
            rospy.logdebug_throttle(0.5, "z = [%.2f, %.2f, %.2f (deg)]" % (z[0], z[1], np.rad2deg(z[2])))
            rospy.logdebug_throttle(0.5, "v = [%.2f, %.2f]" % (v[0], v[1]))

            rospy.logdebug_throttle(0.5, "dIcO = %.2f" % dIcO)
            # rospy.loginfo_throttle(0.5, "nav_path = [%s]" % r)

            if self.core.gvn_status > self.core.IDLE:
                self.core.update(dist=dist, r=r, z=z, v=v, mo_list=mo_list, debug=False)
                if not self.core.gvn_goal_reached_flag:

                    # no intersection or e_stop triggered, governor g to robot position
                    if self.lpg_no_intersection or self.pre.e_stop:
                        self.core.g = self.pre.np_z

                    # next governor on unknown, use previous one
                    if not self.pre.gvn_located_on_unknown_flag:
                        self._last_gvn = self.core.g
                    else:
                        self.core.g = self._last_gvn

                    rospy.logdebug_throttle(0.5, "[ref_gvn_node | actlpg_core] ------    NORMAL    ------")
                else:
                    rospy.logwarn_throttle(5.0, "[ref_gvn_node | actlpg_core] ------ GOAL REACHED ------")
                self.post.send_cmd(gvn=self.core.g)

            else:
                self.post.send_cmd(gvn=z)
                self.core.update(dist=dist, r=r, z=z, v=v, mo_list=mo_list, debug=False)
                # show ref_gvn debug info
                self.show_debug_info()
                if self.core.gvn_status == self.core.HALT:
                    rospy.logwarn_throttle(0.5, "[ref_gvn_node | actlpg_core] ------     HALT     ------")
                elif self.core.gvn_status == self.core.WARN:
                    rospy.logwarn_throttle(0.5, "[ref_gvn_node | actlpg_core] ------     WARN     ------")
                elif self.core.gvn_status == self.core.IDLE:
                    rospy.logwarn_throttle(0.5, "[ref_gvn_node | actlpg_core] ------     IDLE     ------")
                else:
                    rospy.logwarn_throttle(0.5, "[ref_gvn_node | actlpg_core] ------     ERROR     ------")

        self.publish_gvn_state()
        # update moving markers in rviz
        self.update_moving_markers()


if __name__ == '__main__':

    config_dict = {'ctrl_freq': 50.0}

    try:
        rospy.init_node('ref_gvn')
        rospy.loginfo("Node Reference Governor Started!\n")

        # set node frequency by reading parameter server 
        config_dict['ctrl_freq'] = rospy.get_param("~ctrl_freq", 50.0)
        # gvn control gain 
        config_dict['kg'] = rospy.get_param("~kg", 1.0)
        # CBF class-K param
        config_dict['gamma'] = rospy.get_param("~classK_gamma", 1.0)

        actlpg_gvn_ros = ActRefGvnWrapper(config_dict=config_dict)
        rate = rospy.Rate(config_dict['ctrl_freq'])

        while not rospy.is_shutdown():
            actlpg_gvn_ros.update()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
