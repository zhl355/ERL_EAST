#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from ref_gvn_utils.ros_planning_utils import create_path_msg_from_waypoints

jackal_race_waypoints = np.array([
    [0, 0],
    [6, 6],
    [6, 8],
    [0, 8],
    [-8, 2],
    [-8, -1],
    [-3, -1],
], dtype=np.float)


def create_two_point_dummy_path():
    """
    Create a two-point dummy path message.
    """
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()

    pose_stamped_start = PoseStamped()
    pose_stamped_start.pose.position.x = 0.0
    pose_stamped_start.pose.position.y = 0.0
    pose_stamped_start.pose.orientation.w = 1.0

    pose_stamped_end = PoseStamped()
    pose_stamped_end.pose.position.x = 3.0
    pose_stamped_end.pose.position.y = 3.0
    pose_stamped_end.pose.orientation.w = 1.0

    path.poses.append(pose_stamped_start)
    path.poses.append(pose_stamped_end)
    return path


class DummyPlannerGenerator:
    static_path: Path

    def __init__(self, config_dict=None):
        """ Init dummy planner class.

            This node subscribes:
                rviz 2D nav goal (/move_base_simple/goal)
            Publish:
                hard-coded simple straight line path message
        """
        # loading external config parameters
        self._config_dict = config_dict

        self._use_external_path = rospy.get_param('~use_external_path', False)
        if not self._use_external_path:
            waypoints = jackal_race_waypoints
        else:
            path_file = rospy.get_param('~ext_path')
            ext_data = np.load(path_file)  # load npy file contains path
            waypoints = ext_data['path']

        print('Got waypoint path from [%.2f %.2f] to [%.2f, %.2f]' % (
            waypoints[0, 0], waypoints[0, 1],
            waypoints[-1, 0], waypoints[-1, 1]))

        # ------------------- subscribers ----------------------
        self.rviz_Nav2D_button_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.button_callback)

        # ------------------- publishers ---------------------- 
        # rviz goal 
        self.path_pub = rospy.Publisher('/path', Path, queue_size=1)
        self.static_path = create_path_msg_from_waypoints(waypoints=waypoints)

        # -------------------- other status -------------------------
        self._start_planning = False
        self.button_count = 0
        self.rbt_loc_xy = None

    def button_callback(self, msg):
        """
        Trigger dummy planner.
        """
        self._start_planning = True
        rospy.loginfo("[dummy planner] start planning")
        if self.button_count == 1:
            self.replan()
            self.gvn_restart_pub.publish(True)
            rospy.loginfo_throttle(1.0, "[dummy planner] Replan Path")

        self.button_count = self.button_count + 1

    def publish_dummy_path(self):
        """
        Publish cached goal if goal received. 
        """
        if self._start_planning:
            self.static_path.header.stamp = rospy.Time.now()
            self.path_pub.publish(self.static_path)


if __name__ == '__main__':

    try:
        rospy.init_node('dummy_path')
        rospy.loginfo("bring up [dummy_path] node!\n")

        # loading parameters
        pub_path_freq = rospy.get_param("~pub_path_freq", 50.0)
        dummy_path_gen = DummyPlannerGenerator()
        rate = rospy.Rate(pub_path_freq)

        while not rospy.is_shutdown():
            # publish path at certain freq
            dummy_path_gen.publish_dummy_path()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr("[dummy path] node init failed.")
        rospy.signal_shutdown('[dummy path] node init fail')
        pass
