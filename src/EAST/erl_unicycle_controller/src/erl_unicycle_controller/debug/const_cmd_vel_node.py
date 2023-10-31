#!/usr/bin/env python3
""" Constant command velocity generator. 
Use rviz [2D Nav Goal] button as command trigger, one click a time to publish constant twist.
"""



import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from ref_gvn_utils.ros_msg_utils import odom_to_pose2d


class DummyCmdVelGenerator:

    """ A dummy command velocity generator class used for debugging.

        This controller subscribes:
            rviz 2D nav goal (/move_base_simple/goal)
        Publish:
            constant twist command
    """
    def __init__(self):
        # ------------------- subscribers ----------------------
        
        self.dummy_goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)

        self.p3d_odom_sub = rospy.Subscriber('/gazebo_p3d/odom', Odometry, self.p3d_odom_callback)
        self.diff_odom_sub = rospy.Subscriber('/jackal_velocity_controller/odom', Odometry, self.diff_odom_callback)

        # ------------------- publishers ---------------------- 
        # publishers
        # publish velocity command (body twist) for mobile platform hardware / simulated dynamics
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.p3d_odom_pose2d_pub = rospy.Publisher('~p3d_pose2d', Pose2D, queue_size=1)
        self.diff_odom_pose2d_pub = rospy.Publisher('~diff_pose2d', Pose2D, queue_size=1)

        # control limit
        self._v = rospy.get_param("~v", 0.0)
        self._w = rospy.get_param("~w", 1.0)

        self._body_twist = Twist()
        self._body_twist.linear.x = self._v
        self._body_twist.angular.z = self._w
        rospy.logwarn("DummyCmdVelGenerator Inited!")

        # pub message 
        self.p3d_pose2d = None 
        self.diff_pose2d = None

    def p3d_odom_callback(self, msg):
        """
        Convert p3d odom to pose2d msg.
        """
        self.p3d_pose2d = odom_to_pose2d(msg)
        self.p3d_odom_pose2d_pub.publish(self.p3d_pose2d)
        rospy.logdebug_once("p3d odom received, converted and published.")


    def diff_odom_callback(self, msg):
        """
        Convert diff odom to pose2d msg.
        """
        self.diff_pose2d = odom_to_pose2d(msg)
        self.diff_odom_pose2d_pub.publish(self.diff_pose2d)
        rospy.logdebug_once("diff odom received, converted and published.")

    def rviz_goal_callback(self, msg):
        """
        Trigger publisher by chaning rviz 2D nav mannually. 
        """
        # receive goal from rviz button nav2d. The goal is at map frame.
        rospy.logwarn("\npusblish twist cmd [v = %.2f, w = %.2f]" % (self._v, self._w))
        rospy.logdebug("p3d_odom_pose2d  [x = %5.2f, y = %5.2f, theta = %5.2f]" % (self.p3d_pose2d.x, self.p3d_pose2d.y, self.p3d_pose2d.theta))
        rospy.logdebug("diff_odom_pose2d [x = %5.2f, y = %5.2f, theta = %5.2f]" % (self.diff_pose2d.x, self.diff_pose2d.y, self.diff_pose2d.theta))
        # publish cmd vel
        self.cmd_vel_pub.publish(self._body_twist)


if __name__ == '__main__':
    try:
        rospy.init_node('dummy_cmd_vel', log_level=rospy.DEBUG)
        rospy.logwarn("bring up [dummy_cmd_vel] node!\n")
        dummy_goal_gen = DummyCmdVelGenerator()
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr("dummy_cmd_vel node init failed.")
        rospy.signal_shutdown('dummy_cmd_vel node init fail')
        pass
