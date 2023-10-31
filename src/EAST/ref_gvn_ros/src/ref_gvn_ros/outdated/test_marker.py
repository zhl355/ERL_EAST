#! /usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

rospy.init_node('rviz_marker')

marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

marker = Marker()

marker.header.frame_id = "/map"
marker.header.stamp = rospy.Time.now()

# set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
marker.type = 0
marker.id = 0

# # Set the scale of the marker
# marker.scale.x = 0.02
# marker.scale.y = 0.03
# marker.scale.z = 0.02

# Set the color
marker.color.r = 0.5
marker.color.g = 0.0
marker.color.b = 0.5
marker.color.a = 1.0

# Set the pose of the marker
marker.pose.position.x = 0
marker.pose.position.y = 0
marker.pose.position.z = 3
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0


p1 = Point(0, 0, 0)
p2 = Point(1, 1, 1)

# use start and end points to specify arrow 
# You can also specify a start/end point for the arrow, using the points member. 
# If you put points into the points member, it will assume you want to do things this way.
# The point at index 0 is assumed to be the start point, and the point at index 1 is assumed to be the end.
# scale.x is the shaft diameter, 
# scale.y is the head diameter
# If scale.z is not zero, it specifies the head length.

# Set the scale of the marker
marker.scale.x = 0.1
marker.scale.y = 0.2
marker.scale.z = 0.3

marker.points = [p1, p2]

while not rospy.is_shutdown():
  marker_pub.publish(marker)
  rospy.rostime.wallsleep(1.0)
