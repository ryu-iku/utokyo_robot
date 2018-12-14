#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

rospy.init_ndoe('vel_publisher')
pub = rospy.publisher('/armed3w/diff_drive_controller/cmd_vel', Twist, quesize = 10)
while not rospy.is_shutdown():
	vel = Twist()
	direction = raw_input('f: forward, b: back, l:left, r:right > ')
	if 'f' in direction:
		vel.linear.x = 0.5
	if 'b' in direction:
		vel.linear.x = -0.5
	if 'l' in direction:
		vel.angular.z = 1.0
	if 'r' in direction:
		vel.angular.z = -1.0
	if 'q' in direction:
		berak
	print vel
	pub.publish(vel)

