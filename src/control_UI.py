#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist

rospy.init_node('vel_publisher')
pub = rospy.Publisher('/armed3w/diff_drive_controller/cmd_vel', Twist, queue_size = 10)
while not rospy.is_shutdown():
	vel = Twist()
	direction = raw_input('f: forward, b: back, l:left, r:right > ')
	if 'f' in direction:
		vel.linear.x = 2.0
	if 'b' in direction:
		vel.linear.x = -2.0
	if 'l' in direction:
		vel.angular.z = -math.pi/4
	if 'r' in direction:
		vel.angular.z = math.pi/4
	if 'q' in direction:
		berak
	print vel
	pub.publish(vel)
