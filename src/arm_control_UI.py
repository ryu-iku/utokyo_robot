#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('arm_publisher')
pub = rospy.Publisher('/armed3w/arm_controller/cmd_vel', Twist, queue_size = 10)
while not rospy.is_shutdown():
	vel = Twist()
	direction = raw_input('u: up, d: down > ')
	if 'u' in direction:
		vel.linear.x = -2.0
	if 'd' in direction:
		vel.linear.x = 2.0
	if 'q' in direction:
		berak
	print vel
	pub.publish(vel)

