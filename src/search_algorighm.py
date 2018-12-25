#!/usr/bin/env python
import rospy
import numpy as np
import math
from time import sleep
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
rospy.init_node('search_algorithm')
pub = rospy.Publisher('/armed3w/diff_drive_controller/cmd_vel', Twist, queue_size = 10)

class Timer():
	def __init__(self,):
		self.timer = rospy.Time()
		self.interval_s = 0.0
	
	def start(self,t):
		self.timer = rospy.get_rostime()
		self.interval_s = t

	def is_wating(self):
		if self.interval_s == 0.0:
			return False
		if rospy.get_rostime() - self.timer > rospy.Duration.from_sec(self.interval_s):
			self.interval_s = 0.0
			return False
		return True

timer = Timer()

def move(direction):
	vel = Twist()
	if 'f' in direction:
		vel.linear.x = 2.0
	if 'b' in direction:
		vel.linear.x = -2.0
	if 'l' in direction:
		vel.angular.z = -math.pi/4
	if 'r' in direction:
		vel.angular.z = math.pi/4
	pub.publish(vel)

class Controller():
	def __init__(self,):
		self.front_minimum_d = 0.6
		self.right_maximum_d = 1.0
		self.front_i_width = 4
		self.right_minimum_i = 0
		self.right_maximum_i = 3
		self.turn_wait_s = 2
		self.delay_before_right_turn_s = 1.3
		self.delay_after_right_turn_s = 1.8
		self.right_turn_phase = 0
	
	def calc_feature(self,data):
		lower = int((len(data)-self.front_i_width)/2)
		upper = int((len(data)+self.front_i_width)/2)
		self.front_d = data[lower : upper].mean()
		self.right_d = data[self.right_minimum_i : self.right_maximum_i + 1].mean()
	
	def control(self):
		if self.right_turn_phase == 2:
			self.right_turn_phase = 0
			timer.start(self.delay_after_right_turn_s)
			move('f')
		elif self.right_turn_phase == 1:
			self.right_turn_phase = 2
			timer.start(self.turn_wait_s)
			move('r')
		elif self.front_d < self.front_minimum_d:
			timer.start(self.turn_wait_s)
			move('l')
		elif self.right_d > self.right_maximum_d:
			self.right_turn_phase = 1
			timer.start(self.delay_before_right_turn_s)
			move('f')
		else:
			move('f')
		rospy.loginfo("front:%s right:%s", self.front_d, self.right_d)

controller = Controller()
def search(message):
	if timer.is_wating() == True:
		return
	controller.calc_feature(np.array(message.ranges))
	controller.control()

sub = rospy.Subscriber('scan', LaserScan, search)
rospy.spin()


