#!/usr/bin/env python
#convert Ros data to cv http://qiita.com/MENDY/items/3a7159f3f032af05c69b
#color detection: https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html
import rospy
import numpy as np
import math
import cv2
from time import sleep
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
rospy.init_node('search_algorighm')
pub1 = rospy.Publisher('/armed3w/diff_drive_controller/cmd_vel', Twist, queue_size = 10)
pub2 = rospy.Publisher('/armed3w/arm_controller/cmd_vel', Twist, queue_size = 10)

glab_flag = False
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
		
	def reset(self):
		self.interval_s = 0.0

timer = Timer()
right_timer = Timer()

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
	pub1.publish(vel)

def hand(direction):
	vel = Twist()
	if 'u' in direction:
		vel.linear.x = -1.0
	if 'd' in direction:
		vel.linear.x = 1.0
	pub2.publish(vel)

class Controller():
	def __init__(self,):
		self.front_minimum_d = 0.7
		self.right_maximum_d = 1.0
		self.front_i_width = 4
		self.right_minimum_i = 0
		self.right_maximum_i = 25
		self.turn_wait_s = 2
		self.delay_before_right_turn_s = 1.3
		self.delay_after_right_turn_s = 2.5
		self.right_turn_phase = 0
		self.glab_phase = 0
		self.glab_dist = 0.31
		self.hand_wait_s = 4
		self.wait_after_glab_s = 1
	
	def calc_feature(self,data):
		lower = int((len(data)-self.front_i_width)/2)
		upper = int((len(data)+self.front_i_width)/2)
		self.front_d = data[lower : upper].mean()
		self.right_d = data[self.right_minimum_i : self.right_maximum_i + 1].min()
	
	def control(self):
		global glab_flag
		if self.glab_phase != 0 and self.glab_phase != 4:
			if self.glab_phase == 1:
				if self.front_d > self.glab_dist:
					move('f')
				else:
					self.glab_phase = 2
					move('b')
			elif self.glab_phase == 2:
				self.glab_phase = 3
				timer.start(self.hand_wait_s)
				move('')
				hand('u')
			elif self.glab_phase == 3:
				self.glab_phase = 4
				move('b')
				timer.start(self.wait_after_glab_s)
		else:
			if self.right_turn_phase == 1:
				self.right_turn_phase = 2
				timer.start(self.turn_wait_s)
				move('r')
			elif self.right_turn_phase == 2:
				right_timer.start(self.delay_after_right_turn_s)
				self.right_turn_phase = 0
			if self.right_turn_phase == 0:
				if self.glab_phase == 0 and glab_flag == True:
					self.glab_phase = 1
					right_timer.reset()
				elif right_timer.is_wating() == False and self.right_d > self.right_maximum_d:
					self.right_turn_phase = 1
					timer.start(self.delay_before_right_turn_s)
					move('f')
				elif self.front_d < self.front_minimum_d:
					timer.start(self.turn_wait_s)
					move('l')
					right_timer.reset()
				else:
					move('f')
		if self.glab_phase != 0 and self.glab_phase != 4:
			rospy.loginfo("glab phase:%s",self.glab_phase)
		elif self.right_turn_phase != 0:
			rospy.loginfo("right turn phase:%s",self.right_turn_phase)
		else:
			rospy.loginfo("front:%s right:%s", self.front_d, self.right_d)

controller = Controller()
def search(message):
	if timer.is_wating() == True:
		return
	controller.calc_feature(np.array(message.ranges))
	controller.control()

def detect(message):
	global glab_flag
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(message, "bgr8")
	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, np.array([50,0,0]),  np.array([150,255,255]))
	if mask.mean() > 0:
		glab_flag = True
	else:
		glab_flag = False
	
#	data.reshape((channel, rows*cols))
#	r=data[0:]
#	g=data[1:]
#	b=data[2:]
#	rospy.loginfo("%s", mask)
#	rospy.loginfo("%s", g)
#	rospy.loginfo("%s", b)

sub1 = rospy.Subscriber('/mybot/camera1/image_raw', Image, detect)
sub2 = rospy.Subscriber('scan', LaserScan, search)

rospy.spin()


