import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


import math
import time

class TurningControl:
    def __init__(self, goal_orientation):
        self.start_angle = - math.pi
        self.end_angle = math.pi
        self.sample_n = 41
        self.round_angle = 2 * math.pi

        rospy.init_node('MovingControl')
        self.rate = rospy.Rate(10)
        self.cmd_vel_publisher = rospy.Publisher('/armed3w/diff_drive_controller/cmd_vel',
                        Twist, queue_size = 1)

        self.goal_orientation = goal_orientation % self.round_angle
        self.orientation = None

    def start(self):
        while not rospy.is_shutdown():
            rospy.Subscriber("/armed3w/diff_drive_controller/odom", Odometry, self.update_odo_data, queue_size=1)
            rospy.Subscriber("/scan", LaserScan, self.update_scan_data, queue_size=1)
            self.turn_over()
            self.rate.sleep()

    def update_odo_data(self, msg):
        orientation_q = msg.pose.pose.orientation
        _, _, self.orientation = euler_from_quaternion([orientation_q.x,
        orientation_q.y, orientation_q.z, orientation_q.w])

        self.orientation = self.orientation % self.round_angle

    def update_scan_data(self, msg):
        self.laser_ranges = msg.ranges

    def turn_over(self):
        if self.orientation == None:
            return False

        print "goal_orientation", self.goal_orientation
        print "cur_orientation", self.orientation
        if abs(self.goal_orientation - self.orientation) <= .01:
            self.goal_orientation = None
            rospy.signal_shutdown("Turning finished!")
        elif (self.goal_orientation > self.orientation and\
        self.goal_orientation - self.orientation <= math.pi) or\
        (self.goal_orientation < self.orientation and\
        self.orientation - self.goal_orientation > math.pi):
            self.turn_direction(.1)
        else:
            self.turn_direction(-.1)

    def turn_direction(self, speed):
        vel = Twist()
        vel.angular.z = speed
        self.cmd_vel_publisher.publish(vel)


if __name__ == "__main__":
    tc = TurningControl(-1.56)
    tc.start()
