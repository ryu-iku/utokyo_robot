import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import math
import time
import datetime as dt
import numpy as np

class SensorDataRecord:
    def __init__(self):
        self.record_data = []
        self.orientation = None
        self.laser_ranges = None

        self.record_time = 20 #seconds

    def start(self):
        rospy.init_node('MovingControl')
        self.rate = rospy.Rate(10)

        self.cmd_vel_publisher = rospy.Publisher('/armed3w/diff_drive_controller/cmd_vel',
                        Twist, queue_size=1)

        start_time = time.time()
        while time.time() - start_time <= self.record_time:
            rospy.Subscriber("/armed3w/diff_drive_controller/odom", Odometry, self.update_odo_data, queue_size=1)
            rospy.Subscriber("/scan", LaserScan, self.update_scan_data, queue_size=1)

            if self.orientation != None and self.laser_ranges != None:
                self.record_data.append([self.orientation] + list(self.laser_ranges))

            self.turn_direction(.5)
            self.rate.sleep()

        self.record_to_file()
        rospy.signal_shutdown("Record finished!")

    def update_odo_data(self, msg):
        orientation_q = msg.pose.pose.orientation
        _, _, self.orientation = euler_from_quaternion([orientation_q.x,
        orientation_q.y, orientation_q.z, orientation_q.w])

    def update_scan_data(self, msg):
        self.laser_ranges = msg.ranges

    def turn_direction(self, speed):
        vel = Twist()
        vel.angular.z = speed
        self.cmd_vel_publisher.publish(vel)

    def record_to_file(self):
        now = dt.datetime.now()
        file_name = "data/%s%s%s.record"%(now.hour, now.minute, now.second)
        np.savetxt(file_name, np.array(self.record_data), delimiter=",")

if __name__ == "__main__":
    sdr = SensorDataRecord()
    sdr.start()
