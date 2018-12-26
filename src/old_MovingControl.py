import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math
import time

class MovingControl:
    def __init__(self):
        self.start_angle = - math.pi
        self.end_angle = math.pi
        self.sample_n = 41
        self.max_range = 20
        self.min_range = .1
        self.shortest_road = 1

        self.back_i = 0
        self.left_i = int((self.sample_n - 1) / 4)
        self.front_i = int((self.sample_n - 1) / 2)
        self.right_i = int((self.sample_n - 1) * 3 / 4)
        self.left_back_i = int((self.sample_n - 1) * 1 / 8)
        self.left_front_i = int((self.sample_n - 1) * 3 / 8)
        self.right_front_i = int((self.sample_n - 1) * 5 / 8)
        self.right_back_i = int((self.sample_n - 1) * 7 / 8)

        self.state_set = {"start", "straight_road", "corner", "end", "turn_around_at_corner", "turn_around_at_end", "turning_over", "goal"}
        self.state = "start"
        self.mean_window = 1

        rospy.init_node('MovingControl')
        self.rate = rospy.Rate(10)
        self.cmd_vel_publisher = rospy.Publisher('/armed3w/diff_drive_controller/cmd_vel',
                        Twist, queue_size = 1)

    def start(self):
        while not rospy.is_shutdown():
            rospy.Subscriber("/armed3w/diff_drive_controller/odom", Odometry, self.update_odo_data, queue_size=1)
            rospy.Subscriber("/scan", LaserScan, self.update_scan_data, queue_size=1)

        # rospy.spin()

    def update_odo_data(self, msg):
        # self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation.z

        # print "position", self.position
        print "orientation", self.orientation

        self.rate.sleep()

    def update_scan_data(self, msg):
        laser_ranges = msg.ranges
        mean_ranges = []
        for i in range(self.sample_n):
            s_sum = 0
            for j in range(self.mean_window):
                cal_i = (i - int(self.mean_window / 2) + j) % self.sample_n
                s_sum += laser_ranges[cal_i]
            mean_ranges.append(s_sum / self.mean_window)

        self.laser_ranges = mean_ranges
        print self.laser_ranges

        self.update_state_and_move()

        # self.move_robot()

        self.rate.sleep()

    def update_state_and_move(self):
        self.update_sensor_data_analysis()
        print(self.state)

        if self.state == "start":
            self.state_handle()

        elif self.state == "straight_road":
            self.go_straight(1)
            self.state_handle()

        elif self.state == "corner":
            self.stop()
            self.set_moving_goal()
            self.state = "turn_around_at_corner"

        elif self.state == "end":
            self.stop()
            self.set_moving_goal()
            self.state = "turn_around_at_end"

        elif self.state == "turn_around_at_corner":
            self.turn_direction(-.5)
            if self.check_turning_over():
                self.state = "turning_over"

        elif self.state == "turn_around_at_end":
            self.turn_direction(-.5)
            if self.check_turning_over():
                self.state = "turning_over"

        elif self.state == "turning_over":
            self.state = "straight_road"

    def update_sensor_data_analysis(self):
        max_sensor_arr = []
        msa_front = []

        for i in range(self.sample_n):
            curr_s = self.laser_ranges[i]
            prev_s = self.laser_ranges[i - 1]
            next_s = self.laser_ranges[(i + 1) % self.sample_n]

            if curr_s > prev_s and curr_s >= next_s:
                max_sensor_arr.append(i)
                if curr_s > 2 and i <= self.right_back_i and i >= self.left_back_i - 1:
                    msa_front.append(i)

        ava_direction = []
        for i in msa_front:
            direction_minus = i - int(self.sample_n / 8)
            direction_plus = i + int(self.sample_n / 8)
            print "direction_minus", direction_minus
            print "direction_plus", direction_plus
            if min(self.laser_ranges[direction_minus:direction_plus]) > 1:
                ava_direction.append(i)

        self.max_sensor_arr = max_sensor_arr
        self.msa_front = msa_front
        self.ava_direction = ava_direction

        print self.max_sensor_arr
        print self.msa_front
        print self.ava_direction


    def state_handle(self):
        if len(self.ava_direction) == 0:
            self.state = "end"
        elif len(self.ava_direction) == 1:
            self.state = "straight_road"
        else:
            self.state = "corner"


    def set_moving_goal(self):
        ori_position = self.position
        ori_orientation = self.orientation

        self.goal_orientation = None
        for i in range(len(self.max_sensor_arr)):
            next_i = (i + 1) % len(self.max_sensor_arr)
            if (self.front_i >= self.max_sensor_arr[i]) and\
             (self.front_i < self.max_sensor_arr[next_i] or next_i == 0):
                self.goal_orientation = self.si_to_orientation(self.max_sensor_arr[next_i])
                break

    def si_to_orientation(self, si):
        si_theta = 2 * math.pi * si / (self.sample_n - 1)
        return self.orientation - math.pi + si_theta

    def check_turning_over(self):
        print "goal_orientation", self.goal_orientation
        print "cur_orientation", self.orientation
        if self.goal_orientation == None:
            print "No goal orientation!!"
            return False
        return self.goal_orientation - self.orientation < .1

    def stop(self):
        vel = Twist()
        vel.linear.x = 0
        vel.angular.z = 0
        self.cmd_vel_publisher.publish(vel)

    def go_straight(self, speed):
        vel = Twist()
        vel.linear.x = speed
        self.cmd_vel_publisher.publish(vel)

    def turn_direction(self, speed):
        vel = Twist()
        vel.angular.z = speed
        self.cmd_vel_publisher.publish(vel)



if __name__ == "__main__":
    mc = MovingControl()
    mc.start()
