import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

class MovingControl:
    def __init__(self):
        self.start_angle = - math.pi
        self.end_angle = math.pi
        self.sample_n = 21
        self.max_range = 20
        self.min_range = .1
        self.shortest_road = 1

        self.back_i = 0
        self.left_i = int(self.sample_n / 4)
        self.front_i = int(self.sample_n / 2)
        self.right_i = int(self.sample_n * 3 / 4)
        self.left_back_i = int(self.sample_n * 1 / 8)
        self.left_front_i = int(self.sample_n * 3 / 8)
        self.right_front_i = int(self.sample_n * 5 / 8)
        self.right_back_i = int(self.sample_n * 7 / 8)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(10)

        self.state_set = {"start", "straight_road", "corner", "end", "turn_around_at_corner", "turn_around_at_end", "turning_over", "goal"}
        self.state = "start"

        self.cmd_vel_publisher = rospy.Publisher('/armed3w/diff_drive_controller/cmd_vel',
                        Twist, queue_size = 1)
        rospy.Subscriber("/scan", LaserScan, self.update_scan_data, queue_size=1)
        rospy.spin()

    def update_scan_data(self, msg):
        laser_ranges = msg.ranges
        mean_ranges = []
        for i in range(self.sample_n):
            curr_s = laser_ranges[i]
            prev_s = laser_ranges[i - 1]
            next_s = laser_ranges[(i + 1) % self.sample_n]

            mean_s = sum((curr_s, prev_s, next_s)) / 3
            mean_ranges.append(mean_s)

        self.laser_ranges = mean_ranges
        print self.laser_ranges

        self.update_state_and_move()
        # self.update_sensor_data_analysis()

        # self.move_robot()

    def update_state_and_move(self):
        self.update_sensor_data_analysis()
        print(self.local_max_arr)
        print(self.local_min_arr)
        print(self.state)

        if self.state == "start":
            self.state_handle()

        elif self.state == "straight_road":
            self.go_straight(.1)
            self.state_handle()

        elif self.state == "corner":
            self.stop()

            self.record_sensor_data()
            self.state = "turn_around_at_corner"

        elif self.state == "end":
            self.stop()
            self.record_sensor_data()
            self.state = "turn_around_at_end"

        elif self.state == "turn_around_at_corner":
            self.turn_direction(-.05)
            if self.check_turning_over():
                self.state = "turning_over"

        elif self.state == "turn_around_at_end":
            self.turn_direction(-.05)
            if self.check_turning_over():
                self.state = "turning_over"

        elif self.state == "turning_over":
            self.state = "straight_road"

    def update_sensor_data_analysis(self):
        local_max_arr = []
        local_min_arr = []

        for i in range(self.sample_n):
            curr_s = self.laser_ranges[i]
            prev_s = self.laser_ranges[i - 1]
            next_s = self.laser_ranges[(i + 1) % self.sample_n]

            if curr_s > prev_s and curr_s >= next_s:
                local_max_arr.append(i)
            if curr_s < prev_s and curr_s <= next_s:
                local_min_arr.append(i)

        self.front_max = self.front_i in local_max_arr
        self.front_min = self.front_i in local_min_arr

        self.back_max = self.back_i in local_max_arr
        self.back_min = self.back_i in local_min_arr

        self.right_max = self.right_i in local_max_arr
        self.right_min = self.right_i in local_min_arr

        self.left_max = self.left_i in local_max_arr
        self.left_min = self.left_i in local_min_arr

        self.local_max_arr = local_max_arr
        self.local_min_arr = local_min_arr

    def state_handle(self):
        if self.front_max and self.right_min and self.left_min:
            self.state = "straight_road"
        elif self.right_max or self.left_max:
            if min(self.laser_ranges) > 1:
                self.state = "corner"
            else:
                self.state = "straight_road"
        else:
            self.state = "straight_road"

    def record_sensor_data(self):
        self.record_lr = self.laser_ranges
        self.record_lma = self.local_max_arr
        # self.record_lmn = self.local_max_n

    def check_turning_over(self):
        return self.front_max and (self.right_max or self.right_min) and\
                (self.left_max or self.left_min)

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


    ###############
    def sub_callback(self, msg):
        self.init_timer()

    def publish_once_in_cmd_vel(self, cmd):
        while not self.ctrl_c:
            connections = self.cmd_vel_publisher.get_num_connections()
            if connections > 0:
                self.cmd_vel_publisher.publish(cmd)
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        self.stop_robot()
        self.ctrl_c = True

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel(cmd)


if __name__ == "__main__":
    rospy.init_node('MovingControl')
    MovingControl()
    # mc = MovingControl()
    # mc.move_robot()
