import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


import math
import time

PI = math.pi

class MovingControl:
    def __init__(self):
        self.start_angle = - PI
        self.end_angle = PI
        self.sample_n = 101
        self.max_range = 20
        self.min_range = .1
        self.shortest_road = 2
        self.safe_distance = 1
        # self.danger_distance = .6
        self.round_angle = 2 * PI
        self.mean_window = 5
        self.straight_speed = 3 #To be tuned!
        self.turn_speed = .5
        self.turn_angle_bias = .05

        self.quarter_lsi = int((self.sample_n - 1) / 4)
        self.back_i = 0
        self.right_i = self.quarter_lsi
        self.front_i = self.quarter_lsi * 2
        self.left_i = self.quarter_lsi * 3
        self.right_front_i = int((self.sample_n - 1) * 1 / 8)
        self.right_back_i = int((self.sample_n - 1) * 3 / 8)
        self.left_back_i = int((self.sample_n - 1) * 5 / 8)
        self.left_front_i = int((self.sample_n - 1) * 7 / 8)

        # self.state_set = {"start", "straight_road", "corner", "end", "turn_around_at_corner", "turn_around_at_end", "turning_over", "goal"}
        self.state = "start"
        # self.mean_window = 1

        self.laser_ranges = None
        self.orientation = None
        self.goal_orientation = None
        self.turning = False

        rospy.init_node('MovingControl')
        self.rate = rospy.Rate(20)
        self.cmd_vel_publisher = rospy.Publisher('/armed3w/diff_drive_controller/cmd_vel',
                        Twist, queue_size=1)

    def start(self):
        while not rospy.is_shutdown():
            rospy.Subscriber("/armed3w/diff_drive_controller/odom", Odometry, self.update_odo_data, queue_size=1)
            rospy.Subscriber("/scan", LaserScan, self.update_scan_data, queue_size=1)
            if self.laser_ranges != None and self.orientation != None:
                self.move_on()
            self.rate.sleep()

    def update_odo_data(self, msg):
        orientation_q = msg.pose.pose.orientation
        _, _, self.orientation = euler_from_quaternion([orientation_q.x,
        orientation_q.y, orientation_q.z, orientation_q.w])

        self.orientation = self.orientation % self.round_angle

    def update_scan_data(self, msg):
        self.laser_ranges = msg.ranges

    def update_sensor_data_analysis(self):
        lr_moving_average = self.moving_average(self.laser_ranges, int(self.sample_n / 8))

        max_sensor_arr = []
        msa_front = []

        for i in range(self.sample_n):
            curr_s = lr_moving_average[i]
            prev_s = lr_moving_average[i - 1]
            next_s = lr_moving_average[(i + 1) % self.sample_n]

            if curr_s > prev_s and curr_s >= next_s and curr_s >= self.shortest_road :
                max_sensor_arr.append(i)
                if i >= self.right_i and i <= self.left_i:
                    msa_front.append(i)

        self.max_sensor_arr = max_sensor_arr
        self.msa_front = msa_front

        print self.max_sensor_arr
        print self.msa_front
        print map(lambda i:self.si_to_orientation(i), self.msa_front)

    def bump_range(self, si):
        r_start = min(si, self.front_i, (si - self.quarter_lsi / 2))
        r_end = max(si, self.front_i, (si + self.quarter_lsi / 2 + 1))

        front_range = self.laser_ranges[r_start:(r_end + 1)]

        min_rs, itr = min(list(zip(front_range, range(len(front_range)))))
        min_ri = itr + r_start

        return min_rs, min_ri

    # Go along one side wall
    def move_on(self):
        self.update_sensor_data_analysis()

        if self.state == "go_along_wall":
            si = self.ori_to_si(self.real_goal_orientation)
            # front_range = self.laser_ranges[(si - self.quarter_lsi / 4):(si + self.quarter_lsi / 4 + 1)]
            min_rs, min_ri = self.bump_range(si)

            if min_rs >= self.safe_distance:
                self.goal_orientation = self.real_goal_orientation
                if abs(self.real_goal_orientation - self.orientation) <= self.turn_angle_bias:
                    self.state = "go_straight"

            print("state = %s, length = %s"%(self.state, self.laser_ranges[self.front_i]))
            self.turn_over_and_go()

        elif len(self.msa_front) >= 1:
            si = self.msa_front[-1]

            # front_range = self.laser_ranges[si:max((si + self.quarter_lsi / 4 + 1), self.front_i + 1)]
            min_rs, min_ri = self.bump_range(si)
            print("min_rs = %s, min_ri = %s"%(min_rs, min_ri), self.si_to_orientation(min_ri))

            target_ori = self.si_to_orientation(si)

            if min_rs >= self.safe_distance:
                self.goal_orientation = target_ori
                self.state = "go_straight"
            else:
            # elif self.state != "go_along_wall":
                print("The space is not enough!")
                self.real_goal_orientation = target_ori
                self.state = "go_along_wall"
                self.stop()

                self.goal_orientation = (self.si_to_orientation(min_ri) + PI / 2) % self.round_angle

                # left_side_deg = (self.si_to_orientation(min_ri) - PI / 2) % self.round_angle
                # right_side_deg = (self.si_to_orientation(min_ri) + PI / 2) % self.round_angle
                #
                # print(left_side_deg, right_side_deg)
                #
                # if self.get_min_distance(left_side_deg, self.orientation) < PI / 2:
                #     self.goal_orientation = left_side_deg
                #     print("Choose the left side")
                # elif self.get_min_distance(right_side_deg, self.orientation) < PI / 2:
                #     self.goal_orientation = right_side_deg
                #     print("Choose the right side")
                # else:
                #     print(">>>>>>> Error!!")

            print("state = %s, length = %s"%(self.state, self.laser_ranges[self.front_i]))
            self.turn_over_and_go()

        else:
            if self.state != "end":
                self.stop()
                self.state = "end"
            print("state = %s, length = %s"%(self.state, self.laser_ranges[self.front_i]))
            self.turn_direction(self.turn_speed)


    def turn_over_and_go(self):
        print "state = ", self.state
        print "goal_orientation", self.goal_orientation
        print "cur_orientation", self.orientation
        if abs(self.goal_orientation - self.orientation) <= self.turn_angle_bias:
            print("Go straight!")
            self.go_straight(self.straight_speed)
        elif (self.goal_orientation > self.orientation and\
        self.goal_orientation - self.orientation <= math.pi) or\
        (self.goal_orientation < self.orientation and\
        self.orientation - self.goal_orientation > math.pi):
            print("Turn right!")
            self.turn_direction(self.turn_speed)
        else:
            print("Turn left!")
            self.turn_direction(-self.turn_speed)

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

    ###### tool functions
    def moving_average(self, arr, mean_window):
        res = []
        arr_len = len(arr)
        for i in range(arr_len):
            s_sum = 0
            for j in range(mean_window):
                cal_i = (i - int(mean_window / 2) + j) % arr_len
                s_sum += arr[cal_i]
            res.append(s_sum / mean_window)
        return res

    def get_min_distance(self, deg1, deg2):
        dis = (deg1 - deg2) % self.round_angle
        return min(dis, self.round_angle - dis)

    def si_to_orientation(self, si):
        ori_zero = (1. - 2. * si / (self.sample_n - 1)) * PI
        return (self.orientation + ori_zero) % self.round_angle

    def ori_to_si(self, ori):
        ori_zero = (ori - self.orientation) % self.round_angle
        return (1. - ori_zero / PI) * (self.sample_n - 1.) / 2.

    def ori_to_si(self, ori):
        ori_zero = (ori - self.orientation) % self.round_angle
        return int(round((1. - ori_zero / PI) * (self.sample_n - 1.) / 2.) % self.sample_n)

if __name__ == "__main__":
    mc = MovingControl()
    mc.start()
