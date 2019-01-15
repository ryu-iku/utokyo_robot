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
        self.sample_n = 51
        self.max_range = 20
        self.min_range = .1
        self.shortest_road = 3 #2
        self.safe_distance = .7 #1
        self.round_angle = 2 * PI
        self.straight_speed = .5 #3 #To be tuned!
        self.turn_speed = .5 #To be tuned!
        self.turn_angle_bias = .1 #.05
        self.cleared_area = {} #Store visited areas
        self.area_unit = 2.

        self.quarter_lsi = int((self.sample_n - 1) / 4)
        self.back_i = 0
        self.right_i = self.quarter_lsi
        self.front_i = self.quarter_lsi * 2
        self.left_i = self.quarter_lsi * 3
        self.right_back_i = int((self.sample_n - 1) * 1 / 8)
        self.right_front_i = int((self.sample_n - 1) * 3 / 8)
        self.left_front_i = int((self.sample_n - 1) * 5 / 8)
        self.left_back_i = int((self.sample_n - 1) * 7 / 8)

        # self.mean_window = int(self.quarter_lsi / 2)
        self.mean_window = 1

        # self.state_set = {"start", "straight_road", "corner", "end", "turn_around_at_corner", "turn_around_at_end", "turning_over", "goal"}
        self.state = "start"

        self.laser_ranges = None
        self.orientation = None
        self.goal_orientation = None
        self.turning = False

    def start(self):
        rospy.init_node('MovingControl')
        self.rate = rospy.Rate(10)
        self.cmd_vel_publisher = rospy.Publisher('/armed3w/diff_drive_controller/cmd_vel',
                        Twist, queue_size=1)
        while not rospy.is_shutdown():
            rospy.Subscriber("/armed3w/diff_drive_controller/odom", Odometry, self.update_odo_data, queue_size=1)
            rospy.Subscriber("/scan", LaserScan, self.update_scan_data, queue_size=1)
            if self.laser_ranges != None and self.orientation != None:
                self.update_sensor_data_analysis()
                self.move_on()
            self.rate.sleep()

    def update_odo_data(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        _, _, self.orientation = euler_from_quaternion([orientation_q.x,
        orientation_q.y, orientation_q.z, orientation_q.w])
        self.orientation = self.orientation % self.round_angle

    def update_scan_data(self, msg):
        laser_ranges = []

        for i in range(self.sample_n):
            if msg.ranges[i] == float("inf"):
                laser_ranges.append(self.max_range)
            else:
                laser_ranges.append(msg.ranges[i])

        self.laser_ranges = laser_ranges

    def update_sensor_data_analysis(self):
        lr_moving_average = self.moving_average(self.laser_ranges, self.mean_window)

        max_sensor_arr = []
        msa_front = []

        for i in range(self.sample_n):
            curr_s = lr_moving_average[i]
            prev_s = lr_moving_average[i - 1]
            next_s = lr_moving_average[(i + 1) % self.sample_n]

            if curr_s > prev_s and curr_s >= next_s and curr_s >= self.shortest_road :
                max_sensor_arr.append(i)

                # Check the front side to get available directions
                alpha = 0
                lsi_a = int(self.quarter_lsi * alpha)
                if i >= self.right_i + lsi_a and i <= self.left_i - lsi_a:
                # if i >= self.front_i - 6 and i <= self.left_front_i + 6:
                # if i >= self.front_i - 6 and i <= self.left_front_i + 6:
                    msa_front.append(i)

        self.max_sensor_arr = max_sensor_arr
        self.msa_front = msa_front

        # print "Max sensor data numbers", self.max_sensor_arr
        # print "Available direction in front:", self.msa_front
        # print "Available orientation in front:", map(lambda i:self.si_to_orientation(i), self.msa_front)


    # Go along one side wall
    def move_on(self):

        if self.state == "go_along_wall":
            si = self.ori_to_si(self.real_goal_orientation)
            min_rs, min_ri = self.bump_range(si)
            print("min_rs = %s, min_ri = %s"%(min_rs, min_ri), self.si_to_orientation(min_ri))
            print self.laser_ranges

            if min_rs >= self.safe_distance:
                self.goal_orientation = self.real_goal_orientation
                print(">> goal orientation changed to real_goal_orientation")
                if abs(self.real_goal_orientation - self.orientation) <= self.turn_angle_bias:
                    self.state = "go_straight"
                    self.real_goal_orientation = None
            else:
                self.stop()
                # If goal orientatin is not stored, change the state to set the value
                if self.real_goal_orientation == None:
                    self.state = "go_straight"

            print("state = %s, length = %s"%(self.state, self.laser_ranges[self.front_i]))
            self.turn_over_and_go()

        elif len(self.msa_front) >= 1:
            # Select the most left available direction
            # si = self.msa_front[-1]

            # Choose the direction not in the visited area storage.
            si = None
            for tmp_si in reversed(self.msa_front):
                si_area = self.si_to_area(tmp_si, self.laser_ranges[tmp_si])
                print "The area of si number is:", si_area
                if self.cleared_area.has_key(si_area):
                    print "The area was visited!"
                    continue
                else:
                    print "The area was not visited. Choose it!"
                    si = tmp_si
                    break
            if si == None:
                print "No unvisited area, choose the most left direction"
                si = self.msa_front[-1]
            print "Available direction in front: %s, the direction chosen: %s"%(self.msa_front, si)

            # Check bump range
            min_rs, min_ri = self.bump_range(si)
            print("min_rs = %s, min_ri = %s"%(min_rs, min_ri), self.si_to_orientation(min_ri))
            print self.laser_ranges

            target_ori = self.si_to_orientation(si)

            if min_rs >= self.safe_distance:
                self.goal_orientation = target_ori
                print(">> goal orientation changed to target_ori")
                self.state = "go_straight"
            else:
                print("The space is not enough!")
                self.real_goal_orientation = target_ori
                print(">> goal orientation changed to target_ori")
                self.state = "go_along_wall"
                # self.stop()

                # Turn around if there is a risk to bump the wall
                alpha = .3
                angle_go_right = (self.si_to_orientation(min_ri) + PI * alpha) % self.round_angle
                angle_go_left = (self.si_to_orientation(min_ri) - PI * alpha) % self.round_angle

                if self.min_angle_diff(angle_go_left, target_ori) <=\
                self.min_angle_diff(angle_go_right, target_ori):
                    self.goal_orientation = angle_go_left
                    print(">> goal orientation changed to angle_go_left")
                else:
                    self.goal_orientation = angle_go_right
                    print(">> goal orientation changed to angle_go_right")

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
            self.store_visited_area()
        elif (self.goal_orientation > self.orientation and\
        self.goal_orientation - self.orientation <= math.pi) or\
        (self.goal_orientation < self.orientation and\
        self.orientation - self.goal_orientation > math.pi):
            print("Turn right!")
            self.turn_direction(self.turn_speed)
        else:
            print("Turn left!")
            self.turn_direction(-self.turn_speed)

    ###### Action function
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
        # vel.linear.x = .02
        vel.angular.z = speed
        self.cmd_vel_publisher.publish(vel)

    def twist_control(self, linear_x, angular_z):
        vel = Twist()
        vel.linear.x = linear_x
        vel.angular.z = angular_z
        self.cmd_vel_publisher.publish(vel)

    ###### Tool functions
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

    def min_angle_diff(self, deg1, deg2):
        dis = (deg1 - deg2) % self.round_angle
        return min(dis, self.round_angle - dis)

    def bump_range(self, si):
        alpha = .4
        r_start = min(si, self.front_i, (si - int(self.quarter_lsi * alpha)))
        r_end = max(si, self.front_i, (si + int(self.quarter_lsi * alpha) + 1))

        front_range = self.laser_ranges[r_start:(r_end + 1)]

        min_rs, itr = min(list(zip(front_range, range(len(front_range)))))
        min_ri = itr + r_start

        return min_rs, min_ri


    def si_to_orientation(self, si):
        ori_zero = (1. - 2. * si / (self.sample_n - 1)) * PI
        return (self.orientation + ori_zero) % self.round_angle

    def ori_to_si(self, ori):
        ori_zero = (ori - self.orientation) % self.round_angle
        return (1. - ori_zero / PI) * (self.sample_n - 1.) / 2.

    def ori_to_si(self, ori):
        ori_zero = (ori - self.orientation) % self.round_angle
        return int(round((1. - ori_zero / PI) * (self.sample_n - 1.) / 2.) % self.sample_n)

    def coor_to_area(self, x, y):
        area_x_start = math.floor(x / self.area_unit) * self.area_unit
        area_x_end = math.ceil(x / self.area_unit) * self.area_unit

        area_y_start = math.floor(y / self.area_unit) * self.area_unit
        area_y_end = math.ceil(y / self.area_unit) * self.area_unit

        key = "x%dx%dy%dy%d"%(area_x_start, area_x_end, area_y_start, area_y_end)

        return key

    def store_visited_area(self):
        key = self.coor_to_area(self.pos_x, self.pos_y)
        self.cleared_area[key] = True
        print "Visited areas: ", self.cleared_area

    def si_to_area(self, si, laser_r):
        ori = self.si_to_orientation(si)
        x = math.cos(ori) + self.pos_x
        y = math.sin(ori) + self.pos_y

        return self.coor_to_area(x, y)


if __name__ == "__main__":
    mc = MovingControl()
    mc.start()
