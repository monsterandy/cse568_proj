#!/usr/bin/env python
import math
import numpy as np
import rospy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

class LocalPlanner():
    def __init__(self, global_path):
        self.global_path = global_path
        # rospy.init_node('local_planner', anonymous=False)
        self.bin_size = 10  # number of laser lines in a bin
        self.histogram = [0] * 36
        self.pose_subscriber = rospy.Subscriber('odom', Odometry, self.update_pose)
        self.scan_subscriber = rospy.Subscriber('scan', LaserScan, self.update_polar_hg)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        rospy.wait_for_message('odom', Odometry)
        self.follow_path()
        # rospy.spin()

    @classmethod
    def from_path(cls, global_path):
        return cls(global_path)

    def follow_path(self):
        for checkpoint in self.global_path:
            self.go_to_checkpoint(Point(x=checkpoint[0], y=checkpoint[1]))
        print("Reach the GOAL!!!")

    def update_polar_hg(self, scan):
        # generate the binary polar histogram based on the scan data
        histogram = []
        for i in range(0, 360, self.bin_size):
            if min(scan.ranges[i:i+9]) < 0.7:
                histogram.append(1)
            else:
                histogram.append(0)
        self.histogram = histogram

    def update_pose(self, data):
        self.pose = data.pose.pose
        self.pose.position.x = round(self.pose.position.x, 4)
        self.pose.position.y = round(self.pose.position.y, 4)
        explicit_quat = [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
        self.orientation = euler_from_quaternion(explicit_quat)

    def vfh_find_openings(self, goal):
        # find the available openings from the polar histogram
        histogram = self.histogram
        opening = []
        i = 0
        while i < len(histogram):
            if histogram[i] == 0:
                count = 0
                start = i
                for j in range(i, len(histogram)):
                    if histogram[j] == 0:
                        count += 1
                    else:
                        break
                end = start + count - 1
                if count >= 5:
                    if count <= 11: # narrow opening
                        select = int(math.floor((start + end) / 2))
                        opening.append(self.vfh_get_opening_odom_angle(select))
                    else: # wide opening
                        right = self.vfh_get_opening_odom_angle(start + 6)
                        opening.append(right)
                        left = self.vfh_get_opening_odom_angle(end - 6)
                        opening.append(left)
                        if self.vfh_is_target_between_two(goal, right, left):
                            opening.append(goal)
                i += count
            else:
                i += 1
        print(opening)
        return opening

    def vfh_is_target_between_two(self, target, a, b):
        range1 = self.vfh_odom_angle_between(a, target)
        range2 = self.vfh_odom_angle_between(b ,target)
        if ((range1 >= 0 and range2 <= 0) or (range1 <= 0 and range2 >= 0)) and ((abs(range1) + abs(range2)) < math.pi):
            return True
        else:
            return False

    def vfh_odom_angle_between(self, start, goal):
        a = goal - start
        return self.convert_bound(a)

    def steering_angle(self, goal):
        return math.atan2(goal.y - self.pose.position.y, goal.x - self.pose.position.x)
        # return -90*2*math.pi/360

    def angular_vel(self, goal, constant=6):
        a = self.steering_angle(goal) - self.orientation[2]
        return self.convert_bound(a) * constant

    def vfh_angular_vel(self, opening_angle, constant=6):
        a = opening_angle - self.orientation[2]
        return self.convert_bound(a) * constant
    
    def euclidean_distance(self, goal):
        return math.sqrt(pow((goal.x - self.pose.position.x), 2) +
                        pow((goal.y - self.pose.position.y), 2))

    def linear_vel(self, goal, constant=2):
        return constant * self.euclidean_distance(goal)

    def convert_bound(self, a):
        if abs(a) > math.pi:
            if a > 0:
                return -(2 * math.pi - abs(a))
            else:
                return 2 * math.pi - abs(a)
        return a

    def go_to_checkpoint(self, checkpoint):
        print("new checkpoint:")
        print(checkpoint)
        # first: turn to the checkpoint
        move_cmd = Twist()
        while round(self.angular_vel(checkpoint), 5) != 0:
            move_cmd.angular.z = self.angular_vel(checkpoint)
            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()

        self.previous_odom_angle = self.orientation[2]

        # second: go to the checkpoint using modified VFH
        while round(self.linear_vel(checkpoint), 3) >= 0.3:
            move_cmd = Twist()            
            opening_angle = self.vfh_lowest_cost_opening(checkpoint)
            while round(self.vfh_angular_vel(opening_angle), 2) != 0:
                move_cmd.angular.z = self.vfh_angular_vel(opening_angle)
                self.cmd_vel.publish(move_cmd)
                self.rate.sleep()

            move_cmd = Twist()            
            move_cmd.linear.x = 2
            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()

    def vfh_chosen_to_target_cost(self, chosen, target, constant=2):
        cost = target - chosen
        return abs(self.convert_bound(cost)) * constant

    def vfh_current_to_chosen_cost(self, chosen, constant=0.5):
        cost = chosen - self.orientation[2]
        return abs(self.convert_bound(cost)) * constant

    def vfh_previous_to_chosen_cost(self, chosen, constant=0.5):
        cost = chosen - self.previous_odom_angle
        return abs(self.convert_bound(cost)) * constant

    def vfh_get_opening_odom_angle(self, chosen):
        previous_angle = (chosen * 5 + 2.5) * math.pi / 180
        previous_odom_angle = self.orientation[2] - (math.pi / 2) + previous_angle
        previous_odom_angle = self.convert_bound(previous_odom_angle)
        return previous_odom_angle

    def vfh_lowest_cost_opening(self, target_point):
        # calculate the cost of each openings and return the lowest cost angle
        goal = self.steering_angle(target_point)
        opening = self.vfh_find_openings(goal)
        g_cost = [0] * len(opening)
        for i in range(0, len(opening)):
            g_cost[i] += self.vfh_chosen_to_target_cost(opening[i], goal) \
                          + self.vfh_current_to_chosen_cost(opening[i]) \
                          + self.vfh_previous_to_chosen_cost(opening[i])
        print(g_cost)
        lowest_opening_index = np.argmin(g_cost)
        lowest_opening = opening[lowest_opening_index]
        self.previous_odom_angle = lowest_opening
        return lowest_opening



