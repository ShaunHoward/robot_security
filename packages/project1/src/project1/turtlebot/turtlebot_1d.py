#!/usr/bin/env python

import rospy
import math
import helpers
import random
from geometry_msgs.msg import Twist, Pose2D, PoseWithCovarianceStamped
from turtlebot import TurtleBot
from project1.msg import ScanWithVarianceStamped


class TurtleBot1D(TurtleBot, object):

    def __init__(self):
        super(TurtleBot1D, self).__init__()

        rospy.on_shutdown(self.stop)

        self.robot_1_distance = None
        self.robot_3_distance = None
        self.robot_1_position = None
        self.robot_3_position = None
        self.pose11 = PoseWithCovarianceStamped()
        self.pose33 = PoseWithCovarianceStamped()
        self.pose32 = PoseWithCovarianceStamped()
        self.covariance11 = [0] * 36
        self.covariance33 = [0] * 36
        self.covariance32 = [0] * 36
        self.initialize_subscribers()
        self.initialize_publishers()

    def initialize_publishers(self):
        super(TurtleBot1D, self).initialize_publishers()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist,
                                           queue_size=1)

        self.pose_wrt_1_from_1 = rospy.Publisher('pose11',
                                                 PoseWithCovarianceStamped,
                                                 queue_size=1)

        self.pose_wrt_3_from_3 = rospy.Publisher('pose33',
                                                 PoseWithCovarianceStamped,
                                                 queue_size=1)

        self.pose_wrt_3_from_2 = rospy.Publisher('pose32',
                                                 PoseWithCovarianceStamped,
                                                 queue_size=1)

    def initialize_subscribers(self):
        super(TurtleBot1D, self).initialize_subscribers()
        self.robot_1_dist = rospy.Subscriber('/turtlebot1/processed_scan',
                                             ScanWithVarianceStamped,
                                             self.robot_1_dist_cb)

        self.robot_3_dist = rospy.Subscriber('/turtlebot3/processed_scan',
                                             ScanWithVarianceStamped,
                                             self.robot_3_dist_cb)

        self.robot_1_pos = rospy.Subscriber('/turtlebot1/position', Pose2D,
                                            self.robot_1_pos_cb)

        self.robot_3_pos = rospy.Subscriber('/turtlebot3/position', Pose2D,
                                            self.robot_3_pos_cb)

    def scan_callback(self, scan_msg):
        super(TurtleBot1D, self).scan_callback(scan_msg)
        self.update_pose_32()

    def robot_1_dist_cb(self, distance):
        rospy.logdebug("Updated robot 1 distance")
        self.robot_1_distance = distance
        self.update_pose_11()

    def robot_3_dist_cb(self, distance):
        rospy.logdebug("Updated robot 3 distance")
        self.robot_3_distance = distance
        self.update_pose_33()

    def robot_1_pos_cb(self, position):
        self.robot_1_position = position

    def robot_3_pos_cb(self, position):
        self.robot_3_position = position

    def update_pose_11(self):
        if (self.robot_1_distance, self.robot_1_position,
            self.robot_1_distance.scan.std_dev, self.robot_1_distance.scan.median)\
                is not (None, None, None, None):
            median = self.robot_1_distance.scan.median
            std_dev = self.robot_1_distance.scan.std_dev
            mean = self.robot_1_distance.scan.mean
            bias = 0.3556
            self.pose11.pose.pose.position.x = (self.robot_1_position.x + median) - self.pose.x  # - std_dev - bias
            self.covariance11[0] = self.robot_1_distance.scan.variance
            self.pose11.pose.covariance = self.covariance11
            self.pose11.header.stamp = rospy.get_rostime()
            self.pose11.header.frame_id = self.namespace + 'odom'
            self.pose_wrt_1_from_1.publish(self.pose11)
        else:
            if self.robot_1_distance is None:
                rospy.logdebug("Didn't update pose_11 because robot_1_distance = None")
            if self.robot_1_position is None:
                rospy.logdebug("Didn't update pose_11 because robot_1_position = None")

    def update_pose_33(self):
        if (self.robot_3_distance, self.robot_3_position,
            self.robot_3_distance.scan.std_dev, self.robot_3_distance.scan.median)\
                is not (None, None, None, None):
            # width of turtlebot is 14 in == .3556 m, accounting for kinect dists -> .3556/2
            # account for distance from t3 center to t2 center as well as kinect dists
            median = self.robot_3_distance.scan.median
            std_dev = self.robot_3_distance.scan.std_dev
            mean = self.robot_3_distance.scan.mean - (2 * .087)
            bias = 0.3556
            self.pose33.pose.pose.position.x = (self.robot_3_position.x - median) - self.pose.x  # - std_dev - bias
            self.covariance33[0] = self.robot_3_distance.scan.variance
            self.pose33.pose.covariance = self.covariance33
            self.pose33.header.stamp = rospy.get_rostime()
            self.pose33.header.frame_id = self.namespace + 'odom'
            self.pose_wrt_3_from_3.publish(self.pose33)
        else:
            if self.robot_3_distance is None:
                rospy.logdebug("Didn't update pose_33 because robot_3_distance = None")
            if self.robot_3_position is None:
                rospy.logdebug("Didn't update pose_33 because robot_3_position = None")

    def update_pose_32(self):
        # NOTE: this one works well and is on-par with real odom
        if self.processed_scan is not None and self.robot_3_position is not None:
            # account for distance from t2 center to t3 center as well as kinect dists
            mean = self.processed_scan.scan.mean - (2 * .087)
            std_dev = self.processed_scan.scan.std_dev
            median = self.processed_scan.scan.median
            bias = 0.3556
            self.pose32.pose.pose.position.x = (self.robot_3_position.x - median) - self.pose.x  # - std_dev - bias
            self.covariance32[0] = self.processed_scan.scan.variance
            self.pose32.pose.covariance = self.covariance32
            self.pose32.header.stamp = rospy.get_rostime()
            self.pose32.header.frame_id = self.namespace + 'odom'
            self.pose_wrt_3_from_2.publish(self.pose32)

    def move(self, amount, lower_bound=1, upper_bound=3):
        goal_x = self.pose.x + amount

        within_bounds = helpers.check_bounds(goal_x, lower_bound, upper_bound)
        if within_bounds:
            move_cmd = Twist()
            if amount < 0:
                move_cmd.linear.x = -self.speed
                dist_to_goal = -amount
            else:
                move_cmd.linear.x = self.speed
                dist_to_goal = amount
            rospy.logdebug('Robot is heading to x: %s', str(goal_x))

            prev_time = helpers.get_curr_time()
            tot_dist_traveled = 0
            while dist_to_goal > 0.1:
                self.cmd_vel_pub.publish(move_cmd)
                self.rate.sleep()

                curr_time = helpers.get_curr_time()
                diff = curr_time - prev_time
                dist_traveled = self.speed * diff
                tot_dist_traveled += dist_traveled
                dist_to_goal -= dist_traveled
                prev_time = curr_time

            if amount < 0:
                self.pose.x -= tot_dist_traveled
            else:
                self.pose.x += tot_dist_traveled
            rospy.logdebug('Robot reached x: %s', str(goal_x))
        else:
            rospy.logwarn('Goal received out of bounds')

    def stop(self):
        rospy.logdebug('%s has stopped', self.namespace)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


def main():
    robot = TurtleBot1D()

    while not rospy.is_shutdown():
        #robot.move(amount=random.uniform(-1, 1))
        #rospy.sleep(1)
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print "exiting turtle"
        pass
