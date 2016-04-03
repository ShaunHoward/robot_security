#!/usr/bin/env python

import rospy

from project1.msg import ScanWithVarianceStamped, DebugInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D

# initialize all values to values we know are incorrect if any errors arise
gazebo_x = -10
self_x = -10
dist_x = -10
pose11 = -10
pose33 = -10
pose32 = -10
distance_11_mean = -10
distance_33_mean = -10
distance_32_mean = -10
distance_11_median = -10
distance_33_median = -10
distance_32_median = -10
pos1 = -10
pos2 = -10
pos3 = -10


def gazebo_cb(odom_msg):
    global gazebo_x
    gazebo_x = odom_msg.pose.pose.position.x


def self_cb(odom_msg):
    global self_x
    self_x = odom_msg.pose.pose.position.x


def dist_cb(odom_msg):
    global dist_x
    dist_x = odom_msg.pose.pose.position.x


def pose11_cb(pose):
    global pose11
    pose11 = pose.pose.pose.position.x


def pose33_cb(pose):
    global pose33
    pose33 = pose.pose.pose.position.x


def pose32_cb(pose):
    global pose32
    pose32 = pose.pose.pose.position.x


def distance11_cb(scan):
    global distance_11_mean
    global distance_11_median
    distance_11_mean = scan.scan.mean
    distance_11_median = scan.scan.median


def distance33_cb(scan):
    global distance_33_mean
    global distance_33_median
    distance_33_mean = scan.scan.mean
    distance_33_median = scan.scan.median


def distance32_cb(scan):
    global distance_32_mean
    global distance_32_median
    distance_32_mean = scan.scan.mean
    distance_32_median = scan.scan.median


def pos1_cb(pose):
    global pos1
    pos1 = pose.x


def pos2_cb(pose):
    global pos2
    pos2 = pose.x


def pos3_cb(pose):
    global pos3
    pos3 = pose.x


def main():
    rospy.init_node('error_publisher')

    # subscribe to all the relevant topics published by the robot sensor network
    gazebo_subscriber = rospy.Subscriber('turtlebot2/odom', Odometry, gazebo_cb)
    self_subscriber = rospy.Subscriber('turtlebot2/odometry/filtered_self', Odometry, self_cb)
    dist_subscriber = rospy.Subscriber('turtlebot2/odometry/filtered_distributed', Odometry, dist_cb)
    pose11_subscriber = rospy.Subscriber('turtlebot2/pose11', PoseWithCovarianceStamped, pose11_cb)
    pose33_subscriber = rospy.Subscriber('turtlebot2/pose33', PoseWithCovarianceStamped, pose33_cb)
    pose32_subscriber = rospy.Subscriber('turtlebot2/pose32', PoseWithCovarianceStamped, pose32_cb)
    distance11_subscriber = rospy.Subscriber('turtlebot1/processed_scan', ScanWithVarianceStamped, distance11_cb)
    distance33_subscriber = rospy.Subscriber('turtlebot3/processed_scan', ScanWithVarianceStamped, distance33_cb)
    distance32_subscriber = rospy.Subscriber('turtlebot2/processed_scan', ScanWithVarianceStamped, distance32_cb)
    position1_subscriber = rospy.Subscriber('turtlebot1/position', Pose2D, pos1_cb)
    position2_subscriber = rospy.Subscriber('turtlebot2/position', Pose2D, pos2_cb)
    position3_subscriber = rospy.Subscriber('turtlebot3/position', Pose2D, pos3_cb)
    self_publisher = rospy.Publisher('self_err', Float32, queue_size=1)
    dist_publisher = rospy.Publisher('dist_err', Float32, queue_size=1)
    debug_publisher = rospy.Publisher('debug_info', DebugInfo, queue_size=1)

    # make a bunch of global variables to store the values
    global gazebo_x
    global self_x
    global dist_x
    global pose11
    global pose33
    global pose32
    global distance_11_mean
    global distance_33_mean
    global distance_32_mean
    global distance_11_median
    global distance_33_median
    global distance_32_median
    global pos1
    global pos2
    global pos3

    self = Float32()
    dist = Float32()
    debug = DebugInfo()

    # update the values and add to a debug message
    # publish that message repeatedly in this loop
    while not rospy.is_shutdown():
        self_err = gazebo_x - self_x
        dist_err = gazebo_x - dist_x

        self.data = self_err
        dist.data = dist_err

        self_publisher.publish(self)
        dist_publisher.publish(dist)

        debug.gazebo_odom = gazebo_x
        debug.self_odom = self_x
        debug.dist_odom = dist_x
        debug.pose11 = pose11
        debug.pose33 = pose33
        debug.pose32 = pose32
        debug.self_err = self_err
        debug.dist_err = dist_err
        debug.distance_11_mean = distance_11_mean
        debug.distance_33_mean = distance_33_mean
        debug.distance_32_mean = distance_32_mean
        debug.distance_11_median = distance_11_median
        debug.distance_33_median = distance_33_median
        debug.distance_32_median = distance_32_median
        debug.pos1 = pos1
        debug.pos2 = pos2
        debug.pos3 = pos3
        debug_publisher.publish(debug)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass