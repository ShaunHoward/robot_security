#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

gazebo_x = -10
self_x = -10
dist_x = -10


def gazebo_cb(odom_msg):
    global gazebo_x
    gazebo_x = odom_msg.pose.pose.position.x


def self_cb(odom_msg):
    global self_x
    self_x = odom_msg.pose.pose.position.x


def dist_cb(odom_msg):
    global dist_x
    dist_x = odom_msg.pose.pose.positon.x


def main():
    gazebo_subscriber = rospy.Subscriber('turtlebot2/odom', Odometry, gazebo_cb)
    self_subscriber = rospy.Subscriber('turtlebot2/odometry/filtered_self', Odometry, self_cb)
    dist_subscriber = rospy.Subscriber('turtlebot2/odometry/filtered_distributed', Odometry, dist_cb)

    self_publisher = rospy.Publisher('self_err', Float32, queue_size=1)
    dist_publisher = rospy.Publisher('dist_err', Float32, queue_size=1)

    global gazebo_x
    global self_x
    global dist_x

    self = Float32()
    dist = Float32()

    while not rospy.is_shutdown():
        self_err = gazebo_x - self_x
        dist_err = gazebo_x - dist_x

        self.data = self_err
        dist.data = dist_err

        self_publisher.publish(self)
        dist_publisher.publisher(dist)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass