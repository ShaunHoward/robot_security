#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D


class TurtleBot:

    def __init__(self, speed=0.2, rate=10):
        x_pos = rospy.get_param('x_pos')
        y_pos = rospy.get_param('y_pos')
        yaw = rospy.get_param('yaw')

        self.pose = Pose2D()
        self.pose.x = x_pos
        self.pose.y = y_pos
        self.pose.theta = yaw

        self.initialize_subscribers()
        self.initialize_publishers()

        self.position_publisher.publish(self.pose)

        self.scan_received = False  # We haven't received a valid LaserScan yet

        self.speed = speed
        self.rate = rospy.Rate(rate)

        self.namespace = rospy.get_namespace()

        self.start()

    def start(self):
        rospy.init_node('robot')

    def initialize_subscribers(self):
        self.lidar_subscriber = rospy.Subscriber('scan',
                                                 LaserScan,
                                                 self.scan_callback)

    def initialize_publishers(self):
        self.distance_publisher = rospy.Publisher('scan_average',
                                                  Float32,
                                                  queue_size=1)

        self.position_publisher = rospy.Publisher('position',
                                                  Pose2D,
                                                  queue_size=1,
                                                  latch=True)

    def scan_callback(self, scan_msg):
        if not self.scan_received:
            self.angle_min = scan_msg.angle_min
            self.angle_max = scan_msg.angle_max
            self.angle_increment = scan_msg.angle_increment
            self.time_increment = scan_msg.time_increment
            self.scan_time = scan_msg.scan_time
            self.range_min = scan_msg.range_min
            self.range_max = scan_msg.range_max

        particle_number = 0  # how many valid particles have we received?
        particle_sum = 0   # going to calculate the average distance
        for particle in scan_msg.ranges:
            if self.range_min < particle < self.range_max:
                particle_sum += particle
                particle_number += 1

        if particle_number is not 0:
            average_distance = particle_sum / particle_number
        else:
            average_distance = self.range_max

        self.distance_publisher.publish(average_distance)


def main():
    rospy.init_node('robot')

    robot = TurtleBot()

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
