#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan


class StationaryTurtlebot:

    def __init__(self, x_pos):
        self.x = Float32()
        self.x.data = x_pos

        self.distance_publisher = rospy.Publisher('distance_to_mobile_bot',
                                                  Float32,
                                                  queue_size=1)

        self.position_publisher = rospy.Publisher('position',
                                                  Float32,
                                                  queue_size=1,
                                                  latch=True)
        self.position_publisher.publish(self.x)

        self.lidar_subscriber = rospy.Subscriber('scan',
                                                 LaserScan,
                                                 self.scan_callback)

        self.scan_received = False

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
    x_pos = rospy.get_param('x_pos')
    robot = StationaryTurtlebot(x_pos)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass