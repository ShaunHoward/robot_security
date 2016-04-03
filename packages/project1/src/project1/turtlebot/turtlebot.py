#!/usr/bin/env python

import math
import numpy
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from project1.msg import ScanWithVariance, ScanWithVarianceStamped


class TurtleBot:
    """
    Base class for the Turtle Bot 2 in our experiments
    author: Matt Swartwout and Shaun Howard
    """

    def __init__(self, rate=10):
        rospy.init_node('robot')

        # get the initial robot starting positions from the ros parameter server
        x_pos = rospy.get_param('x_pos')
        y_pos = rospy.get_param('y_pos')
        yaw = rospy.get_param('yaw')

        # create a pose object for this t-bot so we can easily know its initial pose
        self.pose = Pose2D()
        self.pose.x = x_pos
        self.pose.y = y_pos
        self.pose.theta = yaw

        self.initialize_subscribers()
        self.initialize_publishers()

        self.position_publisher.publish(self.pose)

        self.scan_received = False  # We haven't received a valid LaserScan yet
        self.processed_scan = None

        self.rate = rospy.Rate(rate)

        self.namespace = rospy.get_namespace()[1:]  # Get rid of the leading /

    def initialize_subscribers(self):
        self.lidar_subscriber = rospy.Subscriber('scan',
                                                 LaserScan,
                                                 self.scan_callback)

    def initialize_publishers(self):
        self.processed_scan_publisher = rospy.Publisher('processed_scan',
                                                        ScanWithVarianceStamped,
                                                        queue_size=1)

        self.position_publisher = rospy.Publisher('position',
                                                  Pose2D,
                                                  queue_size=1,
                                                  latch=True)

    def initialize_scanner(self, scan_msg):
        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.angle_increment = scan_msg.angle_increment
        self.time_increment = scan_msg.time_increment
        self.scan_time = scan_msg.scan_time
        self.range_min = scan_msg.range_min
        self.range_max = scan_msg.range_max
        self.scan_received = True

    def scan_callback(self, scan_msg):
        # initialize scanner properties for this robot
        if not self.scan_received:
            self.initialize_scanner(scan_msg)

        # First throw out invalid particles
        valid_particles = []
        # ranges = numpy.array(scan_msg.ranges)
        for particle in scan_msg.ranges:
            if self.range_min <= particle <= self.range_max:
                valid_particles.append(particle)

        scan = ScanWithVariance()

        # Find the min, max, mean, median, variance, std dev, and std error if there are particles
        if len(valid_particles) is not 0:
            scan.min = numpy.min(valid_particles)
            scan.max = numpy.max(valid_particles)
            scan.mean = numpy.mean(valid_particles)
            scan.median = numpy.median(valid_particles)
            var = numpy.var(valid_particles)
            std = math.sqrt(var)
            scan.variance = var
            scan.std_dev = std
            scan.std_error = std / math.sqrt(len(valid_particles))
        else:
            # otherwise, set all values to 0
            scan.min = 0
            scan.max = 0
            scan.mean = 0
            scan.variance = 0
            scan.std_dev = 0
            scan.median = 0
            scan.std_error = 0

        self.processed_scan = self.stamp_scan_w_variance(scan)
        self.processed_scan_publisher.publish(self.processed_scan)

    @staticmethod
    def stamp_scan_w_variance(scan_w_variance):
        # Create time-stamped scan message including the scan and variance of points
        stamped_scan_w_variance = ScanWithVarianceStamped()
        stamped_scan_w_variance.scan = scan_w_variance
        stamped_scan_w_variance.header.stamp = rospy.get_rostime()

        return stamped_scan_w_variance
