#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

# publisher for imu, odom
imu_publisher = rospy.Publisher('imu_data_remapped', Imu, queue_size=1)
odom_publisher = rospy.Publisher('odom_remapped', Odometry, queue_size=1)

# get the current namespace
namespace = rospy.get_namespace()[1:]


def imu_remap(imu_msg):
    # remap imu message to base_link_self and publish the new message
    remapped_msg = imu_msg
    remapped_msg.header.frame_id = namespace + 'base_link_self'
    try:
        imu_publisher.publish(remapped_msg)
    except rospy.ROSException as e:
        print("%s", e.message)


def odom_remap(odom_msg):
    # remap odom message to base_footprint_self and publish the new message
    remapped_msg = odom_msg
    remapped_msg.child_frame_id = namespace + 'base_footprint_self'
    odom_publisher.publish(remapped_msg)


def main():
    # initialize ros node for the imu remap process
    rospy.init_node('imu_remap')
    # subscribe to imu and odom from the robot
    imu_subscriber = rospy.Subscriber('mobile_base/sensors/imu_data', Imu, imu_remap)
    odom_subscriber = rospy.Subscriber('odom', Odometry, odom_remap)

    # just run this to pub/sub to the robot
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass