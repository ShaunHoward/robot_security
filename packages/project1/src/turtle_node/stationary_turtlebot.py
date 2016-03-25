#!/usr/bin/env python

import rospy
from turtlebot import TurtleBot

class StationaryTurtlebot(TurtleBot, object):

    def __init__(self):
        super(StationaryTurtleBot, self).__init__()

    @staticmethod
    def move():
        rospy.logwarn('I am stationary! You can\'t move me!')

    @staticmethod
    def stop():
        rospy.logwarn('I am stationary! You can\'t stop me')

def main():
    robot = StationaryTurtlebot()

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
