#!/usr/bin/env python

import rospy
import helpers
from geometry_msgs.msg import Twist
from turtlebot import TurtleBot


class TurtleBot1D(TurtleBot, object):

    def __init__(self):
        super(TurtleBot1D, self).__init__()

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist,
                                           queue_size=1)
        rospy.on_shutdown(self.stop)

    def move_on_x(self, amount, lower_bound=1, upper_bound=5):
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
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
