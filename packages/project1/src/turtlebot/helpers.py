import rospy

__author__ = 'shaun'


def check_bounds(val, lower, upper):
    return lower < val < upper


def get_curr_time():
    return rospy.get_time()
