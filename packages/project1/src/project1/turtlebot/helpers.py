import rospy

__author__ = 'shaun'


def check_bounds(val, lower, upper):
    # check that the val is between lower and upper values
    return lower < val < upper


def get_curr_time():
    return rospy.get_time()


def scan_has_none_check(scan):
    # checks if the scan as a none value in its contents, returns True if so
    scan = [scan.min, scan.max, scan.mean, scan.variance, scan.std_dev, scan.median, scan.std_error]
    for item in scan:
        if item is None:
            return True
    return False
