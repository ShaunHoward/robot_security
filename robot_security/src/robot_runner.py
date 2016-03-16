__author__ = 'shaun'
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


# runs the robot(s) around the map on random grids

# runs the robot with the given id string
def run(robot_id):
    pass


# license removed for brevity

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('robot_runner')
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass