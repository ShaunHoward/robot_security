import random
import rospy
from turtle_node.turtle_node import TurtleBot

R2_START_STATE = (4, 0, 0)
moves = [0.1, -0.1]


def run_robot(id=2):
    squirtle = TurtleBot(*R2_START_STATE, id=id)
    # invoke sensors once
    rospy.sleep(2)
    while not rospy.is_shutdown():
        squirtle.move_on_x(amount=random.uniform(-1, 1))
        rospy.sleep(1)

if __name__ == '__main__':
    run_robot()
