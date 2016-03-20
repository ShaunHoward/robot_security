import random
import math
import bisect

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from draw import Maze

#  Credit to Martin J. Laubach for underlying idea of graphical turtlebot maze
#  system using particle filter.

# 0 - empty square
# 1 - occupied square
# 2 - occupied square with a beacon at each corner, detectable by the robot
# Smaller maze
maze_data = ( ( 2, 0, 1, 0, 0 ),
              ( 0, 0, 0, 0, 1 ),
              ( 1, 1, 1, 0, 0 ),
              ( 1, 0, 0, 0, 0 ),
              ( 0, 0, 2, 0, 1 ))

"""
maze_data = ( ( 1, 1, 0, 0, 2, 0, 0, 0, 0, 1 ),
              ( 1, 2, 0, 0, 1, 1, 0, 0, 0, 0 ),
              ( 0, 1, 1, 0, 0, 0, 0, 1, 0, 1 ),
              ( 0, 0, 0, 0, 1, 0, 0, 1, 1, 2 ),
              ( 1, 1, 0, 1, 1, 2, 0, 0, 1, 0 ),
              ( 1, 1, 1, 0, 1, 1, 1, 0, 2, 0 ),
              ( 2, 0, 0, 0, 0, 0, 0, 0, 0, 0 ),
              ( 1, 2, 0, 1, 1, 1, 1, 0, 0, 0 ),
              ( 0, 0, 0, 0, 1, 0, 0, 0, 1, 0 ),
              ( 0, 0, 1, 0, 0, 2, 1, 1, 1, 0 ))
"""
PARTICLE_COUNT = 2000    # Total number of particles

ROBOT_HAS_COMPASS = True # Does the robot know where north is? If so, it
# makes orientation a lot easier since it knows which direction it is facing.
# If not -- and that is really fascinating -- the particle filter can work
# out its heading too, it just takes more particles and more time. Try this
# with 3000+ particles, it obviously needs lots more hypotheses as a particle
# now has to correctly match not only the position but also the heading.

# ------------------------------------------------------------------------
# Some utility functions


def add_noise(level, *coords):
    return [x + random.uniform(-level, level) for x in coords]


def add_little_noise(*coords):
    return add_noise(0.02, *coords)


def add_some_noise(*coords):
    return add_noise(0.1, *coords)

# This is just a gaussian kernel
sigma2 = 0.9 ** 2
def w_gauss(a, b):
    error = a - b
    g = math.e ** -(error ** 2 / (2 * sigma2))
    return g


def compute_mean_point(particles):
    """
    Compute the mean for all particles that have a reasonably good weight.
    This is not part of the particle filter algorithm but rather an
    addition to show the "best belief" for current position.
    """

    m_x, m_y, m_count = 0, 0, 0
    for p in particles:
        m_count += p.w
        m_x += p.x * p.w
        m_y += p.y * p.w

    if m_count == 0:
        return -1, -1, False

    m_x /= m_count
    m_y /= m_count

    # Now compute how good that mean is -- check how many particles
    # actually are in the immediate vicinity
    m_count = 0
    for p in particles:
        if maze.distance(p.x, p.y, m_x, m_y) < 1:
            m_count += 1

    return m_x, m_y, m_count > PARTICLE_COUNT * 0.95


class WeightedDistribution(object):
    def __init__(self, state):
        cum_sum = 0.0
        self.state = [p for p in state if p.w > 0]
        self.distribution = []
        for x in self.state:
            cum_sum += x.w
            self.distribution.append(cum_sum)

    def pick(self):
        try:
            return self.state[bisect.bisect_left(self.distribution, random.uniform(0, 1))]
        except IndexError:
            # Happens when all particles are improbable w=0
            return None


class Particle(object):
    def __init__(self, x, y, heading=None, w=1, noisy=False):
        if heading is None:
            heading = random.uniform(0, 360)
        if noisy:
            x, y, heading = add_some_noise(x, y, heading)

        self.x = x
        self.y = y
        self.h = heading
        self.w = w

    def __repr__(self):
        return "(%f, %f, w=%f)" % (self.x, self.y, self.w)

    @property
    def xy(self):
        return self.x, self.y

    @property
    def xyh(self):
        return self.x, self.y, self.h

    @classmethod
    def create_random(cls, count, maze):
        return [cls(*maze.random_free_place()) for _ in range(0, count)]

    def read_sensor(self, maze):
        """
        Find distance to nearest beacon.
        """
        return maze.distance_to_nearest_beacon(*self.xy)

    def advance_by(self, speed, occupancy_check=None, noisy=False):
        h = self.h
        if noisy:
            speed, h = add_little_noise(speed, h)
            h += random.uniform(-3, 3)  # needs more noise to disperse better
        r = math.radians(h)
        dx = math.sin(r) * speed
        dy = math.cos(r) * speed
        if occupancy_check is None or occupancy_check(self, dx, dy):
            self.move_by(dx, dy)
            return True
        return False

    def move_by(self, x, y):
        self.x += x
        self.y += y


# new version using ros turtle bot
class TurtleBot(Particle):
    my_odom = None
    cmd_vel_pub = None
    pursuing_goal = False
    my_odom_pos = tuple

    def __init__(self, xi=0, yi=0, hi=chose_random_direction(), maze=None, rate=10):
        super(TurtleBot, self).__init__(*maze.random_free_place(), heading=90)
        self.xi = xi
        self.yi = yi
        self.hi = hi
        self.maze = maze
        self.speed = 0.2
        self.steps = 0
        self.ros_rate = rospy.Rate(rate)
        self.start()

    # the distance to the goal of the maze
    def distance_to_goal(self, maze):
        # noise added by default
        return add_little_noise(super(TurtleBot, self).read_sensor(maze))[0]

    def start(self):
        # initialize
        rospy.init_node('TurtleBot', anonymous=False)
        rospy.loginfo('Use ctrl-c to stop TurtleBot')
        rospy.on_shutdown(self.stop)

        # subscribe to robot odom
        self.my_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # publisher to command movement from TurtleBot
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

    # called by ros on shutdown
    def stop(self):
        rospy.loginfo("TurtleBot has stopped")
        self.cmd_vel_pub.publish(Twist())
        # make sure robot gets the message
        rospy.sleep(1)

    def odom_callback(self, msg):
        print msg.pose.pose
        x = msg.pose.pose.Point.x
        y = msg.pose.pose.Point.y
        z = msg.pose.pose.Point.z
        self.my_odom_pos = (x, y, z)

    def move(self, maze):
        """Move turtle bot stochastically"""
        while not self.advance_by(self.speed, noisy=True,
                                  occupancy_check=lambda r, dx, dy: maze.is_free(r.x+dx, r.y+dy)):
            # hit wall, re-orient on new path
            self.h = chose_random_direction()
            self.steps += 1

    def advance_by(self, speed, occupancy_check=None, noisy=False):
        h = self.h
        if noisy:
            speed, h = add_little_noise(speed, h)
            h += random.uniform(-3, 3)  # needs more noise to disperse better
        r = math.radians(h)
        dx = math.sin(r) * speed
        dy = math.cos(r) * speed
        if occupancy_check is None or occupancy_check(self, dx, dy):
            self.move_by(dx, dy)
            return True
        return False

    def move_by(self, x, y):
        move_cmd = Twist()
        move_cmd.linear.x = self.speed
        move_cmd.angular.z = 0

        goal_x = self.x + x
        goal_y = self.y + y

        # my_odom_pos updated by odom callback
        curr_diff_x = abs(goal_x - self.my_odom_pos[0])
        curr_diff_y = abs(goal_y - self.my_odom_pos[1])
        # move until the closeness has met a certain threshold
        while curr_diff_x <= x and curr_diff_y <= y:
            self.cmd_vel_pub.publish(move_cmd)
            # sleep to let robot process msg
            self.ros_rate.sleep()
            curr_diff_x = abs(goal_x - self.my_odom_pos[0])
            curr_diff_y = abs(goal_y - self.my_odom_pos[1])

        # stop robot
        self.cmd_vel_pub.publish(Twist())

        # update coordinates in grid
        self.x += x
        self.y += y


def chose_random_direction():
    heading = random.uniform(0, 360)
    return heading


# old version using turtle graphics
class TRobot(Particle):
    speed = 0.2

    def __init__(self, maze):
        super(TRobot, self).__init__(*maze.random_free_place(), heading=90)
        self.h = chose_random_direction()
        self.step_count = 0

    def read_sensor(self, maze):
        """
        Poor robot, it's sensors are noisy and pretty strange,
        it only can measure the distance to the nearest beacon(!)
        and is not very accurate at that too!
        """
        return add_little_noise(super(TRobot, self).read_sensor(maze))[0]

    def move(self, maze):
        """
        Move the robot. Note that the movement is stochastic too.
        """
        while True:
            self.step_count += 1
            if self.advance_by(self.speed, noisy=True,
                               occupancy_check=lambda r, dx, dy: maze.is_free(r.x+dx, r.y+dy)):
                break
            # Bumped into something or too long in same direction,
            # chose random new direction
            self.h = chose_random_direction()

# ------------------------------------------------------------------------
# main program start

using_ros = True

maze = Maze(maze_data)
maze.draw()

# initial distribution assigns each particle an equal probability
particles = Particle.create_random(PARTICLE_COUNT, maze)
squirtle = TurtleBot(maze=maze)

while True and not (using_ros and rospy.is_shutdown()):
    # Read squirtle's sensor
    r_d = squirtle.read_sensor(maze)

    # Update particle weight according to how good every particle matches
    # squirtle's sensor reading
    for p in particles:
        if maze.is_free(*p.xy):
            p_d = p.read_sensor(maze)
            p.w = w_gauss(r_d, p_d)
        else:
            p.w = 0

    # ---------- Try to find current best estimate for display ----------
    m_x, m_y, m_confident = compute_mean_point(particles)

    # ---------- Show current state ----------
    maze.show_particles(particles)
    maze.show_mean(m_x, m_y, m_confident)
    maze.show_robot(squirtle)

    # ---------- Shuffle particles ----------
    new_particles = []

    # Normalise weights
    nu = sum(p.w for p in particles)
    if nu:
        for p in particles:
            p.w = p.w / nu

    # create a weighted distribution, for fast picking
    dist = WeightedDistribution(particles)

    for _ in particles:
        p = dist.pick()
        if p is None:  # No pick b/c all totally improbable
            new_particle = Particle.create_random(1, maze)[0]
        else:
            new_particle = Particle(p.x, p.y, heading=squirtle.h if ROBOT_HAS_COMPASS else p.h,
                                    noisy=True)
        new_particles.append(new_particle)

    particles = new_particles

    # ---------- Move things ----------
    old_heading = squirtle.h
    squirtle.move(maze)
    d_h = squirtle.h - old_heading

    # Move particles according to my belief of movement (this may
    # be different than the real movement, but it's all I got)
    for p in particles:
        p.h += d_h # in case robot changed heading, swirl particle heading too
        p.advance_by(squirtle.speed)
