__author__ = 'shaun'

import rospy
import random
from turtle_node.turtle_node import TurtleBot, GridMap
from particle_filter import particle_filter_node as pf
from romdp.utils import distance
PARTICLE_COUNT = 2000
START_STATE = (1, 0)
GOAL_STATES = [(3, 3)]


def run_robots(n=3, goal_states=GOAL_STATES):
    # TODO: fork n processes in future
    squirtle = TurtleBot(*START_STATE)
    # invoke sensors once
    rospy.sleep(2)

    # initial distribution assigns each particle an equal probability
    particles = pf.Particle.create_random(PARTICLE_COUNT, squirtle.grid_map)
    while not rospy.is_shutdown():
        # only do robot if robot has goal
        if squirtle.goal is not None:
            # read squirtle's sensor
            (odom_x, odom_y) = squirtle.my_odom_pos

            # read other robot's sensors in reference to squirtle and add good reading in
            perturbed_xy = [pf.add_some_noise(odom_x, odom_y) for i in range(n)] + [(odom_x, odom_y)]

            # Update particle weight according to how good every particle matches
            # squirtle's sensor reading
            for p in particles:
                if squirtle.grid_map.is_free(p.x, p.y):
                    p_d = squirtle.grid_map.dist_to_goal(p.x, p.y)
                    mean_x = sum(perturbed_xy[:][0]) / len(perturbed_xy)
                    mean_y = sum(perturbed_xy[:][1]) / len(perturbed_xy)
                    dist_to_goal = squirtle.grid_map.dist_to_goal(mean_x, mean_y)
                    p.w = pf.gauss_kernel(dist_to_goal, p_d)
                else:
                    p.w = 0

            # ---------- Try to find current best estimate for display ----------
            m_x, m_y, m_confident = pf.compute_mean_point(particles)
            rospy.loginfo("actual x, y: %f %f" % (squirtle.x, squirtle.y))
            rospy.loginfo("mx, my, mconfidence: %f %f %f" % (m_x, m_y, m_confident))

            # ---------- Shuffle particles ----------
            new_particles = []

            # Normalise weights
            nu = sum(p.w for p in particles)
            if nu:
                for p in particles:
                    p.w = p.w / nu

            # create a weighted distribution, for fast picking
            dist = pf.WeightedDistribution(particles)

            for _ in particles:
                p = dist.pick()
                if p is None:  # No pick b/c all totally improbable
                    new_particle = pf.Particle.create_random(1, squirtle.grid_map)[0]
                else:
                    h = squirtle.h
                    new_particle = pf.Particle(p.x, p.y, heading=squirtle.h + (squirtle.h * random.random()))
                new_particles.append(new_particle)

            particles = new_particles

        else:
            # stop robot movement
            squirtle.stop()
            #
            # # ---------- Move things ----------
            # old_heading = squirtle.h
            # squirtle.move(0.2)
            # d_h = squirtle.h - old_heading
            #
            # # Move particles according to my belief of movement (this may
            # # be different than the real movement, but it's all I got)
            # for p in particles:
            #     p.h += d_h  # in case robot changed heading, swirl particle heading too
            #     p.advance_by(squirtle.speed)

if __name__ == '__main__':
    run_robots()