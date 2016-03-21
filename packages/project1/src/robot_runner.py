__author__ = 'shaun'

import rospy

from turtle_node.turtle_node import TurtleBot, GridMap
from particle_filter import particle_filter as pf
from romdp.romdp import ROMDP


PARTICLE_COUNT = 2000
START_STATE = (1, 0)
GOAL_STATES = [(3, 3)]
GRID_MAP = [[-1, 0, 0],
            [-1, 0, 0],
            [-1, 0, 1]]


def run_robots(grid_map):
    particles = pf.Particle.create_random(PARTICLE_COUNT, GRID_MAP)
    romdp = ROMDP(GRID_MAP, GOAL_STATES)
    # initial distribution assigns each particle an equal probability
    squirtle = TurtleBot(*START_STATE, grid_map=grid_map)

    while ~rospy.is_shutdown():
        # read squirtle's sensor
        odom = squirtle.my_odom_pos

        # read other robot's sensors in reference to squirtle
        observations = [odom]

        # run through ROMDP
        policy = romdp.value_iteration(observations)

        # Update particle weight according to how good every particle matches
        # squirtle's sensor reading
        for p in particles:
            if grid_map.is_free(*p.xy):
                p_d = p.dist
                p.w = pf.gauss_kernel(odom, p_d)
            else:
                p.w = 0

        # ---------- Try to find current best estimate for display ----------
        m_x, m_y, m_confident = pf.compute_mean_point(particles)

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
                new_particle = pf.Particle.create_random(1, grid_map)[0]
            else:
                new_particle = pf.Particle(p.x, p.y, heading=squirtle.h)
            new_particles.append(new_particle)

        particles = new_particles

        # ---------- Move things ----------
        old_heading = squirtle.h
        squirtle.move(0.2)
        d_h = squirtle.h - old_heading

        # Move particles according to my belief of movement (this may
        # be different than the real movement, but it's all I got)
        for p in particles:
            p.h += d_h  # in case robot changed heading, swirl particle heading too
            p.advance_by(squirtle.speed)

if __name__ == 'main':
    run_robots()