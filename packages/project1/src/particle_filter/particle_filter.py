import random
import math
import bisect

from romdp.utils import distance

#  Credit to Martin J. Laubach for the particle filter

PARTICLE_COUNT = 2000    # Total number of particles
sigma2 = 0.9 ** 2


def add_noise(level, *coords):
    return [x + random.uniform(-level, level) for x in coords]


def add_little_noise(*coords):
    return add_noise(0.02, *coords)


def add_some_noise(*coords):
    return add_noise(0.1, *coords)


def gauss_kernel(a, b):
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
        if distance((p.x, p.y), (m_x, m_y)) < 1:
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

    @classmethod
    def create_random(cls, count, grid_map):
        return [cls(*grid_map.random_free_place()) for _ in range(0, count)]

    def __repr__(self):
        return "(x=%f, y=%f, h=%f, w=%f)" % (self.x, self.y, self.h, self.w)

    def dist_to(self, x, y):
        return distance((x, y), (self.x, self.y))