__author__ = 'shaun'

from utils import *

orientations = [(1, 0), (0, 1), (-1, 0), (0, -1)]


# utility functions from AIMA Berkeley Chapter 17 samples
def turn_right(orientation):
    return orientations[orientations.index(orientation)-1]


def turn_left(orientation):
    return orientations[(orientations.index(orientation)+1) % len(orientations)]


def best_policy(mdp, U):
    """Given an MDP and a utility function U, determine the best policy,
    as a mapping from state to action. (Equation 17.4)"""
    pi = {}
    for s in mdp.states:
        pi[s] = argmax(mdp.actions(s), lambda a:expected_utility(a, s, U, mdp))
    return pi


def expected_utility(a, s, U, mdp):
    """
    The expected utility of doing a in state s, according to the MDP and U.
    """
    return sum([p * U[s1] for (p, s1) in mdp.T(s, a)])


def to_grid(mapping):
    """Convert a mapping from (x, y) to v into a [[..., v, ...]] grid."""
    return list(reversed([[mapping.get((x, y), None)
                           for x in range(len(mapping[0]))]
                          for y in range(len(mapping))]))


def to_mapping(grid):
    """
    Convert from grid map to reward mapping.
    :param grid:
    :return:
    """
    R = dict()
    for x in range(len(grid[0])):
        for y in range(len(grid)):
            R[x, y] = grid[y][x]
    return R


def to_arrows(self, policy):
    chars = {(1, 0):'>', (0, 1):'^', (-1, 0):'<', (0, -1):'v', None: '.'}
    return self.to_grid(dict([(s, chars[a]) for (s, a) in policy.items()]))


class ROMDP(object):

    C = dict()
    S = set()
    R = dict()

    def __init__(self, grid, goal_states, discount=.9):
        """
        Create the Redundant Observable Markov Decision Process
        object. O and C are two elements added in addition to the
        conventional MDP definition.
        :param grid: a 2-d grid of rewards for each state at x and y (lists within a list)
        :param C: confidence function mapping elements of O into discrete probability distributions in S.
        In other words, C is the probability that the agent is in state s given the observation O.
        :param discount: the discount of probabilities over time
        """
        self.A = orientations
        self.discount = discount
        self.goal_states = goal_states
        self.initialize_c()

        # we need row 0 on the bottom rather than the top
        self.grid = grid.reverse()

        self.rows = len(grid)
        self.cols = len(grid[0])

        # produce S and R from the given grid
        self.construct_states_and_rewards(self.grid)

    def initialize_c(self):
        self.C = dict()
        for s in self.S:
            self.C[s] = 0

    def construct_states_and_rewards(self, grid):
        """
        Constructs the states set S with (0,0) to (n, m)
        where n is the width, m is the height,
        based on width and length of the grid.
        Constructs the rewards dict R from the
        grid given its labeled rewards per coordinate pair.
        :param grid: a list of lists representing the rows of the grid
        """
        for x in range(self.cols):
            for y in range(self.rows):
                self.R[x, y] = grid[y][x]
                if grid[y][x] is not None:
                    self.S.add((x, y))

    def T(self, state, action):
        """
        Calculate the probability of reaching every state from s given
        action a takes place.
        :param action: action that takes place
        :param state: current state
        :return: the probability of reaching s_ from with action a
        """
        if action is None:
            return [(0.0, state)]
        else:
            return [(0.8, self.move(state, action)),
                    (0.1, self.move(state, turn_right(action))),
                    (0.1, self.move(state, turn_left(action)))]

    def move(self, state, direction):
        """
        Return the state that results from moving in this direction.
        """
        state1 = vector_add(state, direction)
        return if_(state1 in self.S, state1, state)

    def actions(self, state):
        """
        Set of actions that can be performed in this state.  By default, a
        fixed list of actions, except for terminal states. Override this
        method if you need to specialize by state.
        """
        if state in self.goal_states:
            return [None]
        else:
            return self.A

    def value_iteration(self, O, K=10, epsilon=0.001):
        """
        Solve a ROMDP by value iteration with redundancy.
        :param O: the set of observations aka N sensor readings from N robots
        :param K: the maximum number of iterations to perform
        :return U1: the optimal policy set
        """
        k = 0
        while k < K:
            U = self.C.copy()
            delta = 0
            # do standard MDP
            for s in self.S:
                self.C[s] = self.R[s] + self.discount * max([sum([p * U[s1]
                                                            for (p, s1) in self.T(s, a)])
                                                            for a in self.A(s)])
                delta = max(delta, abs(self.C[s] - U[s]))
            # continue with redundant MDP
            if len(O) > 1:
                for o in self.O:
                    self.C[o] = self.R[o] + self.discount * max([sum([(p * self.C[o1]) * U[o1]
                                                                for (p, o1) in self.T(o, a)])
                                                                for a in self.A(o)])
                    delta = max(delta, abs(self.C[o] - U[o]))
            # return an optimal policy if found
            if delta < epsilon * (1 - self.discount) / self.discount:
                return self.C
            k += 1
        # return a sub-optimal policy
        return self.C
