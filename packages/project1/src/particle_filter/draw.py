# Based on work created by https://github.com/mjl/particle_filter_demo
# Future: Modified for multiple ROS turtle bots with added unique security algorithm

import math
import turtle
import random
from Tkinter import Canvas


canvas = Canvas(width=5, height=5)
ts = turtle.Screen()
rt = turtle.RawTurtle(ts)
rt.tracer(50000, delay=0)
ts.register_shape("dot", ((-3,-3), (-3,3), (3,3), (3,-3)))
ts.register_shape("tri", ((-3, -2), (0, 3), (3, -2), (0, 0)))
rt.speed(0)

UPDATE_EVERY = 0
DRAW_EVERY = 2


class Maze(object):
    def __init__(self, maze):
        self.maze = maze
        self.width = len(maze[0])
        self.height = len(maze)
        ts.setworldcoordinates(0, 0, self.width, self.height)
        self.blocks = []
        self.update_cnt = 0
        self.one_px = float(ts.window_width()) / float(self.width) / 2

        self.beacons = []
        for y, line in enumerate(self.maze):
            for x, block in enumerate(line):
                if block:
                    nb_y = self.height - y - 1
                    self.blocks.append((x, nb_y))
                    if block == 2:
                        self.beacons.extend(((x, nb_y), (x+1, nb_y), (x, nb_y+1), (x+1, nb_y+1)))

    def draw(self):
        for x, y in self.blocks:
            rt.up()
            rt.setposition(x, y)
            rt.down()
            rt.setheading(90)
            rt.begin_fill()
            for _ in range(0, 4):
                rt.fd(1)
                rt.right(90)
            rt.end_fill()
            rt.up()

        rt.color("#00ffff")
        for x, y in self.beacons:
            rt.setposition(x, y)
            rt.dot()
        ts.update()

    @staticmethod
    def weight_to_color(weight):
        return "#%02x00%02x" % (int(weight * 255), int((1 - weight) * 255))

    def is_in(self, x, y):
        if x < 0 or y < 0 or x > self.width or y > self.height:
            return False
        return True

    def is_free(self, x, y):
        if not self.is_in(x, y):
            return False

        yy = self.height - int(y) - 1
        xx = int(x)
        return self.maze[yy][xx] == 0

    def show_mean(self, x, y, confident=False):
        if confident:
            rt.color("#00AA00")
        else:
            rt.color("#cccccc")
        rt.setposition(x, y)
        rt.shape("circle")
        rt.stamp()

    def show_particles(self, particles):
        self.update_cnt += 1
        if UPDATE_EVERY > 0 and self.update_cnt % UPDATE_EVERY != 1:
            return

        rt.clearstamps()
        rt.shape('triangle')

        draw_cnt = 0
        px = {}
        for p in particles:
            draw_cnt += 1
            if DRAW_EVERY == 0 or draw_cnt % DRAW_EVERY == 1:
                # Keep track of which positions already have something
                # drawn to speed up display rendering
                scaled_x = int(p.x * self.one_px)
                scaled_y = int(p.y * self.one_px)
                scaled_xy = scaled_x * 10000 + scaled_y
                if scaled_xy not in px:
                    px[scaled_xy] = 1
                    rt.setposition(*p.xy)
                    rt.setheading(90 - p.h)
                    rt.color(self.weight_to_color(p.w))
                    rt.stamp()

    @staticmethod
    def show_robot(robot):
        rt.color("green")
        rt.shape('turtle')
        rt.setposition(*robot.xy)
        rt.setheading(90 - robot.h)
        rt.stamp()
        ts.update()

    def random_place(self):
        x = random.uniform(0, self.width)
        y = random.uniform(0, self.height)
        return x, y

    def random_free_place(self):
        """
        Return a new random point that is open on the map.
        :return: random point that is free on the map
        """
        while True:
            x, y = self.random_place()
            if self.is_free(x, y):
                return x, y

    @staticmethod
    def distance(x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def distance_to_nearest_beacon(self, x, y):
        d = 99999
        for c_x, c_y in self.beacons:
            distance = self.distance(c_x, c_y, x, y)
            if distance < d:
                d = distance
                d_x, d_y = c_x, c_y
        return d