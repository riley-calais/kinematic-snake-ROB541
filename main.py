import math
from copy import deepcopy
import csv
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

import Link
import Primitives
import SE3
import GeoUtils


def draw_walls():
    """ draws static lines so we can see the snake's displacement"""
    # a bunch of lines with increasing x values
    glColor3f(1, 1, 1)

    xs = [x / 10 for x in range(-100, 100, 15)]

    for x in xs:
        # draw the floor
        a = [x, -0.2, 10]
        b = [x + 0.2, -0.2, 10]
        c = [x, -0.2, -10]
        d = [x + 0.2, -0.2, -10]
        Primitives.render_rectangle(a, b, c, d, (1, 1, 1), True)
        # draw the wall
        a = [x, -0.2, -10]
        b = [x + 0.2, -0.2, -10]
        c = [x, 10, -10]
        d = [x + 0.2, 10, -10]
        Primitives.render_rectangle(a, b, c, d, (1, 1, 1), True)


def main():
    pygame.init()
    disp_size = 600
    display = (disp_size, disp_size)
    surface = pygame.display.set_mode(display, pygame.locals.DOUBLEBUF | pygame.locals.OPENGL)
    directory = "C:\\Users\\Riley\\Documents\\GeoMech_Gifs\\gif_comp\\"
    pause = 5

    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)

    glRotatef(25, 1, 0, 0)
    glTranslatef(-1.0, -2.5, -10)

    # initialize alphas to 0
    joint_left = 0.0
    joint_right = 0.0
    # center is the center link, g is the system body frame
    center = Link.Link((0.2, 0.6, 0.2))
    g = Link.Link((0, 0, 0))
    # beta the transform from center link to CoM/mean orientation frame
    beta = GeoUtils.calc_beta(center.length, joint_left, joint_right)
    g.update_com(center, beta)
    center.gmp.hard_reset(g.gmp.matrix * beta.matrix.getI())  # welp. this is hideous.
    # left is the index -1 link, right is the index 1 link
    left = Link.Link((0.2, 0.3, 0.2))
    right = Link.Link((0.2, 0.9, 0.2))

    # create a list of vertices that will need to be drawn
    vertex_list = {}  # initialize an empty dictionary each time
    draw_walls()
    left.draw_joint(vertex_list)
    center.draw_joint(vertex_list)
    right.draw_joint(vertex_list)

    # draw everything to the screen, in the correct z and y order
    Primitives.render_all(vertex_list)
    pygame.display.flip()
    pygame.time.wait(pause)

    filename = directory + "max_displacement.csv"
    # while True:
    with open(filename) as alpha_file:
        alpha_reader = csv.reader(alpha_file)
        first_loop = True
        alpha_prev = [0, 0]
        i = 0
        for alpha_cur in alpha_reader:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    quit()

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            vertex_list = {}  # reset the dictionary each time

            draw_walls()

            if first_loop:
                alpha_dot_left = 0.0  # the velocity starts at zero
                alpha_dot_right = 0.0
            else:
                alpha_dot_left = float(alpha_cur[0]) - float(alpha_prev[0])
                alpha_dot_right = float(alpha_cur[1]) - float(alpha_prev[1])

            joint_left = -float(alpha_cur[0])
            joint_right = float(alpha_cur[1])
            beta = GeoUtils.calc_beta(center.length, joint_left, joint_right)
            g.update_com(center, beta)
            g_body_velocity = GeoUtils.system_body_velocity(g.length, joint_left, joint_right, alpha_dot_left,
                                                            alpha_dot_right)
            g_dot = GeoUtils.world_velocity(g_body_velocity, g)
            g.shift(g_dot)
            center_shift = g.gmp.matrix * beta.matrix.getI()
            center.gmp.hard_reset(center_shift)
            print("i: {}, g_dot.x: {}".format(i, g_dot.x()))
            left.update(GeoUtils.g_left(center, joint_left))
            right.update(GeoUtils.g_right(center, joint_right))

            # now draw everything
            left.draw_joint(vertex_list)
            center.draw_joint(vertex_list)
            right.draw_joint(vertex_list)
            Primitives.render_all(vertex_list)
            # save each frame so we can make a gif afterward
            pygame.image.save(surface, directory + "pic" + str(i) + ".png")
            i = i + 1
            pygame.display.flip()
            pygame.time.wait(pause)

            alpha_prev = list(alpha_cur)
            first_loop = False


def print_inputs():
    radius = math.pi * 0.125
    angle = 0
    steps = 100
    center = 3 * math.pi / 8
    for t in reversed(range(steps)):
        angle += 2 * math.pi / steps
        x = center + radius * math.cos(angle)
        y = (-1 * center) + radius * math.sin(angle)
        print("{},{}".format(x, y))


if __name__ == '__main__':
    main()


