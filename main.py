import math
from copy import deepcopy
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
        Primitives.render_rectangle(a, b, c, d, True)
        # draw the wall
        a = [x, -0.2, -10]
        b = [x + 0.2, -0.2, -10]
        c = [x, 10, -10]
        d = [x + 0.2, 10, -10]
        Primitives.render_rectangle(a, b, c, d, True)


def main():
    pygame.init()
    disp_size = 600
    display = (disp_size, disp_size)
    surface = pygame.display.set_mode(display, pygame.locals.DOUBLEBUF | pygame.locals.OPENGL)
    directory = "C:\\Users\\Riley\\Documents\\GeoMech_Gifs\\gif_comp\\"

    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)

    glRotatef(25, 1, 0, 0)
    glTranslatef(-1.0, -2.5, -10)

    # glTranslatef(0.0, 0.0, -5)

    # joint_left = Joint.Joint(0.0)  # the joint between the left link and the center link
    # joint_right = Joint.Joint(0.0)  # the joint between the center link and the right link
    joint_left = math.pi * 0.5
    joint_right = math.pi * 0.5
    center = Link.Link()
    g = deepcopy(center)
    beta = GeoUtils.calc_beta(center, joint_left, joint_right)
    g.update(beta.matrix)
    # center.update(SE3.SE3(-1 * center.length, 0.0, 0.0, GeoUtils.quat_from_theta(0.0)).matrix)
    left = Link.Link()
    h_left = GeoUtils.frame_difference(left.gmp.matrix, GeoUtils.g_left(center, -1 * joint_left).matrix)
    left.update(h_left)
    # left.update(SE3.SE3(-1 * center.displacement(), 0.0, 0.0, GeoUtils.quat_from_theta(0.0)).matrix)
    right = Link.Link()
    h_right = GeoUtils.frame_difference(right.gmp.matrix, GeoUtils.g_right(center, joint_right).matrix)
    right.update(h_right)
    # right.update(SE3.SE3(center.displacement(), 0.0, 0.0, GeoUtils.quat_from_theta(0.0)).matrix)


    vertex_list = {}  # initialize an empty dictionary each time
    draw_walls()
    left.draw_joint(vertex_list)
    # center.update(SE3.SE3(0.0, 0.0, 0.0, GeoUtils.quat_from_theta(math.pi / 50)).matrix)
    center.draw_joint(vertex_list)
    right.draw_joint(vertex_list)

    Primitives.render_all(vertex_list)
    pygame.display.flip()
    pygame.time.wait(80)

    beta = GeoUtils.calc_beta(center, joint_left, joint_right)

    steps = 100
    for i in range(steps):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        vertex_list = {}  # initialize an empty dictionary each time

        draw_walls()

        # TODO: draw links based on relative placement... almost there...
        # TODO: update links based on body velocity of CoM frame

        beta = GeoUtils.calc_beta(center, joint_left, joint_right)
        joint_left += math.pi / 50
        h_left = GeoUtils.frame_difference(left.gmp.matrix, GeoUtils.g_left(center, -1 * joint_left).matrix)
        left.update(h_left)
        joint_right += math.pi / 50
        h_right = GeoUtils.frame_difference(right.gmp.matrix, GeoUtils.g_right(center, joint_right).matrix)
        # right.update(h_right)
        left.draw_joint(vertex_list)
        # center.update(SE3.SE3(0.0, 0.0, 0.0, GeoUtils.quat_from_theta(math.pi / 50)).matrix)
        center.draw_joint(vertex_list)
        right.draw_joint(vertex_list)

        Primitives.render_all(vertex_list)
        pygame.display.flip()
        pygame.time.wait(40)

        pygame.image.save(surface, directory + "pic" + str(i) + ".png")


if __name__ == '__main__':
    main()
    # dict = {1.0: ['a', 'b'], 2.0: ['c', 'd'], 1.5: ['e', 'f']}
    # for i in sorted(dict.keys()):
    #     print("key: {}, value: {}".format(i, dict[i]))


