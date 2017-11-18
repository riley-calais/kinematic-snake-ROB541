import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

import Joint
import Primitives


def draw_ground():
    """"""
    # a bunch of lines with increasing x values
    glColor3f(1, 1, 1)

    xs = [x / 10 for x in range(-100, 100, 15)]

    for x in xs:
        a = [x, -0.7, 10]
        b = [x + 0.2, -0.7, 10]
        c = [x, -0.7, -10]
        d = [x + 0.2, -0.7, -10]
        Primitives.rectangle(a, b, c, d)


def main():
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)

    glRotatef(25, 1, 0, 0)
    glTranslatef(0.0, -2.5, -10)

    # glTranslatef(0.0, 0.0, -5)

    center = Joint.Joint(0.0, 0.0, 3.0, 0.0, 0.0)
    left = Joint.Joint(-2.7, 0.0, 3.0, 0.0, 0.0)
    right = Joint.Joint(2.7, 0.0, 3.0, 0.0, 0.0)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        draw_ground()
        left.draw_joint()
        center.draw_joint()
        right.draw_joint()

        pygame.display.flip()
        pygame.time.wait(80)


if __name__ == '__main__':
    main()




