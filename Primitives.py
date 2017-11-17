import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *


def rectangle(a, b, c, d):
    """ the parameters are each a single vertex of the rectangle, in the following arrangement:
        a - - - b
        |       |
        |       |
        c - - - d"""
    triangle(a, b, c)  # upper left portion
    triangle(b, c, d)  # bottom right portion


def triangle(a, b, c):
    """ the parameters are each a single vertex of the triangle"""
    glBegin(GL_TRIANGLES)
    glVertex3f(a[0], a[1], a[2])
    glVertex3f(b[0], b[1], b[2])
    glVertex3f(c[0], c[1], c[2])
    glEnd()


if __name__ == '__main__':
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)

    glTranslatef(0.0, 0.0, -5)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        a = (-0.5, 0.5, 0.0)
        b = (0.5, 0.5, 0.0)
        c = (-0.5, -0.5, 0.0)
        d = (0.5, -0.5, 0.0)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        rectangle(a, b, c, d)

        pygame.display.flip()
        pygame.time.wait(10)


