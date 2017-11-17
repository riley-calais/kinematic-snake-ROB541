import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

import Joint


def main():
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)

    glRotatef(15, 1, 0, 0)

    glTranslatef(0.0, -1.5, -5)

    joint = Joint.Joint(0.0, 0.0, 0.0, 0.0, 0.0)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        joint.draw_joint()

        pygame.display.flip()
        pygame.time.wait(10)


if __name__ == '__main__':
    main()