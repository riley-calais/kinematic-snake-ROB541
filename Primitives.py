from OpenGL.GL import *


def rectangle(a, b, c, d, vertex_list, color):
    """ the parameters are each a single vertex of the rectangle, in the following arrangement:
        a - - - b
        |       |
        |       |
        c - - - d
        """
    triangle(a, b, c, vertex_list, color)  # upper left portion
    triangle(b, c, d, vertex_list, color)  # bottom right portion


def triangle(a, b, c, vertex_list, color):
    """ adds a set of vertexes to the vertex list, indexed by min z value"""
    min_z = min(a.item(2), b.item(2), c.item(2))
    min_y = min(a.item(1), b.item(1), c.item(1))
    if min_z in vertex_list:
        if min_y in vertex_list[min_z]:
            vertex_list[min_z][min_y].append((a, b, c, color))  # append to an existing list
        else:
            vertex_list[min_z][min_y] = [(a, b, c, color)]  # create a new list with just one tuple
    else:
        vertex_list[min_z] = {min_y: [(a, b, c, color)]}  # create a new dictionary with one list and one tuple


def render_all(vertex_list):
    """loop over all vertices and render them"""
    for z in sorted(vertex_list.keys()):
        for y in sorted(vertex_list[z].keys()):
            for a, b, c, color in vertex_list[z][y]:
                render_triangle(a, b, c, color)


def render_triangle(a, b, c, color):
    """ the parameters are each a single vertex of the triangle"""
    # if not color_flag:  # use the default colors
    glColor3f(color[0], color[1], color[2])
    glBegin(GL_TRIANGLES)
    glVertex3f(a[0], a[1], a[2])
    glVertex3f(b[0], b[1], b[2])
    glVertex3f(c[0], c[1], c[2])
    glEnd()
    # lines from a to b, from b to c, and c to a
    # if not color_flag:
    glColor3f(color[0] * 0.8, color[1] * 0.8, color[2] * 0.8)  # use a slightly darker color for the lines
    glBegin(GL_LINE_STRIP)
    glVertex3f(a[0], a[1], a[2])
    glVertex3f(b[0], b[1], b[2])
    glVertex3f(c[0], c[1], c[2])
    glVertex3f(a[0], a[1], a[2])
    glEnd()


def render_rectangle(a, b, c, d, color_flag):
    """ the parameters are each a single vertex of the rectangle, in the following arrangement:
        a - - - b
        |       |
        |       |
        c - - - d
        """
    render_triangle(a, b, c, color_flag)  # upper left portion
    render_triangle(b, c, d, color_flag)  # bottom right portion


if __name__ == '__main__':
    pass
    # pygame.init()
    # display = (800, 600)
    # pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    #
    # gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    #
    # glTranslatef(0.0, 0.0, -5)
    #
    # while True:
    #     for event in pygame.event.get():
    #         if event.type == pygame.QUIT:
    #             pygame.quit()
    #             quit()
    #
    #     a = (-0.5, 0.5, 0.0)
    #     b = (0.5, 0.5, 0.0)
    #     c = (-0.5, -0.5, 0.0)
    #     d = (0.5, -0.5, 0.0)
    #
    #     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    #     rectangle(a, b, c, d)
    #
    #     pygame.display.flip()
    #     pygame.time.wait(10)


