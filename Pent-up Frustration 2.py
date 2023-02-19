from math import radians, sin, cos, tan, sqrt
import numpy as np
from numpy.linalg import norm

from tqdm import tqdm

import turtle
from time import sleep


def dist(a, b):
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def find_near_points(center, angle):
    near_points = []
    for _ in range(4):
        angle += rotating_angle
        point = (center[0] + cos(radians(angle)) * ct_dt, center[1] + sin(radians(angle)) * ct_dt, angle)
        near_points.append(point)
    return near_points


def find_vertices(center, angle):
    vertices = []
    for _ in range(5):
        vertices.append((center[0] + radius * cos(radians(angle)),
                         center[1] + radius * sin(radians(angle))))
        angle += rotating_angle
    return vertices


def slope_diff(a, b, c):
    """ slope difference between AB and AC """
    return (c[1] - a[1]) * (b[0] - a[0]) - (b[1] - a[1]) * (c[0] - a[0])


def isCollinear(p, a, b):
    if np.isclose(slope_diff(p, a, b), 0):
        return True
    else:
        False


def point_inside_polygon(point, vertices):
    inside = False

    for i in range(5):
        a = vertices[i - 1]
        b = vertices[i]

        if isCollinear(point, a, b):
            if min(a[0], b[0]) + 1e-8 < point[0] < max(a[0], b[0]) - 1e-8:
                inside = not inside
            else:
                continue

        if min(a[1], b[1]) + 1e-8 < point[1] < max(a[1], b[1]) - 1e-8:  # if the point is between y of the side of the pentagon
            x_inters = (point[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]) + a[0]
            if point[0] - 1e-8 < x_inters:
                inside = not inside

    return inside


def test_point(point):
    for n, center_coord in enumerate(center_coords[:-2]):
        if dist(point[:2], center_coord) - ct_dt < 1e-8:
            return False
        elif dist(point[:2], center_coord) - 2 * radius > 1e-8:
            continue
        else:
            vertices = find_vertices(point[:2], point[2])
            confront_to = vertices_coords[n]
            for vex in vertices:
                if dist(vex, center_coord) - radius >= -1e-8:
                    continue
                elif point_inside_polygon(vex, confront_to):
                    return False
    return True


def find_near_points_iterative(starting_points, iters):
    global minimum_distance

    for starting_point in starting_points:

        if not test_point(starting_point):
            continue

        center_coords.append(starting_point[:2])
        vertices_coords.append(find_vertices(starting_point[:2], starting_point[2]))

        if len(center_coords) == iters:
            if dist(starting_point[:2], starting_center) <= 3 * radius:
                distance1 = minimum_pentagons_distance(vertices_coords[0], vertices_coords[-1])
                distance2 = minimum_pentagons_distance(vertices_coords[-1], vertices_coords[0])
                minimum = min(distance1, distance2)
                if minimum < minimum_distance - 1e-8 and not np.isclose(minimum, 0):
                    minimum_distance = min(distance1, distance2)
                    print(round(minimum_distance, 7))
                    draw_pentagons()
            center_coords.pop(-1)
            vertices_coords.pop(-1)
            continue

        near_points = find_near_points(starting_point[:2], starting_point[2] + 180)
        find_near_points_iterative(near_points, iters)

        center_coords.pop(-1)
        vertices_coords.pop(-1)


def straight_point_distance(p, a, b):
    p = np.asarray(p)
    a = np.asarray(a)
    b = np.asarray(b)

    l2 = np.sum((a - b) ** 2)
    t = np.sum((p - a) * (b - a)) / l2

    if t > 1 + 1e-7 or t < -1e-7:
        return False
    else:
        return norm(np.cross(b - a, a - p)) / norm(a - b)


def minimum_pentagons_distance(first, last):
    minimum = 2 * radius

    for p in last:
        p = np.array(p)

        for i in range(5):

            a = first[i - 1]
            b = first[i]
            d = straight_point_distance(p, a, b)

            if not d:
                continue
            elif d < minimum - 1e-8:
                minimum = d

    return minimum


def draw_pentagons():
    turtle.clearscreen()
    turtle.speed(0)
    for vex in vertices_coords:
        turtle.penup()
        x0, y0 = vex[0]
        for x, y in vex:
            turtle.goto(x * 50, y * 50)
            turtle.pendown()
        turtle.goto(x0 * 50, y0 * 50)
    sleep(2.5)


if __name__ == '__main__':
    # SETTING GENERAL VARIABLES OF THE PENTAGON

    TOTAL_PENTAGON = 11

    side_length = 1
    ct_dt = (side_length * 0.5 * tan(radians(54))) * 2  # CENTER DISTANCE
    radius = (ct_dt / 2) / sin(radians(54))
    rotating_angle = 72

    minimum_distance = 1

    # SETTING VARIABLES OF THE FIRST PENTAGON

    starting_center = (0, 0)
    starting_vert = find_vertices(starting_center, 90)
    starting_angle = 54

    center_coords = [starting_center]
    vertices_coords = [starting_vert]

    # SETTING VARIABLES OF THE SECOND PENTAGON

    second_center = (starting_center[0] + cos(radians(starting_angle)) * ct_dt,
                     starting_center[1] + sin(radians(starting_angle)) * ct_dt)
    second_vert = find_vertices(second_center, starting_angle)

    center_coords.append(second_center)
    vertices_coords.append(second_vert)

    second_angle = starting_angle + 180
    points = find_near_points(center=second_center, angle=second_angle)
    find_near_points_iterative(starting_points=points[:2], iters=TOTAL_PENTAGON)
