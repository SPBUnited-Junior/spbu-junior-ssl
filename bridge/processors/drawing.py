"""
draw field with robots and trajectory
"""

import math
import time
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
import pygame

import bridge.processors.auxiliary as aux
import bridge.processors.const as const

v_max = 400
a_max = 50


class Image:
    """
    class with image's specs
    """

    def __init__(self) -> None:
        pygame.init()
        width, heigh = 1200, 900
        self.screen = pygame.display.set_mode((width, heigh), pygame.RESIZABLE)
        pygame.display.set_caption("Football Field")

        goal_dx, goal_dy = abs(const.GOAL_DX), abs(3000)
        self.scale = min(width / 2 / goal_dx, heigh / 2 / goal_dy)
        self.middle_x, self.middle_y = self.screen.get_size()
        self.middle_x = round(self.middle_x / 2)
        self.middle_y = round(self.middle_y / 2)
        self.upper_border = self.middle_y - goal_dy * self.scale
        self.lower_border = self.middle_y + goal_dy * self.scale
        self.left_border = self.middle_x - goal_dx * self.scale
        self.right_border = self.middle_x + goal_dx * self.scale
        self.size_x = goal_dx * self.scale * 2
        self.size_y = goal_dy * self.scale * 2

    def draw_field(self) -> None:
        """
        draw green field and white lines
        """
        back_color = (128, 128, 128)
        field_color = (20, 178, 10)
        line_color = (255, 255, 255)
        self.screen.fill(back_color)
        pygame.draw.rect(self.screen, field_color, (self.left_border, self.upper_border, self.size_x, self.size_y))  # Поле
        pygame.draw.rect(
            self.screen, line_color, (self.left_border, self.upper_border, self.size_x, self.size_y), 2
        )  # Обводка поля
        pygame.draw.line(
            self.screen, line_color, (self.left_border, self.middle_y), (self.right_border, self.middle_y), 2
        )  # Горизонтальная линия
        pygame.draw.line(
            self.screen, line_color, (self.middle_x, self.upper_border), (self.middle_x, self.lower_border), 2
        )  # Вертикальная линия
        pygame.draw.circle(self.screen, line_color, (self.middle_x, self.middle_y), 50, 2)  # Круг в центре

    def draw_robot(self, r: aux.Point, angle: float = 0.0) -> None:
        """
        draw robot
        """
        robot_color = (0, 0, 255)
        robot_radius = 20
        robot_length = 40
        end_point = (
            int(r.x * self.scale + self.middle_x + robot_length * math.cos(angle)),
            int(-r.y * self.scale + self.middle_y - robot_length * math.sin(angle)),
        )
        pygame.draw.circle(self.screen, robot_color, [r.x * self.scale + self.middle_x, -r.y * self.scale + self.middle_y], robot_radius)
        pygame.draw.line(self.screen, robot_color, [r.x * self.scale + self.middle_x, -r.y * self.scale + self.middle_y], end_point, 2)

    def draw_dot(self, pos: aux.Point, size: float = 3, color: tuple[int, int, int] = (255, 0, 0)) -> None:
        """
        draw single point
        """
        pygame.draw.circle(self.screen, color, (pos.x * self.scale + self.middle_x, -pos.y * self.scale + self.middle_y), size)

    def update_window(self) -> None:

        """
        update image
        """
        pygame.display.flip()
        self.draw_field()

    def draw_bang_bang_trajectory(
        self, pos1: aux.Point, v1: aux.Point, pos2: aux.Point, v2: Optional[aux.Point] = None
    ) -> None:
        """
        draw full trajectory with speed v2 in end point if max speed isn't achieved
        """
        v = trapeze(pos2 - pos1, v1, v2)
        if v is None:
            print("error for:", pos2 - pos1, v1, v2)
            return
        # _, v = triangle_dist(pos2 - pos1, 300, v1, v2)
        dt = 0.1
        a1 = (v - v1).unity() * a_max
        t1 = (v - v1).mag() / a_max

        for tt in np.arange(0, t1, dt):
            dot_pos = pos1 + v1 * tt + a1 * tt**2 / 2
            self.draw_dot(dot_pos)
        extra_point = pos1 + (v1 + v) * t1 / 2

        self.draw_dot(pos1, 3, (127, 0, 255))
        if v_max - v.mag() < 10e-3:
            if v2 is None:
                t = (pos2 - pos1 - (v1 + v) * t1 / 2).mag() / v_max

                for tt in np.arange(0, t, dt):
                    dot_pos = extra_point + v * tt
                    self.draw_dot(dot_pos, 3, (0, 0, 255))

                self.draw_dot(pos2, 3, (255, 0, 0))
                print("Trapeze without end speed; path time: ", t1 + t)
            else:
                a2 = (v2 - v).unity() * a_max
                t2 = (v2 - v).mag() / a_max
                t = (pos2 - (v2 + v) * t2 / 2 - pos1 - (v1 + v) * t1 / 2).mag() / v_max

                for tt in np.arange(0, t, dt):
                    dot_pos = extra_point + v * tt
                    self.draw_dot(dot_pos, 3, (0, 0, 255))
                extra_point = pos1 + (v1 + v) * t1 / 2 + v * t
                for tt in np.arange(0, t2, dt):
                    dot_pos = extra_point + v * tt + a2 * tt**2 / 2
                    self.draw_dot(dot_pos)

                self.draw_dot(pos2, 3, (127, 0, 255))
                print("Trapeze with end speed; path time: ", t1 + t + t2)
        else:
            if v2 is None:
                self.draw_dot(pos2, 3, (0, 0, 255))
                print("Triangle without end speed; path time: ", t1)
            else:
                a2 = (v2 - v).unity() * a_max
                t2 = (v2 - v).mag() / a_max

                extra_point = pos1 + (v1 + v) * t1 / 2
                for tt in np.arange(0, t2, dt):
                    dot_pos = extra_point + v * tt + a2 * tt**2 / 2
                    self.draw_dot(dot_pos)

                self.draw_dot(pos1 + (v1 + v) * t1 / 2, 3, (0, 0, 255))
                self.draw_dot(pos2, 3, (127, 0, 255))
                print("Triangle with end speed; path time: ", t1 + t2)

    def draw_graph(self, pos1: aux.Point, v1: aux.Point, pos2: aux.Point, v2: Optional[aux.Point] = None) -> None:
        """
        draw full trajectory with speed v2 in end point if max speed isn't achieved
        """
        v = trapeze(pos2 - pos1, v1, v2)
        if v is None:
            print("error for:", pos2 - pos1, v1, v2)
            return
        # _, v = triangle_dist(pos2 - pos1, 300, v1, v2)
        dt = 0.1
        a1 = (v - v1).unity() * a_max
        t1 = (v - v1).mag() / a_max

        v_arr: list[aux.Point] = []
        a_arr: list[aux.Point] = []

        for tt in np.arange(0, t1, dt):
            v_arr.append(v1 + a1 * tt)
            a_arr.append(a1)

        if v_max - v.mag() < 10e-3:
            if v2 is None:
                t = (pos2 - pos1 - (v1 + v) * t1 / 2).mag() / v_max

                for tt in np.arange(0, t, dt):
                    v_arr.append(v)
                    a_arr.append(aux.Point(0, 0))

                self.draw_dot(pos2, 3, (255, 0, 0))
                print("Trapeze without end speed; path time: ", t1 + t)
            else:
                a2 = (v2 - v).unity() * a_max
                t2 = (v2 - v).mag() / a_max
                t = (pos2 - (v2 + v) * t2 / 2 - pos1 - (v1 + v) * t1 / 2).mag() / v_max

                for tt in np.arange(0, t, dt):
                    v_arr.append(v)
                    a_arr.append(aux.Point(0, 0))
                for tt in np.arange(-t2, 0, dt):
                    v_arr.append(v2 + a2 * tt)
                    a_arr.append(a2)

                # self.draw_dot(pos2, 3, (255, 255, 0))
                print("Trapeze with end speed; path time: ", t1 + t + t2)
        else:
            if v2 is None:
                self.draw_dot(pos2, 3, (0, 0, 255))
                print("Triangle without end speed; path time: ", t1)
            else:
                a2 = (v2 - v).unity() * a_max
                t2 = (v2 - v).mag() / a_max

                for tt in np.arange(-t2, 0, dt):
                    v_arr.append(v2 + a2 * tt)
                    a_arr.append(a2)

                print("Triangle with end speed; path time: ", t1 + t2)

        graf_for_params(v_arr, a_arr)


def trapeze(delta_pos: aux.Point, v1: aux.Point, v2: Optional[aux.Point], n: int = 10) -> Optional[aux.Point]:
    """
    calculate full trajectory for "trapeze" case (if end point if max speed is reached)
    """
    angle_near: float
    last_dist = -1.0
    for i in range(n):
        angle = math.pi * 2 * i / n
        v = aux.Point(math.cos(angle), math.sin(angle)) * v_max
        dist = dist_for_v(delta_pos, v, v1, v2)
        if last_dist == -1.0 or last_dist > dist:
            last_dist = dist
            angle_near = angle

    angle_min = angle_near - math.pi * 2 / n
    angle_max = angle_near + math.pi * 2 / n
    while angle_max - angle_min > 10e-5:
        angle1 = (angle_max - angle_min) * 1 / 3 + angle_min
        v = aux.Point(math.cos(angle1), math.sin(angle1)) * v_max
        dist1 = dist_for_v(delta_pos, v, v1, v2)

        angle2 = (angle_max - angle_min) * 2 / 3 + angle_min
        v = aux.Point(math.cos(angle2), math.sin(angle2)) * v_max
        dist2 = dist_for_v(delta_pos, v, v1, v2)

        if dist1 < dist2:
            angle_max = angle2
            last_dist = dist1
        else:
            angle_min = angle1
            last_dist = dist2

    if last_dist > 1:
        print(f"dist for trapeze: {last_dist}; n = {n}/50")
        return triangle(delta_pos, v1, v2, n)

    # v = aux.Point(math.cos(time.time()), math.sin(time.time())) * v_max # spinning trajectory :)
    print(last_dist)
    return v


def triangle(delta_pos: aux.Point, v1: aux.Point, v2: Optional[aux.Point], n: int) -> Optional[aux.Point]:
    """
    calculate full trajectory for "triangle" case (if end point if max speed isn't reached)
    """
    dist_min = -1.0
    v_near: float
    for i in range(1, n):
        v_mag = v_max * i / n
        dist, _ = triangle_dist(delta_pos, v_mag, v1, v2, n)
        if dist_min - 1.0 or dist < dist_min:
            dist_min = dist
            v_near = v_mag

    mag_min = v_near - v_max / n
    mag_max = v_near + v_max / n
    v: aux.Point
    while mag_max - mag_min > 10e-6:
        v_mag1 = (mag_max - mag_min) * 1 / 3 + mag_min
        v_mag2 = (mag_max - mag_min) * 2 / 3 + mag_min
        dist1, _ = triangle_dist(delta_pos, v_mag1, v1, v2, n)
        dist2, v = triangle_dist(delta_pos, v_mag2, v1, v2, n)
        if dist1 < dist2:
            mag_max = v_mag2
            dist_min = dist1
        else:
            mag_min = v_mag1
            dist_min = dist2

    print(f"dist for triangle: {dist_min}; n = {n}/50")

    # mass: list[float] = []
    # for i in range(1, 1000):
    #     a = triangle_dist(delta_pos, i / 1000, v1, v2)[0]
    #     mass.append(a)
    # print("=======================================")
    # plt.plot(mass)
    # plt.show()

    if dist_min > 10 or v.mag() > v_max:
        if n > 40:
            return None
        return trapeze(delta_pos, v1, v2, n + 10)

    # t1 = (v - v1).mag() / a_max
    # t2 = (v2 - v).mag() / a_max

    # dist = aux.dist((v1 + v) * t1 / 2 + (v2 + v) * t2 / 2, delta_pos)
    v = aux.Point(math.cos(time.time()), math.sin(time.time())) * v.mag()  # spinning trajectory :)

    return v


def triangle_dist(
    delta_pos: aux.Point, v_mag: float, v1: aux.Point, v2: Optional[aux.Point], n: int
) -> tuple[float, aux.Point]:
    """
    calculate angle for v in "triangle" case
    """
    angle_near: float
    last_dist = -1.0
    n += 5
    for i in range(n):
        angle = math.pi * 2 * i / n
        v = aux.Point(math.cos(angle), math.sin(angle)) * v_mag
        dist = dist_for_v(delta_pos, v, v1, v2)
        if last_dist == -1 or last_dist > dist:
            last_dist = dist
            angle_near = angle

    angle_min = angle_near - math.pi * 2 / n
    angle_max = angle_near + math.pi * 2 / n
    while angle_max - angle_min > 10e-5:
        angle1 = (angle_max - angle_min) * 1 / 3 + angle_min
        v = aux.Point(math.cos(angle1), math.sin(angle1)) * v_mag
        dist1 = dist_for_v(delta_pos, v, v1, v2)

        angle2 = (angle_max - angle_min) * 2 / 3 + angle_min
        v = aux.Point(math.cos(angle2), math.sin(angle2)) * v_mag
        dist2 = dist_for_v(delta_pos, v, v1, v2)

        if dist1 < dist2:
            angle_max = angle2
            last_dist = dist1
        else:
            angle_min = angle1
            last_dist = dist2

    # mass: list[float] = []
    # for i in range(1, 1000):
    #     angle1 = math.pi * 2 * i / 1000
    #     v = aux.Point(math.cos(angle1), math.sin(angle1)) * v_mag
    #     dist1 = dist_for_v(delta_pos, v, v1, v2)
    #     mass.append(dist1)
    # print("=======================================")
    # plt.plot(mass)
    # plt.show()

    return last_dist, v


def dist_for_v(delta_pos: aux.Point, v: aux.Point, v1: aux.Point, v2: Optional[aux.Point]) -> float:
    """
    calculate distance between trajectories from start and end for every case
    """
    if v_max - v.mag() < 10e-3:
        if v2 is None:  # trapeze without end speed
            t1 = (v - v1).mag() / a_max
            t = (delta_pos - (v1 + v) * t1 / 2).mag() / v_max
            return aux.dist(delta_pos, (v1 + v) * t1 / 2 + v * t)
        t1 = (v - v1).mag() / a_max  # trapeze with end speed
        t2 = (v2 - v).mag() / a_max
        t = (delta_pos - (v1 + v) * t1 / 2 - (v2 + v) * t2 / 2).mag() / v_max
        return aux.dist(delta_pos, (v1 + v) * t1 / 2 + v * t + (v2 + v) * t2 / 2)
    else:
        if v2 is None:  # triangle without end speed
            t1 = (v - v1).mag() / a_max
            return aux.dist((v1 + v) * t1 / 2, delta_pos)
        t1 = (v - v1).mag() / a_max  # triangle with end speed
        t2 = (v2 - v).mag() / a_max
        return aux.dist((v1 + v) * t1 / 2 + (v2 + v) * t2 / 2, delta_pos)


def graf_for_params(v: list[aux.Point], a: list[aux.Point]) -> None:
    """
    nothing interesting, pls you don't need to watch here
    """
    v_x, v_y = [], []
    for speed in v:
        v_x.append(speed.x)
        v_y.append(speed.y)

    a_x, a_y = [], []
    for acc in a:
        a_x.append(acc.x)
        a_y.append(acc.y)

    t: list[int] = []
    for i in range(len(v)):
        t.append(i)

    _, ax = plt.subplots(2, 1)

    ax[0].plot(t, v_x, t, v_y)
    ax[0].legend(["Speed_x", "Speed_y"])

    ax[1].plot(t, a_x, t, a_y)
    ax[1].legend(["Acceleration_x", "Acceleration_y"])

    for axes in ax:
        axes.grid(True)
    plt.show()
