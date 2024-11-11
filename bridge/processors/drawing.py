"""
draw field with robots and trajectory
"""

import math

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
        self.disable = True

        width, heigh = 1200, 900
        goal_dx, goal_dy = abs(const.GOAL_DX), abs(1500)
        self.scale = min(width / 2 / goal_dx, heigh / 2 / goal_dy)
        self.size_x = goal_dx * self.scale * 2
        self.size_y = goal_dy * self.scale * 2

        if not self.disable:
            pygame.init()
            self.screen = pygame.display.set_mode((width, heigh), pygame.RESIZABLE)
            pygame.display.set_caption("Football Field")
            self.middle_x, self.middle_y = self.screen.get_size()
            self.middle_x = round(self.middle_x / 2)
            self.middle_y = round(self.middle_y / 2)
            self.upper_border = self.middle_y - goal_dy * self.scale
            self.lower_border = self.middle_y + goal_dy * self.scale
            self.left_border = self.middle_x - goal_dx * self.scale
            self.right_border = self.middle_x + goal_dx * self.scale

    def draw_field(self) -> None:
        """
        draw green field and white lines
        """
        if self.disable:
            return

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

    def draw_poly(self, dots: list[aux.Point], color: tuple[int, int, int] = (255, 255, 255)) -> None:
        """
        Connect nearest dots with line
        """
        if self.disable:
            return
        new_dots = dots.copy()
        for i, dot in enumerate(dots):
            new_dots[i] = dot * self.scale + aux.Point(self.middle_x, self.middle_y)

        for i in range(len(new_dots) - 1):
            pygame.draw.line(self.screen, color, (new_dots[i].x, new_dots[i].y), (new_dots[i + 1].x, new_dots[i + 1].y), 2)
        pygame.draw.line(self.screen, color, (dots[len(dots) - 1].x, dots[len(dots) - 1].y), (dots[0].x, dots[0].y), 2)

    def draw_robot(self, r: aux.Point, angle: float = 0.0, robot_color: tuple[int, int, int] = (0, 0, 255)) -> None:
        """
        draw robot
        """
        if self.disable:
            return
        robot_radius = const.ROBOT_R * self.scale
        robot_length = 40
        end_point = (
            int(r.x * self.scale + self.middle_x + robot_length * math.cos(angle)),
            int(-r.y * self.scale + self.middle_y - robot_length * math.sin(angle)),
        )
        pygame.draw.circle(
            self.screen, robot_color, [r.x * self.scale + self.middle_x, -r.y * self.scale + self.middle_y], robot_radius
        )
        pygame.draw.line(
            self.screen, robot_color, [r.x * self.scale + self.middle_x, -r.y * self.scale + self.middle_y], end_point, 2
        )

    def draw_dot(self, pos: aux.Point, size: float = 3, color: tuple[int, int, int] = (255, 0, 0)) -> None:
        """
        draw single point
        """
        if self.disable:
            return
        pygame.draw.circle(
            self.screen, color, (pos.x * self.scale + self.middle_x, -pos.y * self.scale + self.middle_y), size
        )

    def draw_line(
        self, dot1: aux.Point, dot2: aux.Point, size: int = 2, color: tuple[int, int, int] = (255, 255, 0)
    ) -> None:
        """
        draw line
        """
        if self.disable:
            return
        pygame.draw.line(self.screen, color, (dot1.x, dot1.y), (dot2.x, dot2.y), size)

    def update_window(self) -> None:

        """
        update image
        """
        if self.disable:
            return
        pygame.display.flip()
        # self.draw_field()
