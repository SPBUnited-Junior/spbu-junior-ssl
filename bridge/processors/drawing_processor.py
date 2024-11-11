"""Processor for drawing"""

import typing

import attr
import pygame
from strategy_bridge.bus import DataBus, DataReader
from strategy_bridge.processors import BaseProcessor

from bridge import const, drawing
from bridge.auxiliary import aux, fld


@attr.s(auto_attribs=True)
class Drawer(BaseProcessor):
    """Class for drawing"""

    processing_pause: typing.Optional[float] = 1 / 30
    reduce_pause_on_process_time: bool = True
    max_commands_to_persist: int = 20
    commands_sink_reader: DataReader = attr.ib(init=False)

    def initialize(self, data_bus: DataBus) -> None:
        """
        Инициализация
        """
        super(Drawer, self).initialize(data_bus)
        self.field_reader = DataReader(data_bus, const.FIELD_TOPIC)
        self.image_reader = DataReader(data_bus, const.IMAGE_TOPIC)

        width, heigh = 1200, 900
        goal_dx, goal_dy = abs(const.GOAL_DX), abs(const.HALF_HEIGHT)
        self.scale = min(width / 2 / goal_dx, heigh / 2 / goal_dy)
        self.size_x = goal_dx * self.scale * 2
        self.size_y = goal_dy * self.scale * 2

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

    def process(self) -> None:

        message_img = self.image_reader.read_last()
        message_fld = self.field_reader.read_last()
        if message_img is None or message_fld is None:
            return

        external_image: drawing.Image = message_img.content
        field: fld.Field = message_fld.content

        sum_len = 0
        for g in external_image.commands:
            sum_len += len(g.dots)
        # print(sum_len)
        if sum_len > 60 or 1:
            for rbt in field.allies:
                if rbt.is_used():
                    external_image.draw_robot(rbt.get_pos(), rbt.get_angle())
            for rbt in field.enemies:
                if rbt.is_used():
                    external_image.draw_robot(rbt.get_pos(), rbt.get_angle(), (255, 255, 0))

            external_image.draw_dot(field.ball.get_pos(), (255, 0, 0), const.BALL_R)

            for command in external_image.commands:
                self.scale_dots(command)
                self.complete_command(command)

            pygame.display.flip()
            pygame.event.get()

            self.draw_field()
            for goal in [field.ally_goal, field.enemy_goal]:
                new_dots = []
                for dot in goal.hull:
                    new_dots.append(dot * self.scale + aux.Point(self.middle_x, self.middle_y))

                for i, _ in enumerate(new_dots):
                    pygame.draw.line(
                        self.screen,
                        (255, 255, 255),
                        (new_dots[i - 1].x, new_dots[i - 1].y),
                        (new_dots[i].x, new_dots[i].y),
                        2,
                    )
                goal_up = goal.up * self.scale + aux.Point(self.middle_x, self.middle_y)
                goal_down = goal.down * self.scale + aux.Point(self.middle_x, self.middle_y)
                pygame.draw.line(
                    self.screen,
                    (255, 255, 255),
                    (goal_up.x, goal_up.y),
                    (goal_down.x, goal_down.y),
                    10,
                )

    def draw_field(self) -> None:
        """
        draw green field and white lines
        """

        back_color = (128, 128, 128)
        field_color = (20, 178, 10)
        line_color = (255, 255, 255)
        self.screen.fill(back_color)
        pygame.draw.rect(
            self.screen,
            field_color,
            (self.left_border, self.upper_border, self.size_x, self.size_y),
        )  # Поле
        pygame.draw.rect(
            self.screen,
            line_color,
            (self.left_border, self.upper_border, self.size_x, self.size_y),
            2,
        )  # Обводка поля
        pygame.draw.line(
            self.screen,
            line_color,
            (self.left_border, self.middle_y),
            (self.right_border, self.middle_y),
            2,
        )  # Горизонтальная линия
        pygame.draw.line(
            self.screen,
            line_color,
            (self.middle_x, self.upper_border),
            (self.middle_x, self.lower_border),
            2,
        )  # Вертикальная линия
        pygame.draw.circle(self.screen, line_color, (self.middle_x, self.middle_y), 50, 2)  # Круг в центре

    def complete_command(self, command: drawing.Command) -> None:
        """Draw something on a screen by command"""
        if len(command.dots) == 1:
            pygame.draw.circle(self.screen, command.color, command.dots[0], command.size * self.scale)
        elif len(command.dots) == 2:
            pygame.draw.line(
                self.screen,
                command.color,
                command.dots[0],
                command.dots[1],
                round(command.size),
            )
        elif len(command.dots) > 2:
            for i, _ in enumerate(command.dots):
                pygame.draw.line(
                    self.screen,
                    command.color,
                    command.dots[i - 1],
                    command.dots[i],
                    round(command.size),
                )

    def scale_dots(self, command: drawing.Command) -> None:
        """Scale dots from coordinates to pixels"""
        for i, _ in enumerate(command.dots):
            command.dots[i] = (
                command.dots[i][0] * self.scale + self.middle_x,
                -command.dots[i][1] * self.scale + self.middle_y,
            )
