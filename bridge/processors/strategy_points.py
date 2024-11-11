"""
Processor that creates the field
"""

import typing
from time import time
import math

import attr
from scipy.optimize import minimize
from strategy_bridge.bus import DataBus, DataReader, DataWriter
from strategy_bridge.common import config
from strategy_bridge.pb.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from strategy_bridge.processors import BaseProcessor

from bridge import const
from bridge.auxiliary import aux, fld, rbt

global_pass_points = []
for i in range(const.PASS_PS[0]):
    for j in range(const.PASS_PS[1]):
        global_pass_points.append(aux.Point((2 * i - const.PASS_PS[0] + 1) / (const.PASS_PS[0] + 1) *
            const.GOAL_DX, (2 * j - const.PASS_PS[1] + 1) / (const.PASS_PS[1] + 1) * const.HALF_HEIGHT))

@attr.s(auto_attribs=True)
class StrategyPoints(BaseProcessor):
    """class that creates the field"""

    processing_pause: typing.Optional[float] = 0.02
    reduce_pause_on_process_time: bool = False
    commands_sink_reader: DataReader = attr.ib(init=False)
    field_writer: DataWriter = attr.ib(init=False)
    _ssl_converter: SSL_WrapperPacket = attr.ib(init=False)
    ally_color = const.COLOR

    def initialize(self, data_bus: DataBus) -> None:
        """
        Инициализация
        """
        super(StrategyPoints, self).initialize(data_bus)

        self.field = fld.Field()
        self.field_reader = DataReader(data_bus, const.FIELD_TOPIC)

        self.points_topic = DataWriter(data_bus, const.POINTS_TOPIC, 20)
        self.image_writer = DataWriter(data_bus, const.IMAGE_TOPIC, 20)
        self.active_allies: list[rbt.Robot]
        self.active_enemies: list[rbt.Robot]
        self.ball_hull: list[aux.Point] = []

    def process(self) -> None:
        """
        Метод обратного вызова процесса
        """
        self.do_cycle_config()
        # print("aaa")
        points = self.find_pass_points(self.field)
        for point in points:
            self.pass_metrics(point, self.field, True)
        self.points_topic.write(points)
        if self.field.image is not None and self.field.ally_color == const.COLOR:
            self.image_writer.write(self.field.image)

    def do_cycle_config(self) -> None:
        """обновление данных в цикле"""
        new_field = self.field_reader.read_last()
        if new_field is not None:
            self.field = new_field.content
            if self.field.ally_color != self.ally_color:
                self.field.reverse_field()
        self.active_allies = fld.find_nearest_robots(self.field.ball.get_pos(), self.field.allies, None, [self.field.allies[const.GK]])
        self.active_enemies = fld.find_nearest_robots(self.field.ball.get_pos(), self.field.enemies)
        self.ball_hull = [self.field.ball.get_pos(), self.field.enemy_goal.up, self.field.enemy_goal.down]

    def pass_metrics(self, point: aux.Point, field: fld.Field, flag: bool = False) -> float:
        """метрика точки для паса"""
        rb_point = aux.point_on_line(field.ball.get_pos(), point, aux.dist(field.ball.get_pos(), point) + const.BALL_R + const.ROBOT_R)
        gate_angle = fld.gate_angle_size(point, field)[2]
        enemy_dists = list(map(lambda x: aux.dist(aux.closest_point_on_line(rb_point,
            field.ball.get_pos(), x.get_pos()), x.get_pos()), self.active_enemies))
        enemy_dists.append(aux.dist(rb_point, aux.nearest_point_on_poly(rb_point, field.enemy_goal.big_hull)) + 2 * const.ROBOT_R)
        kick_dist = aux.dist(rb_point, aux.nearest_point_on_poly(rb_point, self.ball_hull))
        if aux.is_point_inside_poly(rb_point, self.ball_hull):
            kick_dist *= -1
        enemy_dists.append(kick_dist)
        enemy_dist = sorted(enemy_dists)[0] - 2 * const.ROBOT_R
        pass_dist = (min(aux.dist(field.ball.get_pos(), point), const.NORM_PASS_DIST)
            - max(aux.dist(field.enemy_goal.center, point), const.MIN_PASS_DIST))
        if aux.is_point_inside_poly(rb_point, field.enemy_goal.big_hull):
            return aux.dist(aux.nearest_point_on_poly(rb_point, field.enemy_goal.big_hull), rb_point) + const.DIAGONAL + const.MIN_PASS_DIST
        if enemy_dist < const.MIN_ENEMY_DIST:
            return const.DIAGONAL - enemy_dist + const.MIN_PASS_DIST
        if aux.dist(point, field.ball.get_pos()) < const.MIN_PASS_DIST:
            return const.MIN_PASS_DIST - aux.dist(point, field.ball.get_pos())
        fun = (-gate_angle * const.K_PASS[0] + const.K_PASS[1]
             / enemy_dist - pass_dist * const.K_PASS[2] - const.DIAGONAL * const.K_PASS[2] - const.K_PASS[1] / const.MIN_ENEMY_DIST)
        # if flag:
            # print("start", point)
            # print(fun)
            # print((-gate_angle * const.K_PASS[0], const.K_PASS[1]
            #     / enemy_dist, - pass_dist * const.K_PASS[2], - const.DIAGONAL * const.K_PASS[2] - const.K_PASS[1] / const.MIN_ENEMY_DIST))
            # print("end")
        # col = int(max(min((-fun - 9) * 255 / 7, 255), 0))
        # field.image.draw_dot(point, (255 - col, 255 - col, col), const.BALL_R)
        return fun

    def convert_point_type(self, a: tuple[float, float]) -> float:
        """преобразует координату в точку и загоняет в метрику"""
        return self.pass_metrics(aux.Point(a[0], a[1]), self.field)

    def find_pass_points(self, field: fld.Field) -> list:
        """возвращает точки для пасов"""
        global_pass_points.sort(key = lambda x: self.pass_metrics(x, field))
        pass_points: list[aux.Point] = []
        for k in global_pass_points:
            # best_point_dict = minimize(
            #     self.convert_point_type,
            #     (k.x, k.y),
            #     bounds=[
            #         (-const.GOAL_DX, const.GOAL_DX),
            #         (-const.HALF_HEIGHT, const.HALF_HEIGHT),
            #     ],
            #     method="Nelder-Mead"
            # )
            # best_point = aux.Point(best_point_dict['x'][0], best_point_dict['x'][1])
            # if best_point_dict['fun'] < 0:
            #     col = int(min((-best_point_dict['fun'] - 12.4) * 255 / 7, 255))
            #     if pass_points:
            #         if aux.dist(aux.find_nearest_point(best_point, pass_points), best_point) > const.MIN_ACTION_DIST:
            #             pass_points.append(best_point)
            #             # field.image.draw_dot(best_point, (255 - col, col, 0), const.BALL_R)
            #     else:
            #         pass_points.append(best_point)
            #         # field.image.draw_dot(best_point, (255 - col, col, 0), const.BALL_R)
            # else:
            #     break
            if pass_points:
                if aux.dist(aux.find_nearest_point(k, pass_points), k) > const.MIN_ACTION_DIST:
                    pass_points.append(k)
            else:
                pass_points.append(k)
            if len(pass_points) == const.SENDING_ACTION_POINTS:
                break
        # print(len(pass_points))
        return pass_points
