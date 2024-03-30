"""
Модуль маршрутизации роботов
Получает от стратега требуемые координаты для каждого робота и
считает оптимальный маршрут для достижения этой точки
"""

import math
import typing

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.field as field
import bridge.processors.quickhull as qh
import bridge.processors.route as route
import bridge.processors.waypoint as wp


class Router:
    """
    Маршрутизатор
    """

    def __init__(self, fld: field.Field) -> None:
        """
        Конструктор
        """
        self.routes = [route.Route(fld.allies[i]) for i in range(const.TEAM_ROBOTS_MAX_COUNT)]

    def update(self, fld: field.Field) -> None:
        """
        Обновить маршруты актуальным состоянием поля
        """
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            self.routes[i].update(fld.allies[i])

    def set_dest(self, idx: int, target: wp.Waypoint, fld: field.Field) -> None:
        """
        Установить единственную путевую точку для робота с индексом idx
        """
        if idx != const.GK:
            # self_pos = fld.allies[idx].get_pos()
            dest_pos = target.pos
            for goal in [fld.ally_goal, fld.enemy_goal]:
                if aux.is_point_inside_poly(dest_pos, goal.hull):
                    closest_out = aux.find_nearest_point(dest_pos, goal.hull, [goal.up, aux.GRAVEYARD_POS, goal.down])
                    angle0 = target.angle
                    self.routes[idx].set_dest_wp(
                        wp.Waypoint(goal.center + (closest_out - goal.center) * 1.2, angle0, wp.WType.S_ENDPOINT)
                    )
                    return
        self.routes[idx].set_dest_wp(target)

    def reroute(self, fld: field.Field) -> None:
        """
        Рассчитать маршруты по актуальным путевым точкам
        """
        for idx in range(const.TEAM_ROBOTS_MAX_COUNT):

            self_pos = fld.allies[idx].get_pos()

            if not self.routes[idx].is_used():
                continue

            if (
                self.routes[idx].get_next_type() == wp.WType.S_BALL_KICK
                or self.routes[idx].get_next_type() == wp.WType.S_BALL_GRAB
            ):
                # if not field.allies[idx].is_kick_aligned(self.routes[idx].get_dest_wp()):
                align_wp = self.calc_kick_wp(idx)
                self.routes[idx].insert_wp(align_wp)

            if idx == const.GK:
                pth_wp = self.calc_vector_field(idx, fld)
                if pth_wp is not None:
                    self.routes[idx].insert_wp(pth_wp)
                continue

            for goal in [fld.ally_goal, fld.enemy_goal]:
                if aux.is_point_inside_poly(self_pos, goal.hull):
                    closest_out = aux.find_nearest_point(self_pos, goal.hull, [goal.up, aux.GRAVEYARD_POS, goal.down])
                    angle0 = self.routes[idx].get_dest_wp().angle
                    self.routes[idx].set_dest_wp(
                        wp.Waypoint(goal.center + (closest_out - goal.center) * 1.2, angle0, wp.WType.S_ENDPOINT)
                    )
                    continue
                pint = aux.segment_poly_intersect(self_pos, self_pos + self.routes[idx].get_next_vec(), goal.hull)
                if pint is not None:
                    if aux.is_point_inside_poly(self.routes[idx].get_dest_wp().pos, goal.hull):
                        angle0 = self.routes[idx].get_dest_wp().angle
                        self.routes[idx].set_dest_wp(wp.Waypoint(pint, angle0, wp.WType.S_ENDPOINT))
                        break
                    convex_hull = qh.shortesthull(self_pos, self_pos + self.routes[idx].get_next_vec(), goal.hull)
                    for j in range(len(convex_hull) - 2, 0, -1):
                        self.routes[idx].insert_wp(wp.Waypoint(convex_hull[j], 0, wp.WType.R_PASSTHROUGH))

            pth_wp = self.calc_vector_field(idx, fld)
            if pth_wp is not None:
                is_inside = False
                for goal in [fld.ally_goal, fld.enemy_goal]:
                    if aux.is_point_inside_poly(pth_wp.pos, goal.hull):
                        is_inside = True
                        break
                if not is_inside:
                    self.routes[idx].insert_wp(pth_wp)

    def calc_vector_field(self, idx: int, fld: field.Field) -> typing.Optional[wp.Waypoint]:
        """
        Рассчитать ближайшую промежуточную путевую точку
        согласно первому приближению векторного поля
        """
        self_pos = fld.allies[idx].get_pos()
        target_point = self.routes[idx].get_next_wp()
        dist = (self_pos - target_point.pos).mag()

        vector_field_threshold = 200
        if dist < vector_field_threshold:
            return None

        if (
            self.routes[idx].get_dest_wp().type == wp.WType.S_IGNOREOBSTACLES
            or self.routes[idx].get_dest_wp().type == wp.WType.S_BALL_GO
        ):
            return None

        sep_dist = 500
        ball_sep_dist = 150

        if self.routes[idx].get_dest_wp().type == wp.WType.S_KEEP_BALL_DISTANCE:
            ball_sep_dist = const.KEEP_BALL_DIST

        closest_robot: typing.Any = None
        closest_dist = dist

        # Расчет теней роботов для векторного поля
        for r in fld.all_bots:
            if r == self or r.is_used() == 0:
                continue
            robot_separation = aux.dist(aux.closest_point_on_line(self_pos, target_point.pos, r.get_pos()), r.get_pos())
            robot_dist = aux.dist(self_pos, r.get_pos())
            if robot_dist == 0:
                continue
            if robot_separation < sep_dist and robot_dist < closest_dist:
                closest_robot = r
                closest_dist = robot_dist
                closest_separation = robot_separation
        ball_separation = aux.dist(
            aux.closest_point_on_line(self_pos, target_point.pos, fld.ball.get_pos()), fld.ball.get_pos()
        )
        ball_dist = aux.dist(self_pos, fld.ball.get_pos())
        if ball_separation < ball_sep_dist and ball_dist < closest_dist:
            closest_robot = fld.ball
            closest_dist = ball_dist
            closest_separation = ball_separation

        passthrough_wp_pos = target_point.pos - self_pos
        # angle_cos = aux.scal_mult((closest_robot.get_pos() - self_pos).unity(),
        # (target_point.pos - self_pos).unity())
        if closest_robot is not None and closest_dist != 0:
            side = aux.vec_mult(closest_robot.get_pos() - self_pos, target_point.pos - self_pos)
            # offset_angle_val = 200*math.pi/6 * closest_dist**(-2)
            offset_angle_val = -2 * math.atan((sep_dist - closest_separation) / (2 * (closest_dist)))
            offset_angle = offset_angle_val if side < 0 else -offset_angle_val
            passthrough_wp_pos = self_pos + aux.rotate(target_point.pos - self_pos, offset_angle)

            passthrough_wp = wp.Waypoint(passthrough_wp_pos, 0, wp.WType.R_PASSTHROUGH)
        else:
            passthrough_wp = None

        return passthrough_wp

    def calc_kick_wp(self, idx: int) -> wp.Waypoint:
        """
        Рассчитать точку для выравнивания по мячу
        """
        target_point = self.routes[idx].get_dest_wp()
        target_pos = target_point.pos
        target_angle = target_point.angle

        align_pos = target_pos - aux.rotate(aux.RIGHT, target_angle) * const.KICK_ALIGN_DIST
        align_angle = target_angle
        align_type = wp.WType.R_BALL_ALIGN
        align_wp = wp.Waypoint(align_pos, align_angle, align_type)
        return align_wp

    def get_route(self, idx: int) -> route.Route:
        """
        Получить маршрут для робота с индексом idx
        """
        return self.routes[idx]
