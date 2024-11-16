"""
Модуль маршрутизации роботов
Получает от стратега требуемые координаты для каждого робота и
считает оптимальный маршрут для достижения этой точки
"""

import math
from typing import Optional

import bridge.router.waypoint as wp
from bridge import const
from bridge.auxiliary import aux, entity, fld, rbt
from bridge.router import route


class Router:
    """
    Маршрутизатор
    """

    def __init__(self, field: fld.Field) -> None:
        """
        Конструктор
        """
        self.routes = [route.Route(field.allies[i]) for i in range(const.TEAM_ROBOTS_MAX_COUNT)]
        self.__avoid_ball = False
        self.__avoid_enemy_half = False
        self.__we_active = False
        self.__dont_touch = False

    def put_state(self, avoid_ball: bool, dont_touch: bool, avoid_enemy_half: bool, we_active: bool) -> None:
        """
        Put states from referee to router
        """
        self.__avoid_ball = avoid_ball
        self.__dont_touch = dont_touch
        self.__avoid_enemy_half = avoid_enemy_half
        self.__we_active = we_active

    def update(self, field: fld.Field) -> None:
        """
        Обновить маршруты актуальным состоянием поля
        """
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            self.routes[i].update(field.allies[i])

    def set_dest(self, idx: int, target: wp.Waypoint, field: fld.Field) -> None:
        """
        Установить единственную путевую точку для робота с индексом idx
        """
        # print("aa", target)
        flag = False
        if target.type != wp.WType.S_IGNOREOBSTACLES:
            angle0 = target.angle
            dest_pos = aux.Point(target.pos.x, target.pos.y)
            closest_id = fld.find_nearest_robot(field.ball.get_pos(), field.allies).r_id
            if not (self.__we_active and closest_id == idx) and self.__avoid_enemy_half:
                if dest_pos.x * field.polarity < const.ROBOT_R + const.DELTA_DIST:
                    dest_pos.x = (const.ROBOT_R + const.DELTA_DIST) * field.polarity
                    self.routes[idx].set_dest_wp(wp.Waypoint(dest_pos, angle0, wp.WType.S_ENDPOINT))
                    flag = True

            if self.__dont_touch and closest_id == idx:
                delta = -aux.rotate(aux.Point(const.BALL_R + const.ROBOT_R, 0) * const.K_BALL_DIST, target.angle)
                dest_pos = field.ball.get_pos() + delta
                self.routes[idx].set_dest_wp(wp.Waypoint(dest_pos, angle0, wp.WType.S_ENDPOINT))
                flag = True

            if idx != field.gk_id and target.type != wp.WType.R_IGNORE_GOAl_HULL:
                for goal in [field.ally_goal, field.enemy_goal]:
                    if aux.is_point_inside_poly(dest_pos, goal.big_hull):
                        closest_out = aux.nearest_point_on_poly(dest_pos, goal.big_hull)
                        dest_pos = closest_out
                        self.routes[idx].set_dest_wp(wp.Waypoint(dest_pos, angle0, wp.WType.S_ENDPOINT))
                        flag = True
                    inters = aux.segments_poly_intersect(dest_pos, field.allies[idx].get_pos(), goal.hull)
                    if len(inters) > 1:
                        on_way = []
                        for i in range(2, 5, 2):
                            if aux.get_line_intersection(field.allies[idx].get_pos(), dest_pos, goal.hull[i - 1], goal.hull[i]):
                                on_way.append(goal.hull[max(2, i - 1)])
                                # print(i)
                        if on_way:
                            now_on_way = aux.find_nearest_point(field.allies[idx].get_pos(), on_way)
                            dest_pos = aux.point_on_line(goal.center, now_on_way, aux.dist(goal.center, now_on_way) * 1.2)
                            self.routes[idx].set_dest_wp(wp.Waypoint(dest_pos, angle0, wp.WType.R_PASSTHROUGH))
                            flag = True

            if self.__avoid_ball or (self.__dont_touch and closest_id != idx):
                # print(idx, "defe")
                now_plus = 0
                if closest_id != idx:
                    now_plus = const.PRIORITY_DIST
                now_plus += const.DELTA_DIST
                if aux.dist(dest_pos, field.ball.get_pos()) < const.KEEP_BALL_DIST + now_plus:
                    delta = -aux.rotate(aux.RIGHT, target.angle)
                    closest_out = aux.point_on_line(field.ball.get_pos(), dest_pos + delta, const.KEEP_BALL_DIST + now_plus)
                    dest_pos = closest_out
                    self.routes[idx].set_dest_wp(wp.Waypoint(dest_pos, angle0, wp.WType.S_ENDPOINT))
                    flag = True
        if not flag:
            self.routes[idx].set_dest_wp(target)

    def reroute(self, field: fld.Field) -> None:
        """
        Рассчитать маршруты по актуальным путевым точкам
        """
        # print(self.__avoid_ball)
        closest_id = fld.find_nearest_robot(field.ball.get_pos(), field.allies).r_id
        for idx in range(const.TEAM_ROBOTS_MAX_COUNT):
            if self.routes[idx] != wp.WType.S_IGNOREOBSTACLES:
                self_pos = field.allies[idx].get_pos()

                if not self.routes[idx].is_used():
                    continue

                if self.routes[idx].get_next_type() == wp.WType.S_VELOCITY:
                    continue

                if (
                    self.routes[idx].get_next_type() == wp.WType.S_BALL_KICK
                    or self.routes[idx].get_next_type() == wp.WType.S_BALL_KICK_UP
                    or self.routes[idx].get_next_type() == wp.WType.S_BALL_PASS
                ):
                    # if not field.allies[idx].is_kick_aligned(self.routes[idx].get_dest_wp()):
                    align_wp = self.calc_kick_wp(idx)
                    self.routes[idx].insert_wp(align_wp)
                elif self.routes[idx].get_next_type() == wp.WType.S_BALL_GRAB:
                    align_wp = self.calc_grab_wp(idx)
                    self.routes[idx].insert_wp(align_wp)

                if idx == field.gk_id or self.routes[idx].get_dest_wp().type == wp.WType.R_IGNORE_GOAl_HULL:
                    pth_wp = self.calc_passthrough_wp(field, idx)
                    # pth_wp = None
                    if pth_wp is not None:
                        self.routes[idx].insert_wp(pth_wp)
                    continue

                if not (self.__we_active and closest_id == idx) and self.__avoid_enemy_half:
                    self_pos = field.allies[idx].get_pos()
                    if self_pos.x * field.polarity < const.ROBOT_R:
                        # if idx == 2:
                        #     print("yyy")
                        closest_out = aux.Point(self_pos.x, self_pos.y)
                        closest_out.x = (const.ROBOT_R + const.DELTA_DIST) * field.polarity
                        angle0 = self.routes[idx].get_dest_wp().angle
                        self.routes[idx].insert_wp(wp.Waypoint(closest_out, angle0, wp.WType.R_PASSTHROUGH))
                        # print(self_pos)
                        continue

                for goal in [field.ally_goal, field.enemy_goal]:
                    if aux.is_point_inside_poly(self_pos, goal.hull):
                        closest_out = aux.nearest_point_on_poly(self_pos, goal.big_hull)
                        angle0 = self.routes[idx].get_dest_wp().angle
                        self.routes[idx].insert_wp(wp.Waypoint(closest_out, angle0, wp.WType.R_PASSTHROUGH))
                        continue

                if self.__avoid_ball or (self.__dont_touch and closest_id != idx):
                    # dest_pos = self.routes[idx].get_dest_wp().pos
                    now_plus = 0
                    if closest_id != idx:
                        now_plus = const.PRIORITY_DIST
                    self_pos = field.allies[idx].get_pos()
                    angle0 = self.routes[idx].get_dest_wp().angle
                    if aux.is_point_inside_circle(self_pos, field.ball.get_pos(), const.KEEP_BALL_DIST + now_plus):
                        closest_out = aux.nearest_point_on_circle(
                            self_pos, field.ball.get_pos(), const.KEEP_BALL_DIST + now_plus + const.DELTA_DIST
                        )
                        self.routes[idx].insert_wp(wp.Waypoint(closest_out, angle0, wp.WType.R_PASSTHROUGH))
                        # print("out", closest_out)
                        continue
                # if self.__dont_touch and self.routes[idx].get_dest_wp().type == wp.WType.S_BALL_KICK:
                #     angle0 = self.routes[idx].get_dest_wp().angle
                #     delta = -aux.rotate(aux.Point(const.BALL_R + const.ROBOT_R, 0), angle0) * 2
                #     self.routes[idx].insert_wp(wp.Waypoint(field.ball.get_pos() + delta, angle0, wp.WType.S_ENDPOINT))
                #     continue
                pth_wp = self.calc_passthrough_wp(field, idx)
                if pth_wp is not None:
                    is_inside = False
                    for goal in [field.ally_goal, field.enemy_goal]:
                        if aux.is_point_inside_poly(pth_wp.pos, goal.big_hull):
                            is_inside = True
                            break
                    if not is_inside:
                        self.routes[idx].insert_wp(pth_wp)

    def calc_passthrough_wp(self, field: fld.Field, idx: int) -> Optional[wp.Waypoint]:
        """
        просчитать путь до конечной точки, учитывая объекты на поле
        """
        p_to_go = self.routes[idx].get_next_wp().pos
        target_angle = self.routes[idx].get_next_wp().angle
        start_p = PointTree(field.allies[idx].get_pos().x, field.allies[idx].get_pos().y)
        end_p = PointTree(p_to_go.x, p_to_go.y)
        angle_to_go = self.routes[idx].get_next_wp().angle
        # if 0 <= idx <= 2:
        #     field.image.draw_dot(end_p.point(), (0, int(255 * (idx + 1) / 3), int(255 * (idx + 1) / 3)), 150)
        all_robots: list[entity.Entity | rbt.Robot] = []
        queue: list[PointTree] = [start_p]
        n_max = 100
        n_steps = 0
        finish = False
        point_go_now = None
        now_plus = 0
        wp_ball_r = float(const.BALL_R)
        closest_id = fld.find_nearest_robot(field.ball.get_pos(), field.allies).r_id
        flag_way = True
        if (closest_id != idx and self.__dont_touch) or self.__avoid_ball:
            wp_ball_r = const.KEEP_BALL_DIST - const.ROBOT_R
            if fld.find_nearest_robot(field.ball.get_pos(), field.allies).r_id != idx:
                now_plus = const.PRIORITY_DIST
        else:
            angle_p = aux.rotate(aux.RIGHT, target_angle)
            if (
                abs(aux.get_angle_between_points(start_p.point() + angle_p, start_p.point(), field.ball.get_pos()))
                > math.pi / 6
            ):
                if aux.dist(start_p.point(), field.ball.get_pos()) < (const.BALL_R + const.ROBOT_R) * const.K_BALL_DIST:
                    help_p = aux.point_on_line(
                        field.ball.get_pos(), start_p.point(), (const.BALL_R + const.ROBOT_R) * (const.K_BALL_DIST + 0.5)
                    )
                    point_go_now = PointTree(help_p.x, help_p.y)
                    flag_way = False
                    angle_to_go = field.allies[idx].get_angle()
                    # field.image.draw_dot(point_go_now.point(), (255, 100, 0), 100)
                    # print("aaa")
                wp_ball_r = (const.BALL_R + const.ROBOT_R) * const.K_BALL_DIST - const.ROBOT_R
        wp_ball = rbt.Robot(field.ball.get_pos(), 0, wp_ball_r + now_plus, const.COLOR, 0, 0)
        for ally in field.allies:
            if ally.is_used() and ally.r_id != idx:
                all_robots.append(ally)
        for enemy in field.enemies:
            if enemy.is_used():
                all_robots.append(enemy)
        all_robots.append(wp_ball)
        # field.image.draw_dot(wp_ball.get_pos(), (50, 50, 100), wp_ball.get_radius())
        while n_steps < n_max and n_steps < len(queue) and flag_way:
            on_way: list[int] = []
            for i, robot in enumerate(all_robots):
                if aux.dist2line(queue[n_steps].point(), end_p.point(), robot.get_pos()) < (
                    robot.get_radius() + const.ROBOT_R
                ) and aux.is_on_line(queue[n_steps].point(), end_p.point(), all_robots[i].get_pos()):
                    on_way.append(i)
            if len(on_way) == 0:
                flag = 1
                if end_p.father:
                    old_way_l = 0.0
                    point_mas = end_p
                    while point_mas.father is not None:
                        old_way_l += aux.dist(point_mas.point(), point_mas.father.point())
                        point_mas = point_mas.father
                    new_way_l = aux.dist(end_p.point(), queue[n_steps].point())
                    point_mas = queue[n_steps]
                    while point_mas.father is not None:
                        new_way_l += aux.dist(point_mas.point(), point_mas.father.point())
                        point_mas = point_mas.father
                    if old_way_l < new_way_l:
                        flag = 0
                if flag:
                    end_p.father = queue[n_steps]
                    point_mas = end_p
                    finish = True
                    if point_mas.father is not None:
                        while point_mas.father.father is not None:
                            finish = False
                            point_mas = point_mas.father
                    point_go_now = point_mas
            else:
                min_num_on_way = -1
                min_dist_on_way = float("inf")
                for i, on_w in enumerate(on_way):
                    if aux.dist(all_robots[on_w].get_pos(), queue[n_steps].point()) < min_dist_on_way:
                        min_dist_on_way = aux.dist(all_robots[on_w].get_pos(), queue[n_steps].point())
                        min_num_on_way = i
                all_new = all_robots.copy()
                mas2 = [all_new[on_way[min_num_on_way]]]
                i2 = 0
                while i2 < len(mas2):
                    another = 0
                    while another < len(all_new):
                        if (
                            aux.dist(mas2[i2].get_pos(), all_new[another].get_pos())
                            < mas2[i2].get_radius() + all_new[another].get_radius() + const.ROBOT_R * 2
                            and another not in mas2
                        ):
                            mas2.append(all_new[another])
                            all_new.pop(another)
                            another -= 1
                        another += 1
                    i2 += 1
                all_raw_angles: list[float] = []
                all_angles: list[float] = []
                all_dists: list[float] = []
                gamma = aux.angle_to_point(queue[n_steps].point(), end_p.point())
                for wall in mas2:
                    tangents = aux.get_tangent_points(
                        wall.get_pos(), queue[n_steps].point(), wall.get_radius() + const.ROBOT_R
                    )
                    if len(tangents) > 1:
                        for tang in tangents:
                            new_tang = aux.point_on_line(wall.get_pos(), tang, (wall.get_radius() + const.ROBOT_R) + 50)
                            raw_angle = aux.get_angle_between_points(end_p.point(), queue[n_steps].point(), tang)
                            if abs(raw_angle) < math.pi * 3 / 4:
                                all_raw_angles.append(raw_angle)
                                all_angles.append(
                                    aux.get_angle_between_points(end_p.point(), queue[n_steps].point(), new_tang)
                                )
                                all_dists.append(aux.dist(new_tang, queue[n_steps].point()))
                idxs_max = aux.get_minmax_idxs(all_raw_angles, "max")
                idxs_min = aux.get_minmax_idxs(all_raw_angles, "min")
                if idxs_min:
                    queue.append(
                        PointTree(
                            queue[n_steps].x + math.cos(all_angles[idxs_min[0]] + gamma) * all_dists[idxs_min[0]],
                            queue[n_steps].y + math.sin(all_angles[idxs_min[0]] + gamma) * all_dists[idxs_min[0]],
                            queue[n_steps],
                        )
                    )
                    # field.image.draw_dot(queue[-1].point(), (255, 0, 255), 10)
                if idxs_max:
                    queue.append(
                        PointTree(
                            queue[n_steps].x + math.cos(all_angles[idxs_max[0]] + gamma) * all_dists[idxs_max[0]],
                            queue[n_steps].y + math.sin(all_angles[idxs_max[0]] + gamma) * all_dists[idxs_max[0]],
                            queue[n_steps],
                        )
                    )
                    # field.image.draw_dot(queue[-1].point(), (255, 0, 255), 10)
            n_steps += 1
        if 2 >= idx >= 0:
            point_mas = end_p
            while point_mas.father is not None:
                #field.image.draw_dot(point_mas.point(), (int(255 * (idx + 1) / 3), 0, 0), 50)
                point_mas = point_mas.father
            #field.image.draw_dot(end_p.point(), (0, int(255 * (idx + 1) / 3), int(255 * (idx + 1) / 3)), 100)
            #field.image.draw_dot(start_p.point(), size_in_mms=50)
        if finish or point_go_now is None:
            return None
        # if point_go_now and idx == 2:
        # field.image.draw_dot(point_go_now.point(), (0, 0, 0), 120)
        return wp.Waypoint(point_go_now.point(), angle_to_go, wp.WType.R_PASSTHROUGH)

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
        # align_type = wp.WType.S_ENDPOINT
        align_wp = wp.Waypoint(align_pos, align_angle, align_type)
        return align_wp

    def calc_grab_wp(self, idx: int) -> wp.Waypoint:
        """
        Рассчитать точку для захвата мяча
        """
        target_point = self.routes[idx].get_dest_wp()
        target_pos = target_point.pos
        target_angle = target_point.angle

        align_pos = target_pos - aux.rotate(aux.RIGHT, target_angle) * const.GRAB_ALIGN_DIST
        align_angle = target_angle
        align_type = wp.WType.R_BALL_ALIGN
        # align_type = wp.WType.S_ENDPOINT
        align_wp = wp.Waypoint(align_pos, align_angle, align_type)
        return align_wp

    def get_route(self, idx: int) -> route.Route:
        """
        Получить маршрут для робота с индексом idx
        """
        return self.routes[idx]


class PointTree:
    """
    класс точки в дереве.
    """

    def __init__(self, x: float, y: float, father: Optional["PointTree"] = None) -> None:
        self.x = x
        self.y = y
        self.father = father

    def point(self) -> aux.Point:
        """
        возвращает значение в формате дефолтной точки.
        """
        return aux.Point(self.x, self.y)
