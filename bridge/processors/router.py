"""
Модуль маршрутизации роботов
Получает от стратега требуемые координаты для каждого робота и
считает оптимальный маршрут для достижения этой точки
"""

import math
import typing

import bridge.processors.auxiliary as aux
from bridge.processors import const, field, route
import bridge.processors.quickhull as qh
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
        self.__avoid_ball = False

    def avoid_ball(self, state: bool = True) -> None:
        """
        Stop game state
        """
        self.__avoid_ball = state

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
        if idx != fld.gk_id and target.type != wp.WType.R_IGNORE_GOAl_HULL:
            dest_pos = target.pos
            for goal in [fld.ally_goal, fld.enemy_goal]:
                if aux.is_point_inside_poly(dest_pos, goal.big_hull):
                    closest_out = aux.nearest_point_on_poly(dest_pos, goal.big_hull)
                    angle0 = target.angle
                    self.routes[idx].set_dest_wp(wp.Waypoint(closest_out, angle0, wp.WType.S_ENDPOINT))
                    return

        if self.__avoid_ball:
            dest_pos = target.pos
            if aux.is_point_inside_circle(dest_pos, fld.ball.get_pos(), const.KEEP_BALL_DIST):
                delta = -aux.rotate(aux.RIGHT, target.angle)
                closest_out = aux.nearest_point_on_circle(dest_pos + delta, fld.ball.get_pos(), const.KEEP_BALL_DIST + const.ROBOT_R)
                angle0 = target.angle
                self.routes[idx].set_dest_wp(wp.Waypoint(closest_out, angle0, wp.WType.S_ENDPOINT))
                return

        if abs(target.pos.x) > const.GOAL_DX:
            target.pos.x = const.GOAL_DX * aux.sign(target.pos.x)
        if abs(target.pos.y) > 1500:
            target.pos.y = 1500 * aux.sign(target.pos.y)
        self.routes[idx].set_dest_wp(target)

    def reroute(self, fld: field.Field) -> None:
        """
        Рассчитать маршруты по актуальным путевым точкам
        """
        for idx in range(const.TEAM_ROBOTS_MAX_COUNT):

            self_pos = fld.allies[idx].get_pos()

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

            if idx == fld.gk_id or self.routes[idx].get_dest_wp().type == wp.WType.R_IGNORE_GOAl_HULL:
                pth_wp = self.calc_passthrough_wp(fld, idx)
                if pth_wp is not None:
                    self.routes[idx].insert_wp(pth_wp)
                continue

            for goal in [fld.ally_goal, fld.enemy_goal]:
                if aux.is_point_inside_poly(self_pos, goal.big_hull):
                    closest_out = aux.find_nearest_point(self_pos, goal.big_hull, [goal.up, aux.GRAVEYARD_POS, goal.down])
                    angle0 = self.routes[idx].get_dest_wp().angle
                    self.routes[idx].set_dest_wp(
                        wp.Waypoint(goal.center + (closest_out - goal.center) * 1.2, angle0, wp.WType.S_ENDPOINT)
                    )
                    continue
                pint = aux.segment_poly_intersect(self_pos, self.routes[idx].get_next_wp().pos, goal.big_hull)
                if pint is not None:
                    angle0 = self.routes[idx].get_dest_wp().angle
                    if aux.is_point_inside_poly(self.routes[idx].get_dest_wp().pos, goal.big_hull):
                        self.routes[idx].set_dest_wp(wp.Waypoint(pint, angle0, wp.WType.S_ENDPOINT))
                        break
                    convex_hull = qh.shortesthull(self_pos, self_pos + self.routes[idx].get_next_vec(), goal.big_hull)
                    for j in range(len(convex_hull) - 2, 0, -1):
                        self.routes[idx].insert_wp(wp.Waypoint(convex_hull[j], angle0, wp.WType.R_PASSTHROUGH))

            if self.__avoid_ball:
                dest_pos = self.routes[idx].get_dest_wp().pos
                self_pos = fld.allies[idx].get_pos()
                angle0 = self.routes[idx].get_dest_wp().angle
                if aux.is_point_inside_circle(self_pos, fld.ball.get_pos(), const.KEEP_BALL_DIST):
                    closest_out = aux.nearest_point_on_circle(self_pos, fld.ball.get_pos(), const.KEEP_BALL_DIST)
                    self.routes[idx].insert_wp(wp.Waypoint(closest_out, angle0, wp.WType.R_PASSTHROUGH))
                    continue
                points = aux.line_circle_intersect(self_pos, dest_pos, fld.ball.get_pos(), const.KEEP_BALL_DIST) 
                if points is None:
                    continue
                if len(points) == 2:
                    points = aux.get_tangent_points(fld.ball.get_pos(), self_pos, const.KEEP_BALL_DIST)
                    p = aux.Point(0, 0)
                    if points is None:
                        continue
                    if len(points) == 2:
                        p = points[0] if aux.dist(points[0], dest_pos) < aux.dist(points[1], dest_pos) else points[1]
                    else:
                        p = points[0]
                    self.routes[idx].insert_wp(wp.Waypoint(p + (p - fld.ball.get_pos()).unity() * const.ROBOT_R, angle0, wp.WType.R_PASSTHROUGH))
                        
            pth_wp = self.calc_passthrough_wp(fld, idx)
            if pth_wp is not None:
                is_inside = False
                for goal in [fld.ally_goal, fld.enemy_goal]:
                    if aux.is_point_inside_poly(pth_wp.pos, goal.big_hull):
                        is_inside = True
                        break
                if not is_inside:
                    self.routes[idx].insert_wp(pth_wp)

    def calc_passthrough_wp(self, fld: field.Field, idx: int) -> typing.Optional[wp.Waypoint]:
        pToGo = self.routes[idx].get_next_wp().pos
        #print(pToGo.x, pToGo.y)
        startP = PointTree(fld.allies[idx].get_pos().x, fld.allies[idx].get_pos().y)
        endP = PointTree(pToGo.x, pToGo.y)
        allRobots = []
        queue = [startP]
        nMax = float('inf')
        nSteps = 0
        finish = None
        pointGoNow = None
        for ally in fld.allies:
            if ally.is_used() and ally.r_id != idx:
                allRobots.append(ally)
        for enemy in fld.enemies:
            if enemy.is_used():
                allRobots.append(enemy)
        allRobots.append(fld.ball)
        while nSteps < nMax and nSteps < len(queue):
            onWay = []
            for i in range(len(allRobots)):
                if aux.dist2line(queue[nSteps].point(), endP.point(), allRobots[i].get_pos()) < allRobots[i].get_radius() + allRobots[0].get_radius() and aux.is_on_line(queue[nSteps].point(), endP.point(), allRobots[i].get_pos()) and queue[nSteps].myRobot != i:
                    onWay.append(i)
            if(len(onWay) == 0):
                flag = 1
                if endP.father:
                    oldWayL = 0
                    pointMas = endP
                    #print("start")
                    while pointMas.father != None:
                        #print(pointMas.x, pointMas.y)
                        oldWayL += aux.dist(pointMas, pointMas.father)
                        pointMas = pointMas.father
                    newWayL = aux.dist(endP, queue[nSteps])
                    pointMas = queue[nSteps]
                    while pointMas.father != None:
                        newWayL += aux.dist(pointMas, pointMas.father)
                        pointMas = pointMas.father
                    if oldWayL < newWayL:
                        flag = 0
                if flag:
                    endP.father = queue[nSteps]
                    #print("start")
                    #print(endP.x, endP.y)
                    pointMas = endP
                    finish = True
                    while pointMas.father.father != None:
                        #print(pointMas.x, pointMas.y)
                        finish = False
                        pointMas = pointMas.father
                        #print(pointMas.x, pointMas.y)
                    pointGoNow = pointMas
                    if nMax == float('inf'):
                        nMax = nSteps + 5
            else:
                minNumOnWay = -1
                minDistOnWay = float('inf')
                for i in range(len(onWay)):
                    if aux.dist(allRobots[onWay[i]].get_pos(), queue[nSteps]) < minDistOnWay:
                        minDistOnWay = aux.dist(allRobots[onWay[i]].get_pos(), queue[nSteps])
                        minNumOnWay = i
                allNew = allRobots.copy()
                mas2 = [allNew[onWay[minNumOnWay]]]
                i2 = 0
                while i2 < len(mas2):
                    another = 0
                    while another < len(allNew):
                        if aux.dist(mas2[i2].get_pos(), allNew[another].get_pos()) < mas2[i2].get_radius() + allNew[another].get_radius() + allRobots[0].get_radius() * 2 and another not in mas2:
                            mas2.append(allNew[another])
                            allNew.pop(another)
                            another -= 1
                        another += 1
                    i2 += 1
                angs = [0, 0]
                dists = [0, 0]
                flagNone = False
                gamma = aux.angle_to_point(queue[nSteps].point(), endP.point())
                for wall in mas2:
                    tangents = aux.get_tangent_points(wall.get_pos(), queue[nSteps], wall.get_radius() + allRobots[0].get_radius())
                    if not tangents:
                        flagNone = True
                    else:
                        for tang in tangents:
                            tang = aux.point_on_line(wall.get_pos(), tang, (wall.get_radius() + allRobots[0].get_radius()) * 1.1)
                            tetta = aux.get_angle_between_points(endP.point(), queue[nSteps].point(), tang)
                            #print(tetta)
                            c = aux.dist(tang, queue[nSteps])
                            if abs(angs[bool(tetta < 0)]) < abs(tetta):
                                angs[bool(tetta < 0)] = tetta
                                dists[bool(tetta < 0)] = c
                #print(angs[0], angs[1], gamma)
                if flagNone:
                    pointGoNow = endP
                    break
                else:
                    queue.append(PointTree(queue[nSteps].x + math.cos(angs[1] + gamma) * dists[1], queue[nSteps].y + math.sin(angs[1] + gamma) * dists[1], queue[nSteps], onWay[minNumOnWay]))
                    queue.append(PointTree(queue[nSteps].x + math.cos(angs[0] + gamma) * dists[0], queue[nSteps].y + math.sin(angs[0] + gamma) * dists[0], queue[nSteps], onWay[minNumOnWay]))
                #print("start")
                #for i in queue:
                #    print(i.x, i.y)
            nSteps += 1
        # if idx == 0:
        #     print(pointGoNow.x, pointGoNow.y)
        return wp.Waypoint(pointGoNow.point(), self.routes[idx].get_next_wp().angle, wp.WType.R_PASSTHROUGH)

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
    def __init__(self, x: float, y: float, father: Optional[int] = None, myRobot: Optional[int] = None) -> None:
        self.x = x
        self.y = y
        self.father = father
        self.myRobot = myRobot

    def point(self):
        return aux.Point(self.x, self.y)
