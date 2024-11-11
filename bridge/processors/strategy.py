"""Верхнеуровневый код стратегии"""
# pylint: disable=redefined-outer-name

# @package Strategy
# Расчет требуемых положений роботов исходя из ситуации на поле

import math

# !v DEBUG ONLY
from enum import Enum
from time import time
from typing import Optional

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.drawing as draw
import bridge.processors.field as fld
import bridge.processors.ref_states as refs
import bridge.processors.robot as robot
import bridge.processors.signal as signal
import bridge.processors.waypoint as wp
from bridge.processors.referee_state_processor import State as GameStates
from bridge.processors.referee_state_processor import Color as ActiveTeam


class Strategy:
    """Основной класс с кодом стратегии"""

    def __init__(self, dbg_game_status: GameStates = GameStates.RUN, dbg_state: States = States.ATTACK) -> None:
        self.refs = refs.RefStates()

        self.game_status = dbg_game_status
        self.active_team: ActiveTeam = ActiveTeam.ALL
        self.we_kick = False
        self.we_active = False
        self.state = dbg_state
        self.action = 0
        self.image = draw.Image()

    def change_game_state(self, new_state: GameStates, upd_active_team: ActiveTeam) -> None:
        """Изменение состояния игры и цвета команды"""
        self.game_status = new_state
        self.active_team = upd_active_team

    def process(self, field: fld.Field) -> list[wp.Waypoint]:
        """
        Рассчитать конечные точки для каждого робота
        """
        if (
            self.active_team == ActiveTeam.ALL
            or field.ally_color == self.active_team
        ):
            self.we_active = True
        else:
            self.we_active = False


        waypoints: list[wp.Waypoint] = []
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            waypoints.append(wp.Waypoint(field.allies[i].get_pos(), field.allies[i].get_angle(), wp.WType.S_STOP))

        if self.game_status == GameStates.RUN:
            self.run(field, waypoints)
        else:
            if self.game_status == GameStates.TIMEOUT:
                self.refs.timeout(field, waypoints)
            elif self.game_status == GameStates.HALT:
                pass
                # self.halt(field, waypoints)
            elif self.game_status == GameStates.PREPARE_PENALTY:
                self.refs.prepare_penalty(field, waypoints)
            elif self.game_status == GameStates.PENALTY:
                if self.refs.we_kick or 1:
                    self.refs.penalty_kick(field, waypoints)
                else:
                    robot_with_ball = fld.find_nearest_robot(field.ball.get_pos(), field.enemies)
                    waypoints[field.gk_id] = self.goalk(field, 0, robot_with_ball)[0]
            elif self.game_status == GameStates.PREPARE_KICKOFF:
                self.prepare_kickoff(field, waypoints)
            elif self.game_status == GameStates.KICKOFF:
                self.kickoff(field, waypoints)
            elif self.game_status == GameStates.FREE_KICK:
                self.run(field, waypoints)
            elif self.game_status == GameStates.STOP:
                self.run(field, waypoints)

        for rbt in field.allies:
            if rbt.is_used():
                self.image.draw_robot(rbt.get_pos(), rbt.get_angle())
        for rbt in field.enemies:
            if rbt.is_used():
                self.image.draw_robot(rbt.get_pos(), rbt.get_angle(), (255, 255, 0))
        self.image.draw_dot(field.ball.get_pos(), 5)
        self.image.draw_poly(field.ally_goal.hull)
        self.image.draw_poly(field.enemy_goal.hull)
        self.image.update_window()
        self.image.draw_field()

        return waypoints

    def run(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """
        Определение глобального состояния игры
        """
        self.defense(waypoints, field)
        #waypoints[0] = wp.Waypoint(aux.Point(500, 0), 0, wp.WType.S_ENDPOINT)
        # global_state = return_state()
        # if global_state == 1:
        #     if self.action == 0:
        #         global_state = 0
        #     else:
        #         global_state = 2
        # a = aux.get_tangent_points(aux.Point(1000, 0), aux.Point(0, 0), 1)
        # print(a[0].x, a[0].y, a[1].x, a[1].y)
        #print(aux.get_angle_between_points(aux.Point(1000, 100), aux.Point(0, 0), aux.Point(1000, 0)))
        


    def go_to_ball(self, idx: int, waypoints: list[wp.Waypoint], field: fld.Field) -> None:
        angle_point = aux.closest_point_on_line(field.enemy_goal.up, field.enemy_goal.down, field.ball.get_pos(), is_inf = "L")
        gates = [[aux.get_angle_between_points(angle_point, field.ball.get_pos(), field.enemy_goal.down), aux.get_angle_between_points(angle_point, field.ball.get_pos(), field.enemy_goal.up)]]
        print("gates =", gates[0][0], gates[0][1])
        gates.sort()
        minuses = []
        for enemy in field.enemies:
            if enemy.is_used():
                try:
                    tangents = aux.get_tangent_points(enemy.get_pos(), field.ball.get_pos(), enemy.get_radius() + field.ball.get_radius())
                    minus = [aux.get_angle_between_points(angle_point, field.ball.get_pos(), tangents[0]), aux.get_angle_between_points(angle_point, field.ball.get_pos(), tangents[1])]
                    minus.sort()
                    if abs(minus[0]) < math.pi / 2 or abs(minus[1]) < math.pi / 2:
                        minus = [min(abs(minus[0]), math.pi / 2) * aux.sign(minus[0]), min(abs(minus[1]), math.pi / 2) * aux.sign(minus[1])]
                        minuses.append(minus)
                except:
                    pass
        gates = aux.range_minus(gates, minuses)
        print("start")
        for i in minuses:
            print(i[0], i[1])
        print("start 1")
        for i in gates:
            print(i[0], i[1])
        max_angle = [1, -1]
        for ang in gates:
            if max_angle[1] - max_angle[0] < ang[1] - ang[0]:
                max_angle = ang
        if max_angle[1] - max_angle[0] < 0:
            cick_angle = aux.get_angle_between_points(angle_point, field.ball.get_pos(), field.enemy_goal.center)
        else:
            cick_angle = (max_angle[1] + max_angle[0]) / 2
        cick_angle = aux.wind_down_angle(cick_angle + aux.angle_to_point(aux.Point(0, 0), field.enemy_goal.center))
        print(cick_angle)
        # if len(minuses) > 0:
        waypoints[idx] = wp.Waypoint(field.ball.get_pos(), cick_angle, wp.WType.S_BALL_KICK_UP)

    def kick_ball_out(self, waypoints: list[wp.Waypoint], field: fld.Field):
        angle_point = aux.closest_point_on_line(field.enemy_goal.up, field.enemy_goal.down, field.ball.get_pos(), is_inf = "L")
        ally_angles = []
        enemy_angles = []
        for ally in field.allies:
            if ally.is_used() and ally.r_id != const.GK:
                try:
                    tangents = aux.get_tangent_points(ally.get_pos(), field.ball.get_pos(), ally.get_radius() + field.ball.get_radius())
                    ally_angle = [aux.get_angle_between_points(angle_point, field.ball.get_pos(), tangents[0]), aux.get_angle_between_points(angle_point, field.ball.get_pos(), tangents[1])]
                    ally_angle.sort()
                    if abs(ally_angle[0]) < math.pi / 2 or abs(ally_angle[1]) < math.pi / 2:
                        ally_angle = [min(abs(ally_angle[0]), math.pi / 2) * aux.sign(ally_angle[0]), min(abs(ally_angle[1]), math.pi / 2) * aux.sign(ally_angle[1])]
                        ally_angles.append(ally_angle)
                except:
                    pass
        for enemy in field.enemies:
            if enemy.is_used() and enemy.r_id != const.ENEMY_GK:
                try:
                    tangents = aux.get_tangent_points(enemy.get_pos(), field.ball.get_pos(), enemy.get_radius() + field.ball.get_radius())
                    enemy_angle = [aux.get_angle_between_points(angle_point, field.ball.get_pos(), tangents[0]), aux.get_angle_between_points(angle_point, field.ball.get_pos(), tangents[1])]
                    enemy_angle.sort()
                    if abs(enemy_angle[0]) < math.pi / 2 or abs(enemy_angle[1]) < math.pi / 2:
                        enemy_anlge = [min(abs(enemy_angle[0]), math.pi / 2) * aux.sign(enemy_angle[0]), min(abs(enemy_angle[1]), math.pi / 2) * aux.sign(enemy_angle[1])]
                        enemy_angles.append(enemy_angle)
                except:
                    pass
        ally_angles = aux.range_minus(ally_angles, enemy_angles, False)
        max_angle = [1, -1]
        for ang in ally_angles:
            if max_angle[1] - max_angle[0] < ang[1] - ang[0]:
                max_angle = ang
        if max_angle[1] - max_angle[0] < 0:
            cick_angle = aux.get_angle_between_points(angle_point, field.ball.get_pos(), field.enemy_goal.center)
        else:
            cick_angle = (max_angle[1] + max_angle[0]) / 2
        cick_angle = aux.wind_down_angle(cick_angle + aux.angle_to_point(aux.Point(0, 0), field.enemy_goal.center))
        waypoints[const.GK] = wp.Waypoint(field.ball.get_pos(), cick_angle, wp.WType.S_BALL_KICK_UP)

    def return_state(self, waypoints: list[wp.Waypoint], field: fld.Field):
        min_ally = float("inf")
        min_enemy = float("inf")
        for i in range(12):
            if field.allies[i].is_used():
                a = aux.dist(field.ball.get_pos(), field.allies[i].get_pos())
                if a < min_ally:
                    min_ally = a
            if field.enemies[i].is_used():
                a = aux.dist(field.ball.get_pos(), field.enemies[i].get_pos())
                if a < min_enemy:
                    min_enemy = a
        st = 0
        if min_enemy > field.enemies[0].get_radius() + field.ball.get_radius() * 2:
            state = 2
        elif min_ally < field.allies[0].get_radius() + field.ball.get_radius() * 2:
            state = 1
        return st

    def defense(self, waypoints: list[wp.Waypoint], field: fld.Field):
        enemy_with_ball = field.find_nearest_enemies(field.ball.get_pos(), 1)[0]
        ally_with_ball = field.find_nearest_allies(field.ball.get_pos(), 1)[0]
        now_enemies = []
        now_allies = []
        for enemy in field.enemies:
            if enemy.is_used() and enemy.r_id != const.ENEMY_GK and enemy.r_id != enemy_with_ball.r_id:
                now_enemies.append(enemy)
        for ally in field.allies:
            if ally.is_used() and ally.r_id != const.GK and ally.r_id != ally_with_ball.r_id:
                now_allies.append(ally)
        angle_point = aux.closest_point_on_line(field.ally_goal.up, field.ally_goal.down, field.ball.get_pos(), is_inf = "L")
        gates = [[aux.get_angle_between_points(angle_point, field.ball.get_pos(), field.ally_goal.down), aux.get_angle_between_points(angle_point, field.ball.get_pos(), field.ally_goal.up)]]
        gates.sort()
        def_angle = (gates[0][0] + gates[0][1]) / 2
        def_angle = aux.wind_down_angle(def_angle + aux.angle_to_point(aux.Point(0, 0), field.ally_goal.center))
        print(def_angle)
        help_point = aux.Point(field.ally_goal.center.x, field.ball.get_pos().y + (field.ally_goal.center.x - field.ball.get_pos().x) * math.tan(def_angle))
        pToGo = aux.average_point([aux.closest_point_on_line(field.ball.get_pos(), help_point, ally_with_ball.get_pos(), is_inf = "R"), aux.point_on_line(field.ball.get_pos(), help_point, field.ball.get_radius() + ally_with_ball.get_radius())])
        waypoints[ally_with_ball.r_id] = wp.Waypoint(pToGo, aux.wind_down_angle(def_angle - math.pi), wp.WType.S_ENDPOINT)
        now_enemies = field.find_nearest_robots(field.ball.get_pos(), now_enemies, len(now_enemies))
        for enemy in now_enemies:
            if len(now_allies) == 0:
                break
            now_ally = fld.find_nearest_robot(enemy.get_pos(), now_allies)
            for ally in now_allies:
                if ally.r_id != now_ally.r_id:
                    now_allies.append(ally)
            pToGo = aux.average_point([aux.closest_point_on_line(field.ball.get_pos(), enemy.get_pos(), now_ally.get_pos(), is_inf = "R"), aux.point_on_line(enemy.get_pos(), field.ball.get_pos(), enemy.get_radius() + now_ally.get_radius())])
            waypoints[now_ally.r_id] = wp.Waypoint(pToGo, aux.angle_to_point(field.ball.get_pos(), enemy.get_pos()), wp.WType.S_ENDPOINT)

