"""Верхнеуровневый код стратегии"""
# pylint: disable=redefined-outer-name

# @package Strategy
# Расчет требуемых положений роботов исходя из ситуации на поле

import math

# !v DEBUG ONLY
from enum import Enum
from typing import Optional

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.drawing as draw
import bridge.processors.field as field
import bridge.processors.robot as rb
import bridge.processors.waypoint as wp


class States(Enum):
    """Класс с глобальными состояниями игры"""

    DEBUG = 0
    DEFENSE = 1
    ATTACK = 2


class GameStates(Enum):
    """Класс с командами от судей"""

    HALT = 0
    STOP = 1
    RUN = 2
    TIMEOUT = 3
    PREPARE_KICKOFF = 5
    KICKOFF = 6
    PREPARE_PENALTY = 7
    PENALTY = 8
    FREE_KICK = 9
    BALL_PLACEMENT = 11


class ActiveTeam(Enum):
    """Класс с командами"""

    ALL = 0
    YELLOW = 1
    BLUE = 2


class Strategy:
    def __init__(self, dbg_game_status: GameStates = GameStates.RUN, dbg_state: States = States.DEBUG) -> None:
        self.game_status = GameStates.RUN
        self.active_team = 0
        self.status = States.ATTACK
        self.n = 3
        self.ballRadius = 110
        self.robotRadius = 200
        self.goalUp = 500
        self.goalDown = -500

        # self.image = draw.Image()
        self.ball_start_point: Optional[aux.Point] = None

    def process(self, field: field.Field) -> list[wp.Waypoint]:
        """
        Рассчитать конечные точки для каждого робота
        """

        waypoints: list[wp.Waypoint] = [
                                           wp.Waypoint(aux.Point(const.GRAVEYARD_POS_X, 0), 0,
                                                       wp.WType.S_IGNOREOBSTACLES)
                                       ] * const.TEAM_ROBOTS_MAX_COUNT

        if self.game_status == GameStates.RUN:
            self.run(field, waypoints)
        """else:
            if self.game_status == GameStates.TIMEOUT:
                self.timeout(field, waypoints)
            elif self.game_status == GameStates.HALT:
                pass
                # self.halt(field, waypoints)
            elif self.game_status == GameStates.PREPARE_PENALTY:
                self.prepare_penalty(field, waypoints)
            elif self.game_status == GameStates.PENALTY:
                self.penalty(field, waypoints)
            elif self.game_status == GameStates.BALL_PLACEMENT:
                self.keep_distance(field, waypoints)
            elif self.game_status == GameStates.PREPARE_KICKOFF:
                self.prepare_kickoff(field, waypoints)
            elif self.game_status == GameStates.KICKOFF:
                self.kickoff(field, waypoints)
            elif self.game_status == GameStates.FREE_KICK:
                self.free_kick(field, waypoints)
            elif self.game_status == GameStates.STOP:
                self.keep_distance(field, waypoints)"""

        # print(self.game_status, self.state)
        return waypoints

    def run(self, field: field.Field, waypoints: list[wp.Waypoint]) -> None:
        # for robo in field.allies:
        #     waypoints[robo.r_id] = wp.Waypoint(
        #         field.ball.get_pos(), aux.angle_to_point(robo.get_pos(), field.ally_goal.center), wp.WType.S_BALL_KICK
        #     )

        # field.ball.get_pos() - координаты мяча
        # field.enemies[i].get_pos() - координаты робота соперника с id i
        # field.allies[i].get_pos() - координаты робота союзника с id i

        robot_with_ball = rb.find_nearest_robot(field.ball.get_pos(), field.allies)

        # waypoints[9]  = wp.Waypoint(field.ball.get_pos(), aux.angle_to_point(field.allies[9].get_pos(), field.allies[10].get_pos()), wp.WType.S_BALL_KICK)
        #self.goalk(field, waypoints, [const.GK], robot_with_ball)
        #waypoints[0] = wp.Waypoint(field.ball.get_pos(), self.choose_kick_point(field, 0), wp.WType.S_BALL_KICK)
        self.choose_kick_point(field, 0)
        # self.image.draw_robot(field.allies[const.GK].get_pos(), field.allies[const.GK].get_angle())

        # self.image.update_window()
        # self.image.draw_field()

    def choose_kick_point(self, field: field.Field, robot_inx: int) -> None:
        kicker = field.allies[robot_inx]
        my_pos = field.allies[robot_inx].get_pos()
        ball_pos = field.ball.get_pos()

        #positions = [field.ally_goal.goal_up]
        positions = []
        for robot in field.allies:
            if robot.r_id != field.allies[robot_inx].r_id:
                if aux.dist(robot.get_pos(), field.enemy_goal.center) < aux.dist(field.enemy_goal.center, my_pos):
                    positions.append(robot.get_pos())
        #positions.append(field.ally_goal.goal_down)

        positions = sorted(positions, key=lambda x: x.y)

        for s in positions:
            print(s.x, end=" ")
        print()
        segments = []

        TMP_CONST = -4500

        xR = TMP_CONST

        for p in positions:
            tangents = aux.get_tangent_points(p, my_pos, const.ROBOT_R)
            if tangents is None or len(tangents) != 2:
                print(p, my_pos, tangents)
                continue
            int1 = aux.get_line_intersection(
                my_pos,
                tangents[0],
                field.enemy_goal.goal_down,
                field.enemy_goal.goal_up,
                "RS",
            )
            int2 = aux.get_line_intersection(
                my_pos,
                tangents[1],
                field.enemy_goal.goal_down,
                field.enemy_goal.goal_up,
                "RS",
            )
            if int1 is None and int2 is None:
                continue
            if int1 is None:
                segments.append([-const.GOAL_DY / 2, int2])
            elif int2 is None:
                segments.append([int1, const.GOAL_DY / 2])
            else:
                segments.append([int1.y, int2.y])

        for s in segments:
            print(s)
        return 0

    def goalk(
            self, field: field.Field, waypoints: list[wp.Waypoint], gk_wall_idx_list: list[int],
            robot_with_ball: rb.Robot
    ) -> None:
        gk_pos = None
        if robot_with_ball is not None:
            predict = aux.get_line_intersection(
                robot_with_ball.get_pos(),
                robot_with_ball.get_pos() + aux.rotate(aux.RIGHT, robot_with_ball.get_angle()),
                field.ally_goal.goal_down,
                field.ally_goal.goal_up,
                "RS",
            )
            if predict is not None:
                p_ball = (field.ball.get_pos() - predict).unity()
                gk_pos = aux.lerp(
                    aux.point_on_line(field.ally_goal.center, field.ball.get_pos(), const.GK_FORW),
                    p_ball * const.GK_FORW
                    + aux.get_line_intersection(
                        robot_with_ball.get_pos(),
                        robot_with_ball.get_pos() + aux.rotate(aux.RIGHT, robot_with_ball.get_angle()),
                        field.ally_goal.goal_down,
                        field.ally_goal.goal_up,
                        "RS",
                    ),
                    0.5,
                )
                # print("PREDICTION: ", robot_with_ball.getAngle())

        # print(field.ball.vel.mag())
        if field.is_ball_moves_to_goal():
            if self.ball_start_point is None:
                self.ball_start_point = field.ball.get_pos()
            elif (self.ball_start_point - field.ball.get_pos()).mag() > const.GK_INTERCEPT_SPEED:
                tmpPos = aux.get_line_intersection(
                    self.ball_start_point, field.ball.get_pos(), field.ally_goal.down, field.ally_goal.up, "RS"
                )
                gk_pos = aux.closest_point_on_line(field.ball.get_pos(), tmpPos,
                                                   field.allies[gk_wall_idx_list[0]].get_pos())
        else:
            self.ball_start_point = None
        # gk_pos = aux.Point(-1000, -100)

        if gk_pos is None:
            gk_pos = aux.point_on_line(field.ally_goal.center, field.ball.get_pos(), const.GK_FORW)

            # self.image.draw_dot(gk_pos, 10, [255, 255, 255])

        gk_angle = math.pi / 2
        waypoints[gk_wall_idx_list[0]] = wp.Waypoint(gk_pos, gk_angle, wp.WType.S_IGNOREOBSTACLES)

        # self.image.draw_dot(field.ball.get_pos(), 5)

        # print(field.isBallInGoalSq(), field.ball.get_pos())
        if field.is_ball_stop_near_goal():
            waypoints[gk_wall_idx_list[0]] = wp.Waypoint(
                field.ball.get_pos(), field.ally_goal.eye_forw.arg(), wp.WType.S_BALL_KICK
            )

        # wallline = [field.ally_goal.forw + field.ally_goal.eye_forw * const.GOAL_WALLLINE_OFFSET]
        # wallline.append(wallline[0] + field.ally_goal.eye_up)

        walline = aux.point_on_line(field.ally_goal.center, field.ball.get_pos(), const.GOAL_WALLLINE_OFFSET)
        walldir = aux.rotate((field.ally_goal.center - field.ball.get_pos()).unity(), math.pi / 2)
        dirsign = -aux.sign(aux.vec_mult(field.ally_goal.center, field.ball.get_pos()))

        wall = []
        for i in range(len(gk_wall_idx_list) - 1):
            wall.append(walline - walldir * (i + 1) * dirsign * (1 + (i % 2) * -2) * const.GOAL_WALL_ROBOT_SEPARATION)
            waypoints[gk_wall_idx_list[i + 1]] = wp.Waypoint(wall[i], walldir.arg(), wp.WType.S_ENDPOINT)
