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

        self.image = draw.Image()
        self.ball_start_point: Optional[aux.Point] = None

    def process(self, field: field.Field) -> list[wp.Waypoint]:
        """
        Рассчитать конечные точки для каждого робота
        """

        waypoints: list[wp.Waypoint] = [
            wp.Waypoint(aux.Point(const.GRAVEYARD_POS_X, 0), 0, wp.WType.S_IGNOREOBSTACLES)
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

    def getIndexHolding(
        self, field: field.Field
    ) -> Optional[
        int
    ]:  # Возвращает индекс атакующего робота (для врагов индекс + 3), None -- в случае неопределенного статуса
        minDistEnemy = 4500.0
        iEnemy = -1
        for i in range(self.n):
            d = aux.dist(field.ball.get_pos(), field.enemies[i].get_pos())
            if d < minDistEnemy:
                minDistEnemy = d
                iEnemy = i

        minDistAllies = 4500.0
        iAllies = -1
        for i in range(self.n):
            d = aux.dist(field.ball.get_pos(), field.allies[i].get_pos())
            if d < minDistAllies:
                minDistAllies = d
                iAllies = i

        if minDistEnemy < minDistAllies and minDistAllies - minDistEnemy > 30:
            return 3 + iEnemy
        elif minDistAllies < minDistEnemy and minDistEnemy - minDistAllies > 30:
            return iAllies
        return None

    def run(self, field: field.Field, waypoints: list[wp.Waypoint]) -> None:
        print(field.ball.get_vel().mag() < 100)
        for robo in field.allies:
            waypoints[robo.r_id] = wp.Waypoint(
                field.ball.get_pos(), aux.angle_to_point(robo.get_pos(), field.ally_goal.center), wp.WType.S_BALL_KICK
            )

        # field.ball.get_pos() - координаты мяча
        # field.enemies[i].get_pos() - координаты робота соперника с id i
        # field.allies[i].get_pos() - координаты робота союзника с id i

        robot_with_ball = rb.find_nearest_robot(field.ball.get_pos(), field.allies)

        # waypoints[9]  = wp.Waypoint(field.ball.get_pos(), aux.angle_to_point(field.allies[9].get_pos(), field.allies[10].get_pos()), wp.WType.S_BALL_KICK)
        self.goalk(field, waypoints, [const.GK], robot_with_ball)

        self.image.draw_robot(field.allies[const.GK].get_pos(), field.allies[const.GK].get_angle())

        self.image.update_window()
        self.image.draw_field()

    def chooseKick(self, field: field.Field, robotInx: int) -> None:
        central = []

        myPos = field.allies[robotInx].get_pos()
        ballPos = field.ball.get_pos()

        for i in range(self.n):
            dist = aux.dist(myPos, field.enemies[i].get_pos())
            D = dist * (4500 - ballPos.x) / (field.enemies[i].get_pos().x - ballPos.x)
            if (self.robotRadius + self.ballRadius + 20) / dist > 1:
                alphaNew = math.asin(1)
            else:
                alphaNew = math.asin((self.robotRadius + self.ballRadius + 20) / dist)

            gamma = math.acos((field.enemies[i].get_pos().x - ballPos.x) / dist) - alphaNew

            downDist = math.sqrt(D**2 - (4500 - ballPos.x) ** 2) - (4500 - ballPos.x) * math.tan(gamma)  # HASHUV

            if abs(D * math.sin(alphaNew) / downDist) <= 1:
                bettaNew = math.asin(D * math.sin(alphaNew) / downDist)
            elif math.sin(alphaNew) > 0:
                bettaNew = math.asin(1)
            else:
                bettaNew = math.asin(-1)

            upDist = D * math.sin(alphaNew) / math.sin(math.pi - 2 * alphaNew - bettaNew)  # HASHUV

            if ballPos.y > field.enemies[i].get_pos().y:
                (downDist, upDist) = (upDist, downDist)
                ycc = ballPos.y - D * math.sin(alphaNew + gamma)
            else:
                ycc = ballPos.y + D * math.sin(alphaNew + gamma)

            if ycc < self.goalDown or ycc > self.goalUp:
                central.append([ycc, ycc + upDist, ycc - downDist])

        central = sorted(central, key=lambda x: x[0])

        maxiAngle = 0.0
        rememberI = -1
        for i in range(len(central) - 1):
            lookUp = central[i + 1][2]
            lookDown = central[i][1]

            bokDown = math.sqrt((myPos.x - 4500) ** 2 + (myPos.y - lookDown) ** 2)
            bokUp = math.sqrt((myPos.x - 4500) ** 2 + (myPos.y - lookUp) ** 2)
            v1 = (4500 - myPos.x, lookDown - myPos.y)
            v2 = (4500 - myPos.x, lookUp - myPos.y)

            if (v1[0] * v2[0] + v1[1] * v2[1]) / (bokDown * bokUp) > 1:
                angleBetweenVectors = math.acos(1)
            elif (v1[0] * v2[0] + v1[1] * v2[1]) / (bokDown * bokUp) < -1:
                angleBetweenVectors = math.acos(-1)
            else:
                angleBetweenVectors = math.acos((v1[0] * v2[0] + v1[1] * v2[1]) / (bokDown * bokUp))

            if angleBetweenVectors > maxiAngle:
                maxiAngle = angleBetweenVectors
                rememberI = i

        if rememberI != -1:
            lookUp = central[rememberI + 1][2]
            lookDown = central[rememberI][1]
            bokDown = math.sqrt((myPos.x - 4500) ** 2 + (myPos.y - lookDown) ** 2)
            bokUp = math.sqrt((myPos.x - 4500) ** 2 + (myPos.y - lookUp) ** 2)

            osn = lookUp - lookDown
            distUp = osn * bokUp / (bokUp + bokDown)

            self.xR = 4500
            self.yR = lookUp - distUp
        else:
            self.xR = 4500
            self.yR = 0
        # pass

    def goalk(
        self, field: field.Field, waypoints: list[wp.Waypoint], gk_wall_idx_list: list[int], robot_with_ball: rb.Robot
    ) -> None:
        gk_pos = None
        if robot_with_ball is not None:
            predict = aux.get_line_intersection(
                robot_with_ball.get_pos(),
                robot_with_ball.get_pos() + aux.rotate(aux.RIGHT, robot_with_ball.get_angle()),
                field.ally_goal.down,
                field.ally_goal.up,
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
                        field.ally_goal.down,
                        field.ally_goal.up,
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
                gk_pos = aux.closest_point_on_line(field.ball.get_pos(), tmpPos, field.allies[gk_wall_idx_list[0]].get_pos())
        else:
            self.ball_start_point = None
        #gk_pos = aux.Point(-1000, -100)

        if gk_pos is None:
            gk_pos = aux.point_on_line(field.ally_goal.center, field.ball.get_pos(), const.GK_FORW)

            self.image.draw_dot(gk_pos, 10, [255, 255, 255])
        else:
            self.image.draw_dot(gk_pos, 10, [0, 0, 0])

        gk_angle = math.pi / 2
        waypoints[gk_wall_idx_list[0]] = wp.Waypoint(gk_pos, gk_angle, wp.WType.S_IGNOREOBSTACLES)

        self.image.draw_dot(field.ball.get_pos(), 5)

        # print(field.isBallInGoalSq(), field.ball.get_pos())
        """if field.is_ball_stop_near_goal():
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
            waypoints[gk_wall_idx_list[i + 1]] = wp.Waypoint(wall[i], walldir.arg(), wp.WType.S_ENDPOINT)"""
