"""Верхнеуровневый код стратегии"""
# pylint: disable=redefined-outer-name

# @package Strategy
# Расчет требуемых положений роботов исходя из ситуации на поле

import math

# !v DEBUG ONLY
from enum import Enum
from typing import Optional
from time import time

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.drawing as draw
import bridge.processors.field as field
import bridge.processors.robot as robot
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

        self.n = 3
        self.angleMyRobot = 0#aux.Point(0, 0)
        self.choosedKick = False
        self.passInd = -1
        self.xR = 2250

        self.image = draw.Image()
        self.ball_start_point: Optional[aux.Point] = None

    def process(self, field: field.Field) -> list[wp.Waypoint]:
        """
        Рассчитать конечные точки для каждого робота
        """

        #const.GRAVEYARD_POS_X
        waypoints: list[wp.Waypoint] = [
            wp.Waypoint(aux.Point(0, 0), 0, wp.WType.S_STOP)
        ] * const.TEAM_ROBOTS_MAX_COUNT

        if self.game_status == GameStates.RUN:
            self.run(field, waypoints)
        else:
            if self.game_status == GameStates.TIMEOUT:
                self.timeout(field, waypoints)
            elif self.game_status == GameStates.HALT:
                self.halt(field, waypoints)
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
                self.keep_distance(field, waypoints)

        # print(self.game_status, self.state)
        return waypoints
    
    def change_game_state(self, new_state: GameStates, upd_active_team: int) -> None:
        """Изменение состояния игры и цвета команды"""
        self.game_status = new_state
        if upd_active_team == 0:
            self.active_team = ActiveTeam.ALL
        elif upd_active_team == 2:
            self.active_team = ActiveTeam.YELLOW
        elif upd_active_team == 1:
            self.active_team = ActiveTeam.BLUE

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

        if minDistAllies - minDistEnemy > 30:
            return 3 + iEnemy
        elif minDistEnemy - minDistAllies > 30:
            return iAllies
        return None
    
    def trueBallCoordinate(self, p):
        return not(p.x == -10000 and p.y == 0)

    def kickToGoal(self, field: field.Field, robotInx):
        myPos = field.allies[robotInx].get_pos()
        ballPos = field.ball.get_pos()#getPos() 

        poses = []
        poses.append(aux.Point(self.xR, self.goalUp))
        for i in range(self.n):
            poses.append(field.enemies[i].get_pos())
            if i != robotInx: 
                poses.append(field.allies[i].get_pos())
        poses.append(aux.Point(self.xR, self.goalDown))
        
        central = []
        for i in range(len(poses)):
            #print("COORD:", poses[i].x, poses[i].y)
            dist = aux.dist(ballPos, poses[i])
            if dist == 0:
                continue
            #print()
            if poses[i].x != ballPos.x: D = abs(dist * (self.xR - ballPos.x) / (poses[i].x - ballPos.x))
            else: D = dist
            
            if (5 + self.robotRadius) / dist > 1: alphaNew = math.asin(1)
            else: alphaNew = math.asin((5 + self.robotRadius) / dist)
            
            gamma = math.acos(abs(poses[i].x - ballPos.x) / dist) - alphaNew
            downDist = math.sqrt(D**2 - (self.xR - ballPos.x)**2) - abs(self.xR - ballPos.x) * math.tan(gamma) #HASHUV
            
            if abs(D * math.sin(alphaNew) / downDist) <= 1:
                bettaNew = math.asin(D * math.sin(alphaNew) / downDist) 
            elif math.sin(alphaNew) > 1: bettaNew = math.asin(1)
            else: bettaNew = math.asin(-1)
            
            #upDist = D * math.sin(alphaNew) / math.sin(math.pi - 2 * alphaNew - bettaNew) #HASHUV
            upDist = abs(self.xR - ballPos.x) * math.tan(gamma + 2 * alphaNew) - abs(self.xR - ballPos.x) * math.tan(gamma) - downDist

            if ballPos.y > poses[i].y: 
                #(downDist, upDist) = (upDist, downDist)
                ycc = ballPos.y - D * math.sin(alphaNew + gamma)
            else:
                (downDist, upDist) = (upDist, downDist) 
                ycc = ballPos.y + D * math.sin(alphaNew + gamma)
        
            if upDist < downDist: (downDist, upDist) = (upDist, downDist)

            #if ycc + upDist > self.goalDown and ycc - downDist  < self.goalUp:
            if ycc > self.goalDown - self.robotRadius and ycc < self.goalUp + self.robotRadius:
                central.append([ycc, ycc + upDist, ycc - downDist])
        
        central = sorted(central, key = lambda x: x[0])
        #for i in range(len(central)):
        #   print(central[i][0], central[i][1], central[i][2])

        maxiAngle = -4 * math.pi
        rememberI = -2
        for i in range(len(central) - 1):
            lookUp = central[i + 1][2]
            lookDown = central[i][1]

            #print(lookUp, lookDown)
            if lookUp < lookDown: 
                continue

            if lookDown < lookUp:
                bokDown = math.sqrt((ballPos.x - self.xR)**2 + (ballPos.y - lookDown)**2)
                bokUp = math.sqrt((ballPos.x - self.xR)**2 + (ballPos.y - lookUp)**2)
                v1 = aux.Point(self.xR - ballPos.x, lookDown - ballPos.y)
                v2 = aux.Point(self.xR - ballPos.x, lookUp - ballPos.y)
                
                if (v1.x * v2.x + v1.y * v2.y) / (bokDown * bokUp) > 1: 
                    angleBetweenVectors = math.acos(1)
                elif (v1.x * v2.x + v1.y * v2.y) / (bokDown * bokUp) < -1: 
                    angleBetweenVectors = math.acos(-1)
                else: 
                    angleBetweenVectors = math.acos((v1.x * v2.x + v1.y * v2.y) / (bokDown * bokUp))

                if angleBetweenVectors > maxiAngle:
                    maxiAngle = angleBetweenVectors
                    rememberI = i
        
        canKickToGoal = False
        if rememberI != -2:
            lookUp = central[rememberI + 1][2]
            lookDown = central[rememberI][1]

            bokDown = math.sqrt((ballPos.x - self.xR)**2 + (ballPos.y - lookDown)**2)
            bokUp = math.sqrt((ballPos.x - self.xR)**2 + (ballPos.y - lookUp)**2)
            osn = lookUp - lookDown
            distUp = osn * bokUp / (bokUp + bokDown)
            
            self.yR = lookUp - distUp
            canKickToGoal = True
        else: self.yR = 0
        
        if canKickToGoal: return math.atan2(self.yR - myPos.y, self.xR - myPos.x)
        else: return None

    def run(self, field: field.Field, waypoints: list[wp.Waypoint]) -> None:
        if (field.ball.get_pos().x == 0 and field.ball.get_pos().y == 0) \
            or self.trueBallCoordinate(field.ball.get_pos()): 
            self.angleMyRobot = self.kickToGoal(field, 1) 

        if self.angleMyRobot == None: 
            if self.xR > 0: 
                self.angleMyRobot = math.pi/2 + math.atan2(field.allies[1].get_pos().y, 
                           field.allies[1].get_pos().x - self.xR)
            else: self.angleMyRobot = math.pi + math.atan2(field.allies[1].get_pos().y, 
                           field.allies[1].get_pos().x - self.xR)
        #pass
        #waypoints[1] = wp.Waypoint(aux.Point(0, 0), 0, wp.WType.S_BALL_KICK)
        #print(field.ball.get_pos().x, field.ball.get_pos().y)
        #angle = math.pi + math.atan2(field.allies[1].get_pos().y - field.ball.get_pos().y, 
        #                   field.allies[1].get_pos().x - field.ball.get_pos().x)
        robot_with_ball = robot.find_nearest_robot(field.ball.get_pos(), field.enemies)
        waypoints[1] = wp.Waypoint(field.ball.get_pos(), self.angleMyRobot, wp.WType.S_BALL_KICK)# - задать точку для езды. Куда, с каким углом, тип.
        self.goalk(field, waypoints, [0, 2], robot_with_ball)

  
    def choose_kick_point(self, field: field.Field, robot_inx: int) -> Optional[aux.Point]:
        ball_pos = field.ball.get_pos()

        positions = []
        for robot in field.allies:
            if robot.r_id != field.allies[robot_inx].r_id:
                if aux.dist(robot.get_pos(), field.enemy_goal.center) < aux.dist(field.enemy_goal.center, ball_pos):
                    positions.append(robot.get_pos())

        positions = sorted(positions, key=lambda x: x.y)

        segments = [field.enemy_goal.goal_up]
        for p in positions:
            tangents = aux.get_tangent_points(p, ball_pos, const.ROBOT_R * 100)
            if tangents is None or len(tangents) != 2:
                print(p, ball_pos, tangents)
                continue
            
            int1 = aux.get_line_intersection(
                ball_pos,
                tangents[0],
                field.enemy_goal.goal_down,
                field.enemy_goal.goal_up,
                "RS",
            )
            int2 = aux.get_line_intersection(
                ball_pos,
                tangents[1],
                field.enemy_goal.goal_down,
                field.enemy_goal.goal_up,
                "RS",
            )
            if int1 is None and int2 is None:
                continue
            elif int1 is None:
                segments.append(field.enemy_goal.goal_up)
                segments.append(int2)
            elif int2 is None:
                segments.append(int1)
                segments.append(field.enemy_goal.goal_down)
            else:
                segments.append(int1)
                segments.append(int2)

        segments.append(field.enemy_goal.goal_down)
        max_ = 0.0
        maxId = -1
        for i in range(0, len(segments), 2):
            c = segments[i]
            a = segments[i + 1]
            b = ball_pos
            if c.y > a.y: continue #Shadow intersection
            ang = aux.get_angle_between_points(a, b, c)
            print(ang, c.y, a.y)
            if (ang > max_):
                max_ = ang
                maxId = i

        if maxId == -1:
            return None

        A = segments[maxId + 1]
        B = ball_pos
        C = segments[maxId]
        tmp1 = (C - B).mag()
        tmp2 = (A - B).mag()
        CA = (A - C)
        pnt = C + CA * 0.5 * (tmp1 / tmp2)
        self.image.draw_dot(pnt, 10, [255, 0, 0])
        return pnt
    
    def goalk(
        self, field: field.Field, waypoints: list[wp.Waypoint], gk_wall_idx_list: list[int], robot_with_ball
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

        if gk_pos is None:
            gk_pos = aux.point_on_line(field.ally_goal.center - field.ally_goal.eye_forw * 1000, field.ball.get_pos(), const.GK_FORW + 1000)
            gk_pos.x = min(field.ally_goal.center.x + field.ally_goal.eye_forw.x * 300, gk_pos.x, key=lambda x: abs(x))
            if abs(gk_pos.y) > abs(field.ally_goal.goal_up.y):
                gk_pos.y = abs(field.ally_goal.goal_up.y) * abs(gk_pos.y) / gk_pos.y
            self.image.draw_dot(gk_pos, 10, [255, 255, 255])
        else:
            self.image.draw_dot(gk_pos, 10, [0, 0, 0])

        gk_angle = math.pi / 2
        waypoints[gk_wall_idx_list[0]] = wp.Waypoint(gk_pos, gk_angle, wp.WType.S_IGNOREOBSTACLES)

        self.image.draw_dot(field.ball.get_pos(), 5)

        if field.is_ball_stop_near_goal():
            waypoints[gk_wall_idx_list[0]] = wp.Waypoint(
                field.ball.get_pos(), field.ally_goal.eye_forw.arg(), wp.WType.S_BALL_KICK_UP
            )

        wallline = [field.ally_goal.forw + field.ally_goal.eye_forw * const.GOAL_WALLLINE_OFFSET]
        wallline.append(wallline[0] + field.ally_goal.eye_up)

        walline = aux.point_on_line(field.ally_goal.center, field.ball.get_pos(), const.GOAL_WALLLINE_OFFSET)
        walldir = aux.rotate((field.ally_goal.center - field.ball.get_pos()).unity(), math.pi / 2)
        dirsign = -aux.sign(aux.vec_mult(field.ally_goal.center, field.ball.get_pos()))

        wall = []
        for i in range(len(gk_wall_idx_list) - 1):
            wall.append(walline - walldir * (i + 1) * dirsign * (1 + (i % 2) * -2) * const.GOAL_WALL_ROBOT_SEPARATION)
            waypoints[gk_wall_idx_list[i + 1]] = wp.Waypoint(wall[i], walldir.arg(), wp.WType.S_IGNOREOBSTACLES)

