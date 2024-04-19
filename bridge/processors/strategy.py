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

class Team(Enum):
    ENEMY_TEAM = 0
    MY_TEAM = 1
    UNCERTAIN_TEAM = 2

class Strategy:
    def __init__(self, dbg_game_status: GameStates = GameStates.RUN, dbg_state: States = States.DEBUG) -> None:
        self.game_status = GameStates.RUN
        self.active_team = 0
        self.status = States.ATTACK
        self.atackerInx = 1
        self.defenderInx = 0

        self.angleMyRobot = 0#aux.Point(0, 0)
        self.choosedKick = False
        self.passInd = -1

        self.image = draw.Image()
        self.ball_start_point: Optional[aux.Point] = None
        self.haveAngle = False
        self.angleDefender = 0

    def change_game_state(self, newState, activeTeam):
        self.game_status = newState
        if activeTeam == 0:
            self.active_team = ActiveTeam.ALL
        elif activeTeam == 2:
            self.active_team = ActiveTeam.YELLOW
        elif activeTeam == 1:
            self.active_team = ActiveTeam.BLUE

    def process(self, field: field.Field) -> list[wp.Waypoint]:
        """
        Рассчитать конечные точки для каждого робота
        """
        field.enemies[2] = rb.Robot(aux.Point(0, 0), 0, const.ROBOT_R, "y", 2, 2)

        #const.GRAVEYARD_POS_X
        waypoints: list[wp.Waypoint] = [
            wp.Waypoint(aux.Point(0, 0), 0, wp.WType.S_STOP)
        ] * const.TEAM_ROBOTS_MAX_COUNT

        if self.game_status == GameStates.RUN:
            self.run(field, waypoints)
        else:
            waypoints: list[wp.Waypoint] = [
            wp.Waypoint(aux.Point(0, 0), 0, wp.WType.S_STOP)] * const.TEAM_ROBOTS_MAX_COUNT
            '''if self.game_status == GameStates.TIMEOUT:
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
                self.keep_distance(field, waypoints)'''

        # print(self.game_status, self.state)
        return waypoints

    def getIndexHolding(
        self, field: field.Field
    ) -> Optional[
        int
    ]:  # Возвращает индекс атакующего робота (для врагов индекс + 3), None -- в случае неопределенного статуса
        minDistEnemy = 4500.0
        iEnemy = -1
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            d = aux.dist(field.ball.get_pos(), field.enemies[i].get_pos())
            if d < minDistEnemy:
                minDistEnemy = d
                iEnemy = i

        minDistAllies = 4500.0
        iAllies = -1
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            d = aux.dist(field.ball.get_pos(), field.allies[i].get_pos())
            if d < minDistAllies:
                minDistAllies = d
                iAllies = i

        if minDistAllies - minDistEnemy > 30:
            return 3 + iEnemy
        elif minDistEnemy - minDistAllies > 30:
            return iAllies
        return None
    
    def passBall(self, field: field.Field, robotInx):
        myPos = field.ball.get_pos()
        passRobot = -1
        minDist = 10000
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if field.allies[i].is_used() and i != robotInx:
                alliePos = field.allies[i].get_pos()
                if aux.dist(myPos, alliePos) < minDist and self.canPass(field, robotInx, i):
                    minDist = aux.dist(myPos, alliePos)
                    passRobot = i

        if passRobot != -1: return passRobot 
        else: return None

    def between(self, posFrom, posTo, posBet):
        xMax = max(posFrom.x, posTo.x)
        xMin = min(posFrom.x, posTo.x)
        yMax = max(posFrom.y, posTo.y)
        yMin = min(posFrom.y, posTo.y)
        return (xMin - 0.8 * const.ROBOT_R < posBet.x < xMax + 0.8 * const.ROBOT_R) \
                and (yMin - 0.8 * const.ROBOT_R < posBet.y < yMax + 0.8 * const.ROBOT_R)

    def canPass(self, field: field.Field, my, any):
        fromPoses = (field.allies[my].get_pos(), field.ball.get_pos())
        toPos = field.allies[any].get_pos()
        
        state = True
        for fromPos in fromPoses:
            if toPos.x != fromPos.x: kl = (toPos.y - fromPos.y) / (toPos.x - fromPos.x)
            else: kl = 1
            bl = fromPos.y - kl * fromPos.x

            a = -kl
            b = 1
            c = -bl

            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if (i != my and i != any and 
                    self.between(fromPos, toPos, field.allies[i].get_pos()) 
                    and self.intersection(a, b, c, field.allies[i].get_pos())):
                    state = False
                    break
            
        return state
    
    def intersection(self, a, b, c, pos):
        return abs(a * pos.x + b * pos.y + c) / math.sqrt(a**2 + b**2) < const.ROBOT_R * 0.8


    def kickToGoal(self, field: field.Field, robotInx):
        #ballPos = field.allies[robotInx].get_pos()
        ballPos = field.ball.get_pos()#get_pos() 

        poses = []
        #poses.append(aux.Point(const.GOAL_DX, (const.GOAL_DY / 2)))
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if field.enemies[i].is_used(): poses.append(field.enemies[i].get_pos())
            if field.allies[i].is_used() and i != robotInx: 
                poses.append(field.allies[i].get_pos())
        
        #poses.append(field.allies[8].get_pos())
        #poses.append(aux.Point(const.GOAL_DX, (-const.GOAL_DY / 2)))
        
        central = []
        for i in range(len(poses)):
            #print("COORD:", poses[i].x, poses[i].y)
            dist = aux.dist(ballPos, poses[i])
            if dist == 0: dist = 1
            #print()
            if poses[i].x != ballPos.x: D = abs(dist * (const.GOAL_DX - ballPos.x) / (poses[i].x - ballPos.x))
            else: D = dist
        
            if (5 + (const.ROBOT_R * 100)) / dist > 1: alphaNew = math.asin(1)
            else: alphaNew = math.asin((5 + (const.ROBOT_R * 100)) / dist)
            
            gamma = math.acos(abs(poses[i].x - ballPos.x) / dist) - alphaNew
            downDist = math.sqrt(abs(D**2 - (const.GOAL_DX - ballPos.x)**2)) - abs(const.GOAL_DX - ballPos.x) * math.tan(gamma) #HASHUV
            
            if abs(D * math.sin(alphaNew) / downDist) <= 1:
                bettaNew = math.asin(D * math.sin(alphaNew) / downDist) 
            elif math.sin(alphaNew) > 1: bettaNew = math.asin(1)
            else: bettaNew = math.asin(-1)
            
            #upDist = D * math.sin(alphaNew) / math.sin(math.pi - 2 * alphaNew - bettaNew) #HASHUV
            upDist = abs(const.GOAL_DX - ballPos.x) * math.tan(gamma + 2 * alphaNew) - abs(const.GOAL_DX - ballPos.x) * math.tan(gamma) - downDist

            if ballPos.y > poses[i].y: 
                ycc = ballPos.y - D * math.sin(alphaNew + gamma)
            else:
                (downDist, upDist) = (upDist, downDist) 
                ycc = ballPos.y + D * math.sin(alphaNew + gamma)
        
            if upDist < downDist: (downDist, upDist) = (upDist, downDist)

            #if ycc + upDist > (-const.GOAL_DY / 2) and ycc - downDist  < (const.GOAL_DY / 2):
            if ycc > (-const.GOAL_DY / 2) - (const.ROBOT_R * 100) and ycc < (const.GOAL_DY / 2) + (const.ROBOT_R * 100):
                central.append([ycc, ycc + upDist, ycc - downDist])
        
        central = sorted(central, key = lambda x: x[0])
        
        #for i in range(len(central)):
        #   print(central[i][0], central[i][1], central[i][2])

        maxiAngle = -4 * math.pi
        rememberI = -2
        #print('YES')
        if len(central) != 0:
            for i in range(-1, len(central)):
                if i == -1:
                    lookUp = central[0][2]
                    lookDown = -const.GOAL_DY / 2
                elif i == len(central) - 1:
                    lookUp = const.GOAL_DY / 2
                    lookDown = central[i][1]
                else:
                    lookUp = central[i + 1][2]
                    lookDown = central[i][1]
                #print(lookUp, lookDown)

                if lookUp < lookDown: 
                    continue

                if lookDown < lookUp:
                    bokDown = math.sqrt((ballPos.x - const.GOAL_DX)**2 + (ballPos.y - lookDown)**2)
                    bokUp = math.sqrt((ballPos.x - const.GOAL_DX)**2 + (ballPos.y - lookUp)**2)
                    v1 = aux.Point(const.GOAL_DX - ballPos.x, lookDown - ballPos.y)
                    v2 = aux.Point(const.GOAL_DX - ballPos.x, lookUp - ballPos.y)
                    
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
            if rememberI == -1:
                lookUp = central[0][2]
                lookDown = -const.GOAL_DY / 2
            elif rememberI == len(central) - 1:
                lookUp = const.GOAL_DY / 2
                lookDown = central[rememberI][1]
            else:
                lookUp = central[rememberI + 1][2]
                lookDown = central[rememberI][1]

            bokDown = math.sqrt((ballPos.x - const.GOAL_DX)**2 + (ballPos.y - lookDown)**2)
            bokUp = math.sqrt((ballPos.x - const.GOAL_DX)**2 + (ballPos.y - lookUp)**2)
            osn = lookUp - lookDown
            distUp = osn * bokUp / (bokUp + bokDown)
            
            self.yR = lookUp - distUp
            canKickToGoal = True
        else: self.yR = 0
        
        if canKickToGoal: return math.atan2(self.yR - ballPos.y, const.GOAL_DX - ballPos.x)
        else: return None

    def getRobotsList(self, field: field.Field):
        rlist = []
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if field.allies[i].is_used(): rlist.append(field.allies[i].get_pos())
            if field.enemies[i].is_used(): rlist.append(field.enemies[i].get_pos())
        return rlist

    def teamOfRobot(self, field: field.Field, point):
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if field.allies[i].is_used() and aux.pointsAreEqual(field.allies[i].get_pos(), point): 
                return (i, Team.MY_TEAM)
            if field.enemies[i].is_used() and aux.pointsAreEqual(field.enemies[i].get_pos(), point):
                return (i, Team.ENEMY_TEAM)
        
        return (-1, Team.UNCERTAIN_TEAM)

    def getHoldingRobot(self, field: field.Field):
        robotList = self.getRobotsList(field)
        if len(robotList) != 0:
            #print('YES')
            nearestRobot2Ball = aux.find_nearest_point(field.ball.get_pos(), robotList)
            #print(nearestRobot2Ball)
            return self.teamOfRobot(field, nearestRobot2Ball)
        else: return (-1, Team.UNCERTAIN_TEAM)

    def robotBlockedGoal(self, field: field.Field, robotPos):
        upperDist = aux.dist2line(field.ball.get_pos(), 
                                  aux.Point(const.GOAL_DX, const.GOAL_DY / 2),
                                  robotPos)
        downerDist = aux.dist2line(field.ball.get_pos(), 
                                   aux.Point(const.GOAL_DX, -const.GOAL_DY / 2),
                                   robotPos)
        #print('up down', upperDist, downerDist)
        return upperDist <= 1000 * const.ROBOT_R and downerDist <= 1000 * const.ROBOT_R  

    def enemyRobotMayKick2Goal(self, field: field.Field, enemyRobotPos):
        enemyMayKick = aux.get_line_intersection(field.ball.get_pos(),
                                                enemyRobotPos,
                                                aux.Point(const.GOAL_DX, -const.GOAL_DY / 2),
                                                aux.Point(const.GOAL_DX, const.GOAL_DY / 2),
                                                "LL")
        
        if enemyMayKick == aux.Point(None, None): return None
        
        return -const.GOAL_DY <= enemyMayKick.y <= const.GOAL_DY

    def run(self, field: field.Field, waypoints: list[wp.Waypoint]) -> None:
        '''inxGetter = self.passBall(field, self.atackerInx)#self.kickToGoal(field, self.atackerInx) 
        if inxGetter != None: 
            if not self.haveAngle:
                self.haveAngle = True
                self.angleMyRobot = aux.angle_to_point(field.allies[self.atackerInx].get_pos(), field.allies[inxGetter].get_pos())
 
        waypoints[self.atackerInx] = wp.Waypoint(field.ball.get_pos(), self.angleMyRobot, wp.WType.S_BALL_KICK)# - задать точку для езды. Куда, с каким углом, тип.
        '''
        
        if not hasattr(self.run, "atackerOld"):
            atackerOld = ()

        atacker = self.getHoldingRobot(field)
        #print(self.robotBlockedGoal(field, field.allies[self.defenderInx].get_pos()))
        #print(self.enemyRobotMayKick2Goal(field, field.enemies[0].get_pos()))
        if atacker[1] == Team.ENEMY_TEAM:
            print(self.enemyRobotMayKick2Goal(field, field.enemies[atacker[0]].get_pos()))
            if not self.enemyRobotMayKick2Goal(field, field.enemies[atacker[0]].get_pos()):
                point2DefenseLine = aux.closest_point_on_line(aux.Point(const.GOAL_DX, 0), 
                                                              field.ball.get_pos(), 
                                                              field.allies[self.defenderInx].get_pos(),
                                                              True)
            else:
                point2DefenseLine = aux.closest_point_on_line(field.ball.get_pos(), 
                                                              field.enemies[atacker[0]].get_pos(), 
                                                              field.allies[self.defenderInx].get_pos(),
                                                              True)
                #print(aux.isPointOnLine(point2DefenseLine, field.ball.get_pos(), field.allies[self.defenderInx].get_pos()))
                
                if aux.isPointOnLine(point2DefenseLine, field.ball.get_pos(), field.allies[self.defenderInx].get_pos()):
                    print("my Blocked:", self.robotBlockedGoal(field, field.allies[self.defenderInx].get_pos()))
                    if self.robotBlockedGoal(field, field.allies[self.defenderInx].get_pos()):
                        point2DefenseLine = field.allies[self.defenderInx].get_pos()
                    else:
                        point2DefenseLine = field.ball.get_pos()
            
            if atackerOld != atacker:
                self.angleDefender = aux.angle_to_point(point2DefenseLine, field.ball.get_pos())
            waypoints[self.defenderInx] = wp.Waypoint(point2DefenseLine, self.angleDefender, wp.WType.S_ENDPOINT)
            atackerOld = atacker

        #print(atacker)
        #print(atacker[1], field.enemies[atacker[0]].get_pos())
        
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
        self, field: field.Field, waypoints: list[wp.Waypoint], gk_wall_idx_list: list[int], robot_with_ball: rb.Robot
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
                field.ball.get_pos(), field.ally_goal.eye_forw.arg(), wp.WType.S_BALL_KICK
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