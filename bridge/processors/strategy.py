## @package Stratery
# 
# Расчет требуемых положений роботов исходя из ситуации на поле

import bridge.processors.field as field
import bridge.processors.waypoint as wp
import bridge.processors.const as const
import bridge.processors.auxiliary as aux
import bridge.processors.signal as signal
import bridge.processors.robot as robot
import math
from enum import Enum

#!v DEBUG ONLY
import time

class goalKeeper:
    def __init__(self):
        self.p1 = aux.Point(0,0)
        self.p2 = aux.Point(0,0)
        self.gotPoint = False
        self.time_start = time.time()
        self.intersection_point = aux.Point(0,0)

class States(Enum):
    DEBUG = 0
    DEFENCE = 1
    ATTACK = 2

class GameStates(Enum):
    HALT = 0
    STOP = 1
    RUN = 2
    TIMEOUT = 3
    PREPARE_KICKOFF = 5
    KICKOFF = 6
    PREPARE_PENALTY = 7
    PENALTY = 8
    FREE_KICK = 9
    BALL_PLACMENT = 11

class ActiveTeam(Enum):
    ALL = 0
    YELLOW = 1
    BLUE = 2

class Strategy:
    def __init__(self, dbg_game_status = GameStates.RUN, dbg_state = States.DEBUG) -> None:
        self.game_status = GameStates.RUN
        self.active_team = 0
        self.status = States.ATTACK
        self.n = 3
        self.ballRadius = 110
        self.robotRadius = 200
        self.goalUp = 500
        self.goalDown = -500

        self.gk = goalKeeper()
        ### goalkeeper strategy
        #self.gotPoint = False
        #self.time_start = time.time()

    def process(self, field: field.Field):
        """
        Рассчитать конечные точки для каждого робота
        """
        waypoints = [None]*const.TEAM_ROBOTS_MAX_COUNT
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            waypoint = wp.Waypoint(field.allies[i].getPos(), field.allies[i].getAngle(), wp.WType.S_ENDPOINT)
            waypoints[i] = waypoint

        self.run(field, waypoints)
        #self.goalkeeperProcced(field, waypoints)
        #print(waypoints)
        
        return waypoints

    def distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
    
    def get_intersection(self, p1, p2, p3, p4, old_point):
        denom = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x)

        if denom == 0:
            return old_point
        
        px = ((p1.x * p2.y - p1.y * p2.x) * (p3.x - p4.x) - (p1.x - p2.x) * (p3.x * p4.y - p3.y * p4.x)) / denom
        py = ((p1.x * p2.y - p1.y * p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x * p4.y - p3.y * p4.x)) / denom

        return aux.Point(-5000, max(-1200, min(1200, py)))
    

    def getIndexHolding(self, field: field.Field): # Возвращает индекс атакующего робота (для врагов индекс + 3), None -- в случае неопределенного статуса
        minDistEnemy = 4500
        iEnemy = -1
        for i in range(self.n):
            d = self.distance(field.ball.getPos(), field.enemies[i].getPos()) 
            if d < minDistEnemy:
                minDistEnemy = d
                iEnemy = i

        minDistAllies = 4500
        iAllies = -1
        for i in range(self.n):
            d = self.distance(field.ball.getPos(), field.allies[i].getPos()) 
            if d < minDistAllies:
                minDistAllies = d
                iAllies = i

        if minDistEnemy < minDistAllies and minDistAllies - minDistEnemy > 30:
            return 3 + iEnemy
        elif minDistAllies < minDistEnemy and minDistEnemy - minDistAllies > 30:
            return iAllies
        else: 
            return None

    def goalkeeperProcced(self, field: field.Field, waypoints):
        goalpoint1 = aux.Point(900, -1200)
        goalpoint2 = aux.Point(900, 1200)
        

        if not self.gk.gotPoint:
            self.gk.p1 = field.ball.getPos()
            self.gk.time_start = time.time()
            self.gk.gotPoint = True
        else:
            if (time.time() - self.gk.time_start) > 0.05:
                self.gk.p2 = field.ball.getPos()
                #print(self.gk.p1.x, self.gk.p1.y, self.gk.p2.x, self.gk.p2.y)
                
                self.gk.intersection_point = self.get_intersection(self.gk.p1, self.gk.p2, goalpoint1, goalpoint2, self.gk.intersection_point)
                print(self.gk.intersection_point.x, self.gk.intersection_point.y)
                self.gk.gotPoint = False
                waypoints[0] = wp.Waypoint(self.gk.intersection_point, 0, wp.WType.S_ENDPOINT)

    def run(self, field: field.Field, waypoints):
        # field.ball.getPos() - координаты мяча
        # field.enemies[i].getPos() - координаты робота соперника с id i
        # field.allies[i].getPos() - координаты робота союзника с id i
        # waypoints[i] = wp.Waypoint(field.allies[i].getPos(), field.allies[i].getAngle(), wp.WType.S_ENDPOINT) - задать точку для езды. Куда, с каким углом, тип.

        goal_pos = aux.Point(-6000, 0)
        enemy_pos = field.allies[2].getPos()
        self_pos = field.allies[1].getPos()

        alpha = math.atan2(enemy_pos.y - goal_pos.y, enemy_pos.x - goal_pos.x)
        beta = math.atan2(self_pos.y - goal_pos.y, self_pos.x - goal_pos.x) - alpha

        if(abs(beta) > 0.085):
            dist_to_goal = math.sqrt((self_pos.x - goal_pos.x) ** 2 + (self_pos.y - goal_pos.y ** 2))
            length = dist_to_goal * math.cos(beta)
            length = min(length, math.sqrt((enemy_pos.x - goal_pos.x) ** 2 + enemy_pos.y ** 2) - 50)
        else:
            length = math.sqrt((enemy_pos.x - goal_pos.x) ** 2 + enemy_pos.y ** 2) - 50
            #length = 1
        
        path_point = aux.Point(goal_pos.x + length * math.cos(alpha), goal_pos.y + length * math.sin(alpha))
        waypoints[1] = wp.Waypoint(path_point, alpha, wp.WType.S_ENDPOINT)
        
        #target_point = aux.Point(path_point.x + 100 * math.cos(alpha + (math.pi / 2)), path_point.y + 100 * math.sin(alpha + (math.pi / 2)))

        enemy2_pos = field.enemies[0].getPos()
        enemy2_angle = math.atan2(0 - self_pos.y, -6000 - self_pos.x)
        #waypoints[2] = wp.Waypoint(field.ball.getPos(), enemy_angle, wp.WType.S_BALL_KICK)

        

        pass


    def chooseKick(self, field: field.Field, robotInx):
        central = []
        
        myPos = field.allies[robotInx].getPos()
        ballPos = field.ball.getPos() 

        for i in range(self.n):
            dist = self.distance(myPos, field.enemies[i].getPos())
            D = dist * (4500 - ballPos.x) / (field.enemies[i].getPos().x - ballPos.x)
            if (self.robotRadius + self.ballRadius + 20) / dist > 1: alphaNew = math.asin(1)
            else: alphaNew = math.asin((self.robotRadius + self.ballRadius + 20) / dist)
            
            gamma = math.acos((field.enemies[i].getPos()- ballPos.x) / dist) - alphaNew
            
            downDist = math.sqrt(D**2 - (4500 - ballPos.x)**2) - (4500 - ballPos.x) * math.tan(gamma) #HASHUV
            
            if abs(D * math.sin(alphaNew) / downDist) <= 1:  bettaNew = math.asin(D * math.sin(alphaNew) / downDist) 
            elif math.sin(alphaNew) > 0: bettaNew = math.asin(1)
            else: bettaNew = math.asin(-1)
            
            upDist = D * math.sin(alphaNew) / math.sin(math.pi - 2 * alphaNew - bettaNew) #HASHUV

            if ballPos.y > field.enemies[i]: 
                (downDist, upDist) = (upDist, downDist)
                ycc = ballPos.y - D * math.sin(alphaNew + gamma)
            else:
                ycc = ballPos.y + D * math.sin(alphaNew + gamma)


            if ycc < self.goalDown or ycc > self.goalUp:
                central.append([ycc, ycc + upDist, ycc - downDist])
        
        central = sorted(central, key=lambda x: x[0])

        maxiAngle = 0
        rememberI = -1
        for i in range(len(central) - 1):
            lookUp = central[i + 1][2]
            lookDown = central[i][1]

            bokDown = math.sqrt((myPos.x - 4500)**2 + (myPos.y - lookDown)**2)
            bokUp = math.sqrt((myPos.x - 4500)**2 + (myPos.y - lookUp)**2)
            v1 = (4500 - myPos.x, lookDown - myPos.y)
            v2 = (4500 - myPos.x, lookUp - myPos.y)
            
            if (v1[0] * v2[0] + v1[1] * v2[1]) / (bokDown * bokUp) > 1: angleBetweenVectors = math.acos(1)
            elif (v1[0] * v2[0] + v1[1] * v2[1]) / (bokDown * bokUp) < -1: angleBetweenVectors = math.acos(-1)
            else: angleBetweenVectors = math.acos((v1[0] * v2[0] + v1[1] * v2[1]) / (bokDown * bokUp))

            if angleBetweenVectors > maxiAngle:
                maxiAngle = angleBetweenVectors
                rememberI = i
            
        if rememberI != -1:
            lookUp = central[rememberI + 1][2]
            lookDown = central[rememberI][1]
            bokDown = math.sqrt((myPos.x - 4500)**2 + (myPos.y - lookDown)**2)
            bokUp = math.sqrt((myPos.x - 4500)**2 + (myPos.y - lookUp)**2)
            
            osn = lookUp - lookDown
            distUp = osn * bokUp / (bokUp + bokDown)
            
            self.xR = 4500
            self.yR = lookUp - distUp
        else:
            self.xR = 4500
            self.yR = 0
        #pass

