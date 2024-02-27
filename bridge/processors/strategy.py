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
        self.angleMyRobot = 0#aux.Point(0, 0)
<<<<<<< Updated upstream
=======
        self.choosedKick = False
        self.passInd = -1
>>>>>>> Stashed changes

    def process(self, field: field.Field):
        """
        Рассчитать конечные точки для каждого робота
        """
        waypoints = [None]*const.TEAM_ROBOTS_MAX_COUNT
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            waypoint = wp.Waypoint(field.allies[i].getPos(), field.allies[i].getAngle(), wp.WType.S_ENDPOINT)
            waypoints[i] = waypoint

        self.run(field, waypoints)
        #print(waypoints)

        return waypoints

    def distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

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

        if minDistAllies - minDistEnemy > 30:
            return 3 + iEnemy
        elif minDistEnemy - minDistAllies > 30:
            return iAllies
        else: 
            return None

    def trueBallCoordinate(self, p):
        return not(p.x == -10000 and p.y == 0)
    
    def angleToBall(self, ball, myPos):
        return math.atan2(ball.y - myPos.y, ball.x - myPos.x)

    def kickToGoal(self, field: field.Field, robotInx):
        myPos = field.allies[robotInx].getPos()
        ballPos = field.ball.getPos() 

        poses = []
        poses.append(aux.Point(4500, self.goalUp))
        for i in range(self.n):
            poses.append(field.enemies[i].getPos())
        poses.append(aux.Point(4500, self.goalDown))
        
        central = []
        self.xR = 4500
        for i in range(len(poses)):
            #print("COORD:", poses[i].x, poses[i].y)
            dist = self.distance(ballPos, poses[i])
            #print()
            if poses[i].x != ballPos.x: D = abs(dist * (4500 - ballPos.x) / (poses[i].x - ballPos.x))
            else: D = dist
            
            if (15 + self.robotRadius) / dist > 1: alphaNew = math.asin(1)
            else: alphaNew = math.asin((15 + self.robotRadius) / dist)
            
            gamma = math.acos(abs(poses[i].x - ballPos.x) / dist) - alphaNew
            downDist = math.sqrt(D**2 - (4500 - ballPos.x)**2) - (4500 - ballPos.x) * math.tan(gamma) #HASHUV
            
            if abs(D * math.sin(alphaNew) / downDist) <= 1:
                bettaNew = math.asin(D * math.sin(alphaNew) / downDist) 
            elif math.sin(alphaNew) > 1: bettaNew = math.asin(1)
            else: bettaNew = math.asin(-1)
            
            #upDist = D * math.sin(alphaNew) / math.sin(math.pi - 2 * alphaNew - bettaNew) #HASHUV
            upDist = (4500 - ballPos.x) * math.tan(gamma + 2 * alphaNew) - (4500 - ballPos.x) * math.tan(gamma) - downDist

            if ballPos.y > poses[i].y: 
                #(downDist, upDist) = (upDist, downDist)
                ycc = ballPos.y - D * math.sin(alphaNew + gamma)
            else:
                (downDist, upDist) = (upDist, downDist) 
                ycc = ballPos.y + D * math.sin(alphaNew + gamma)

            #if ycc + upDist > self.goalDown and ycc - downDist  < self.goalUp:
            if ycc > self.goalDown - self.robotRadius and ycc < self.goalUp + self.robotRadius:
                central.append([ycc, ycc + upDist, ycc - downDist])
        
        central = sorted(central, key = lambda x: x[0])
        #for i in range(len(central)):
        #   print(central[i][0], central[i][1], central[i][2])

        maxiAngle = -2 * math.pi
        rememberI = -2
        for i in range(len(central) - 1):
            lookUp = central[i + 1][2]
            lookDown = central[i][1]

            #print(lookUp, lookDown)
            if lookUp < lookDown: continue

            if lookDown < lookUp:
                bokDown = math.sqrt((ballPos.x - 4500)**2 + (ballPos.y - lookDown)**2)
                bokUp = math.sqrt((ballPos.x - 4500)**2 + (ballPos.y - lookUp)**2)
                v1 = aux.Point(4500 - ballPos.x, lookDown - ballPos.y)
                v2 = aux.Point(4500 - ballPos.x, lookUp - ballPos.y)
                
                if (v1.x * v2.x + v1.y * v2.y) / (bokDown * bokUp) > 1: 
                    angleBetweenVectors = math.acos(1)
                elif (v1.x * v2.x + v1.y * v2.y) / (bokDown * bokUp) < -1: 
                    angleBetweenVectors = math.acos(-1)
                else: 
                    angleBetweenVectors = math.acos((v1.x * v2.x + v1.y * v2.y) / (bokDown * bokUp))

                if angleBetweenVectors > maxiAngle:
                    maxiAngle = angleBetweenVectors
                    rememberI = i
            
        if rememberI != -2:
            lookUp = central[rememberI + 1][2]
            lookDown = central[rememberI][1]

            bokDown = math.sqrt((ballPos.x - 4500)**2 + (ballPos.y - lookDown)**2)
            bokUp = math.sqrt((ballPos.x - 4500)**2 + (ballPos.y - lookUp)**2)
            osn = lookUp - lookDown
            distUp = osn * bokUp / (bokUp + bokDown)
            
            self.yR = lookUp - distUp
        else: self.yR = 0
        
        return math.atan2(self.yR - myPos.y, self.xR - myPos.x)
<<<<<<< Updated upstream

=======
    
    def between(self, posFrom, posTo, posBet):
        xMax = max(posFrom.x, posTo.x)
        xMin = min(posFrom.x, posTo.x)
        yMax = max(posFrom.y, posTo.y)
        yMin = min(posFrom.y, posTo.y)
        return (xMin - 0.8 * self.robotRadius < posBet.x < xMax + 0.8 * self.robotRadius) \
                and (yMin - 0.8 * self.robotRadius < posBet.y < yMax + 0.8 * self.robotRadius)
>>>>>>> Stashed changes
    def passBall(self, field: field.Field, robotInx):
        myPos = field.ball.getPos()
        myAngle = field.allies[robotInx].getAngle()
        passRobot = -1
        minDist = 10000
        for i in range(3):
            if i != robotInx:
                alliePos = field.allies[i].getPos()
                allieAngle = field.allies[i].getAngle()
                if self.canPass(field, robotInx, i) and self.distance(myPos, alliePos) < minDist:
                    minDist = self.distance(myPos, alliePos)
                    passRobot = i
                    #passRobots.append(i)

        return passRobot 

<<<<<<< Updated upstream
    def canPass(self, field: field.Field, my, any):#, fromPos, formAngle, toPos, toAngle):
=======
    def canPass(self, field: field.Field, my, any):
>>>>>>> Stashed changes
        fromPos = field.allies[my].getPos()
        toPos = field.allies[any].getPos()
        if toPos.x != fromPos.x: kl = (toPos.y - fromPos.y) / (toPos.x - fromPos.x)
        else: kl = 1
        bl = fromPos.y - kl * fromPos.x

<<<<<<< Updated upstream
        a = 1
        b = -kl
        c = -bl

        state = True
        for i in range(6):
            if (i < 3 and i != my and i != any and self.intersection(a, b, c, field.allies[i].getPos())) \
                or (i >= 3 and self.intersection(a, b, c, field.enemies[i - 3].getPos())):
=======
        a = -kl
        b = 1
        c = -bl
        #print(a, b, c)

        state = True
        for i in range(6):
            if (i < 3 and i != my and i != any and self.between(fromPos, toPos, field.allies[i].getPos()) and self.intersection(a, b, c, field.allies[i].getPos())) \
                or (i >= 3 and self.between(fromPos, toPos, field.enemies[i - 3].getPos()) and self.intersection(a, b, c, field.enemies[i - 3].getPos())):
>>>>>>> Stashed changes
                state = False
                break
        
        #if state:
        return state
        #else: return False
    
    def intersection(self, a, b, c, pos):
<<<<<<< Updated upstream
        return abs(a * pos.x + b * pos.y + c) / math.sqrt(a**2 + b**2) < self.robotRadius + 10

    def run(self, field: field.Field, waypoints):
        if (field.ball.getPos().x == 0 and field.ball.getPos().y == 0) \
            or self.trueBallCoordinate(field.ball.getPos()): self.angleMyRobot = self.kickToGoal(field, 0) 

        if self.distance(field.ball.getPos(), field.allies[0].getPos()) < 1.5 * self.robotRadius:
            passInd = self.passBall(field, 0)
            print("GOOOOO")
            if passInd != -1:
                ang = math.atan2(field.allies[passInd].getPos().y - field.allies[0].getPos().y, field.allies[passInd].getPos().x - field.allies[0].getPos().x)
                waypoints[0] = wp.Waypoint(field.ball.getPos(), ang, wp.WType.S_BALL_KICK)# - задать точку для езды. Куда, с каким углом, тип.
            else:
                waypoints[0] = wp.Waypoint(field.ball.getPos(), 0, wp.WType.S_BALL_KICK)
=======
        return abs(a * pos.x + b * pos.y + c) / math.sqrt(a**2 + b**2) < self.robotRadius * 0.8

    def run(self, field: field.Field, waypoints):
        #if (field.ball.getPos().x == 0 and field.ball.getPos().y == 0) \
        #    or self.trueBallCoordinate(field.ball.getPos()): self.angleMyRobot = self.kickToGoal(field, 0) 

        if not self.choosedKick and self.distance(field.ball.getPos(), field.allies[0].getPos()) < 2 * self.robotRadius:
            self.passInd = self.passBall(field, 0)
            self.choosedKick = True
        elif self.choosedKick:
            #self.passInd = self.passBall(field, 0)
            print(self.passInd)

            if self.passInd != -1:
                ang = math.atan2(field.allies[self.passInd].getPos().y - field.allies[0].getPos().y, field.allies[self.passInd].getPos().x - field.allies[0].getPos().x)
                waypoints[0] = wp.Waypoint(field.ball.getPos(), ang, wp.WType.S_BALL_KICK)# - задать точку для езды. Куда, с каким углом, тип.
            else:
                waypoints[0] = wp.Waypoint(field.ball.getPos(), 0, wp.WType.S_BALL_KICK)
            
            if self.distance(field.ball.getPos(), field.allies[0].getPos()) > 2.2 * self.robotRadius:
                self.choosedKick = False
            #if field.is_ball_moves_to_goal(): self.choosedKick = False
>>>>>>> Stashed changes
        else:
            ang = math.atan2(field.ball.getPos().y - field.allies[0].getPos().y, field.ball.getPos().x - field.allies[0].getPos().x)
            waypoints[0] = wp.Waypoint(field.ball.getPos(), ang, wp.WType.S_BALL_KICK)