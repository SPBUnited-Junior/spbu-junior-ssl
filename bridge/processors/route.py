"""
Класс, описывающий текущий маршрут робота
"""

import bridge.processors.const as const
import bridge.processors.waypoint as wp
import bridge.processors.field as field
import bridge.processors.auxiliary as aux
import bridge.processors.robot as robot

class Route:

    def __init__(self, robot: robot.Robot):
        self._robot = [wp.Waypoint(robot.getPos(), robot._angle, wp.WType._ROBOT)]
        self._destination = [wp.Waypoint(const.GRAVEYARD_POS, 0, wp.WType._GRAVEYARD)]
        self._routewp = []
        # self.__route = [*self.robot, *self.__routewp, *self.__destination]

    def update(self, robot: robot.Robot):
        self._robot = [wp.Waypoint(robot.getPos(), robot._angle, wp.WType._ROBOT)]
        # if self.getNextVec().mag() < const.VANISH_DIST and \
        #     len(self._routewp) >= 1:
        #     del self._routewp[0]

    def clear(self):
        self._routewp = []

    def __getRoute(self):
        return [*self._robot, *self._routewp, *self._destination]

    def setDestWP(self, dest: wp.Waypoint):
        self._destination = [dest]
   
    def getDestWP(self):
        return self._destination[0]

    def getNextWP(self) -> wp.Waypoint:
        return self.__getRoute()[1]

    def getNextSegment(self):
        return self.__getRoute()[0:1]

    def getNextVec(self) -> aux.Point:
        return (self.__getRoute()[1].pos - self.__getRoute()[0].pos)

    def getNextAngle(self) -> float:
        return self.__getRoute()[1].angle

    def getNextType(self) -> wp.WType:
        return self.__getRoute()[1].type
    
    def insertWP(self, wp: wp.Waypoint):
        self._routewp.insert(0, wp)

    def isUsed(self):
        return self._destination[0].type != wp.WType._GRAVEYARD

    def getLenght(self):
        dist = 0
        last_wp_pos = self._robot[0].pos
        for wpt in self.__getRoute():
            if wpt.type == wp.WType.S_BALL_GO or \
                wpt.type == wp.WType.S_BALL_KICK or \
                wpt.type == wp.WType.S_BALL_GRAB:
                    break
            dist += (wpt.pos - last_wp_pos).mag()
            last_wp_pos = wpt.pos
        
        #if wpt.type == wp.WType.S_BALL_GRAB: dist = 0
        return dist
        
    def __str__(self):
        strin = "ROUTE: "
        for wp in self.__getRoute():
            strin += " ->\n" + str(wp)
        # for wp in [*self.robot, *self.__routewp, *self.__destination]:
        #     strin += " -> " + str(wp)
        return strin

    