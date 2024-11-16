"""код состояний игры"""
import math
import time

from bridge import const
import bridge.router.waypoint as wp
from bridge.auxiliary import aux, fld

def halt(field: fld.Field, waypoints: list[wp.Waypoint]) -> None:  # TODO: проверить что роботы останавливаются на самом деле
    """Пауза по команде от судей"""
    for i in range(const.TEAM_ROBOTS_MAX_COUNT):
        if field.allies[i].is_used():
            waypoint = wp.Waypoint(
                field.allies[i].get_pos(),
                field.allies[i].get_angle(),
                wp.WType.S_ENDPOINT,
            )
            waypoints[i] = waypoint


def timeout(field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
    """Таймаут по команде от судей""" 
    active_allies = []
    for ally in field.allies:
        if ally.is_used():
            active_allies.append(ally)
    # start_p = field.ball.get_pos()
    start_p = aux.Point(0, 0)
    for i, robot in enumerate(active_allies):
        delta_p = aux.rotate(aux.Point(600 + 300 * math.sin(time.time() * const.K_TIMEOUT), 0), time.time() * const.K_TIMEOUT + 2 * math.pi / len(active_allies) * i)
        waypoints[robot.r_id] = wp.Waypoint(start_p + delta_p, 0, wp.WType.R_IGNORE_GOAl_HULL)
