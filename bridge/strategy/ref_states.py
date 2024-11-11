"""код состояний игры"""
import bridge.const as const
import bridge.router.waypoint as wp
from bridge.auxiliary import aux, fld
from bridge.strategy.easy_strategy import attacker, goalkeeper


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
    rC = 0
    for i in range(const.TEAM_ROBOTS_MAX_COUNT):
        if field.allies[i].is_used():
            waypoint = wp.Waypoint(
                aux.Point(800 * field.polarity, 300 - 300 * rC),
                0,
                wp.WType.S_ENDPOINT,
            )
            waypoints[i] = waypoint
            rC += 1


def prepare_penalty(field: fld.Field, waypoints: list[wp.Waypoint], we_active: bool) -> None:
    """Подготовка пенальти по команде от судей"""
    stopped_robots = []
    for ally in field.allies:
        if ally.is_used() and ally.r_id not in [field.gk_id, const.PENALTY_KICKER]:
            stopped_robots.append(ally.r_id)

    x0 = 700
    delta_x = const.ROBOT_R * 2.5
    if field.ally_color == const.Color.BLUE:
        y0 = 800
    else:
        y0 = -800

    if we_active:
        waypoints[const.PENALTY_KICKER] = wp.Waypoint(
            aux.Point(250 * field.polarity, 0),
            aux.angle_to_point(field.allies[const.PENALTY_KICKER].get_pos(), field.ball.get_pos()),
            wp.WType.S_ENDPOINT,
        )
    else:
        x0 *= -1
        delta_x *= -1
        waypoints[const.PENALTY_KICKER] = wp.Waypoint(
            aux.Point((x0 - delta_x) * field.polarity, y0),
            aux.angle_to_point(field.allies[const.PENALTY_KICKER].get_pos(), field.ball.get_pos()),
            wp.WType.S_ENDPOINT,
        )

    poses = [aux.Point((x0 + delta_x * i) * field.polarity, y0) for i in range(len(stopped_robots))]

    for robot_id in stopped_robots:
        robot_pos = field.allies[robot_id].get_pos()
        best_pos = None
        min_dist: float = 0
        for pos in enumerate(poses):
            dist = aux.dist(pos[1], robot_pos)
            if best_pos is None or dist < min_dist:
                best_pos = pos
                min_dist = dist
        if best_pos is None:
            continue

        poses.pop(best_pos[0])
        waypoints[robot_id] = wp.Waypoint(
            best_pos[1],
            aux.angle_to_point(robot_pos, field.ball.get_pos()),
            wp.WType.S_ENDPOINT,
        )

    waypoints[field.gk_id] = wp.Waypoint(
        field.ally_goal.center + field.ally_goal.eye_forw * const.ROBOT_R,
        aux.angle_to_point(field.ally_goal.center, field.ball.get_pos()),
        wp.WType.S_ENDPOINT,
    )
    # print(field.ally_goal.center + field.ally_goal.eye_forw * const.ROBOT_R)


def prepare_kickoff(field: fld.Field, waypoints: list[wp.Waypoint], we_active: bool) -> None:
    """Настройка перед состоянием kickoff по команде судей"""
    poses = []
    if we_active:
        if const.DIV == "B":
            poses = [
                aux.Point(250 * field.polarity, 0),
                aux.Point(2000 * field.polarity, 0),
                aux.Point(3500 * field.polarity, 0),
                aux.Point(300 * field.polarity, 2000),
                aux.Point(300 * field.polarity, -2000),
            ]
        elif const.DIV == "C":
            poses = [
                aux.Point(250 * field.polarity, 0),
            ]
    else:
        if const.DIV == "B":
            poses = [
                aux.Point(650 * field.polarity, -150),
                aux.Point(650 * field.polarity, 150),
                aux.Point(3500 * field.polarity, -150),
                aux.Point(3500 * field.polarity, 150),
                aux.Point(300 * field.polarity, 1250),
            ]
        elif const.DIV == "C":
            poses = [
                aux.Point(650 * field.polarity, 0),
                aux.Point(300 * field.polarity, -800),
            ]

    for i in range(const.TEAM_ROBOTS_MAX_COUNT):
        if field.allies[i].is_used() and field.allies[i].r_id != field.gk_id:
            robot_pos = field.allies[i].get_pos()
            best_pos = None
            min_dist: float = 0
            for pos in enumerate(poses):
                dist = aux.dist(pos[1], robot_pos)
                if best_pos is None or dist < min_dist:
                    best_pos = pos
                    min_dist = dist
            if best_pos is None:
                continue

            poses.pop(best_pos[0])
            waypoints[i] = wp.Waypoint(
                best_pos[1],
                aux.angle_to_point(robot_pos, field.ball.get_pos()),
                wp.WType.S_ENDPOINT,
            )

    waypoints[field.gk_id] = wp.Waypoint(
        field.ally_goal.center + field.ally_goal.eye_forw * const.ROBOT_R,
        aux.angle_to_point(field.ally_goal.center, field.ball.get_pos()),
        wp.WType.S_ENDPOINT,
    )


def kickoff(field: fld.Field, waypoints: list[wp.Waypoint], we_active: bool) -> None:
    """Удар мяча из аута"""
    prepare_kickoff(field, waypoints, we_active)
    if we_active:
        go_kick = fld.find_nearest_robot(field.ball.get_pos(), field.allies)
        attacker(field, waypoints, go_kick.r_id)
    else:
        goalkeeper(field, waypoints)
