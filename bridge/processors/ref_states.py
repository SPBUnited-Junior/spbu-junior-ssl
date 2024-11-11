import math

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.field as fld
import bridge.processors.waypoint as wp


class RefStates:
    def __init__(self) -> None:
        self.we_active = 1

        # PENALTY
        self.we_kick = 0
        self.is_started = 0
        self.penalty_timer = 0

    def prepare_penalty(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """Подготовка пенальти по команде от судей"""
        if self.we_active:
            self.we_kick = 1
        else:
            self.we_kick = 0
        if self.we_kick:
            rC = 0
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if (
                    field.allies[i].is_used()
                    and field.allies[i].r_id != const.PENALTY_KICKER
                    and field.allies[i].r_id != field.gk_id
                ):
                    waypoint = wp.Waypoint(
                        aux.Point(1500 * field.polarity, 250 - 500 * rC), field.allies[i].get_angle(), wp.WType.S_ENDPOINT
                    )
                    waypoints[i] = waypoint
                    rC += 1
            waypoint = wp.Waypoint(
                aux.Point(550 * field.polarity, 0),
                aux.angle_to_point(aux.Point(550 * field.polarity, 0), field.ball.get_pos()),
                wp.WType.S_ENDPOINT,
            )
            waypoints[field.allies[const.PENALTY_KICKER].r_id] = waypoint

            waypoint = wp.Waypoint(
                field.ally_goal.center, 0, wp.WType.S_ENDPOINT
            )
            waypoints[field.allies[field.gk_id].r_id] = waypoint
        else:
            rC = 0
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if field.allies[i].is_used() and field.allies[i].r_id != field.gk_id:
                    waypoint = wp.Waypoint(aux.Point(1500 * -field.polarity, 250 - 500 * rC), 0, wp.WType.S_ENDPOINT)
                    waypoints[i] = waypoint
                    rC += 1
            waypoint = wp.Waypoint(
                field.ally_goal.center, 0, wp.WType.S_ENDPOINT
            )
            waypoints[field.allies[field.gk_id].r_id] = waypoint

    def halt(
        self, field: fld.Field, waypoints: list[wp.Waypoint]
    ) -> None:  # TODO: проверить что роботы останавливаются на самом деле
        """Пауза по команде от судей"""
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if field.allies[i].is_used():
                waypoint = wp.Waypoint(field.allies[i].get_pos(), field.allies[i].get_angle(), wp.WType.S_ENDPOINT)
                waypoints[i] = waypoint

    def timeout(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """Таймаут по команде от судей"""
        rC = 0
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if field.allies[i].is_used():
                waypoint = wp.Waypoint(aux.Point(1000 * field.polarity, 1250 - 500 * rC), 0, wp.WType.S_ENDPOINT)
                waypoints[i] = waypoint
                rC += 1

    def penalty_kick(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """Пенальти (атака)"""
        field.allies[const.PENALTY_KICKER].dribbler_enable_ = 1
        field.allies[const.PENALTY_KICKER].dribbler_speed_ = 10
        kick_delta = 350

        ball = field.ball.get_pos()
        enemy_gk = field.enemies[const.ENEMY_GK].get_pos()

        if abs(aux.get_angle_between_points(enemy_gk, ball, field.enemy_goal.up)) > abs(aux.get_angle_between_points(enemy_gk, ball, field.enemy_goal.down)):
            target = aux.Point(field.enemy_goal.center.x, kick_delta)
        else:
            target = aux.Point(field.enemy_goal.center.x, -kick_delta)

        waypoints[const.PENALTY_KICKER] = wp.Waypoint(ball, aux.angle_to_point(ball, target), wp.WType.S_BALL_KICK)

    def prepare_kickoff(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """Настройка перед состоянием kickoff по команде судей"""
        if self.we_active:
            self.we_kick = 1
        else:
            self.we_kick = 0
        self.put_kickoff_waypoints(field, waypoints)

    def put_kickoff_waypoints(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """Подготовка перед состоянием kickoff"""
        rC = 0
        if self.we_kick:
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if field.allies[i].is_used() and field.allies[i].r_id != field.gk_id:
                    if rC < 3:
                        if rC == 1:
                            waypoint = wp.Waypoint(
                                aux.Point(700 * field.polarity, 0),
                                aux.angle_to_point(field.allies[i].get_pos(), aux.Point(0, 0)),
                                wp.WType.S_ENDPOINT,
                            )
                        else:
                            waypoint = wp.Waypoint(
                                aux.Point(700 * field.polarity, 2000 - 2000 * rC),
                                aux.angle_to_point(field.allies[i].get_pos(), aux.Point(0, 0)),
                                wp.WType.S_ENDPOINT,
                            )
                        waypoints[i] = waypoint
                    else:
                        waypoint = wp.Waypoint(
                            aux.Point(200 * field.polarity, 1500 - 3000 * (rC - 3)),
                            aux.angle_to_point(field.allies[i].get_pos(), aux.Point(0, 0)),
                            wp.WType.S_ENDPOINT,
                        )
                        waypoints[i] = waypoint
                    rC += 1
        else:
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if field.allies[i].is_used() and field.allies[i].r_id != field.gk_id:
                    if rC == 0:
                        waypoint = wp.Waypoint(
                            aux.Point(700 * field.polarity, 0),
                            aux.angle_to_point(field.allies[i].get_pos(), aux.Point(0, 0)),
                            wp.WType.S_ENDPOINT,
                        )
                    elif rC < 3:
                        waypoint = wp.Waypoint(
                            aux.Point(200 * field.polarity, 1000 - 2000 * (rC - 1)),
                            aux.angle_to_point(field.allies[i].get_pos(), aux.Point(0, 0)),
                            wp.WType.S_ENDPOINT,
                        )
                    else:
                        waypoint = wp.Waypoint(
                            aux.Point(200 * field.polarity, 2000 + 4000 * (rC - 4)),
                            aux.angle_to_point(field.allies[i].get_pos(), aux.Point(0, 0)),
                            wp.WType.S_ENDPOINT,
                        )
                    waypoints[i] = waypoint
                    rC += 1
        waypoint = wp.Waypoint(
            field.ally_goal.center, aux.angle_to_point(field.ally_goal.center, field.ball.get_pos()), wp.WType.S_ENDPOINT
        )
        waypoints[field.allies[field.gk_id].r_id] = waypoint

    def kickoff(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """Удар мяча из аута"""
        self.put_kickoff_waypoints(field, waypoints)
        # self.we_kick = 0
        if self.we_kick:
            go_kick = fld.find_nearest_robot(field.ball.get_pos(), field.allies)
            target = field.enemy_goal.center
            target.y = 300
            waypoint = wp.Waypoint(
                field.ball.get_pos(), (target - field.allies[go_kick.r_id].get_pos()).arg(), wp.WType.S_BALL_KICK
            )
            waypoints[go_kick.r_id] = waypoint
        else:
            go_kick = fld.find_nearest_robot(field.ball.get_pos(), field.allies)
            target = aux.point_on_line(field.ball.get_pos(), aux.Point(field.polarity * const.GOAL_DX, 0), 200)
            waypoint = wp.Waypoint(
                target, (field.ball.get_pos() - field.allies[go_kick.r_id].get_pos()).arg(), wp.WType.S_IGNOREOBSTACLES
            )
            waypoints[go_kick.r_id] = waypoint