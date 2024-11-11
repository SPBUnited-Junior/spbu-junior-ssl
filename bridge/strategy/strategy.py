"""Верхнеуровневый код стратегии"""
# pylint: disable=redefined-outer-name

# @package Strategy
# Расчет требуемых положений роботов исходя из ситуации на поле

import math

# !v DEBUG ONLY
from time import time
from typing import Optional
import enum

import bridge.router.waypoint as wp
from bridge import const
from bridge.auxiliary import aux, fld, rbt
from bridge.processors.referee_state_processor import Color as ActiveTeam
from bridge.processors.referee_state_processor import State as GameStates
from bridge.strategy import ref_states as refs


class DoingAction:
    """
    Класс действий некоторого робота, когда нужно зафиксировать положение
    """

    def __init__(self) -> None:
        self.id: Optional[int] = None
        self.angle: Optional[float] = None
        self.point: Optional[aux.Point] = None

class State(enum.Enum):
    """Класс состояния игры"""
    DEFENSE = 0
    ATTACK = 1

class Role(enum.IntEnum):
    """Класс ролей роботов во время игры"""
    BALL = 0
    WALL = 1
    RDEF = 2
    PASS = 3

class Strategy:
    """Основной класс с кодом стратегии"""

    def __init__(self, dbg_game_status: GameStates = GameStates.RUN) -> None:
        self.game_status = dbg_game_status
        self.active_team: ActiveTeam = ActiveTeam.ALL
        self.we_kick = False
        self.we_active = False
        self.ball_point = aux.Point(0, 0)
        self.doing_pass = DoingAction()
        self.doing_kick = DoingAction()
        self.new_st = State.DEFENSE
        self.global_st = State.DEFENSE
        self.time_st = time()
        self.pos_count = 0
        self.pos_ball: list[aux.Point] = []
        self.pos_n = 5
        self.active_allies: list[rbt.Robot]
        self.active_enemies: list[rbt.Robot]
        self.rbt_roles: list[rbt.Robot]
        self.ally_poses: list[tuple[int, aux.Point, float]]
        self.n_roles: list[int]
        for _ in range(self.pos_n):
            self.pos_ball.append(aux.Point(0, 0))
        self.pass_points: list[aux.Point] = []
        self.ball_hull: list[aux.Point] = []

    def change_game_state(self, new_state: GameStates, upd_active_team: ActiveTeam) -> None:
        """Изменение состояния игры и цвета команды"""
        self.game_status = new_state
        self.active_team = upd_active_team

    def process(self, field: fld.Field) -> list[wp.Waypoint]:
        """
        Рассчитать конечные точки для каждого робота
        """
        if self.active_team == ActiveTeam.ALL or field.ally_color == self.active_team:
            self.we_active = True
        else:
            self.we_active = False

        waypoints: list[wp.Waypoint] = []
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            waypoints.append(wp.Waypoint(field.allies[i].get_pos(), field.allies[i].get_angle(), wp.WType.S_STOP))
        if self.game_status == GameStates.RUN or 1:
            self.run(field, waypoints)
            # waypoints[1] = wp.Waypoint(aux.Point(1000,1000),0,wp.WType.S_SLOWDOWN)
        else:
            if self.game_status == GameStates.TIMEOUT:
                refs.timeout(field, waypoints)
            elif self.game_status == GameStates.HALT:
                pass
                # self.halt(field, waypoints)
            elif self.game_status == GameStates.PREPARE_PENALTY:
                refs.prepare_penalty(field, waypoints, self.we_active)
            elif self.game_status == GameStates.PENALTY:
                if self.we_active:
                    self.penalty_kick(waypoints, field)
                else:
                    self.goalkeeper(field)
            elif self.game_status == GameStates.PREPARE_KICKOFF:
                self.prepare_kickoff(field, waypoints)
            elif self.game_status == GameStates.KICKOFF:
                self.kickoff(field, waypoints)
            elif self.game_status == GameStates.FREE_KICK:
                self.run(field, waypoints)
            elif self.game_status == GameStates.STOP:
                self.stop(field, waypoints)
            elif self.game_status == GameStates.BALL_PLACEMENT:
                pass

        return waypoints

    def stop(self, field: fld.Field, waypoints: list[wp.Waypoint])->None:
        """Роботы стоят (почти)"""
        for ally in field.allies:
            if (ally.get_pos()-field.ball.get_pos()).mag()<500:
                waypoints[ally.r_id] = wp.Waypoint((ally.get_pos()-field.ball.get_pos()).unity()*700,ally.get_angle(),wp.WType.S_ENDPOINT)
            else:
                waypoints[ally.r_id] = wp.Waypoint(ally.get_pos(),ally.get_angle(),wp.WType.S_STOP)

    def run(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """
        Определение глобального состояния игры
        """
        self.do_cycle_config(field)
        self.do_roles(field, waypoints)

    def debug(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """
        дебаг
        # """
        # start = aux.Point(700, 0)
        # finish = aux.Point(-700, 0)

        # vec = finish - start
        # sgn = math.sin(time()) > 0

        waypoints[1] = wp.Waypoint(field.ball.get_pos(), 0, wp.WType.S_BALL_KICK)

    def get_pass_points(self, pass_points: list[aux.Point]) -> None:
        """Принимает точки для паса"""
        self.pass_points = pass_points

    def do_cycle_config(self, field: fld.Field) -> None:
        """
        функция, обновляющая направляющую точку мяча, активных ботов, состояние игры
        """
        self.ally_poses = []
        self.rbt_roles = []
        self.n_roles = [0, 0, 0, 0]
        start_st: State
        self.pos_ball[self.pos_count] = field.ball.get_pos()
        self.pos_count = (self.pos_count + 1) % self.pos_n
        self.ball_point = self.pos_ball[self.pos_count]
        self.ball_hull = [field.ball.get_pos(), field.enemy_goal.up, field.enemy_goal.down]
        # field.image.draw_dot(self.ball_point, (0, 255, 0), 30)
        self.active_allies = fld.find_nearest_robots(field.ball.get_pos(), field.allies, None, [field.allies[const.GK]])
        self.active_enemies = fld.find_nearest_robots(field.ball.get_pos(), field.enemies)
        if self.active_enemies:
            closest_dist = aux.dist(self.active_enemies[0].get_pos(), field.ball.get_pos())
        else:
            self.new_st = State.ATTACK
            self.global_st = State.ATTACK
            return
        if self.active_allies:
            closest_dist_ally = aux.dist(self.active_allies[0].get_pos(), field.ball.get_pos())
        else:
            self.new_st = State.DEFENSE
            self.global_st = State.DEFENSE
            return
        if (closest_dist < const.ROBOT_R + const.BALL_R * 2 or closest_dist < closest_dist_ally) and not field.is_ball_moves():
            start_st = State.DEFENSE
        else:
            start_st = State.ATTACK
        if start_st != self.new_st:
            self.new_st = start_st
            self.time_st = time()
        if aux.is_point_inside_poly(field.ball.get_pos(), field.ally_goal.hull):
            self.new_st = State.ATTACK
        if self.new_st == State.DEFENSE and self.global_st != 0 and (time() - self.time_st > const.DEFENSE_TIME or field.is_ball_moves()):
            self.global_st = State.DEFENSE
        if self.new_st == State.ATTACK and self.global_st != 1 and not field.is_ball_moves():
            self.global_st = State.ATTACK


    def goalkeeper(self, field: fld.Field) -> aux.Point:
        """
        функция вратаря с несколькими режимами поведения
        """
        p_ball = None
        if field.is_ball_moves_to_goal():
            help_p = aux.rotate(aux.Point(1, 0), aux.angle_to_point(self.ball_point, field.ball.get_pos())) + field.ball.get_pos()
            return aux.closest_point_on_line(field.ball.get_pos(), help_p, field.allies[const.GK].get_pos(), "R")
        elif self.new_st == State.DEFENSE:
            sorted_enemies = fld.find_nearest_robots(field.ball.get_pos(), field.enemies)
            if sorted_enemies:
                enemy_with_ball = sorted_enemies[0]
                if aux.dist(enemy_with_ball.get_pos(), field.ball.get_pos()) < const.ROBOT_R * 2 + const.BALL_R:
                    help_p = aux.rotate(aux.Point(1, 0), enemy_with_ball.get_angle()) + field.ball.get_pos()
                    if (
                        abs(aux.get_angle_between_points(help_p, enemy_with_ball.get_pos(), field.ball.get_pos()))
                        < const.TRUE_ANGLE
                    ):
                        p_ball = help_p
                        if not aux.get_line_intersection(
                            field.ball.get_pos(), p_ball, field.ally_goal.down, field.ally_goal.up, "RS"
                        ):
                            p_ball = None
        if p_ball is None:
            p_ball = aux.rotate(aux.Point(1, 0), fld.gate_angle_size(field.ball.get_pos(), field, False)[0]) + field.ball.get_pos()
        p_to_go = aux.average_point(aux.closest_point_on_poly(field.ball.get_pos(), p_ball, field.ally_goal.small_hull, "L"))
        field.image.draw_line(field.ball.get_pos(), aux.point_on_line(field.ball.get_pos(), p_to_go, 10000))
        return p_to_go

    def prepare_kickoff(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """Настройка перед состоянием kickoff по команде судей"""
        if self.we_active:
            self.we_kick = True
        else:
            self.we_kick = False
        self.put_kickoff_waypoints(field, waypoints)

    def find_wall_points(self, field: fld.Field, n_rbts: int, min_n: int = 1) -> Optional[list[aux.Point]]:
        """ищет точки для стенки"""
        robot_poses: list[aux.Point] = []
        if n_rbts == 0:
            return [self.goalkeeper(field)]
        goal_angle = aux.get_angle_between_points(field.ally_goal.up, field.ball.get_pos(), field.ally_goal.down) / (n_rbts + 1)
        helps2:list[aux.Point] = []
        for i in range(n_rbts + 1):
            helps2.append(aux.rotate(field.ally_goal.up - field.ball.get_pos(), goal_angle * (i + 0.5)) + field.ball.get_pos())
        robot_points:list[aux.Point] = []
        for help2 in helps2:
            robot_points.append(aux.find_nearest_point(field.ball.get_pos(),
                aux.segments_poly_intersect(field.ball.get_pos(), help2, field.ally_goal.big_hull, "L")))
        if field.ball.get_pos().y * field.ally_goal.up.y < 0:
            gk_point = robot_points[0]
            robot_points.pop(0)
        else:
            gk_point = robot_points[-1]
            robot_points.pop()
        robot_poses.append(aux.average_point(aux.closest_point_on_poly(field.ball.get_pos(), gk_point, field.ally_goal.small_hull, "L")))
        corner_in_hull: Optional[aux.Point] = None
        wall_hull = [field.ball.get_pos(), robot_points[0], robot_points[-1]]
        for i in range(2, 4):
            if aux.is_point_inside_poly(field.ally_goal.big_hull[i], wall_hull):
                corner_in_hull = field.ally_goal.big_hull[i]
        if corner_in_hull:
            r_wall = aux.dist(corner_in_hull, field.ball.get_pos())
        else:
            r_wall = aux.dist(field.ball.get_pos(), aux.find_nearest_point(field.ball.get_pos(), robot_points))
        for robot_point in robot_points:
            robot_poses.append(aux.point_on_line(field.ball.get_pos(), robot_point, r_wall))
        if abs(math.sin(goal_angle / 2) * r_wall) < const.ROBOT_R and n_rbts > min_n:
            return None
        return robot_poses

    def do_roles(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """Распределение ролей между роботами и их перемещение"""
        p_to_go: aux.Point = aux.Point(0, 0)
        angle_to_go: float = 0
        best_point: aux.Point
        pass_robots: list[rbt.Robot] = []
        if (self.global_st == State.ATTACK and (aux.is_point_inside_poly(field.ball.get_pos(), field.ally_goal.hull)
            or aux.dist(aux.nearest_point_on_poly(field.ball.get_pos(), field.ally_goal.hull), field.ball.get_pos())
            < const.ROBOT_R * 2 + const.BALL_R) and field.allies[const.GK].is_used()):
            self.active_allies.insert(0, field.allies[const.GK])
        if self.active_allies:
            self.rbt_roles.append(self.active_allies[0])
            self.n_roles[Role.BALL] = 1
        else:
            return
        if self.global_st == State.ATTACK:
            if field.is_ball_moves():
                self.doing_kick.id = None
                self.doing_kick.angle = None
            elif self.doing_kick.id is None:
                self.doing_pass.id = None
                self.doing_pass.point = None
            if self.doing_kick.id is not None:
                if aux.dist(field.ball.get_pos(), field.allies[self.doing_kick.id].get_pos()) > const.ROBOT_R + const.BALL_R * 3:
                    self.doing_kick.id = None
                    self.doing_kick.angle = None
            self.n_roles[Role.PASS] = len(self.active_allies) - self.n_roles[Role.BALL]
        else:
            self.n_roles[Role.WALL] = round(abs(field.ball.get_pos().x - field.enemy_goal.center.x) /
                const.GOAL_DX / 2 * (len(self.active_allies) - sum(self.n_roles[Role.BALL:Role.WALL])))
            self.n_roles[Role.RDEF] = len(self.active_allies) - sum(self.n_roles[Role.BALL:Role.RDEF])
            self.doing_kick.id = None
            self.doing_kick.angle = None
            self.doing_pass.id = None
            self.doing_pass.point = None
        robot_poses = self.find_wall_points(field, self.n_roles[Role.WALL])
        while robot_poses is None:
            self.n_roles[Role.WALL] -= 1
            self.n_roles[Role.RDEF] += 1
            robot_poses = self.find_wall_points(field, self.n_roles[Role.WALL])
        complete_wall = True
        for n in range(1, self.n_roles[Role.WALL] + 1):
            now_ally = fld.find_nearest_robot(robot_poses[n], self.active_allies, self.rbt_roles)
            p_to_go = robot_poses[n]
            if aux.dist(p_to_go, now_ally.get_pos()) > const.WALL_DIST:
                complete_wall = False
            angle_to_go = aux.angle_to_point(p_to_go, field.ball.get_pos())
            self.ally_poses.append((now_ally.r_id, p_to_go, angle_to_go))
            self.rbt_roles.append(now_ally)
        if self.rbt_roles[0].r_id != const.GK:
            if complete_wall and not field.is_ball_moves():
                p_to_go = robot_poses[0]
            else:
                p_to_go = self.goalkeeper(field)
            field.image.draw_dot(p_to_go, (255, 255, 255), const.ROBOT_R * 1.2)
            waypoints[const.GK] = wp.Waypoint(p_to_go,
                aux.angle_to_point(field.ally_goal.center, aux.Point(0, 0)), wp.WType.S_IGNOREOBSTACLES)
        for n in range(1, self.n_roles[Role.RDEF] + 1):
            if n < len(self.active_enemies):
                now_ally = fld.find_nearest_robot(self.active_enemies[n].get_pos(), self.active_allies, self.rbt_roles)
                p_to_go = aux.point_on_line(self.active_enemies[n].get_pos(), field.ball.get_pos(), const.ROBOT_R * 2)
                angle_to_go = aux.angle_to_point(p_to_go, field.ball.get_pos())
                self.ally_poses.append((now_ally.r_id, p_to_go, angle_to_go))
                self.rbt_roles.append(now_ally)
            else:
                self.n_roles[Role.RDEF] -= 1
                self.n_roles[Role.PASS] += 1
        for n in range(self.n_roles[Role.PASS]):
            if n < len(self.pass_points):
                now_ally = fld.find_nearest_robot(self.pass_points[n], self.active_allies, self.rbt_roles)
                best_point = self.pass_points[n]
                crossings = aux.segments_poly_intersect(now_ally.get_pos(), best_point, self.ball_hull)
                if crossings:
                    if aux.dist(best_point, now_ally.get_pos()) > aux.dist(self.rbt_roles[0].get_pos(), field.ball.get_pos()):
                        if len(crossings) == 1:
                            crossings = aux.segments_poly_intersect(now_ally.get_pos(), best_point, self.ball_hull, "L")
                        crossings.sort(key=lambda x: aux.dist(best_point, x))
                        best_point = aux.point_on_line(crossings[0], crossings[1], aux.dist(crossings[0], crossings[1]) + 2 * const.ROBOT_R)
                if (
                    aux.dist(
                        now_ally.get_pos(),
                        aux.closest_point_on_line(field.ball.get_pos(), best_point, now_ally.get_pos(), is_inf="R"),
                    )
                    < const.ROBOT_R + const.BALL_R
                ):
                    pass_robots.append(now_ally)
                self.rbt_roles.append(now_ally)
                self.ally_poses.append((now_ally.r_id, best_point, aux.angle_to_point(best_point, field.ball.get_pos())))
            else:
                break
        if self.global_st == State.ATTACK:
            pass_robots.sort(key=lambda x: fld.gate_angle_size(x.get_pos(), field))
            gate_kick = False
            now_angle_size = fld.gate_angle_size(field.ball.get_pos(), field)
            if pass_robots:
                if fld.gate_angle_size(pass_robots[0].get_pos(), field)[2] <= now_angle_size[2] or now_angle_size[1] > const.DELTA_ANGLE:
                    gate_kick = True
            else:
                gate_kick = True
            if gate_kick:
                angle_to_go = now_angle_size[0]
            else:
                angle_to_go = aux.angle_to_point(field.ball.get_pos(), pass_robots[0].get_pos())
            angle_point = aux.rotate(aux.Point(1, 0), angle_to_go) + self.rbt_roles[0].get_pos()
            if (aux.dist(field.ball.get_pos(), self.rbt_roles[0].get_pos()) < const.FIX_KICK_DIST and self.doing_kick.id is None and
                abs(aux.get_angle_between_points(angle_point, self.rbt_roles[0].get_pos(), field.ball.get_pos())) < math.pi / 8):
                self.doing_kick.id = self.rbt_roles[0].r_id
                self.doing_kick.angle = angle_to_go
                if not gate_kick and self.doing_pass.id is None:
                    self.doing_pass.id = pass_robots[0].r_id
                    self.doing_pass.point = pass_robots[0].get_pos()
        else:
            angle_to_go = (aux.average_angle([(field.ally_goal.up - field.ball.get_pos()).arg(),
                (field.ally_goal.down - field.ball.get_pos()).arg()]) - math.pi)
        p_to_go = aux.rotate(-aux.Point(const.BALL_R + const.ROBOT_R, 0), angle_to_go) + field.ball.get_pos()
        self.ally_poses.append((self.rbt_roles[0].r_id, p_to_go, angle_to_go))
        for f in self.ally_poses:
            field.image.draw_dot(f[1], (255, 255, 255), const.ROBOT_R * 1.2)
            # print("bbb")
        if self.global_st == State.ATTACK:
            for ally_pose in self.ally_poses:
                if ally_pose[0] == self.doing_pass.id and self.doing_pass.point is not None:
                    if field.is_ball_moves():
                        p_to_go = aux.closest_point_on_line(
                            self.ball_point, field.ball.get_pos(), field.allies[ally_pose[0]].get_pos(), is_inf="L"
                        )
                        p_to_go = aux.point_on_line(
                            field.ball.get_pos(),
                            p_to_go,
                            max(
                                field.ball.get_vel().mag() * const.SUMM_DELAY + const.ACTION_DIST,
                                aux.dist(field.ball.get_pos(), field.allies[ally_pose[0]].get_pos()),
                            ),
                        )
                        waypoints[ally_pose[0]] = wp.Waypoint(
                            p_to_go, aux.angle_to_point(p_to_go, self.ball_point), wp.WType.S_IGNOREOBSTACLES
                        )
                    else:
                        waypoints[ally_pose[0]] = wp.Waypoint(
                            self.doing_pass.point, aux.angle_to_point(self.doing_pass.point, self.ball_point), wp.WType.S_ENDPOINT
                        )
                elif ally_pose[0] == self.doing_kick.id and self.doing_kick.angle is not None:
                    waypoints[ally_pose[0]] = wp.Waypoint(field.ball.get_pos(), self.doing_kick.angle, wp.WType.S_BALL_KICK)
                else:
                    waypoints[ally_pose[0]] = wp.Waypoint(ally_pose[1], ally_pose[2], wp.WType.S_ENDPOINT)
        else:
            for ally_pose in self.ally_poses:
                type_to_go = wp.WType.S_ENDPOINT
                robo_point = field.allies[ally_pose[0]].get_pos()
                help_p = aux.rotate(aux.Point(1, 0), aux.angle_to_point(self.ball_point, field.ball.get_pos())) + field.ball.get_pos()
                closest_point = aux.closest_point_on_line(field.ball.get_pos(), help_p, robo_point, "R")
                if aux.dist(robo_point, closest_point) < aux.dist(closest_point, field.ball.get_pos()) / const.K_VEL and field.is_ball_moves():
                    p_to_go = closest_point
                    angle_to_go = field.allies[ally_pose[0]].get_angle()
                    type_to_go = wp.WType.S_IGNOREOBSTACLES
                else:
                    if ally_pose[0] == self.active_allies[0].r_id:
                        type_to_go = wp.WType.S_BALL_KICK
                    p_to_go = aux.average_point([ally_pose[1], aux.closest_point_on_line(field.ball.get_pos(), ally_pose[1], robo_point)])
                    angle_to_go = ally_pose[2]
                waypoints[ally_pose[0]] = wp.Waypoint(p_to_go, angle_to_go, type_to_go)

    def penalty_kick(self, waypoints: list[wp.Waypoint], field: fld.Field) -> None:
        """состояние бияния пенальти"""
        waypoints[const.PENALTY_KICKER] = wp.Waypoint(field.ball.get_pos(),
            fld.gate_angle_size(field.ball.get_pos(), field)[0], wp.WType.S_BALL_KICK)
        return

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
