"""
Модуль описания структуры Field для хранения информации об объектах на поле (роботы и мяч)
"""

from functools import reduce
from math import cos, pi
from typing import Optional

from bridge import const, drawing
from bridge.auxiliary import aux, entity, rbt


class Goal:
    """
    Структура, описывающая ключевые точки ворот
    """

    def __init__(self, goal_dx: float, goal_dy: float, pen_dx: float, pen_dy: float) -> None:

        # Абсолютный центр
        self.center = aux.Point(goal_dx, 0)

        # Относительные вектора
        self.eye_forw = aux.Point(-aux.sign(goal_dx), 0)
        self.eye_up = aux.Point(0, aux.sign(goal_dy))

        self.vec_up = aux.Point(0, goal_dy / 2)
        self.vec_pen = aux.Point(-pen_dx, 0)
        self.vec_pen_up = aux.Point(0, pen_dy / 2)

        # Абсолютные вектора
        self.up = self.center + self.vec_up
        self.down = self.center - self.vec_up
        self.frw = self.center + self.vec_pen
        self.frw_up = self.frw + self.vec_pen_up
        self.frw_down = self.frw - self.vec_pen_up

        self.center_up = self.center + self.vec_pen_up
        self.center_down = self.center - self.vec_pen_up

        # Оболочка штрафной зоны
        self.hull = [
            aux.FIELD_INF * self.eye_forw.x,
            self.center_up,
            self.frw_up,
            self.frw_down,
            self.center_down,
        ]

        self.big_hull = [
            aux.FIELD_INF * self.eye_forw.x,
            self.center_up + self.eye_up * const.ROBOT_R,
            self.frw_up + (self.eye_forw + self.eye_up) * const.ROBOT_R,
            self.frw_down + (self.eye_forw - self.eye_up) * const.ROBOT_R,
            self.center_down - self.eye_up * const.ROBOT_R,
        ]

        self.small_hull = [
            aux.Point(self.up.x + const.ROBOT_R * self.eye_forw.x, self.up.y),
            aux.Point(self.frw_up.x, self.up.y),
            aux.Point(self.frw_down.x, self.down.y),
            aux.Point(self.down.x - const.ROBOT_R * self.eye_forw.x, self.down.y),
        ]


class Field:
    """
    Класс, хранящий информацию о всех объектах на поле и ключевых точках
    """

    def __init__(self) -> None:
        """
        Конструктор
        Инициализирует все нулями

        TODO Сделать инициализацию реальными параметрами для корректного
        определения скоростей и ускорений в первые секунды
        """
        self.last_update = 0.0
        self.robot_with_ball: Optional[rbt.Robot] = None
        self.image: drawing.Image = drawing.Image()

        self.gk_id = const.GK
        self.enemy_gk_id = const.ENEMY_GK

        self.ally_color = const.COLOR

        self.be_slow = False

        if self.ally_color == const.Color.BLUE:
            self.polarity = const.POLARITY * -1
        else:
            self.polarity = const.POLARITY

        self.ball = entity.Entity(aux.GRAVEYARD_POS, 0, const.BALL_R, 0.2)
        ctrl_mapping = const.CONTROL_MAPPING
        self.b_team = [
            rbt.Robot(
                aux.GRAVEYARD_POS,
                0,
                const.ROBOT_R,
                const.Color.BLUE,
                i,
                ctrl_mapping[i],
            )
            for i in range(const.TEAM_ROBOTS_MAX_COUNT)
        ]
        self.y_team = [
            rbt.Robot(
                aux.GRAVEYARD_POS,
                0,
                const.ROBOT_R,
                const.Color.YELLOW,
                i,
                ctrl_mapping[i],
            )
            for i in range(const.TEAM_ROBOTS_MAX_COUNT)
        ]
        self.all_bots = [*self.b_team, *self.y_team]
        self.ally_goal = Goal(
            const.GOAL_DX * self.polarity,
            const.GOAL_DY * self.polarity,
            const.GOAL_PEN_DX * self.polarity,
            const.GOAL_PEN_DY * self.polarity,
        )
        self.enemy_goal = Goal(
            -const.GOAL_DX * self.polarity,
            -const.GOAL_DY * self.polarity,
            -const.GOAL_PEN_DX * self.polarity,
            -const.GOAL_PEN_DY * self.polarity,
        )

        if const.SELF_PLAY:
            self.enemy_goal = self.ally_goal

        if self.ally_color == const.Color.BLUE:
            self.allies = [*self.b_team]
            self.enemies = [*self.y_team]
        elif self.ally_color == const.Color.YELLOW:
            self.allies = [*self.y_team]
            self.enemies = [*self.b_team]

        self.ball_history: list[Optional[aux.Point]] = [None] * round(0.2 / const.Ts)
        self.ball_history_idx = 0
        self.ball_start_point: Optional[aux.Point] = None

    def reverse_field(self) -> None:
        """
        меняет местами синих и желтых
        """
        self.gk_id, self.enemy_gk_id = self.enemy_gk_id, self.gk_id

        if self.ally_color == const.Color.BLUE:
            self.ally_color = const.Color.YELLOW
        else:
            self.ally_color = const.Color.BLUE

        self.polarity *= -1

        self.ally_goal, self.enemy_goal = self.enemy_goal, self.ally_goal

        self.allies, self.enemies = self.enemies, self.allies

    def update_ball(self, pos: aux.Point, t: float) -> None:
        """
        Обновить положение мяча
        !!! Вызывать один раз за итерацию с постоянной частотой !!!
        """
        self.ball.update(pos, 0, t)

        if self.ball_history[self.ball_history_idx] is None:
            self.ball_start_point = self.ball_history[0]
        else:
            self.ball_start_point = self.ball_history[self.ball_history_idx]

        self.ball_history[self.ball_history_idx] = self.ball.get_pos()
        self.ball_history_idx += 1
        self.ball_history_idx %= len(self.ball_history)

        if self.robot_with_ball is not None:
            length = len(self.ball_history)
            self.ball_history = [self.robot_with_ball.get_pos() for _ in range(length)]

    def _is_ball_in(self, robo: rbt.Robot) -> bool:
        """
        Определить, находится ли мяч внутри дрибблера
        """
        return (robo.get_pos() - self.ball.get_pos()).mag() < const.BALL_GRABBED_DIST and abs(
            aux.wind_down_angle((self.ball.get_pos() - robo.get_pos()).arg() - robo.get_angle())
        ) < const.BALL_GRABBED_ANGLE

    def is_ball_in(self, robo: rbt.Robot) -> bool:
        """
        Определить, находится ли мяч внутри дрибблера
        """
        return robo == self.robot_with_ball

    def update_blu_robot(self, idx: int, pos: aux.Point, angle: float, t: float) -> None:
        """
        Обновить положение робота синей команды
        !!! Вызывать один раз за итерацию с постоянной частотой !!!
        """
        self.b_team[idx].update(pos, angle, t)

    def update_yel_robot(self, idx: int, pos: aux.Point, angle: float, t: float) -> None:
        """
        Обновить положение робота желтой команды
        !!! Вызывать один раз за итерацию с постоянной частотой !!!
        """
        self.y_team[idx].update(pos, angle, t)

    def get_blu_team(self) -> list[rbt.Robot]:
        """
        Получить массив роботов синей команды

        @return Массив entity.Entity[]
        """
        return self.b_team

    def get_yel_team(self) -> list[rbt.Robot]:
        """
        Получить массив роботов желтой команды

        @return Массив entity.Entity[]
        """
        return self.y_team

    def is_ball_stop_near_goal(self) -> bool:
        """
        Определить, остановился ли мяч в штрафной зоне
        """
        return aux.is_point_inside_poly(self.ball.get_pos(), self.ally_goal.hull) and not self.is_ball_moves()

    def is_ball_moves(self) -> bool:
        """
        Определить, движется ли мяч
        """
        return self.ball.get_vel().mag() > const.INTERCEPT_SPEED

    def is_ball_moves_to_point(self, point: aux.Point) -> bool:
        """
        Определить, движется ли мяч в сторону точки
        """
        vec_to_point = point - self.ball.get_pos()
        return (
            self.ball.get_vel().mag() * (cos(vec_to_point.arg() - self.ball.get_vel().arg()) ** 5)
            > const.INTERCEPT_SPEED * 10
            and self.robot_with_ball is None
            and vec_to_point.arg() - self.ball.get_vel().arg() < pi / 2
        )

    def is_ball_moves_to_goal(self) -> bool:
        """
        Определить, движется ли мяч в сторону ворот
        """
        inter = aux.get_line_intersection(
            self.ally_goal.up,
            self.ally_goal.down,
            self.ball.get_pos(),
            self.ball.get_pos() + self.ball.get_vel(),
            "SR",
        )
        return (
            inter is not None
            and self.is_ball_moves()
            # or self.is_ball_moves_to_point(self.ally_goal.up)
            # or self.is_ball_moves_to_point(self.ally_goal.down)
        )


def find_nearest_robot(point: aux.Point, team: list[rbt.Robot], avoid: Optional[list[rbt.Robot]] = None) -> rbt.Robot:
    """
    Найти ближайший робот из массива team к точке point, игнорируя точки avoid
    """
    if avoid is None:
        avoid = []
    robo_id = -1
    min_dist = 10e10

    for i, player in enumerate(team):
        if player in avoid or not player.is_used():
            continue
        if aux.dist(point, player.get_pos()) < min_dist:
            min_dist = aux.dist(point, player.get_pos())
            robo_id = i
    return team[robo_id]


def find_nearest_robots(
    point: aux.Point,
    team: list[rbt.Robot],
    num: Optional[int] = None,
    avoid: Optional[list[rbt.Robot]] = None,
) -> list[rbt.Robot]:
    """
    Найти num роботов из team, ближайших к точке point
    """
    if num is None:
        num = len(team)
    if avoid is None:
        avoid = []

    robot_dist: list[tuple[rbt.Robot, float]] = []

    for robot in team:  # in [field.enemies, field.allies]
        dist = (robot.get_pos() - point).mag()
        if robot.is_used() and robot not in avoid:
            robot_dist.append((robot, dist))

    sorted_robot_dist = sorted(robot_dist, key=lambda x: x[1])

    sorted_robots: list[rbt.Robot] = [rbt_dst[0] for rbt_dst in sorted_robot_dist]

    return sorted_robots[: min(len(sorted_robots), num)]


def exclude(team: list[rbt.Robot], avoid: list[rbt.Robot]) -> list[rbt.Robot]:
    """
    Возвращает team без роботов avoid
    """
    new_team = []
    for i in team:
        if i not in avoid:
            new_team.append(i)
    return new_team


def get_active_bots(team: list[rbt.Robot]) -> list[rbt.Robot]:
    """
    Возвращает используемых роботов команды
    """
    active_bots = []
    for r in team:
        if r.is_used():
            active_bots.append(r)
    return active_bots

def gate_angle_size(main_p: aux.Point, field: Field, enemy_goal: bool = True, flag: bool = False) -> list:
    """
    функция, определяющая оптимальный угол для удара по воротам соперников из данной точки
    """
    if enemy_goal:
        now_rbts = get_active_bots(field.enemies)
        now_goal = field.enemy_goal
    else:
        now_rbts = exclude(get_active_bots(field.allies), [field.allies[const.GK]])
        now_goal = field.ally_goal
    angle_point = aux.closest_point_on_line(now_goal.up, now_goal.down, main_p, is_inf="L")
    gates: list[list[float]] = sorted(
        [
            [
                aux.get_angle_between_points(angle_point, main_p, now_goal.down),
                aux.get_angle_between_points(angle_point, main_p, now_goal.up),
            ]
        ]
    )
    minuses = []
    for now_rbt in now_rbts:
        tangents = aux.get_tangent_points(now_rbt.get_pos(), main_p, const.ROBOT_R + const.BALL_R)
        if len(tangents) > 1:
            minus: list[float] = sorted(
                [
                    aux.get_angle_between_points(angle_point, main_p, tangents[0]),
                    aux.get_angle_between_points(angle_point, main_p, tangents[1]),
                ]
            )
            if minus[1] - minus[0] < pi:
                minuses.append(minus)

    gates = sorted(aux.range_minus(gates, minuses), key=lambda a: a[1] - a[0], reverse=True)
    if not gates:
        cick_angle = aux.get_angle_between_points(angle_point, main_p, now_goal.center)
        gates = [[0, 0]]
    else:
        cick_angle = (gates[0][1] + gates[0][0]) / 2
    cick_angle = aux.wind_down_angle(cick_angle + aux.angle_to_point(aux.Point(0, 0), now_goal.center))
    summ_angle = reduce((lambda x, y: [x[0] + y[0], x[1] + y[1]]), gates)
    if flag:
        for gate in gates:
            print(gate[1], gate[0], '', end='')
        print(summ_angle[1], summ_angle[0])
    return [cick_angle, gates[0][1] - gates[0][0], summ_angle[1] - summ_angle[0]]
