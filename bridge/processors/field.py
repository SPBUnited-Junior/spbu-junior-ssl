"""
Модуль описания структуры Field для хранения информации об объектах на поле (роботы и мяч)
"""

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.entity as entity
import bridge.processors.robot as robot


class Goal:
    """
    Структура, описывающая ключевые точки ворот
    """

    def __init__(self, goal_dx: float, goal_dy: float, goal_pen: float) -> None:

        # Абсолютный центр
        self.center = aux.Point(goal_dx, 0)

        # Относительные вектора
        self.eye_forw = aux.Point(-aux.sign(goal_dx), 0)
        self.eye_up = aux.Point(0, aux.sign(goal_dy))
        self.vup = aux.Point(0, goal_dy)
        self.vdown = aux.Point(0, -goal_dy)
        self.vpen = aux.Point(-goal_pen, 0)

        # Абсолютные вектора
        self.up = self.center + self.vup
        self.down = self.center + self.vdown
        self.forw = self.center + self.vpen
        self.forwup = self.forw + self.vup
        self.forwdown = self.forw + self.vdown

        # Оболочка штрафной зоны
        self.hull = [
            self.up + self.eye_up * const.GOAL_BOUND_OFFSET,
            self.forwup + (self.eye_forw + self.eye_up) * const.GOAL_BOUND_OFFSET,
            self.forw + (self.eye_forw) * const.GOAL_BOUND_OFFSET,
            self.forwdown + (self.eye_forw - self.eye_up) * const.GOAL_BOUND_OFFSET,
            self.down - self.eye_up * const.GOAL_BOUND_OFFSET,
            aux.GRAVEYARD_POS * self.eye_forw.x,
        ]

        # Попуск
        self.popusk_positions = [
            aux.Point(0, 0),
            aux.Point(0, 2000),
            aux.Point(0, -2000),
            aux.Point(aux.sign(goal_dx) * 2000, 2000),
            aux.Point(aux.sign(goal_dx) * 2000, -2000),
        ]


class Field:
    """
    Класс, хранящий информацию о всех объектах на поле и ключевых точках
    """

    def __init__(self, ctrl_mapping: dict[int, int], ally_color: str) -> None:
        """
        Конструктор
        Инициализирует все нулями

        TODO Сделать инициализацию реальными параметрами для корректного
        определения скоростей и ускорений в первые секунды
        """
        self.ally_color = ally_color
        if self.ally_color == "b":
            self.polarity = const.POLARITY * -1
        else:
            self.polarity = const.POLARITY
        self.ball = entity.Entity(aux.GRAVEYARD_POS, 0, const.BALL_R)
        self.b_team = [
            robot.Robot(aux.GRAVEYARD_POS, 0, const.ROBOT_R, "b", i, ctrl_mapping[i])
            for i in range(const.TEAM_ROBOTS_MAX_COUNT)
        ]
        self.y_team = [
            robot.Robot(aux.GRAVEYARD_POS, 0, const.ROBOT_R, "y", i, ctrl_mapping[i])
            for i in range(const.TEAM_ROBOTS_MAX_COUNT)
        ]
        self.all_bots = [*self.b_team, *self.y_team]
        self.ally_goal = Goal(const.GOAL_DX * self.polarity, const.GOAL_DY * self.polarity, const.GOAL_PEN * self.polarity)
        self.enemy_goal = Goal(
            -const.GOAL_DX * self.polarity, -const.GOAL_DY * self.polarity, -const.GOAL_PEN * self.polarity
        )

        if self.ally_color == "b":
            self.allies = [*self.b_team]
            self.enemies = [*self.y_team]
        elif self.ally_color == "y":
            self.allies = [*self.y_team]
            self.enemies = [*self.b_team]

    def update_ball(self, pos: aux.Point, t: float) -> None:
        """
        Обновить положение мяча
        !!! Вызывать один раз за итерацию с постоянной частотой !!!
        """
        self.ball.update(pos, 0, t)

    def is_ball_in(self, robo: robot.Robot) -> bool:
        """
        Определить, находится ли мяч внутри дриблера
        """
        return (robo.get_pos() - self.ball.get_pos()).mag() < const.BALL_GRABBED_DIST and abs(
            aux.wind_down_angle((self.ball.get_pos() - robo.get_pos()).arg() - robo.get_angle())
        ) < const.BALL_GRABBED_ANGLE

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

    def get_ball(self) -> entity.Entity:
        """
        Получить объект мяча

        @return Объект entity.Entity
        """
        return self.ball

    def get_blu_team(self) -> list[robot.Robot]:
        """
        Получить массив роботов синей команды

        @return Массив entity.Entity[]
        """
        return self.b_team

    def get_yel_team(self) -> list[robot.Robot]:
        """
        Получить массив роботов желтой команды

        @return Массив entity.Entity[]
        """
        return self.y_team

    def is_ball_stop_near_goal(self) -> bool:
        """
        Определить, находится ли мяч в штрафной зоне
        """
        # print(self.ally_goal.center.x - self.ball.get_pos().x,
        #       self.ball.get_pos().x - self.ally_goal.forw.x,
        #       self.ally_goal.up.y - self.ball.get_pos().y,
        #       self.ball.get_pos().y - self.ally_goal.down.y)
        return (
            aux.sign(self.ally_goal.center.x - self.ball.get_pos().x)
            == aux.sign(self.ball.get_pos().x - self.ally_goal.forw.x)
            and aux.sign(self.ally_goal.up.y - self.ball.get_pos().y)
            == aux.sign(self.ball.get_pos().y - self.ally_goal.down.y)
            and not self.is_ball_moves_to_goal()
        )

    def is_ball_moves_to_goal(self) -> bool:
        """
        Определить, движется ли мяч в сторону ворот
        """
        print("ball_vel", self.ball.get_vel())
        return (
            aux.scal_mult(self.ball.get_vel(), self.ally_goal.center - self.ball.get_pos().unity())
            > const.GK_INTERCEPT_SPEED
        )
