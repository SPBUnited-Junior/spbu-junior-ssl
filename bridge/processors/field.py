"""
Модуль описания структуры Field для хранения информации об объектах на поле (роботы и мяч)
"""
import bridge.processors.entity as entity
import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.robot as robot

class Goal:
    """
    Структура, описывающая ключевые точки ворот
    """
    def __init__(self, goal_dx, goal_dy, goal_pen) -> None:

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
        self.hull = [self.up + self.eye_up * const.GOAL_BOUND_OFFSET,
                     self.forwup + (self.eye_forw + self.eye_up) * const.GOAL_BOUND_OFFSET,
                     self.forw + (self.eye_forw) * const.GOAL_BOUND_OFFSET,
                     self.forwdown + (self.eye_forw - self.eye_up) * const.GOAL_BOUND_OFFSET,
                     self.down - self.eye_up * const.GOAL_BOUND_OFFSET,
                     const.GRAVEYARD_POS*self.eye_forw.x]

        # Попуск
        self.popusk_positions = [aux.Point(0, 0), aux.Point(0, 2000), aux.Point(0, -2000), aux.Point(aux.sign(goal_dx) * 2000, 2000), aux.Point(aux.sign(goal_dx) * 2000, -2000)]

class Field:
    """
    Класс, хранящий информацию о всех объектах на поле и ключевых точках
    """
    def __init__(self, ctrl_mapping, ally_color = 'b') -> None:
        """
        Конструктор
        Инициализирует все нулями
        
        TODO Сделать инициализацию реальными параметрами для корректного
        определения скоростей и ускорений в первые секунды
        """
        self.ally_color = ally_color
        self.ball = entity.Entity(const.GRAVEYARD_POS, 0, const.BALL_R)
        self.b_team = [ robot.Robot(const.GRAVEYARD_POS, 0, const.ROBOT_R, 'b', i, ctrl_mapping[i]) for i in range(const.TEAM_ROBOTS_MAX_COUNT)]
        self.y_team = [ robot.Robot(const.GRAVEYARD_POS, 0, const.ROBOT_R, 'y', i, ctrl_mapping[i]) for i in range(const.TEAM_ROBOTS_MAX_COUNT)]
        self.all_bots = [*self.b_team, *self.y_team]
        self.y_goal = Goal(const.GOAL_DX, const.GOAL_DY, const.GOAL_PEN)
        self.b_goal = Goal(-const.GOAL_DX, -const.GOAL_DY, -const.GOAL_PEN)

        if ally_color == 'b':
            self.allies = [*self.b_team]
            self.ally_goal = self.b_goal
            self.enemies = [*self.y_team]
            self.enemy_goal = self.y_goal
            self.side = -const.POLARITY # TODO УДАЛИТЬ АААААААААААААААААААААА
        elif ally_color == 'y':
            self.allies = [*self.y_team]
            self.ally_goal = self.y_goal
            self.enemies = [*self.b_team]
            self.enemy_goal = self.b_goal
            self.side = const.POLARITY # TODO УДАЛИИИИТЬ

    def updateBall(self, pos):
        """
        Обновить положение мяча
        !!! Вызывать один раз за итерацию с постоянной частотой !!!
        """
        self.ball.update(pos, 0)

    def updateBluRobot(self, idx, pos, angle, t):
        """
        Обновить положение робота синей команды
        !!! Вызывать один раз за итерацию с постоянной частотой !!!
        """
        self.b_team[idx].update(pos, angle, t)

    def updateYelRobot(self, idx, pos, angle, t):
        """
        Обновить положение робота желтой команды
        !!! Вызывать один раз за итерацию с постоянной частотой !!!
        """
        self.y_team[idx].update(pos, angle, t)

    def getBall(self):
        """
        Получить объект мяча

        @return Объект entity.Entity
        """
        return self.ball

    def getBluTeam(self):
        """
        Получить массив роботов синей команды

        @return Массив entity.Entity[]
        """
        return self.b_team

    def getYelTeam(self):
        """
        Получить массив роботов желтой команды

        @return Массив entity.Entity[]
        """
        return self.y_team

    def isBallInGoalSq(self):
        """
        Определить, находится ли мяч в штрафной зоне
        """
        # print(self.ally_goal.center.x - self.ball.getPos().x, self.ball.getPos().x - self.ally_goal.forw.x,
        #     self.ally_goal.up.y - self.ball.getPos().y, self.ball.getPos().y - self.ally_goal.down.y)
        return aux.sign(self.ally_goal.center.x - self.ball.getPos().x) == aux.sign(self.ball.getPos().x - self.ally_goal.forw.x) and \
            aux.sign(self.ally_goal.up.y - self.ball.getPos().y) == aux.sign(self.ball.getPos().y - self.ally_goal.down.y)

    def is_ball_moves_to_goal(self) -> bool:
        """
        Определить, движется ли мяч в сторону ворот
        """
        return (
            self.ball.get_vel().mag() > const.GK_INTERCEPT_SPEED
        )  # and self.ball._vel.x / self.ally_goal.center.x > 0 нужно протестить