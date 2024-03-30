"""
Описание полей и интерфейсов взаимодействия с роботом
"""
import math
import typing

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.entity as entity
import bridge.processors.tau as tau
import bridge.processors.waypoint as wp


class Robot(entity.Entity):
    """
    Описание робота
    """

    def __init__(self, pos: aux.Point, angle: float, R: float, color: str, r_id: int, ctrl_id: int) -> None:
        super().__init__(pos, angle, R)

        self.r_id = r_id
        self.ctrl_id = ctrl_id
        self.__is_used = 0
        self.color = color
        self.last_update_ = 0.0

        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_r = 0.0
        self.kick_up_ = 0
        self.kick_forward_ = 0
        self.auto_kick_ = 0
        self.kicker_voltage_ = 0
        self.dribbler_enable_ = 0
        self.dribbler_speed_ = 0
        self.kicker_charge_enable_ = 1
        self.beep = 0
        self.role = 0

        # v! SIM
        if const.IS_SIMULATOR_USED:
            self.k_xx = -833 / 20
            self.k_yy = 833 / 20
            self.k_ww = 1.25 / 20
            self.k_wy = -0.001
            self.t_wy = 0.15
            self.r_comp_f_dy = tau.FOD(self.t_wy, const.Ts)
            self.r_comp_f_fy = tau.FOLP(self.t_wy, const.Ts)

        # v! REAL
        else:
            self.k_xx = -250 / 20
            self.k_yy = 250 / 20
            self.k_ww = 6 / 20
            self.k_wy = 0
            self.t_wy = 0.15
            self.r_comp_f_dy = tau.FOD(self.t_wy, const.Ts)
            self.r_comp_f_fy = tau.FOLP(self.t_wy, const.Ts)

        # self.max_acc = 1000
        # self.xxRL = tau.RateLimiter(const.Ts, self.max_acc)
        # self.yyRL = tau.RateLimiter(const.Ts, self.max_acc)
        self.xx_t = 0.2
        self.xx_flp = tau.FOLP(self.xx_t, const.Ts)
        self.yy_t = 0.2
        self.yy_flp = tau.FOLP(self.yy_t, const.Ts)
        # self.a0TF = 0.5
        # self.a0Flp = tau.FOLP(self.a0TF, const.Ts)

        # !v REAL
        # if self.r_id != const.GK:
        gains_full = [16, 0.6, 0, const.MAX_SPEED]  #3, 0.3
        gains_soft = [16, 0.6, 0, const.SOFT_MAX_SPEED]
        # gains_soft = gains_full
        a_gains_full = [7, 0.31, 0.1, const.MAX_SPEED_R]
        a_gains_soft = [7, 0.08, 4, const.SOFT_MAX_SPEED_R]
        # a_gains_soft = a_gains_full
        # else:
        #     gains_full = [6, 0.8, 0, const.MAX_SPEED]
        #     gains_soft = [6, 1, 0.1, const.SOFT_MAX_SPEED]
        #     a_gains_full = [6, 0.1, 0, const.MAX_SPEED_R]
        #     a_gains_soft = [2, 0.07, 1, const.SOFT_MAX_SPEED_R]

        # !v SIM
        # gains_full = [2, 0.3, 0, const.MAX_SPEED]
        # gains_soft = [0.5, 0.1, 0, const.SOFT_MAX_SPEED]
        # a_gains_full = [2, 0.1, 0.1, const.MAX_SPEED_R]
        # a_gains_soft = [1, 0.07, 0, const.SOFT_MAX_SPEED_R]

        self.pos_reg = tau.PISD(
            const.Ts,
            [gains_full[0], gains_soft[0]],
            [gains_full[1], gains_soft[1]],
            [gains_full[2], gains_soft[2]],
            [gains_full[3], gains_soft[3]],
        )
        self.angle_reg = tau.PISD(
            const.Ts,
            [a_gains_full[0], a_gains_soft[0]],
            [a_gains_full[1], a_gains_soft[1]],
            [a_gains_full[2], a_gains_soft[2]],
            [a_gains_full[3], a_gains_soft[3]],
        )

        self.is_kick_commited = False

    def used(self, a: int) -> None:
        """
        Выставить флаг использования робота
        """
        self.__is_used = a

    def is_used(self) -> int:
        """
        Узнать, используется ли робот
        """
        return self.__is_used

    def last_update(self) -> float:
        """
        Получить время последнего обновления робота
        """
        return self.last_update_

    def update(self, pos: aux.Point, angle: float, t: float) -> None:
        """
        Обновить состояние робота согласно SSL Vision
        """
        super().update(pos, angle, t)
        self.kick_forward_ = 0
        self.kick_up_ = 0
        self.last_update_ = t

    def kick_forward(self) -> None:
        """
        Ударить вперед
        """
        self.kick_forward_ = 1

    def kick_up(self) -> None:
        """
        Ударить вверх
        """
        self.kick_up_ = 1

    def copy_control_fields(self, robot: "Robot") -> None:
        """
        Скопировать в данный робот поля управления робота robot
        """
        self.speed_x = robot.speed_x
        self.speed_y = robot.speed_y
        self.speed_r = robot.speed_r
        self.kick_up_ = robot.kick_up_
        self.kick_forward_ = robot.kick_forward_
        self.auto_kick_ = robot.auto_kick_
        self.kicker_voltage_ = robot.kicker_voltage_
        self.dribbler_enable_ = robot.dribbler_enable_
        self.dribbler_speed_ = robot.dribbler_speed_
        self.kicker_charge_enable_ = robot.kicker_charge_enable_
        self.beep = robot.beep

    def clear_fields(self) -> None:
        """
        Очистить поля управления
        """
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_r = 0.0
        self.kick_up_ = 0
        self.kick_forward_ = 0
        self.auto_kick_ = 0
        self.kicker_voltage_ = 0
        self.dribbler_enable_ = 0
        self.dribbler_speed_ = 0
        self.kicker_charge_enable_ = 0
        self.beep = 0

    def is_kick_aligned(self, target: wp.Waypoint) -> bool:
        """
        Определить, выровнен ли робот относительно путевой точки target
        """
        # print(round((self.get_pos() - target.pos).mag(), 2),
        #       const.KICK_ALIGN_DIST*const.KICK_ALIGN_DIST_MULT, \
        #       round(abs(aux.wind_down_angle(self._angle - target.angle)), 2),
        #       const.KICK_ALIGN_ANGLE, \
        #     # round(abs(aux.vec_mult(
        #           aux.rotate(aux.i, target.angle), target.pos - self._pos)), 2),
        #           const.KICK_ALIGN_OFFSET)
        #       round(aux.dist(aux.closest_point_on_line(
        #           target.pos,
        #           target.pos - aux.rotate(aux.i, target.angle)*const.KICK_ALIGN_DIST, self._pos),
        #           self._pos)),
        #           const.KICK_ALIGN_OFFSET)
        # print(aux.i, target.angle, aux.rotate(aux.i, target.angle), target.pos,
        #                                       self._pos, target.pos - self._pos)

        commit_scale = 1.2 if self.is_kick_commited else 1
        is_dist = (self.get_pos() - target.pos).mag() < const.KICK_ALIGN_DIST * const.KICK_ALIGN_DIST_MULT * commit_scale
        is_angle = abs(aux.wind_down_angle(self._angle - target.angle)) < const.KICK_ALIGN_ANGLE * commit_scale
        is_offset = (
            aux.dist(
                aux.closest_point_on_line(
                    target.pos, target.pos - aux.rotate(aux.RIGHT, target.angle) * const.KICK_ALIGN_DIST, self._pos
                ),
                self._pos,
            )
            < const.KICK_ALIGN_OFFSET * commit_scale
        )
        is_aligned = is_dist and is_angle and is_offset

        if is_aligned:
            self.is_kick_commited = True
        else:
            self.is_kick_commited = False
        # print(is_dist, is_angle, is_offset)
        return is_aligned

    def update_vel_xyw(self, vel: aux.Point, wvel: float) -> None:
        """
        Выполнить тик низкоуровневых регуляторов скорости робота

        vel - требуемый вектор скорости [мм/с] \\
        wvel - требуемая угловая скорость [рад/с]
        """
        self.speed_x = self.xx_flp.process(1 / self.k_xx * aux.rotate(vel, -self._angle).x)
        self.speed_y = self.yy_flp.process(1 / self.k_yy * aux.rotate(vel, -self._angle).y)

        # RcompY = self.Kwy * self.RcompFfy.process(self.RcompFdy.process(self.speed_y))
        # RcompY = self.Kwy * self.RcompFdy.process(abs(float(self.speed_y)**2))
        r_comp_y = 0
        self.speed_r = 1 / self.k_ww * (wvel - r_comp_y)
        if abs(self.speed_r) > const.MAX_SPEED_R:
            self.speed_r = const.MAX_SPEED_R * abs(self.speed_r) / self.speed_r

        vec_speed = math.sqrt(self.speed_x**2 + self.speed_y**2)
        r_speed = abs(self.speed_r)

        vec_speed *= ((const.MAX_SPEED_R - r_speed) / const.MAX_SPEED_R) ** 4
        ang = math.atan2(self.speed_y, self.speed_x)
        self.speed_x = vec_speed * math.cos(ang)
        self.speed_y = vec_speed * math.sin(ang)

    def clamp_motors(self) -> None:
        """
        Ограничить управляющее воздействие
        """
        # TODO

    def __str__(self) -> str:
        return (
            str(
                str(self.color)
                + " "
                + str(self.r_id)
                + " "
                + str(self.get_pos())
                + " "
                + str(self.speed_x)
                + " "
                + str(self.speed_y)
            )
            + " "
            + str(self.speed_r)
        )


def find_nearest_robot(robo: aux.Point, team: list[Robot], avoid: typing.Optional[list[int]] = None) -> Robot:
    """
    Найти ближайший робот из массива team к точке robot, игнорируя точки avoid
    """
    if avoid is None:
        avoid = []
    robo_id = -1
    min_dist = 10e10
    for i, player in enumerate(team):
        if i in avoid or not player.is_used():
            continue
        if aux.dist(robo, player.get_pos()) < min_dist:
            min_dist = aux.dist(robo, player.get_pos())
            robo_id = i
    return team[robo_id]


def probability(inter: list[aux.Point], bots: list[Robot], pos: aux.Point) -> float:
    """
    TODO написать доку
    """
    res = 1.0
    # print(len(inter), end = ' ')
    for i, intr in enumerate(inter):
        # koef = 1
        # print([inter[i].x, inter[i].y, bots[i].get_pos().x, bots[i].get_pos().y])
        tmp_res_x = aux.dist(intr, bots[i].get_pos())
        tmp_res_y = math.sqrt(aux.dist(pos, bots[i].get_pos()) ** 2 - tmp_res_x**2)
        ang = math.atan2(tmp_res_y, tmp_res_x)
        # abs(ang) < math.pi / 4
        if abs(ang) > math.pi / 2:
            continue
        # print(tmpRes)
        # if tmpResX < 0:
        #     koef = 0
        # elif tmpResX > const.ROBOT_R * 100 * 15:
        #     koef = 1
        # else:
        #     koef = tmpResX / (const.ROBOT_R * 100 * 15)
        # res *= (2 * abs(ang) / math.pi) * (dist(st, bots[i].get_pos()) / 54e6)
        res *= 1 / (2 * abs(ang) / math.pi)
    return res


def bot_position(pos: aux.Point, vecx: float, vecy: float) -> aux.Point:
    """
    TODO написать доку
    """
    modul = (vecx**2 + vecy**2) ** (0.5)
    vecx = (vecx / modul) * const.ROBOT_R * 1000 * 2
    vecy = (vecy / modul) * const.ROBOT_R * 1000 * 2
    return aux.Point(pos.x - vecx, pos.y - vecy)


def shot_decision(pos: aux.Point, end: list[aux.Point], tobj: list[Robot]) -> aux.Point:
    """
    TODO написать доку
    """
    objs = tobj.copy()
    tmp_counter = 0
    for obj in range(len(objs)):
        if not objs[obj - tmp_counter].is_used():
            objs.pop(obj - tmp_counter)
            tmp_counter += 1
    # mx_shot_prob = 0
    # shot_point = pos
    mx = 0.0
    # tmp_sum = Point(0, 0)
    # n = 0
    # print(st)
    # for bot in obj:
    #     # print([bot.get_pos().x, bot.get_pos().y], end = " ")
    #     plt.plot(bot.get_pos().x, bot.get_pos().y, 'bo')
    # t = np.arange(-4500*1.0, 1000*1.0, 10)
    for point in end:  # checkai
        A = -(point.y - pos.y)
        B = point.x - pos.x
        C = pos.x * (point.y - pos.y) - pos.y * (point.x - pos.x)
        tmp_line = aux.BobLine(A, B, C)
        # plt.plot(t, (tmpLine.A*t + tmpLine.C)/tmpLine.B, 'g--')
        lines = []
        for bot in objs:
            tmp_c = -(B * bot.get_pos().x - A * bot.get_pos().y)
            line2 = aux.BobLine(B, -A, tmp_c)
            lines.append(line2)
        inter = aux.line_intersect(tmp_line, lines)
        # plt.plot(inter[0].x, inter[0].y, 'bx')
        # plt.plot(inter[1].x, inter[1].y, 'gx')
        tmp_prob = probability(inter, objs, pos)
        # print(tmp_prob, end = " ")
        if tmp_prob > mx:
            mx = tmp_prob
            point_res = point
            # shot_point = bot_position(pos, point.x - pos.x, point.y - pos.y)
        # if tmp_prob > mx:
        #     mx = tmp_prob
        #     shot_point = botPosition(st, point.x - st.x, point.y - st.y)
        #     point_res = point
        #     n = 1
        #     sum = point
        # elif tmp_prob == mx:
        #     sum += point
        #     n += 1
        # else:
        #     point_res = sum / n
        #     shot_point = botPosition(st, point_res.x - st.x, point_res.y - st.y)
        #     sum = Point(0, 0)
        #     n = 0
    # plt.plot(t, -(point_res.A*t + point_res.C)/point_res.B, 'r-')
    # plt.plot(shot_point.x, shot_point.y, 'r^')
    # plt.axis('equal')
    # plt.grid(True)
    # plt.show()
    return point_res
