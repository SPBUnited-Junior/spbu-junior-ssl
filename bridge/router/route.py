"""
Класс, описывающий текущий маршрут робота
"""

import math

import bridge.router.waypoint as wp
from bridge import const
from bridge.auxiliary import aux, fld, rbt, tau


class Route:
    """
    Класс описание произвольного маршрута
    """

    def __init__(self, rbt: rbt.Robot):
        """
        Конструктор
        """
        self._robot = [wp.Waypoint(rbt.get_pos(), rbt._angle, wp.WType.T_ROBOT)]
        self._destination = [wp.Waypoint(aux.GRAVEYARD_POS, 0, wp.WType.T_GRAVEYARD)]
        self._routewp: list[wp.Waypoint] = []
        # self.__route = [*self.robot, *self.__routewp, *self.__destination]

        self.go_flag = 0
        self.go_time = 0

    def update(self, rbt: rbt.Robot) -> None:
        """
        Обновить маршрут

        Обновляет текущее положение робота в маршрутной карте
        """
        self._robot = [wp.Waypoint(rbt.get_pos(), rbt.get_angle(), wp.WType.T_ROBOT)]

    def clear(self) -> None:
        """
        Очистить промежуточные точки маршрута
        """
        self._routewp = []

    def __get_route(self) -> list[wp.Waypoint]:
        """
        Получить маршрут в виде списка путевых точек
        """
        return [*self._robot, *self._routewp, *self._destination]

    def set_dest_wp(self, dest: wp.Waypoint) -> None:
        """
        Задать конечную точку
        """
        self._destination = [dest]

    def get_dest_wp(self) -> wp.Waypoint:
        """
        Получить конечную точку
        """
        return self._destination[0]

    def get_next_wp(self) -> wp.Waypoint:
        """
        Получить следующую путевую точку
        """
        return self.__get_route()[1]

    def get_next_segment(self) -> list[wp.Waypoint]:
        """
        Получить следующий сегмент маршрута в виде списка двух точек
        """
        return self.__get_route()[0:1]

    def get_next_vec(self) -> aux.Point:
        """
        Получить следующий сегмент маршрута в виде вектора
        """
        return self.__get_route()[1].pos - self.__get_route()[0].pos

    def get_next_angle(self) -> float:
        """
        Получить угол следующей путевой точки
        """
        return self.__get_route()[1].angle

    def get_next_type(self) -> wp.WType:
        """
        Получить тип следующей путевой точки
        """
        return self.__get_route()[1].type

    def insert_wp(self, wpt: wp.Waypoint) -> None:
        """
        Вставить промежуточную путевую точку в начало маршрута
        """
        self._routewp.insert(0, wpt)

    def is_used(self) -> bool:
        """
        Определить, используется ли маршрут
        """
        return self._destination[0].type != wp.WType.T_GRAVEYARD

    def get_length(self) -> float:
        """
        Получить длину маршрута
        """
        dist = 0.0
        last_wp_pos = self._robot[0].pos
        for wpt in self.__get_route():
            if wpt.type in [
                wp.WType.S_BALL_GO,
                wpt.type == wp.WType.S_BALL_KICK,
                wpt.type == wp.WType.S_BALL_GRAB,
                wpt.type == wp.WType.S_BALL_KICK_UP,
                wpt.type == wp.WType.S_BALL_PASS,
            ]:
                break
            dist += (wpt.pos - last_wp_pos).mag()
            last_wp_pos = wpt.pos
        return dist

    def __str__(self) -> str:
        strin = "ROUTE: "
        for wpt in self.__get_route():
            strin += " ->\n" + str(wpt)
        # for wp in [*self.robot, *self.__routewp, *self.__destination]:
        #     strin += " -> " + str(wp)
        return strin

    def go_route(self, rbt: rbt.Robot, field: fld.Field) -> None:
        """
        Двигаться по маршруту route
        """
        target_point = self.get_next_wp()

        rbt.kicker_charge_enable_ = 1

        if target_point.type == wp.WType.S_VELOCITY:  # and self.go_flag == 0:
            wvel = target_point.angle
            vel = target_point.pos
            rbt.speed_x = rbt.xx_flp.process(1 / rbt.k_xx * vel.x)
            rbt.speed_y = rbt.yy_flp.process(1 / rbt.k_yy * vel.y)
            rbt.speed_r = 1 / rbt.k_ww * wvel
            return

        cur_vel = rbt.get_vel()

        dist = self.get_length()

        end_point = self.get_dest_wp()

        vec_err = target_point.pos - rbt.get_pos()

        # #   NOTE: kostil!!!!!!!
        # if (dist > 1500):
        #     end_point.angle = aux.angle_to_point(rbt.get_pos(), end_point.pos)

        vel0 = (rbt.get_pos() - target_point.pos).unity()

        dangle = (target_point.pos - rbt.get_pos()).arg()
        rangle = aux.wind_down_angle(rbt.get_angle() - dangle)
        twpangle = aux.wind_down_angle(target_point.angle - dangle)

        angle60_abs = math.pi / 6 if abs(rangle) < math.pi / 2 else 2 * math.pi / 6
        angle60_sign = aux.sign(twpangle + rangle)

        angle60 = dangle + angle60_abs * angle60_sign

        lerp_angles = [target_point.angle, angle60]

        angle0 = aux.lerp(lerp_angles[0], lerp_angles[1], aux.minmax((dist - 100) / 1000, 0, 1))

        rbt.pos_reg_x.select_mode(tau.Mode.NORMAL)
        rbt.pos_reg_y.select_mode(tau.Mode.NORMAL)

        if (
            end_point.type
            in [
                wp.WType.S_BALL_KICK,
                wp.WType.S_BALL_KICK_UP,
                wp.WType.S_BALL_GRAB,
                wp.WType.S_BALL_GO,
                wp.WType.S_BALL_PASS,
            ]
        ) and dist < 750:
            rbt.pos_reg_x.select_mode(tau.Mode.SOFT)
            rbt.pos_reg_y.select_mode(tau.Mode.SOFT)

            if end_point.type == wp.WType.S_BALL_GO:
                angle0 = end_point.angle

            rbt.dribbler_enable_ = True
            rbt.dribbler_speed_ = 15
            if rbt.kicker_voltage_ == 0:
                rbt.kicker_voltage_ = const.VOLTAGE_SHOOT
                if end_point.type in [wp.WType.S_BALL_GRAB, wp.WType.S_BALL_GO]:
                    rbt.kicker_voltage_ = const.VOLTAGE_ZERO
                elif end_point.type == wp.WType.S_BALL_PASS:
                    rbt.kicker_voltage_ = const.VOLTAGE_PASS
                elif end_point.type == wp.WType.S_BALL_KICK_UP:
                    rbt.kicker_voltage_ = const.VOLTAGE_UP
        else:
            pass

        if (
            end_point.type
            in [
                wp.WType.S_BALL_KICK,
                wp.WType.S_BALL_KICK_UP,
                wp.WType.S_BALL_GRAB,
                wp.WType.S_BALL_PASS,
            ]
        ) and rbt.is_kick_aligned(end_point):
            # vel0 = (rbt.get_pos() - end_point.pos).unity()
            vel0 = -aux.rotate(aux.RIGHT, rbt.get_angle())
            # angle0 = end_point.angle
            angle0 = rbt.get_angle()

            transl_vel = vel0 * 300

            # if end_point.type == wp.WType.S_BALL_GRAB:
            #     transl_vel = vel0 * 200
            #     # if self.go_flag == 0:
            #     #     self.go_flag = 1
            #     #     self.go_time = time.time()

        else:
            u_x = -rbt.pos_reg_x.process(vec_err.x, -cur_vel.x)
            u_y = -rbt.pos_reg_y.process(vec_err.y, -cur_vel.y)
            # transl_vel = vel0 * u
            transl_vel = aux.Point(u_x, u_y)
            angle0 = end_point.angle

            if target_point.type == wp.WType.R_PASSTHROUGH:
                transl_vel = -vec_err.unity() * const.MAX_SPEED  # TODO: change speed by dist to final point
            if(target_point.type == wp.WType.S_SLOWDOWN):
                if(transl_vel.mag()>=const.MAX_STOP_SPEED):
                    transl_vel = transl_vel.unity()*const.MAX_STOP_SPEED
                    
        aerr = aux.wind_down_angle(angle0 - rbt.get_angle())

        ang_vel = rbt.angle_reg.process(aerr, -rbt.get_anglevel())

        # if self.go_flag == 1:  #NOTE: kostil
        #     if time.time() - self.go_time < 1:
        #         vel0 = -aux.rotate(aux.RIGHT, rbt.get_angle())
        #         transl_vel = vel0 * 200
        #         ang_vel = 0
        #     else:
        #         self.go_flag = 0

        if end_point.type == wp.WType.S_STOP:
            transl_vel = aux.Point(0, 0)
            ang_vel = 0.0

        # vel0 = -aux.rotate(aux.RIGHT, rbt.get_angle())
        # transl_vel = vel0 * 200
        # ang_vel = 0.7

        if (end_point.type in [wp.WType.S_BALL_KICK, wp.WType.S_BALL_PASS]) and rbt.is_kick_aligned_by_angle(
            end_point.angle
        ):
            if const.IS_SIMULATOR_USED:
                rbt.auto_kick_ = 1
            else:
                rbt.auto_kick_ = 2
        elif end_point.type == wp.WType.S_BALL_KICK_UP and rbt.is_kick_aligned_by_angle(end_point.angle):
            rbt.auto_kick_ = 1
        else:
            rbt.auto_kick_ = 0

        rbt.update_vel_xyw(transl_vel, ang_vel)

        # if rbt.r_id == 0:
        #     print("cord: ", int(rbt.get_pos().x), int(rbt.get_pos().y))
        #     print("speed:", int(transl_vel.mag()), rbt.get_vel().mag())
        #     print("ball:", field.ball.get_pos(), field.ball.get_vel())
