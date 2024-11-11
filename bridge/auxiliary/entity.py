"""
Структура для хранения информации об одном объекте на поле (робот или мяч)

Хранит:
- положение
- скорость
- ускорение
- угол
- радиус
"""

from time import time

import numpy as np
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter

from bridge import const
from bridge.auxiliary import aux, tau


class Entity:
    """
    Класс для описания геометрического объекта на поле

    Хранит положение, скорость, угол и тп.
    """

    def __init__(self, pos: aux.Point, angle: float, R: float, T: float = const.Ts) -> None:
        """
        Конструктор

        @param pos Изначальное положение объекта. Тип: aux.Point
        @param angle Угол поворота объекта [рад]
        @param R Радиус объекта [м]
        """
        # T = 0.05
        Ts = const.Ts

        self._pos = pos
        self._vel = aux.Point(0, 0)

        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.H = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])
        self.kf.R *= 0.001
        self.kf.P *= 900000.0

        self._angle = angle
        self._anglevel = 0.0
        self._vel_fr = tau.FOD(T, Ts, True)
        self._radius = R
        self.last_update_ = time()

    def update(self, pos: aux.Point, angle: float, t: float) -> None:
        """
        Обновить положение и рассчитать исходя из этого скорость и ускорение
        !!! Вызывать один раз за итерацию с постоянной частотой !!!

        TODO Реализовать расчет скоростей и ускорений
        """
        dt = t - self.last_update_
        self.kf.F = np.array([[1, dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, dt], [0, 0, 0, 1]])
        self.kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=35, block_size=2)
        self.kf.predict()
        self.kf.update(np.array([pos.x, pos.y]))
        state = self.kf.x.copy()
        self._pos = aux.Point(state[0].item(), state[2].item())
        self._vel = aux.Point(state[1].item(), state[3].item())

        self._pos = pos

        self._angle = angle
        self._anglevel = self._vel_fr.process(self._angle)
        self.last_update_ = t

    def last_update(self) -> float:
        """
        Получить время последнего обновления
        """
        return self.last_update_

    def get_pos(self) -> aux.Point:
        """Геттер положения"""
        return self._pos

    def get_anglevel(self) -> float:
        """Геттер скорости"""
        return self._anglevel

    def get_vel(self) -> aux.Point:
        """Геттер скорости"""
        return self._vel

    # def get_acc(self) -> aux.Point:
    #     """Геттер ускорения"""
    #     return self._acc

    def get_angle(self) -> float:
        """Геттер угла"""
        return self._angle

    def get_radius(self) -> float:
        """Геттер радиуса"""
        return self._radius

    def __str__(self) -> str:
        """Для print"""
        return str(self._pos)
