"""
Структура для хранения информации об одном объекте на поле (робот или мяч)

Хранит:
- положение
- скорость
- ускорение
- угол
- радиус
"""

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.tau as tau


class Entity:
    """
    Класс для описания геометрического объекта на поле

    Хранит положение, скорость, угол и тп.
    """

    def __init__(self, pos: aux.Point, angle: float, R: float) -> None:
        """
        Конструктор

        @param pos Изначальное положение объекта. Тип: aux.Point
        @param angle Угол поворота объекта [рад]
        @param R Радиус объекта [м]
        """
        T = 0.05
        Ts = const.Ts

        self._pos = pos
        self._vel = aux.Point(0, 0)
        self._vel_fx = tau.FOD(T, Ts)
        self._vel_fy = tau.FOD(T, Ts)
        self._acc = aux.Point(0, 0)
        self._acc_fx = tau.FOD(T, Ts)
        self._acc_fy = tau.FOD(T, Ts)
        self._angle = angle
        self._anglevel = 0.0
        self._vel_fr = tau.FOD(T, Ts, True)
        self._radius = R
        self.last_update_ = 0.0

    def update(self, pos: aux.Point, angle: float, t: float) -> None:
        """
        Обновить положение и рассчитать исходя из этого скорость и ускорение
        !!! Вызывать один раз за итерацию с постоянной частотой !!!

        TODO Реализовать расчет скоростей и ускорений
        """
        self._pos = pos
        self._angle = angle
        self._vel.x = self._vel_fx.process(self._pos.x)
        self._vel.y = self._vel_fy.process(self._pos.y)
        self._acc.x = self._acc_fx.process(self._vel.x)
        self._acc.y = self._acc_fy.process(self._vel.y)
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

    def get_acc(self) -> aux.Point:
        """Геттер ускорения"""
        return self._acc

    def get_angle(self) -> float:
        """Геттер угла"""
        return self._angle

    def get_radius(self) -> float:
        """Геттер радиуса"""
        return self._radius

    def __str__(self) -> str:
        """Для print"""
        return str(self._pos)
