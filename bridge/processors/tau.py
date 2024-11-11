"""
Динамические звенья и регуляторы
"""

import math
from enum import Enum, auto

import bridge.processors.auxiliary as aux


class FOD:
    """
    Реальное дифференцирующее звено первого порядка
    """

    def __init__(self, T: float, Ts: float, is_angle: bool = False) -> None:
        """
        Конструктор

        T - постоянная времени ФНЧ
        dT - период квантования
        """
        self._t = T
        self._ts = Ts
        self._int = 0.0
        self._out = 0.0
        self._is_angle = is_angle

    def process(self, x: float) -> float:
        """
        Рассчитать и получить следующее значение выхода звена

        ВЫЗЫВАТЬ РАЗ В ПЕРИОД КВАНТВАНИЯ

        x - новое значение входа
        """
        err = x - self._int
        if self._is_angle:
            if err > math.pi:
                err -= 2 * math.pi
                self._int += 2 * math.pi
            elif err < -math.pi:
                err += 2 * math.pi
                self._int -= 2 * math.pi
        self._out = err / self._t
        self._int += self._out * self._ts
        return self._out

    def get_val(self) -> float:
        """
        Получить последнее значение выхода звена без расчета
        """
        return self._out


class FOLP:
    """
    Фильтр низких частот первого порядка
    """

    def __init__(self, T: float, Ts: float) -> None:
        """
        Конструктор

        T - постоянная времени ФНЧ
        dT - период квантования
        """
        self._t = T
        self._ts = Ts
        self._int = 0.0
        self._out = 0.0

    def process(self, x: float) -> float:
        """
        Рассчитать и получить следующее значение выхода звена

        ВЫЗЫВАТЬ РАЗ В ПЕРИОД КВАНТВАНИЯ

        x - новое значение входа
        """
        err = x - self._out
        self._int += err * self._ts
        self._out = self._int / self._t
        return self._out

    def get_val(self) -> float:
        """
        Получить последнее значение выхода звена без расчета
        """
        return self._out


class Integrator:
    """
    Интегратор
    """

    def __init__(self, Ts: float) -> None:
        """
        Конструктор

        dT - период квантования
        """
        self._ts = Ts
        self._int = 0.0
        self._out = 0.0

    def reset(self) -> None:
        """
        Сбросить значение интегратора
        """
        self._int = 0

    def process(self, x: float) -> float:
        """
        Рассчитать и получить следующее значение выхода звена

        ВЫЗЫВАТЬ РАЗ В ПЕРИОД КВАНТВАНИЯ

        x - новое значение входа
        """
        self._int += x * self._ts
        self._out = self._int
        return self._out

    def get_val(self) -> float:
        """
        Получить последнее значение выхода звена без расчета
        """
        return self._out


class Mode(Enum):
    """
    Названия наборов коэффициентов регулятора
    """

    NORMAL = 0
    SOFT = auto()


class PISD:
    """
    Пропорционально-скользяще-интегральный регулятор

    (В отличие от ПИД беред производную от скорости изменения регулируемой
    величины, а не ошибки)
    """

    def __init__(
        self,
        dT: float,
        gain: list[float],
        kd: list[float],
        ki: list[float],
        max_out: list[float],
    ) -> None:
        """
        Конструктор

        каждый параметр - список коэффициентов для разных режимов
        gain - коэффициент усиления регулятора (П составляющая)
        kd - коэффициент дифференциальной части (типа Д составляющая)
        ki - коэффициент интегрирующей части (И составляющая)
        max_out - Максимальное значение управляющего воздействия
        """
        self.__gain = gain
        self.__kd = kd
        self.__ki = ki
        self.__max_out = max_out
        self.__int = Integrator(dT)
        self.__out = 0.0
        self.__mode = Mode.NORMAL

    def select_mode(self, mode: Mode) -> None:
        """
        Выбрать набор коэффициентов регулятора
        """
        self.__mode = mode
        self.__int.reset()

    def __get_gains(self) -> tuple[float, float, float, float]:
        """
        Получить коэффициенты регулятора
        """
        return (
            self.__gain[self.__mode.value],
            self.__kd[self.__mode.value],
            self.__ki[self.__mode.value],
            self.__max_out[self.__mode.value],
        )

    def process(self, xerr: float, x_i: float) -> float:
        """
        Рассчитать следующий тик регулятора
        """
        gain, k_d, k_i, max_out = self.__get_gains()

        s = xerr + k_d * x_i + k_i * self.__int.get_val()
        u = gain * s

        u_clipped = aux.minmax(u, max_out)

        if u != u_clipped:
            self.__int.process(xerr + k_d * x_i)

        self.__out = u_clipped
        return self.__out

    def get_val(self) -> float:
        """
        Получить последнее значение выхода звена без расчета
        """
        return self.__out


class RateLimiter:
    """
    Ограничитель скорости роста
    """

    def __init__(self, Ts: float, max_der: float) -> None:
        """
        Конструктор
        """
        self.__out = 0.0
        self.__int = Integrator(Ts)
        self.__k = 1 / Ts
        self.__max_der = max_der

    def process(self, x: float) -> float:
        """
        Рассчитать следующий тик звена
        """
        u = aux.minmax(self.__k * (x - self.__out), self.__max_der)
        self.__out = self.__int.process(u)
        return self.__out

    def get_val(self) -> float:
        """
        Получить последнее значение выхода звена без расчета
        """
        return self.__out
