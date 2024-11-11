"""
Описание структуры путевой точки

Хранит:
- Координата путевой точки
- Желаемый угол поворота робота в точке
- Тип путевой точки
"""

from enum import Enum

import bridge.processors.auxiliary as aux


class WType(Enum):
    """
    Типы путевых точек
    """

    T_ROBOT = -1  # хз, нужен в Route
    S_ENDPOINT = 0  # Двигаться в точку и остановиться (конечная точка пути)
    S_IGNOREOBSTACLES = 1  # Двигаться в точку игнорируя препятствия
    S_BALL_GO = 2  # Двигаться с мячом
    S_BALL_KICK = 3  # Захватить мяч и мгновенно его пнуть
    S_BALL_GRAB = 4  # Захватить мяч не пиная
    S_KEEP_BALL_DISTANCE = 5
    S_STOP = 6  # Kostil for stopping
    S_BALL_KICK_UP = 7  # Захватить мяч и мгновенно его пнуть up
    S_VELOCITY = 8  # Разворачиваться с мячом
    # waypoint.pos - скорость; waypoint.angle - угловая скорость
    S_BALL_PASS = 9  # Give pass to point

    R_PASSTHROUGH = 10  # Двигаться в точку не останавливаясь (промежуточная точка пути)
    R_BALL_ALIGN = 11  # Выровняться напротив мяча, приготовившись его захватить
    R_IGNORE_GOAl_HULL = 12

    T_GRAVEYARD = 100


class Waypoint:
    """
    Описание путевой точки
    """

    def __init__(self, pos: aux.Point, angle: float, wp_type: WType) -> None:
        """
        Конструктор
        """
        self.pos = pos
        self.angle = angle
        self.type = wp_type

    def __str__(self) -> str:
        return f"WP:  {str(self.pos)}; angle = {self.angle:0.2f}; type =  {str(self.type)}"
