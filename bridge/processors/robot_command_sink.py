"""
Модуль-прослойка между стратегией и отправкой пакетов на роботов
"""
import struct
import typing

import attr
from strategy_bridge.bus import DataBus, DataReader, DataWriter
from strategy_bridge.common import config
from strategy_bridge.processors import BaseProcessor

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.robot as robot


@attr.s(auto_attribs=True)
class CommandSink(BaseProcessor):
    """
    Прослойка между стратегией и отправкой пакетов на роботов
    """

    processing_pause: typing.Optional[float] = 0.01
    max_commands_to_persist: int = 20
    commands_sink_reader: DataReader = attr.ib(init=False)
    commands_writer: DataWriter = attr.ib(init=False)

    b_control_team = [
        robot.Robot(aux.GRAVEYARD_POS, 0, const.ROBOT_R, "b", i, 0) for i in range(const.TEAM_ROBOTS_MAX_COUNT)
    ]
    y_control_team = [
        robot.Robot(aux.GRAVEYARD_POS, 0, const.ROBOT_R, "y", i, 0) for i in range(const.TEAM_ROBOTS_MAX_COUNT)
    ]

    def initialize(self, data_bus: DataBus) -> None:
        """
        Инициализация
        """
        super(CommandSink, self).initialize(data_bus)
        self.commands_sink_reader = DataReader(data_bus, const.TOPIC_SINK, 20)
        self.commands_writer = DataWriter(data_bus, config.ROBOT_COMMANDS_TOPIC, self.max_commands_to_persist)

    def process(self) -> None:
        """
        Метод обратного вызова процесса
        """

        cmds = self.commands_sink_reader.read_new()

        for cmd in cmds:
            r = cmd.content
            ctrl_id = r.ctrl_id

            if ctrl_id is None:
                continue

            if r.color == "b":
                self.b_control_team[ctrl_id].copy_control_fields(r)
            elif r.color == "y":
                self.y_control_team[ctrl_id].copy_control_fields(r)

        rules = self.get_rules()

        self.commands_writer.write(rules)

    def get_rules(self) -> bytes:
        """
        Сформировать массив команд для отправки на роботов
        """
        rules: list[float] = []

        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if abs(self.b_control_team[i].speed_x) < 1.5:
                self.b_control_team[i].speed_x = 0
            if abs(self.b_control_team[i].speed_y) < 1.5:
                self.b_control_team[i].speed_y = 0
            if abs(self.b_control_team[i].speed_r) < 1.5:
                self.b_control_team[i].speed_r = 0
            rules.append(0)
            rules.append(self.b_control_team[i].speed_x)
            rules.append(self.b_control_team[i].speed_y)
            rules.append(self.b_control_team[i].speed_r)
            rules.append(self.b_control_team[i].kick_forward_)
            rules.append(self.b_control_team[i].kick_up_)
            rules.append(self.b_control_team[i].auto_kick_)
            rules.append(self.b_control_team[i].kicker_voltage_)
            rules.append(self.b_control_team[i].dribbler_enable_)
            rules.append(self.b_control_team[i].dribbler_speed_)
            rules.append(self.b_control_team[i].kicker_charge_enable_)
            rules.append(self.b_control_team[i].beep)
            rules.append(0)

        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if abs(self.y_control_team[i].speed_x) < 1.5:
                self.y_control_team[i].speed_x = 0
            if abs(self.y_control_team[i].speed_y) < 1.5:
                self.y_control_team[i].speed_y = 0
            if abs(self.y_control_team[i].speed_r) < 1.5:
                self.y_control_team[i].speed_r = 0
            rules.append(0)
            rules.append(self.y_control_team[i].speed_x)
            rules.append(self.y_control_team[i].speed_y)
            rules.append(self.y_control_team[i].speed_r)
            rules.append(self.y_control_team[i].kick_forward_)
            rules.append(self.y_control_team[i].kick_up_)
            rules.append(self.y_control_team[i].auto_kick_)
            rules.append(self.y_control_team[i].kicker_voltage_)
            rules.append(self.y_control_team[i].dribbler_enable_)
            rules.append(self.y_control_team[i].dribbler_speed_)
            rules.append(self.y_control_team[i].kicker_charge_enable_)
            rules.append(self.y_control_team[i].beep)
            rules.append(0)

        # rules = [15] * 15 * 32
        b = bytes()
        return b.join((struct.pack("d", rule) for rule in rules))
