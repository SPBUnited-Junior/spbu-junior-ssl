"""
Модуль-прослойка между стратегией и отправкой пакетов на роботов
"""

import struct
import typing
from time import time

import attr
from strategy_bridge.bus import DataBus, DataReader, DataWriter
from strategy_bridge.common import config
from strategy_bridge.processors import BaseProcessor

from bridge import const
from bridge.auxiliary import aux, rbt


@attr.s(auto_attribs=True)
class CommandSink(BaseProcessor):
    """
    Прослойка между стратегией и отправкой пакетов на роботов
    """

    processing_pause: typing.Optional[float] = 0.01
    reduce_pause_on_process_time: bool = False
    max_commands_to_persist: int = 20
    commands_sink_reader: DataReader = attr.ib(init=False)
    commands_writer: DataWriter = attr.ib(init=False)

    b_control_team = [
        rbt.Robot(aux.GRAVEYARD_POS, 0, const.ROBOT_R, const.Color.BLUE, i, 0) for i in range(const.TEAM_ROBOTS_MAX_COUNT)
    ]
    y_control_team = [
        rbt.Robot(aux.GRAVEYARD_POS, 0, const.ROBOT_R, const.Color.YELLOW, i, 0) for i in range(const.TEAM_ROBOTS_MAX_COUNT)
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

        if cmds is None:
            return

        # if len(cmds) > 0:
        #     print(len(cmds), "total delay:", time() - cmds[0].content.last_update())

        for cmd in cmds:
            r: rbt.Robot = cmd.content
            if not r.is_used():
                continue
            ctrl_id = r.ctrl_id

            if ctrl_id is None:
                continue

            if r.color == const.Color.BLUE:
                self.b_control_team[ctrl_id].copy_control_fields(r)
            elif r.color == const.Color.YELLOW:
                self.y_control_team[ctrl_id].copy_control_fields(r)
                # self.y_control_team[ctrl_id].used(1)

        rules = self.get_rules()

        self.commands_writer.write(rules)

    def get_rules(self) -> bytes:
        """
        Сформировать массив команд для отправки на роботов
        """
        rules: list[float] = []

        if const.IS_SIMULATOR_USED:
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if not const.IS_DRIBBLER_USED:
                    self.b_control_team[i].dribbler_enable_ = 0
                    self.b_control_team[i].dribbler_speed_ = 0

                if abs(self.b_control_team[i].speed_x) < 1:
                    self.b_control_team[i].speed_x = 0
                if abs(self.b_control_team[i].speed_y) < 1:
                    self.b_control_team[i].speed_y = 0
                if abs(self.b_control_team[i].speed_r) < 1:
                    self.b_control_team[i].speed_r = 0
                rules.append(0)
                rules.append(self.b_control_team[i].speed_x)
                rules.append(self.b_control_team[i].speed_y)
                rules.append(self.b_control_team[i].speed_r)
                rules.append(self.b_control_team[i].kick_up_)
                rules.append(self.b_control_team[i].kick_forward_)
                rules.append(self.b_control_team[i].auto_kick_)
                rules.append(self.b_control_team[i].kicker_voltage_ // 2)
                rules.append(self.b_control_team[i].dribbler_enable_)
                rules.append(self.b_control_team[i].dribbler_speed_)
                rules.append(self.b_control_team[i].kicker_charge_enable_)
                rules.append(self.b_control_team[i].beep)
                rules.append(0)

            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if not const.IS_DRIBBLER_USED:
                    self.y_control_team[i].dribbler_enable_ = 0
                    self.y_control_team[i].dribbler_speed_ = 0
                if abs(self.y_control_team[i].speed_x) < 1:
                    self.y_control_team[i].speed_x = 0
                if abs(self.y_control_team[i].speed_y) < 1:
                    self.y_control_team[i].speed_y = 0
                if abs(self.y_control_team[i].speed_r) < 1:
                    self.y_control_team[i].speed_r = 0
                rules.append(0)
                rules.append(self.y_control_team[i].speed_x)
                rules.append(self.y_control_team[i].speed_y)
                rules.append(self.y_control_team[i].speed_r)
                rules.append(self.y_control_team[i].kick_up_)
                rules.append(self.y_control_team[i].kick_forward_)
                rules.append(self.y_control_team[i].auto_kick_)
                rules.append(self.y_control_team[i].kicker_voltage_ // 2)
                rules.append(self.y_control_team[i].dribbler_enable_)
                rules.append(self.y_control_team[i].dribbler_speed_)
                rules.append(self.y_control_team[i].kicker_charge_enable_)
                rules.append(self.y_control_team[i].beep)
                rules.append(0)
        else:
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                control_team = self.y_control_team if self.y_control_team[i].is_used() else self.b_control_team

                if self.y_control_team[i].is_used():
                    pass
                elif self.b_control_team[i].is_used():
                    pass
                else:
                    for _ in range(13):
                        rules.append(0)
                    continue

                if not const.IS_DRIBBLER_USED:
                    if round(time() * 2) % 10 == 0:
                        control_team[i].dribbler_enable_ = 1
                        control_team[i].dribbler_speed_ = 1
                    else:
                        control_team[i].dribbler_enable_ = 0
                        control_team[i].dribbler_speed_ = 0

                if abs(control_team[i].speed_x) < 1:
                    control_team[i].speed_x = 0
                if abs(control_team[i].speed_y) < 1:
                    control_team[i].speed_y = 0
                if abs(control_team[i].speed_r) < 1:
                    control_team[i].speed_r = 0
                rules.append(0)
                rules.append(control_team[i].speed_x)
                rules.append(control_team[i].speed_y)
                rules.append(control_team[i].speed_r)
                rules.append(control_team[i].kick_up_)
                rules.append(control_team[i].kick_forward_)
                rules.append(control_team[i].auto_kick_)
                rules.append(control_team[i].kicker_voltage_)
                rules.append(control_team[i].dribbler_enable_)
                rules.append(control_team[i].dribbler_speed_)
                rules.append(control_team[i].kicker_charge_enable_)
                rules.append(control_team[i].beep)
                rules.append(0)
            for _ in range(const.TEAM_ROBOTS_MAX_COUNT):
                for _ in range(13):
                    rules.append(0)

        # rules = [15] * 13 * 32
        b = bytes()
        return b.join((struct.pack("d", rule) for rule in rules))
