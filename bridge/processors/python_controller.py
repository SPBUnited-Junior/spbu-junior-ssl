"""
Модуль стратегии игры
"""

import time

import attr
from strategy_bridge.bus import DataBus, DataReader, DataWriter
from strategy_bridge.common import config
from strategy_bridge.model.referee import RefereeCommand
from strategy_bridge.processors import BaseProcessor
from strategy_bridge.utils.debugger import debugger

import bridge.processors.referee_state_processor as state_machine
from bridge import const
from bridge.auxiliary import aux, fld
from bridge.router import router
from bridge.strategy import strategy


@attr.s(auto_attribs=True)
class SSLController(BaseProcessor):
    """
    Процессор стратегии SSL
    """

    processing_pause: float = const.Ts
    reduce_pause_on_process_time: bool = True
    max_commands_to_persist: int = 20

    ally_color: const.Color = const.Color.BLUE

    field_reader: DataReader = attr.ib(init=False)
    referee_reader: DataReader = attr.ib(init=False)
    commands_sink_writer: DataWriter = attr.ib(init=False)

    dbg_game_status: strategy.GameStates = strategy.GameStates.TIMEOUT
    # dbg_state: strategy.States = strategy.States.DEBUG

    cur_time = time.time()
    delta_t = 0.0

    ctrl_mapping = const.CONTROL_MAPPING
    count_halt_cmd = 0

    def initialize(self, data_bus: DataBus) -> None:
        """
        Инициализировать контроллер
        """
        super(SSLController, self).initialize(data_bus)
        self.field_reader = DataReader(data_bus, const.FIELD_TOPIC)
        self.referee_reader = DataReader(data_bus, config.REFEREE_COMMANDS_TOPIC)
        self.commands_sink_writer = DataWriter(data_bus, const.TOPIC_SINK, 20)
        self.image_writer = DataWriter(data_bus, const.IMAGE_TOPIC, 20)
        self.points_topic = DataReader(data_bus, const.POINTS_TOPIC)

        self.field = fld.Field()
        self.router = router.Router(self.field)

        self.strategy = strategy.Strategy()

        # Referee fields
        self.state_machine = state_machine.StateMachine()
        self.cur_cmd_state = None
        self.wait_10_sec_flag = False
        self.wait_10_sec = 0.0
        self.wait_ball_moved_flag = False
        self.wait_ball_moved = aux.Point(0, 0)
        self.tmp = 0
        self.pass_points: list[aux.Point] = []
        self.we_kick = False

    def get_last_referee_command(self) -> RefereeCommand:
        """
        Получить последнюю команду рефери
        """
        referee_commands = self.referee_reader.read_new()
        if referee_commands:
            return referee_commands[-1].content
        return RefereeCommand(-1, 0, False)

    def read_vision(self) -> None:
        """
        Прочитать новые пакеты из SSL-Vision
        """
        new_field = self.field_reader.read_last()
        if new_field is not None:
            self.field = new_field.content
            if self.field.ally_color != self.ally_color:
                self.field.reverse_field()
        else:
            print("No new field")

    def read_pass_points(self) -> None:
        """Прочитать точки для паса"""
        points = self.points_topic.read_last()
        if points is not None:
            self.pass_points = points.content
            # for point in points:
            #     self.pass_points.append(point.content)
        self.strategy.get_pass_points(self.pass_points)

    def draw_image(self) -> None:
        """Send commands to drawer processor"""
        if self.field.image is not None and self.field.ally_color == const.COLOR:
            self.image_writer.write(self.field.image)

    def control_loop(self) -> None:
        """
        Рассчитать стратегию, тактику и физику для роботов на поле
        """
        self.router.update(self.field)
        waypoints = self.strategy.process(self.field)

        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            self.router.get_route(i).clear()
            self.router.set_dest(i, waypoints[i], self.field)
        self.router.reroute(self.field)

        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            self.router.get_route(i).go_route(self.field.allies[i], self.field)

    def control_assign(self) -> None:
        """
        Определить связь номеров роботов с каналами управления
        """
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if self.field.allies[i].is_used():
                self.field.allies[i].color = self.ally_color
                self.commands_sink_writer.write(self.field.allies[i])

    def process_referee_cmd(self) -> None:
        """Get referee commands"""
        cur_cmd = self.get_last_referee_command()
        cur_state, cur_active = self.state_machine.get_state()
        # cur_state = state_machine.State.TIMEOUT
        # cur_active = self.field.ally_color
        self.strategy.change_game_state(cur_state, cur_active)
        avoid_ball = False
        dont_touch = False
        we_active = False
        avoid_enemy_half = False

        if cur_cmd.state == -1:
            return

        if cur_state == state_machine.State.PREPARE_KICKOFF:
            if cur_active == self.field.ally_color:
                self.we_kick = True
            else:
                self.we_kick = False

        if ((cur_state == state_machine.State.PREPARE_KICKOFF and cur_active == self.field.ally_color)):
            dont_touch = True

        if (
            cur_state == state_machine.State.STOP or cur_active not in [const.Color.ALL, self.field.ally_color]
            or (cur_state == state_machine.State.KICKOFF and not self.we_kick)
        ) and cur_state not in [state_machine.State.PREPARE_PENALTY, state_machine.State.PENALTY]:
            avoid_ball = True

        if cur_state in [state_machine.State.KICKOFF, state_machine.State.PREPARE_KICKOFF]:
            avoid_enemy_half = True
        if (
            cur_state == state_machine.State.PREPARE_KICKOFF and cur_active == self.field.ally_color
        ) or cur_state == state_machine.State.KICKOFF:
            we_active = True

        if cur_state == state_machine.State.STOP:
            self.field.be_slow = True
        else:
            self.field.be_slow = False
        
        if cur_state == state_machine.State.TIMEOUT:
            self.router.put_state(False, False, False, True)
        else:
            self.router.put_state(avoid_ball, dont_touch, avoid_enemy_half, we_active)

        if cur_cmd.state != self.cur_cmd_state:
            self.state_machine.make_transition(cur_cmd.state)
            self.state_machine.active_team(cur_cmd.commandForTeam)
            self.cur_cmd_state = cur_cmd.state
            cur_state, _ = self.state_machine.get_state()

            self.wait_10_sec_flag = False
            self.wait_ball_moved_flag = False

            if cur_state in [
                state_machine.State.KICKOFF,
                state_machine.State.FREE_KICK,
                state_machine.State.PENALTY,
            ]:
                self.wait_10_sec_flag = True
                self.wait_10_sec = time.time()
            if cur_state in [
                state_machine.State.KICKOFF,
                state_machine.State.FREE_KICK,
            ]:
                self.wait_ball_moved_flag = True
                self.wait_ball_moved = self.field.ball.get_pos()
        else:
            if self.wait_10_sec_flag and time.time() - self.wait_10_sec > 10:
                self.state_machine.make_transition_(state_machine.Command.PASS_10_SECONDS)
                self.state_machine.active_team(0)
                self.wait_10_sec_flag = False
                self.wait_ball_moved_flag = False
            if self.wait_ball_moved_flag and self.field.is_ball_moves():
                self.state_machine.make_transition_(state_machine.Command.BALL_MOVED)
                self.state_machine.active_team(0)
                self.wait_10_sec_flag = False
                self.wait_ball_moved_flag = False
        self.tmp += 1

    @debugger
    def process(self) -> None:
        """
        Выполнить цикл процессора
        """

        self.delta_t = time.time() - self.cur_time
        self.cur_time = time.time()

        self.read_vision()
        self.read_pass_points()
        self.process_referee_cmd()
        self.control_loop()

        self.control_assign()
        self.draw_image()
