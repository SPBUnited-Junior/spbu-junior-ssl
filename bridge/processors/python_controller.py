"""
Модуль стратегии игры
"""
import time

import attr
import numpy as np
from strategy_bridge.bus import DataBus, DataReader, DataWriter
from strategy_bridge.common import config
from strategy_bridge.model.referee import RefereeCommand
from strategy_bridge.pb.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from strategy_bridge.processors import BaseProcessor
from strategy_bridge.utils.debugger import debugger
import bridge.processors.referee_state_processor as state_machine

import bridge.processors.auxiliary as aux
from bridge.processors import const, field, router, signal, strategy


# TODO: Refactor this class and corresponding matlab scripts
@attr.s(auto_attribs=True)
class SSLController(BaseProcessor):
    """
    Процессор стратегии SSL
    """

    max_commands_to_persist: int = 20
    ally_color: const.Color = const.Color.BLUE

    vision_reader: DataReader = attr.ib(init=False)
    referee_reader: DataReader = attr.ib(init=False)
    commands_sink_writer: DataWriter = attr.ib(init=False)
    _ssl_converter: SSL_WrapperPacket = attr.ib(init=False)

    dbg_game_status: strategy.GameStates = strategy.GameStates.TIMEOUT
    dbg_state: strategy.States = strategy.States.DEBUG

    cur_time = time.time()
    delta_t = 0.0

    ctrl_mapping = const.CONTROL_MAPPING
    count_halt_cmd = 0

    def initialize(self, data_bus: DataBus) -> None:
        """
        Инициализировать контроллер
        """
        super(SSLController, self).initialize(data_bus)
        self.vision_reader = DataReader(data_bus, config.VISION_DETECTIONS_TOPIC)
        self.referee_reader = DataReader(data_bus, config.REFEREE_COMMANDS_TOPIC)
        self.commands_sink_writer = DataWriter(data_bus, const.TOPIC_SINK, 20)
        self._ssl_converter = SSL_WrapperPacket()

        self.field = field.Field(self.ctrl_mapping, self.ally_color)
        self.router = router.Router(self.field)

        self.strategy = strategy.Strategy()

        # Referee fields
        self.state_machine = state_machine.StateMachine()
        self.cur_cmd_state = None
        self.wait_10_sec_flag = False
        self.wait_10_sec = 0
        self.wait_ball_moved_flag = False
        self.wait_ball_moved = aux.Point(0, 0)
        self.tmp = 0

    def get_last_referee_command(self) -> RefereeCommand:
        """
        Получить последнюю команду рефери
        """
        referee_commands = self.referee_reader.read_new()
        if referee_commands:
            return referee_commands[-1].content
        return RefereeCommand(-1, 0, False)

    def read_vision(self) -> bool:
        """
        Прочитать новые пакеты из SSL-Vision
        """
        status = False

        balls: list[aux.Point] = []
        b_bots_id: list[int] = []
        b_bots_pos: list[list] = [[] for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]
        b_bots_ang: list[list] = [[] for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]

        y_bots_id: list[int] = []
        y_bots_pos: list[list] = [[] for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]
        y_bots_ang: list[list] = [[] for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]
        field_info = np.zeros(const.GEOMETRY_PACKET_SIZE)

        queue = self.vision_reader.read_new()

        for ssl_package in queue:
            try:
                ssl_package_content = ssl_package.content
            except AttributeError:
                pass
                # None
            if not ssl_package_content:
                continue

            status = True

            ssl_package_content = self._ssl_converter.FromString(ssl_package_content)
            geometry = ssl_package_content.geometry
            if geometry:
                field_info[0] = geometry.field.field_length
                field_info[1] = geometry.field.field_width
                if geometry.field.field_length != 0 and geometry.field.goal_width != 0:
                    const.GOAL_DX = geometry.field.field_length / 2
                    const.GOAL_DY = geometry.field.goal_width

            detection = ssl_package_content.detection
            # camera_id = detection.camera_id
            for ball in detection.balls:
                balls.append(aux.Point(ball.x, ball.y))
            
            cur_state, cur_active = self.state_machine.get_state()
            self.strategy.change_game_state(cur_state, cur_active)
            self.router.avoid_ball(False)
            if cur_state in [state_machine.State.STOP] or (cur_active != const.Color.ALL and cur_active != self.field.ally_color):
                self.router.avoid_ball(True)

            # TODO: Barrier states
            for robot_det in detection.robots_blue:
                b_bots_id.append(robot_det.robot_id)
                b_bots_pos[robot_det.robot_id].append(aux.Point(robot_det.x, robot_det.y))
                b_bots_ang[robot_det.robot_id].append(robot_det.orientation)

            for robot_det in detection.robots_yellow:
                y_bots_id.append(robot_det.robot_id)
                y_bots_pos[robot_det.robot_id].append(aux.Point(robot_det.x, robot_det.y))
                y_bots_ang[robot_det.robot_id].append(robot_det.orientation)

        if len(balls) != 0:
            balls_sum = aux.Point(0, 0)
            for ball in balls:
                balls_sum += ball
            ball = balls_sum / len(balls)
            self.field.update_ball(ball, time.time())
        elif self.field.ally_with_ball is not None:
            ally = self.field.ally_with_ball
            ball = ally.get_pos() + aux.rotate(aux.RIGHT, ally.get_angle()) * ally.get_radius() / 2
            self.field.update_ball(ball, time.time())

        self.field.ally_with_ball = None
        for r in self.field.allies:
            if self.field._is_ball_in(r):
                self.field.ally_with_ball = r

        for r_id in set(b_bots_id):
            position = aux.average_point(b_bots_pos[r_id])
            angle = aux.average_angle(b_bots_ang[r_id])
            if position != self.field.b_team[r_id].get_pos() or const.IS_SIMULATOR_USED:
                self.field.update_blu_robot(r_id, position, angle, time.time())
        for robot in self.field.b_team:
            if time.time() - robot.last_update() > 0.5:
                robot.used(0)
            else:
                robot.used(1)

        for r_id in set(y_bots_id):
            position = aux.average_point(y_bots_pos[r_id])
            angle = aux.average_angle(y_bots_ang[r_id])
            if position != self.field.y_team[r_id].get_pos() or const.IS_SIMULATOR_USED:
                self.field.update_yel_robot(r_id, position, angle, time.time())
        for robot in self.field.y_team:
            if time.time() - robot.last_update() > 0.5:
                robot.used(0)
            else:
                robot.used(1)

        return status

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

    square = signal.Signal(2, "SQUARE", lohi=(-20, 20))
    sine = signal.Signal(2, "SINE", ampoffset=(1000, 0))
    cosine = signal.Signal(2, "COSINE", ampoffset=(1000, 0))

    def control_assign(self) -> None:
        """
        Определить связь номеров роботов с каналами управления
        """
        # self.field.allies[const.DEBUG_ID].speed_x = 0
        # self.field.allies[const.DEBUG_ID].speed_y = 0
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if self.field.allies[i].is_used():
                self.field.allies[i].color = self.ally_color
            # self.field.allies[i].speed_r = self.square.get()
            self.commands_sink_writer.write(self.field.allies[i])

    def process_referee_cmd(self) -> None:
        cur_cmd = self.get_last_referee_command()
        if cur_cmd.state == -1:
            return

        # self.state_machine.make_transition_(state_machine.Command.HALT)
        # self.cur_cmd_state = None
        # print(self.state_machine)
        # cur_cmd = RefereeCommand(0, 0, False)
        # cur_cmd.state = self.cur_cmd_state if self.cur_cmd_state is not None else 0
        #
        # if self.tmp == 3:
        #     cur_cmd = RefereeCommand(1, 0, False)
        # elif self.tmp == 9:
        #     cur_cmd = RefereeCommand(5, 1, False)
        # elif self.tmp == 26:
        #     cur_cmd = RefereeCommand(6, 1, False)
        # elif self.tmp == 220:
        #     cur_cmd = RefereeCommand(1, 0, False)
        # elif self.tmp == 235:
        #     cur_cmd = RefereeCommand(9, 2, False)
        if cur_cmd.state != self.cur_cmd_state:
            self.state_machine.make_transition(cur_cmd.state)
            self.state_machine.active_team(cur_cmd.commandForTeam)
            self.cur_cmd_state = cur_cmd.state
            cur_state, _ = self.state_machine.get_state()

            self.wait_10_sec_flag = False
            self.wait_ball_moved_flag = False

            if cur_state in [state_machine.State.KICKOFF, state_machine.State.FREE_KICK, state_machine.State.PENALTY]:
                self.wait_10_sec_flag = True
                self.wait_10_sec = time.time()
            if cur_state in [state_machine.State.KICKOFF, state_machine.State.FREE_KICK]:
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
        self.process_referee_cmd()
        self.control_loop()


        self.control_assign()
