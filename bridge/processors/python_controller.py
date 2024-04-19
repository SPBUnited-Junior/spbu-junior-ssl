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

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.field as field
import bridge.processors.router as router
import bridge.processors.signal as signal
import bridge.processors.strategy as strategy


# TODO: Refactor this class and corresponding matlab scripts
@attr.s(auto_attribs=True)
class SSLController(BaseProcessor):
    """
    Процессор стратегии SSL
    """

    max_commands_to_persist: int = 20
    ally_color: str = "y"

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

    def get_last_referee_command(self) -> RefereeCommand:
        """
        Получить последнюю команду рефери
        """
        referee_commands = self.referee_reader.read_new()
        if referee_commands:
            return referee_commands[-1].content
        return RefereeCommand(0, 0, False)

    def read_vision(self) -> bool:
        """
        Прочитать новые пакеты из SSL-Vision
        """
        status = False

        balls = [[], []]
        field_info = np.zeros(const.GEOMETRY_PACKET_SIZE)

        queue = self.vision_reader.read_new()

        for ssl_package in queue:
            # print("new paket")
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
                # if geometry.field.field_length != 0 and geometry.field.goal_width != 0:
                #     print("aaaa")
                #     const.GOAL_DX = geometry.field.field_length / 2
                #     const.GOAL_DY = geometry.field.goal_width

            detection = ssl_package_content.detection
            detection.camera_id
            for ball_ind, ball in enumerate(detection.balls):
                balls[0].append(ball.x)
                balls[1].append(ball.y)

            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if time.time() - self.field.b_team[i].last_update() > 1:
                    self.field.b_team[i].used(0)
                if time.time() - self.field.y_team[i].last_update() > 1:
                    self.field.y_team[i].used(0)

            # self.strategy.changeGameState(strategy.GameStates.RUN, 0)
                    
            '''game_controller_mapping = {
                1: strategy.GameStates.STOP,
                2: strategy.GameStates.RUN,
                3: strategy.GameStates.TIMEOUT,
                4: strategy.GameStates.HALT,
                5: strategy.GameStates.PREPARE_KICKOFF,
                6: strategy.GameStates.KICKOFF,
                7: strategy.GameStates.PREPARE_PENALTY,
                8: strategy.GameStates.PENALTY,
                9: strategy.GameStates.FREE_KICK,
                10: strategy.GameStates.HALT,
                11: strategy.GameStates.BALL_PLACMENT,
            }

            cur_cmd = self.get_last_referee_command()
            if cur_cmd.state == 0:
                self.count_halt_cmd += 1
            else:
                self.count_halt_cmd = 0
                self.strategy.change_game_state(game_controller_mapping[cur_cmd.state], cur_cmd.commandForTeam)
                if cur_cmd.state == 4:
                    print("End game")
                elif cur_cmd.state == 10:
                    print("Uknown command 10")
            
            if self.count_halt_cmd > 10:
               self.strategy.change_game_state(strategy.GameStates.HALT, cur_cmd.commandForTeam)'''
            self.strategy.change_game_state(strategy.GameStates.RUN, 0)

            # TODO: Barrier states
            for robot_det in detection.robots_blue:
                if time.time() - self.field.b_team[robot_det.robot_id].last_update() > 0.3:
                    self.field.b_team[robot_det.robot_id].used(0)
                else:
                    self.field.b_team[robot_det.robot_id].used(1)
                self.field.update_blu_robot(
                    robot_det.robot_id, aux.Point(robot_det.x, robot_det.y), robot_det.orientation, time.time()
                )

            for robot_det in detection.robots_yellow:
                if time.time() - self.field.y_team[robot_det.robot_id].last_update() > 0.3:
                    self.field.y_team[robot_det.robot_id].used(0)
                else:
                    self.field.y_team[robot_det.robot_id].used(1)
                self.field.update_yel_robot(
                    robot_det.robot_id, aux.Point(robot_det.x, robot_det.y), robot_det.orientation, time.time()
                )

        if len(balls[0]) != 0:
            midBallX = sum(balls[0]) / len(balls[0])
            midBallY = sum(balls[1]) / len(balls[1])
            self.field.update_ball(aux.Point(midBallX, midBallY), time.time())
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

        # for i in range(const.TEAM_ROBOTS_MAX_COUNT):
        #     print(self.field.y_team[i])
        # for i in range(const.TEAM_ROBOTS_MAX_COUNT):
        #     print(self.field.b_team[i])

    square = signal.Signal(2, "SQUARE", lohi=(-20, 20))
    sine = signal.Signal(2, "SINE", ampoffset=(1000, 0))
    cosine = signal.Signal(2, "COSINE", ampoffset=(1000, 0))

    def control_assign(self) -> None:
        """
        Определить связь номеров роботов с каналами управления
        """
        # self.field.allies[const.DEBUG_ID].speed_x = 0
        # self.field.allies[const.DEBUG_ID].speed_y = 0
        # print(self.square.get())
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if self.field.allies[i].is_used():
                self.field.allies[i].color = self.ally_color
            # self.field.allies[i].speed_r = self.square.get()
            self.commands_sink_writer.write(self.field.allies[i])

    @debugger
    def process(self) -> None:
        """
        Выполнить цикл процессора

        OFFTOP Что означает @debugger?
        """

        self.delta_t = time.time() - self.cur_time
        self.cur_time = time.time()

        # print("delta_t", self.delta_t)

        self.read_vision()
        self.control_loop()

        # print(self.router.getRoute(const.DEBUG_ID))

        self.control_assign()
