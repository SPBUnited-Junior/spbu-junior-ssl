"""
Точка входа в стратегию
"""

from strategy_bridge.processors import RobotCommandsSender, VisionDetectionsCollector
from strategy_bridge.processors.referee_commands_collector import (
    RefereeCommandsCollector,
)
from strategy_bridge.runner import Runner

from bridge import const
from bridge.processors.drawing_processor import Drawer
from bridge.processors.field_creator import FieldCreator
from bridge.processors.python_controller import SSLController
from bridge.processors.robot_command_sink import CommandSink
from bridge.processors.strategy_points import StrategyPoints
from bridge.strategy import strategy

if __name__ == "__main__":

    # config.init_logging("./logs")

    PROCESSORS = [
        VisionDetectionsCollector(processing_pause=0.001),
        RefereeCommandsCollector(processing_pause=0.001),
        FieldCreator(processing_pause=0.01),
        SSLController(
            ally_color=const.COLOR,
            processing_pause=const.Ts,
            reduce_pause_on_process_time=True,
            dbg_game_status=strategy.GameStates.RUN,
            # dbg_state=strategy.States.ATTACK,
        ),
        StrategyPoints(processing_pause=0.001),
        # SSLController(
        #     ally_color=const.Color.YELLOW,
        #     processing_pause=const.Ts,
        #     reduce_pause_on_process_time=True,
        #     dbg_game_status=strategy.GameStates.RUN,
        #     #dbg_state=strategy.States.ATTACK,
        # ),
        Drawer(),
        CommandSink(processing_pause=0.001),  # , should_debug=True
        RobotCommandsSender(processing_pause=0.001),
    ]

    RUNNER = Runner(processors=PROCESSORS)
    RUNNER.run()
