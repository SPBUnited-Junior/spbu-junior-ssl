from enum import Enum
from bridge.processors.const import Color


class State(Enum):
    """Класс с состояниями игры"""
    HALT = 0
    TIMEOUT = 1
    STOP = 2
    PREPARE_KICKOFF = 3
    BALL_PLACEMENT = 4
    PREPARE_PENALTY = 5
    KICKOFF = 6
    FREE_KICK = 7
    PENALTY = 8
    RUN = 9


class Command(Enum):
    """Класс с командами от судей"""
    HALT = 0
    STOP = 1
    FORCE_START = 2
    TIMEOUT = 3
    PREPARE_KICKOFF = 5
    NORMAL_START = 6
    PREPARE_PENALTY = 7
    NORMAL_START2 = 8  # я не понимаю почему они разные ну ладно..
    FREE_KICK = 9
    CONTINUE = 10  # не факт, что правда
    BALL_PLACEMENT = 11

    BALL_MOVED = 101
    PASS_10_SECONDS = 102


CommandMap = {command.value: command for command in Command}



class StateMachine:
    def __init__(self, initial_state:State = State.HALT) -> None:
        self.__state = initial_state
        self.__transitions = {}
        self.__active = Color.ALL

        self.add_transition(State.HALT, State.STOP, Command.STOP)
        self.add_transition(State.HALT, State.RUN, Command.FORCE_START)
        self.add_transition(State.HALT, State.FREE_KICK, Command.FREE_KICK)
        self.add_transition(State.HALT, State.PREPARE_KICKOFF, Command.PREPARE_KICKOFF)
        self.add_transition(State.HALT, State.PREPARE_PENALTY, Command.PREPARE_PENALTY)
        self.add_transition(State.HALT, State.KICKOFF, Command.NORMAL_START)

        for state in State:
            self.add_transition(state, State.HALT, Command.HALT)
            self.add_transition(state, State.STOP, Command.STOP)

        self.add_transition(State.TIMEOUT, State.STOP, Command.STOP)

        self.add_transition(State.STOP, State.PREPARE_KICKOFF, Command.PREPARE_KICKOFF)
        self.add_transition(State.STOP, State.BALL_PLACEMENT, Command.BALL_PLACEMENT)
        self.add_transition(State.STOP, State.PREPARE_PENALTY, Command.PREPARE_PENALTY)
        self.add_transition(State.STOP, State.FREE_KICK, Command.FREE_KICK)
        self.add_transition(State.STOP, State.RUN, Command.FORCE_START)
        self.add_transition(State.STOP, State.TIMEOUT, Command.TIMEOUT)

        self.add_transition(State.PREPARE_KICKOFF, State.KICKOFF, Command.NORMAL_START)

        self.add_transition(State.BALL_PLACEMENT, State.FREE_KICK, Command.CONTINUE)
        self.add_transition(State.BALL_PLACEMENT, State.STOP, Command.STOP)

        self.add_transition(State.PREPARE_PENALTY, State.PENALTY, Command.NORMAL_START2)

        self.add_transition(State.PENALTY, State.STOP, Command.PASS_10_SECONDS)

        self.add_transition(State.KICKOFF, State.RUN, Command.PASS_10_SECONDS)
        self.add_transition(State.KICKOFF, State.RUN, Command.BALL_MOVED)

        self.add_transition(State.FREE_KICK, State.RUN, Command.PASS_10_SECONDS)
        self.add_transition(State.FREE_KICK, State.RUN, Command.BALL_MOVED)

        self.add_transition(State.RUN, State.STOP, Command.STOP)

    def add_transition(self, from_state: State, to_state: State, transition: Command) -> None:
        if from_state not in self.__transitions:
            self.__transitions[from_state] = {}
        self.__transitions[from_state][transition] = to_state

    def make_transition(self, transition: int) -> None:
        self.make_transition_(CommandMap.get(transition))

    def make_transition_(self, transition: Command) -> None:
        if transition in self.__transitions[self.__state]:
            self.__state = self.__transitions[self.__state][transition]
        else:
            raise ValueError(f"No transition '{transition}' from state '{self.__state}'")

    def get_possible_transitions(self):
        return list(self.__transitions[self.__state].keys()) if self.__state in self.__transitions else []

    def active_team(self, num: int) -> None:
        if num == 0:
            self.__active = Color.ALL
        elif num == 1:
            self.__active = Color.BLUE
        elif num == 2:
            self.__active = Color.YELLOW

    def get_state(self) -> tuple[State, Color]:
        return self.__state, self.__active

    def __str__(self) -> str:
        return f'State: {self.__state}, Active: {self.__active}'
