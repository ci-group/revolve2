from enum import Enum, auto


class SimulatorState(Enum):

    PAUSED = auto()
    READY = auto()
    RUNNING = auto()
    WAITING = auto()

    def __eq__(self, other):
        return self.value == other.value

