from enum import Enum, auto


class SimulatorType(Enum):

    NONE = auto()
    GENETIC = auto()
    GAZEBO = auto()
    COPPELIA = auto()
    UNITY = auto()
    MALMO = auto()
    EVOMAN = auto()
