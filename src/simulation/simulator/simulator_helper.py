from enum import Enum, auto


class SimulatorType(Enum):
    NONE = auto()
    GAZEBO = auto()
    V_REP = auto()
    UNITY = auto()
    MALMO = auto()


class TaskPriority(Enum):
    LOW = auto()
    MEDIUM = auto()
    HIGH = auto()
