from enum import Enum, auto

from nca.core.abstract.behavioral.command import Command
from nca.core.agent.agents import Agents
from revolve.evosphere.biosphere import Ecosphere


class TaskPriority(Enum):

    LOW = auto()
    MEDIUM = auto()
    HIGH = auto()


class SimulateCommand(Command):

    def __init__(self, agents: Agents, ecosphere: Ecosphere, task_priority: TaskPriority = TaskPriority.LOW):
        self.agents: Agents = agents
        self.ecosphere: Ecosphere = ecosphere
        self.task_priority: TaskPriority = task_priority

    def __eq__(self, other):
        return self.agents == other.agents and self.ecosphere == other.ecosphere

    def __hash__(self):
        return hash((self.agents, self.ecosphere))
