from enum import Enum, auto

from nca.core.abstract.behavioral.command import Command
from nca.core.actor.actors import Actors
from revolve.evosphere.ecosphere import Ecosphere


class TaskPriority(Enum):

    LOW = auto()
    MEDIUM = auto()
    HIGH = auto()


class SimulateCommand(Command):

    def __init__(self, agents: Actors, ecosphere: Ecosphere, task_priority: TaskPriority = TaskPriority.LOW):
        self.agents: Actors = agents
        self.ecosphere: Ecosphere = ecosphere
        self.task_priority: TaskPriority = task_priority

    def __eq__(self, other):
        return self.agents == other.agents and self.ecosphere == other.ecosphere

    def __hash__(self):
        return hash((self.agents, self.ecosphere))

    def develop(self):
        return self.ecosphere.birth_clinic.develop(self.agents)
