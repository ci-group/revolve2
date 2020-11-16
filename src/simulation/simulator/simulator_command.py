from enum import Enum, auto

from nca.core.abstract.behavioral.command import Command
from nca.core.actor.actors import Actors
from revolve.evosphere.ecosphere import Ecosphere
from revolve.robot.birth_clinic import BirthClinic


class TaskPriority(Enum):

    LOW = auto()
    MEDIUM = auto()
    HIGH = auto()


class SimulateCommand(Command):

    def __init__(self, agents: Actors, ecosphere: Ecosphere, birth_clinic: BirthClinic, task_priority: TaskPriority = TaskPriority.LOW):
        self.agents: Actors = agents
        self.ecosphere: Ecosphere = ecosphere
        self.birth_clinic: BirthClinic = birth_clinic
        self.task_priority: TaskPriority = task_priority

    def __eq__(self, other):
        return self.agents == other.agents and self.ecosphere == other.ecosphere

    def __hash__(self):
        return hash((self.agents, self.ecosphere))

    def develop(self):
        return self.birth_clinic.build(self.agents, self.ecosphere)
