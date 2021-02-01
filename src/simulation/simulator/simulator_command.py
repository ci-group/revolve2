from enum import Enum, auto

from nca.core.abstract.behavioral.command import Command
from nca.core.abstract.sequential_identifier import SimulatorRequestIdentifier
from nca.core.actor.actors import Actors
from revolve.evosphere.ecosphere import Ecosphere
from revolve.robot.birth_clinic import BirthClinic
from test.evosphere.mock_ecosphere import MockEcosphere
from test.evosphere.robot.test_birth_clinic import MockBirthClinic


class TaskPriority(Enum):

    LOW = auto()
    MEDIUM = auto()
    HIGH = auto()


class SimulationRequest(Command):

    identifier = SimulatorRequestIdentifier()

    def __init__(self, agents: Actors, ecosphere: Ecosphere, birth_clinic: BirthClinic, task_priority: TaskPriority = TaskPriority.LOW, number_of_workers: int = 4):
        self.agents: Actors = agents
        self.ecosphere: Ecosphere = ecosphere
        self.birth_clinic: BirthClinic = birth_clinic
        self.task_priority: TaskPriority = task_priority
        self.number_of_workers: int = number_of_workers
        self.id = self.identifier.id()

    def __eq__(self, other):
        return self.agents == other.agents and self.ecosphere == other.ecosphere

    def __hash__(self):
        return hash(self.id)


class MockSimulationRequest(SimulationRequest):

    def __init__(self):
        super().__init__(Actors([]), MockEcosphere(), MockBirthClinic(), number_of_workers=8)
