from abc import ABC, abstractmethod

from revolve2.core.modular_robot import Body as ModularRobotBody
from revolve2.core.modular_robot import Brain as ModularRobotBrain


class BrainGenotype(ABC):
    @abstractmethod
    def develop(self, body: ModularRobotBody) -> ModularRobotBrain:
        pass
