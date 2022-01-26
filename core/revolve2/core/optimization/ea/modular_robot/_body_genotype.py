from abc import ABC, abstractmethod

from revolve2.core.modular_robot import Body as ModularRobotBody


class BodyGenotype(ABC):
    @abstractmethod
    def develop(self) -> ModularRobotBody:
        pass
