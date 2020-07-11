
# Multiple Selection
from abc import ABC, abstractmethod

from pyrevolve.evolutionary.robotics import Agents
from pyrevolve.shared.configurations import PopulationConfiguration


class Selection(ABC):

    def __init__(self):
        self.configuration = PopulationConfiguration()

    def select(self, agents: Agents) -> Agents:
        assert self.configuration.selection_size <= len(agents)

        new_agents = Agents()

        for _ in range(self.configuration.selection_size):

            new_agents.add(self.algorithm(agents))

        return new_agents

    @abstractmethod
    def algorithm(self, agents: Agents):
        raise Exception("Abstract selection did not implement algorithm")


class ParentSelection(Selection, ABC):

    def __init__(self):
        super().__init__()


class SurvivorSelection(Selection, ABC):

    def __init__(self):
        super().__init__()