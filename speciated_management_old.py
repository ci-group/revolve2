from typing import List

from pyrevolve.evolutionary.robotics import Agents
from pyrevolve.patterns.configurations import SpeciationConfiguration
from pyrevolve.ecology import Population
from pyrevolve.ecology import PopulationManagement
from pyrevolve.evolutionary.algorithm import Selection
from pyrevolve.ecology import Genus


class SpeciatedManagement(PopulationManagement):

    def __init__(self, selection: Selection, configuration=SpeciationConfiguration()):
        super().__init__(selection, configuration)

        self.genus: Genus = None




    def populations(self) -> List[Population]:
        if self.genus is None:
            raise Exception("Genus uninitialized")

        return self.genus.species

    def create(self, agents: Agents):
        self.genus: Genus = Genus(self.identifier.increment())

        for agent in agents:
            self.genus.assign(agent)

        pass

    def speciate(self):
        agents: Agents = self.agents()

        self.create(agents)
