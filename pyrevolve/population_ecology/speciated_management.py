from pyrevolve.agent.agents import Agents
from pyrevolve.patterns.configuration import PopulationConfiguration
from pyrevolve.patterns.sequential_identifier import SpeciationIdentifier
from pyrevolve.population_ecology.population_management import PopulationManagement
from pyrevolve.population_ecology.selection import Selection
from pyrevolve.population_ecology.genus import Genus


class SpeciatedManagement(PopulationManagement):

    def __init__(self,
                 configuration: PopulationConfiguration,
                 selection: Selection
                 ):
        super().__init__(configuration, selection)
        self.identifier = SpeciationIdentifier.Instance()
        self.genus: Genus = None

    def create(self, agents: Agents):
        self.genus: Genus = Genus(self.identifier.increment())
        # TODO setup species
        pass

    def speciate(self):
        # TODO speciation algorithm
        pass
