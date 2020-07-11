from typing import List

from pyrevolve.evolutionary.robotics import Agents
from pyrevolve.evolutionary.robotics import BirthClinic
from pyrevolve.evolutionary.things.environment import Environment
from pyrevolve.patterns.configurations import EvoSphereConfiguration
from pyrevolve.ecology import Population
from pyrevolve.ecology import PopulationEcology
from pyrevolve.simulator.simulator import Simulator


class EvoSphere:

    def __init__(self, population_ecology: PopulationEcology, environments: List[Environment] = []):
        self.configuration = EvoSphereConfiguration()

        self.birth_clinic: BirthClinic = BirthClinic()
        self.population_ecology: PopulationEcology = population_ecology

        self.environments: List[Environment] = environments

        self.simulator = Simulator()

    def run(self):

        self.population_ecology.load()

        for _ in range(self.configuration.number_of_generations):

            agents: Agents = self.birth_clinic.create_agents()

            for environment in self.environments:
                self.population_ecology.create(agents, environment)
                for population in self.population_ecology.populations():
                    environment.population = population
                    self.simulator.evaluate(environment)

            self.population_ecology.select()

            #TODO evolve
            self.population_ecology.reproduce()

            self.population_ecology.evolve()

            self.population_ecology.export()

    def evolve(self, populations: List[Population]):
        pass
