import string

from pyrevolve.evolutionary.fitness import DisplacementFitness
from pyrevolve.shared.configuration import Configuration


class PopulationConfiguration(Configuration):

    def __init__(self, filename: string = "population.config"):
        super().__init__(filename)
        self.selection_size: int = 5
        self.tournament_size: int = 2


class SpeciationConfiguration(PopulationConfiguration):

    def __init__(self):
        super().__init__("speciation.config")


class RobotConfiguration(Configuration):

    def __init__(self):
        super().__init__("agent.config")
        self.number_of_robots = 10
        self.fitness = DisplacementFitness()


class EvoSphereConfiguration(Configuration):

    def __init__(self):
        super().__init__("evosphere.config")
        self.number_of_environments = 1
