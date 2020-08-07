import string

from nca.core.agent.fitness import DisplacementFitness
from nca.core.abstract.configuration import Configuration


class PopulationConfiguration(Configuration):

    def __init__(self, filename: string = "population.config"):
        super().__init__(filename)
        self.selection_size: int = 5
        self.tournament_size: int = 2
        self.population_size: int = 10
        self.number_of_parents: int = 2


class SpeciationConfiguration(PopulationConfiguration):

    def __init__(self):
        super().__init__("speciation.config")


class EvolutionaryConfiguration(Configuration):

    def __init__(self):
        super().__init__("agent.config")
        self.number_of_robots = 10
        self.fitness = DisplacementFitness()


class EvoSphereConfiguration(Configuration):

    def __init__(self):
        super().__init__("revolve.config")
        self.number_of_environments = 1
        self.number_of_generations = 50


class RepresentationConfiguration(Configuration):

    def __init__(self):
        super().__init__("representation.config")
        self.genome_size = 10
        self.minimum_value = -1.0
        self.maximum_value = 1.0


class MutationConfiguration(Configuration):
    def __init__(self):
        super().__init__("mutation.config")
        self.mutation_probability = 0.25


class RecombinationConfiguration(Configuration):
    def __init__(self):
        super().__init__("mutation.config")
        self.recombination_probability = 0.5


class InitializationConfiguration(Configuration):
    def __init__(self):
        super().__init__("initialization.config")
        self.min_range = 0.0
        self.max_range = 1.0


class SimulatorConfiguration(Configuration):
    def __init__(self):
        super().__init__("simulator.config")
