import string

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


class EvoSphereConfiguration(Configuration):

    def __init__(self):
        super().__init__("revolve.config")
        self.number_of_generations = 100
        self.number_of_agents = 10


class RepresentationConfiguration(Configuration):

    def __init__(self):
        super().__init__("representation.config")
        self.genome_size = 10


class OperatorConfiguration(Configuration):
    def __init__(self):
        super().__init__("operator.config")
        self.mutation_probability = 0.25
        self.recombination_probability = 0.25


class InitializationConfiguration(Configuration):
    def __init__(self):
        super().__init__("initialization.config")
        self.min_range = 0.0
        self.max_range = 2.0


class SimulatorConfiguration(Configuration):
    def __init__(self):
        super().__init__("simulation_test.config")
