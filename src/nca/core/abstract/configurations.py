import string

from nca.core.abstract.configuration import Configuration


class PopulationConfiguration(Configuration):

    def __init__(self, filename: string = "population.config"):
        super().__init__(filename)
        self.selection_size: int = 5
        self.tournament_size: int = 2
        self.population_size: int = 25
        self.number_of_parents: int = 2


class SpeciationConfiguration(PopulationConfiguration):

    def __init__(self):
        super().__init__("speciation.config")


class EvolutionConfiguration(Configuration):

    def __init__(self):
        super().__init__("revolve.config")
        self.number_of_generations = 250
        self.number_of_agents = 25


class EvoSphereConfiguration(Configuration):

    def __init__(self):
        super().__init__("revolve.config")
        self.number_of_generations = 100
        self.number_of_agents = 100


class RepresentationConfiguration(Configuration):

    def __init__(self):
        super().__init__("representation.config")
        self.genome_size = 10
        self.number_of_chromosomes = 1


class OperatorConfiguration(Configuration):
    def __init__(self):
        super().__init__("operator.config")
        self.mutation_probability = 0.25
        self.recombination_probability = 0.25
        self.min_value = -1
        self.max_value = 1


class InitializationConfiguration(Configuration):
    def __init__(self):
        super().__init__("initialization.config")
        self.min_range = -1.0
        self.max_range = 1.0


class SimulatorConfiguration(Configuration):
    def __init__(self):
        super().__init__("simulation_test.config")
