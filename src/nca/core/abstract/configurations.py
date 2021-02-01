import string

from nca.core.abstract.configuration import Configuration


class PopulationConfiguration(Configuration):

    def __init__(self, filename: string = "population.config"):
        super().__init__(filename)
        self.selection_size: int = 50
        self.tournament_size: int = 2
        self.population_size: int = 100
        self.number_of_parents: int = 2


class SpeciationConfiguration(PopulationConfiguration):

    def __init__(self):
        super().__init__("speciation.config")
        self.number_of_species = 3


class EvolutionConfiguration(Configuration):

    def __init__(self):
        super().__init__("revolve.config")
        self.number_of_generations = 500
        self.number_of_agents = 100


class EvoSphereConfiguration(Configuration):

    def __init__(self):
        super().__init__("revolve.config")
        self.number_of_generations = 20
        self.number_of_agents = 20


class RepresentationConfiguration(Configuration):

    def __init__(self, size: int = 10, chromosomes: int = 1):
        super().__init__("representation.config")
        self.genome_size = size
        self.number_of_chromosomes = chromosomes


class OperatorConfiguration(Configuration):
    def __init__(self):
        super().__init__("operator.config")
        self.mutation_probability = 0.75
        self.recombination_probability = 0.75
        self.min_value = -1
        self.max_value = 1


class InitializationConfiguration(Configuration):
    def __init__(self, min_range: float = -1, max_range: float = 1):
        super().__init__("initialization.config")
        self.min_range: float = min_range
        self.max_range: float = max_range


class SimulatorConfiguration(Configuration):
    def __init__(self):
        super().__init__("simulation_test.config")
