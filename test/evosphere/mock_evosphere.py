from nca.core.agent.individual_factory import IndividualFactory, AgentFactory
from nca.core.ecology import PopulationEcology
from nca.core.evolution.evolutionary_algorithm import EvolutionaryAlgorithm
from nca.core.genome.representations.valued_representation import IntegerRepresentation
from revolve.evosphere.evosphere import EvoSphere
from revolve.evosphere.world import World
from simulation.simulation_manager import SimulationManager


class MockWorld(World):

    def __init__(self):
        super().__init__(PopulationEcology(), IndividualFactory(IntegerRepresentation))


class MockEvoSphere(EvoSphere):

    def __init__(self, world: World = MockWorld(),
                 evolutionary_algorithm: EvolutionaryAlgorithm = EvolutionaryAlgorithm(),
                 simulation: SimulationManager = SimulationManager()):
        super().__init__(world, evolutionary_algorithm, simulation)


def run_mock():
    evosphere = MockEvoSphere()
    evosphere.evolve()


if __name__ == "__main__":
    run_mock()
