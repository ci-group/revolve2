from evosphere.mock_ecosphere import MockEcosphere
from nca.core.agent.individual_factory import IndividualFactory
from nca.core.ecology import PopulationEcology
from nca.core.evolution.evolutionary_algorithm import EvolutionaryAlgorithm
from nca.core.genome.initialization import IntegerInitialization
from nca.core.genome.representations.valued_representation import ValuedRepresentation
from revolve.evosphere.evosphere import Evosphere
from revolve.evosphere.biosphere import Biosphere
from simulation.simulation_manager import SimulationManager


class MockBiosphere(Biosphere):

    def __init__(self):
        super().__init__(PopulationEcology(), IndividualFactory(ValuedRepresentation).initialize(IntegerInitialization()), [MockEcosphere()])


class MockEvosphere(Evosphere):

    def __init__(self, biosphere: Biosphere = MockBiosphere(),
                 evolutionary_algorithm: EvolutionaryAlgorithm = EvolutionaryAlgorithm(),
                 simulation: SimulationManager = SimulationManager()):
        super().__init__(biosphere, evolutionary_algorithm, simulation, False)
