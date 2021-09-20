from revolve2.nca.core.actor.individual_factory import ActorFactory
from revolve2.nca.core.ecology import PopulationEcology
from revolve2.nca.core.evolution.evolutionary_configurations import EvolutionaryConfiguration, GeneticAlgorithmConfiguration

from revolve2.revolve.evosphere.evosphere import Evosphere
from revolve2.revolve.evosphere.biosphere import Biosphere
from revolve2.revolve.robot.birth_clinic import IndividualBirthClinic
from revolve2.simulation.simulation_manager import SimulationManager


class MockBiosphere(Biosphere):

    def __init__(self):
        super().__init__(PopulationEcology(), ActorFactory(), birth_clinic=IndividualBirthClinic())


class MockEvosphere(Evosphere):

    def __init__(self, biosphere: Biosphere = MockBiosphere(),
                 evolutionary_configuration: EvolutionaryConfiguration = GeneticAlgorithmConfiguration(),
                 simulation: SimulationManager = SimulationManager()):
        super().__init__(biosphere, evolutionary_configuration, simulation, False)
