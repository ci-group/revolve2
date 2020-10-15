from nca.core.actor.individual_factory import IndividualFactory
from nca.core.ecology import PopulationEcology
from nca.core.evolution.evolutionary_configurations import EvolutionaryConfiguration, GeneticAlgorithmConfiguration
from revolve.evosphere.evosphere import Evosphere
from revolve.evosphere.biosphere import Biosphere
from revolve.robot.birth_clinic import AgentBirthClinic
from simulation.simulation_manager import SimulationManager


class MockBiosphere(Biosphere):

    def __init__(self):
        super().__init__(PopulationEcology(), IndividualFactory(), birth_clinic=AgentBirthClinic())


class MockEvosphere(Evosphere):

    def __init__(self, biosphere: Biosphere = MockBiosphere(),
                 evolutionary_configuration: EvolutionaryConfiguration = GeneticAlgorithmConfiguration(),
                 simulation: SimulationManager = SimulationManager()):
        super().__init__(biosphere, evolutionary_configuration, simulation, False)
