from nca.core.evolution.evolutionary_configurations import EvolutionaryConfiguration, GeneticAlgorithmConfiguration
from revolve.evosphere.biosphere import Biosphere
from revolve.evosphere.evosphere import Evosphere
from simulation.simulation_manager import SimulationManager


class RobogenBiosphere(Biosphere):
    pass


class RobotEvosphere(Evosphere):
    def __init__(self, biosphere: Biosphere = RobogenBiosphere(),
                 evolutionary_configuration: EvolutionaryConfiguration = GeneticAlgorithmConfiguration(),
                 simulation: SimulationManager = SimulationManager(), debug=True):
        super().__init__(biosphere, evolutionary_configuration, simulation, debug)
