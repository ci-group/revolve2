import os
import string
from typing import List

from revolve2.nca.core.actor.agent import Agent
from revolve2.nca.core.actor.fitness import Fitness
from revolve2.nca.core.actor.individual_factory import ActorFactory
from revolve2.nca.core.ecology import PopulationEcology
from revolve2.nca.core.evolution.evolutionary_configurations import EvolutionaryConfiguration, GeneticAlgorithmConfiguration
from revolve2.nca.core.genome.genotype import Genotype
from revolve2.revolve.evosphere.biosphere import Biosphere, IndividualBiosphere

from revolve2.revolve.evosphere.ecosphere import Ecosphere
from revolve2.revolve.evosphere.evosphere import AgentEvosphere
from revolve2.revolve.robot.birth_clinic import BirthClinic, AgentBirthClinic
from revolve2.revolve.robot.brain.brain import Brain
from revolve2.revolve.robot.brain.brain_builder import BrainBuilder, AgentBrainBuilder
from revolve2.revolve.robot.brain.neural_network_brain import NeuralNetworkBrain
from revolve2.revolve.robot.development_request import BrainDevelopmentRequest
from revolve2.simulation.simulation_manager import SimulationManager
from revolve2.simulation.simulator.simulator_type import SimulatorType


class EvomanBrainBuilder(AgentBrainBuilder):
    def __init__(self, brain_type: type(Brain) = NeuralNetworkBrain):
        super().__init__(brain_type)

    def create(self, brain_development_request: BrainDevelopmentRequest) -> Brain:
        return self.brain_type()


class EvomanBirthClinic(AgentBirthClinic):
    def __init__(self, brain_builder: BrainBuilder = EvomanBrainBuilder()):
        super().__init__()
        self.brain_builder: BrainBuilder = brain_builder

    def _create(self, development_request: BrainDevelopmentRequest) -> object:
        brain: NeuralNetworkBrain = self.brain_builder.create(development_request)
        genotype = Genotype(brain.representation())
        agent = Agent.create(development_request.individual, genotype, brain)
        return agent


class EvomanEcosphere(Ecosphere):

    def __init__(self, filename: string = "evoman", fitness_type: type(Fitness) = None):
        super().__init__(filename, fitness_type, SimulatorType.EVOMAN)

        if not os.path.exists(filename):
            os.makedirs(filename)

        self.filename = filename


class EvomanBiosphere(IndividualBiosphere):

    def __init__(self,
                 population_ecology: PopulationEcology = PopulationEcology(),
                 actor_factory: ActorFactory = ActorFactory(),
                 birth_clinic: BirthClinic = EvomanBirthClinic(),
                 ecospheres: List[Ecosphere] = None):
        if ecospheres is None:
            ecospheres = [EvomanEcosphere()]
        super().__init__(population_ecology, actor_factory, birth_clinic, ecospheres)


class EvomanEvosphere(AgentEvosphere):
    def __init__(self, biosphere: Biosphere = EvomanBiosphere(),
                 evolutionary_configuration: EvolutionaryConfiguration = GeneticAlgorithmConfiguration(),
                 simulation: SimulationManager = SimulationManager(), debug=True):
        super().__init__(biosphere, evolutionary_configuration, simulation, debug)
