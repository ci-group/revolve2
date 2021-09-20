import unittest

from revolve2.nca.core.actor.agent import Agent
from revolve2.nca.core.evolution.evolutionary_configurations import GeneticAlgorithmConfiguration
from revolve2.nca.core.genome.genotype import Genotype
from revolve2.nca.evolution import Evolution
from revolve2.revolve.evosphere.biosphere import Biosphere
from revolve2.revolve.evosphere.evoman import EvomanEcosphere, EvomanBiosphere
from revolve2.revolve.evosphere.evosphere import Evosphere
from revolve2.revolve.robot.brain.neural_network_brain import NeuralNetworkBrain
from revolve2.simulation.simulation_manager import SimulationManager
from revolve2.simulation.simulation_supervisor import ThreadedSimulationSupervisor
from test.evosphere.robot.test_birth_clinic import MockBirthClinic
from revolve2.nca.core.actor.actors import Actors
from revolve2.simulation.simulator.simulator_command import SimulationRequest, TaskPriority
from visualization.population_visualization import PopulationFitnessVisualization


class SimulationSupervisorTest(unittest.TestCase):

    def test_evoman(self):
        request_command = SimulationRequest(Actors(), EvomanEcosphere(), MockBirthClinic(), TaskPriority.MEDIUM)

        supervisor = ThreadedSimulationSupervisor(request_command)
        for i in range(10):
            brain = NeuralNetworkBrain()
            genotype = Genotype(brain.representation())
            agent = Agent(genotype, brain)
            supervisor.work(agent)

        results = supervisor.manager()
        self.assertTrue(True)

    def test_evoman_evolution(self):
        biosphere = EvomanBiosphere()
        simulation_manager = SimulationManager(threads=20)
        evosphere = Evosphere(biosphere=biosphere,
                              evolutionary_configuration=GeneticAlgorithmConfiguration(),
                              simulation=simulation_manager)
        population_ecology = evosphere.evolve()

        for population in population_ecology.management.populations():
            visualization = PopulationFitnessVisualization(population=population)
            visualization.prepare()
            visualization.visualize(True)



