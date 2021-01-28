import unittest

from nca.core.actor.agent import Agent
from nca.core.actor.individual import Individual
from nca.core.genome.genotype import Genotype
from revolve.evosphere.evoman import EvomanEcosphere, EvomanEvosphere
from revolve.robot.brain.neural_network_brain import NeuralNetworkBrain
from simulation.simulation_supervisor import ThreadedSimulationSupervisor
from test.evosphere.mock_evosphere import MockEvosphere
from test.evosphere.robot.test_birth_clinic import MockBirthClinic
from nca.core.actor.actors import Actors
from simulation.simulator.simulator_command import SimulationRequest, TaskPriority


class SimulationSupervisorTest(unittest.TestCase):

    def test_evoman(self):
        request_command = SimulationRequest(Actors(), EvomanEcosphere(), MockBirthClinic(), TaskPriority.MEDIUM)

        supervisor = ThreadedSimulationSupervisor(request_command, 4)
        for i in range(10):
            brain = NeuralNetworkBrain()
            genotype = Genotype(brain.representation())
            agent = Agent(genotype, brain)
            supervisor.work(agent)

        results = supervisor.manager()
        print([result.fitness for result in results])
        self.assertTrue(True)

