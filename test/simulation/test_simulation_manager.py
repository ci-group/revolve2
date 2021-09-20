
import unittest

from test.evosphere.robot.test_birth_clinic import MockBirthClinic
from revolve2.nca.core.actor.individual_factory import ActorFactory
from revolve2.simulation.simulator.simulator_command import SimulationRequest, TaskPriority
from src.simulation.simulation_manager import SimulationManager


from test.evosphere.mock_ecosphere import MockEcosphereA


class SimulationManagerTest(unittest.TestCase):

    def test_supervisor(self):

        manager = SimulationManager()
        agents = ActorFactory().create(3)
        ecosphere = MockEcosphereA()
        birth_clinic = MockBirthClinic()
        manager.simulate(SimulationRequest(agents, ecosphere, birth_clinic, TaskPriority.MEDIUM))

