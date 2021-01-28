import unittest

from test.evosphere.robot.test_birth_clinic import MockBirthClinic
from nca.core.actor.actors import Actors
from revolve.evosphere.ecosphere import GazeboEcosphere
from simulation.simulator.adapter.gazebo_simulator_adapter import GazeboConnectorAdapter
from simulation.simulator.simulator_command import SimulationRequest, TaskPriority
from simulation.simulator.simulator_factory import SimulatorFactory

from test.evosphere.mock_ecosphere import MockEcosphere
from test.simulation.simulator.test_connector_adapter import TestConnectorAdapter


class SimulationSupervisorTest(unittest.TestCase):

    def test_none(self):
        request_command = SimulationRequest(Actors(), MockEcosphere(), MockBirthClinic(), TaskPriority.MEDIUM)

        factory = SimulatorFactory(request_command)
        connector = factory.create()

        self.assertIsInstance(connector, TestConnectorAdapter)

    def test_gazebo(self):
        request_command = SimulationRequest(Actors(), GazeboEcosphere(), MockBirthClinic(), TaskPriority.MEDIUM)

        factory = SimulatorFactory(request_command)
        connector = factory.create()

        self.assertIsInstance(connector, GazeboConnectorAdapter)
