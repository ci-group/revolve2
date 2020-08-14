import unittest

from nca.core.agent.agents import Agents
from simulation.simulator.simulator_command import SimulateCommand
from simulation.simulator.simulator_factory import SimulatorFactory
from src.simulation.simulator.simulator_helper import TaskPriority

from evosphere.TestEcosphere import TestEcosphere
from simulation_test.simulator.test_connector_adapter import TestConnectorAdapter


class SimulationSupervisorTest(unittest.TestCase):

    def test_none(self):
        request_command = SimulateCommand(Agents(), TestEcosphere(), TaskPriority.MEDIUM)

        factory = SimulatorFactory(request_command)
        connector = factory.create()

        self.assertIsInstance(connector, TestConnectorAdapter)

    """
    def test_gazebo(self):
        request_command = RequestCommand(TestEnvironment(), SimulatorType.GAZEBO, TaskPriority.MEDIUM)

        factory = SimulatorFactory(request_command)
        connector = factory.create()

        self.assertIsInstance(connector, GazeboConnectorAdapter)
    """