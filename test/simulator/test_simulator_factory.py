import unittest

from simulation.simulator.simulator_helper import SimulatorType, RequestCommand, SimulatorFactory, TaskPriority

from test.evosphere.TestEnvironment import TestEnvironment
from test.simulator.test_connector_adapter import TestConnectorAdapter


class SimulationSupervisorTest(unittest.TestCase):

    def test_none(self):
        request_command = RequestCommand(TestEnvironment(), SimulatorType.NONE, TaskPriority.MEDIUM)

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