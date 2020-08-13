import unittest

from revolve.robot.robot import Robot
from simulation.simulator.simulation_connector import SimulatorState
from evosphere.TestEnvironment import TestEnvironment
from simulation_test.simulator.test_connector_adapter import TestConnectorAdapter


class TestSimulatorConnector(unittest.TestCase):

    def test_connector_port(self):
        connector_1 = TestConnectorAdapter(TestEnvironment())
        connector_2 = TestConnectorAdapter(TestEnvironment())

        self.assertNotEqual(connector_1.port, connector_2.port)

    def test_connector_ready(self):
        connector = TestConnectorAdapter(TestEnvironment())

        self.assertEqual(connector.state, SimulatorState.READY)

        connector.stop()

        self.assertEqual(connector.state, SimulatorState.STOPPED)

        connector.restart()

        self.assertEqual(connector.state, SimulatorState.READY)

        connector.restart()

        self.assertEqual(connector.state, SimulatorState.READY)

    def test_connector_robot(self):
        connector = TestConnectorAdapter(TestEnvironment())

        robot = Robot()
        connector.add_robot(robot)

        connector.remove_robot(robot)

        self.assertTrue(True)

    def test_connector_robot_changed(self):
        connector = TestConnectorAdapter(TestEnvironment())

        robot = Robot()
        connector.add_robot(robot)

        self.assertRaises(Exception, connector.remove_robot, Robot())
