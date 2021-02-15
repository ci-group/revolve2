import unittest

from nca.core.actor.individual_factory import ActorFactory
from revolve.robot.robot import Robot
from simulation.simulator.adapter.simulation_adapters import SimulatorState
from test.evosphere.mock_ecosphere import MockEcosphere
from test.simulation.simulator.test_connector_adapter import TestConnectorAdapter


class TestSimulatorConnector(unittest.TestCase):

    def test_connector_port(self):
        connector_1 = TestConnectorAdapter(MockEcosphere())
        connector_2 = TestConnectorAdapter(MockEcosphere())

        self.assertNotEqual(connector_1.port, connector_2.port)

    def test_connector_ready(self):
        connector = TestConnectorAdapter(MockEcosphere())
        self.assertEqual(connector.state, SimulatorState.WAITING)

        connector.play()
        print(connector.state)
        self.assertEqual(connector.state, SimulatorState.RUNNING)

        connector.pause()
        self.assertEqual(connector.state, SimulatorState.PAUSED)

        connector.play()
        self.assertEqual(connector.state, SimulatorState.RUNNING)

        connector.stop()
        self.assertEqual(connector.state, SimulatorState.READY)

        connector.play()
        self.assertEqual(connector.state, SimulatorState.RUNNING)

    def test_connector_robot(self):
        connector = TestConnectorAdapter(MockEcosphere())

        robot = ActorFactory(actor_type=Robot).create(1)[0]
        connector.execute(robot)

        self.assertTrue(True)

    def test_connector_robot_changed(self):
        connector = TestConnectorAdapter(MockEcosphere())

        robot = ActorFactory(actor_type=Robot).create(1)[0]
        connector.execute(robot)
