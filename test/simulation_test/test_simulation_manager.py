
import unittest

from nca.core.agent.agents import Agents
from revolve.robot.birth_clinic import RobotFactory
from revolve.robot.body.body import BodyRepresentation
from revolve.robot.brain.brain import BrainRepresentation
from simulation.simulator.simulator_command import SimulateCommand
from src.simulation.simulation_manager import SimulationManager

from src.simulation.simulator.simulator_helper import TaskPriority

from evosphere.TestEcosphere import TestEcosphereB, TestEcosphereA, TestEcosphereC


class MockBodyRepresentation(BodyRepresentation):

    def develop(self):
        pass


class MockBrainRepresentation(BrainRepresentation):

    def develop(self):
        pass


class SimulationManagerTest(unittest.TestCase):

    def test_supervisor(self):

        manager = SimulationManager()
        manager.simulate(SimulateCommand(RobotFactory(MockBodyRepresentation, MockBrainRepresentation).create(3),
                                         TestEcosphereA(), TaskPriority.MEDIUM))
        manager.simulate(SimulateCommand(RobotFactory(MockBodyRepresentation, MockBrainRepresentation).create(3),
                                         TestEcosphereB(), TaskPriority.MEDIUM))
        manager.simulate(SimulateCommand(RobotFactory(MockBodyRepresentation, MockBrainRepresentation).create(3),
                                         TestEcosphereC(), TaskPriority.MEDIUM))

        self.assertEqual(len(manager.supervisors.keys()), 3)
