
import unittest

from revolve.robot.birth_clinic import RobotFactory
from revolve.robot.body.body import BodyRepresentation
from revolve.robot.brain.brain import BrainRepresentation
from simulation.simulator.simulator_command import SimulateCommand, TaskPriority
from src.simulation.simulation_manager import SimulationManager


from evosphere.mock_ecosphere import MockEcosphereB, MockEcosphereA, MockEcosphereC


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
                                         MockEcosphereA(), TaskPriority.MEDIUM))
        manager.simulate(SimulateCommand(RobotFactory(MockBodyRepresentation, MockBrainRepresentation).create(3),
                                         MockEcosphereB(), TaskPriority.MEDIUM))
        manager.simulate(SimulateCommand(RobotFactory(MockBodyRepresentation, MockBrainRepresentation).create(3),
                                         MockEcosphereC(), TaskPriority.MEDIUM))

        self.assertEqual(len(manager.supervisors.keys()), 3)
