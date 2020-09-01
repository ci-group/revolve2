
import unittest

from evosphere.robot.test_birth_clinic import MockBirthClinic
from revolve.robot.birth_clinic import BirthClinic
from revolve.robot.body.body_builder import MockBodyBuilder
from revolve.robot.brain.brain_builder import MockBrainBuilder
from simulation.simulator.simulator_command import SimulateCommand, TaskPriority
from src.simulation.simulation_manager import SimulationManager


from evosphere.mock_ecosphere import MockEcosphereB, MockEcosphereA, MockEcosphereC


class SimulationManagerTest(unittest.TestCase):

    def test_supervisor(self):

        manager = SimulationManager()
        manager.simulate(SimulateCommand(MockBirthClinic().build(3),
                                         MockEcosphereA(), TaskPriority.MEDIUM))
        manager.simulate(SimulateCommand(BirthClinic(brain_builder_type=MockBrainBuilder,
                                                     body_builder_type=MockBodyBuilder).create(3),
                                         MockEcosphereB(), TaskPriority.MEDIUM))
        manager.simulate(SimulateCommand(BirthClinic(brain_builder_type=MockBrainBuilder,
                                                     body_builder_type=MockBodyBuilder).create(3),
                                         MockEcosphereC(), TaskPriority.MEDIUM))

        self.assertEqual(len(manager.supervisors.keys()), 3)
