
import unittest

from nca.core.agent.agents import Agents
from revolve.robot.birth_clinic import BirthClinic
from simulation.simulator.simulator_command import SimulateCommand
from src.simulation.simulation_manager import SimulationManager

from src.simulation.simulator.simulator_helper import TaskPriority

from evosphere.TestEnvironment import TestEnvironmentB, TestEnvironmentA, TestEnvironmentC


class SimulationManagerTest(unittest.TestCase):

    def test_supervisor(self):

        manager = SimulationManager()
        manager.simulate(SimulateCommand(BirthClinic().create(3), TestEnvironmentA(), TaskPriority.MEDIUM))
        manager.simulate(SimulateCommand(BirthClinic().create(3), TestEnvironmentB(), TaskPriority.MEDIUM))
        manager.simulate(SimulateCommand(BirthClinic().create(3), TestEnvironmentC(), TaskPriority.MEDIUM))

        self.assertEqual(len(manager.supervisors.keys()), 3)
