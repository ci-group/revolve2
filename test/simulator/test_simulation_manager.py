
import unittest

from nca.core.agent.agents import Agents
from revolve.robot.robot import Robot
from simulation.simulation_manager import SimulationManager

from simulation.simulation_supervisor import SimulationSupervisor
from simulation.simulator.simulator_helper import SimulatorType, RequestCommand, TaskPriority

from test.evosphere.TestEnvironment import TestEnvironment, TestEnvironmentB, TestEnvironmentA, TestEnvironmentC


class SimulationManagerTest(unittest.TestCase):

    def test_supervisor(self):
        agents_a = Agents()
        agents_b = Agents()
        agents_c = Agents()

        request_command_a = RequestCommand(TestEnvironmentA(), SimulatorType.NONE, TaskPriority.MEDIUM)
        request_command_b = RequestCommand(TestEnvironmentB(), SimulatorType.NONE, TaskPriority.MEDIUM)
        request_command_c = RequestCommand(TestEnvironmentC(), SimulatorType.NONE, TaskPriority.MEDIUM)

        manager = SimulationManager()
        manager.simulate(agents_a, request_command_a)
        manager.simulate(agents_b, request_command_b)
        manager.simulate(agents_c, request_command_c)

        self.assertEqual(len(manager.supervisors.keys()), 3)

        agents_d = Agents()
        request_command_d = RequestCommand(TestEnvironmentA(), SimulatorType.NONE, TaskPriority.MEDIUM)
        manager.simulate(agents_d, request_command_d)

        self.assertEqual(len(manager.supervisors.keys()), 3)
