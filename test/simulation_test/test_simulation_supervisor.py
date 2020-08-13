import unittest

from nca.core.agent.agents import Agents
from revolve.robot.robot import Robot
from simulation.simulator.simulator_command import SimulateCommand

from src.simulation.simulation_supervisor import SimulationSupervisor
from src.simulation.simulator.simulator_helper import SimulatorType, TaskPriority

from evosphere.TestEnvironment import TestEnvironment


class SimulationSupervisorTest(unittest.TestCase):

    def test_manager(self):
        request_command = SimulateCommand(Agents(), TestEnvironment(), TaskPriority.MEDIUM)

        supervisor = SimulationSupervisor(request_command)

        self.assertEqual(supervisor.robot_queue.size(), 0)

        supervisor.work(Robot(), request_command)

        self.assertEqual(supervisor.robot_queue.size(), 1)

    def test_priority(self):
        request_command_high = SimulateCommand(Agents(), TestEnvironment(), TaskPriority.HIGH)
        request_command_medium = SimulateCommand(Agents(), TestEnvironment(), TaskPriority.MEDIUM)
        request_command_low = SimulateCommand(Agents(), TestEnvironment(), TaskPriority.LOW)

        supervisor = SimulationSupervisor(request_command_medium)

        self.assertEqual(supervisor.robot_queue.size(), 0)

        low_robot = Robot()
        medium_robot = Robot()
        high_robot = Robot()
        supervisor.work(low_robot, request_command_low)
        supervisor.work(medium_robot, request_command_medium)
        supervisor.work(high_robot, request_command_high)

        self.assertEqual(supervisor.robot_queue.size(), 3)
        
        self.assertEqual(supervisor.robot_queue.get(), high_robot)
        self.assertEqual(supervisor.robot_queue.get(), medium_robot)
        self.assertEqual(supervisor.robot_queue.get(), low_robot)