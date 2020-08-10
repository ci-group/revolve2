import unittest

from revolve.robot.robot import Robot

from src.simulation.simulation_supervisor import SimulationSupervisor
from src.simulation.simulator.simulator_helper import SimulatorType, RequestCommand, TaskPriority

from test.evosphere.TestEnvironment import TestEnvironment


class SimulationSupervisorTest(unittest.TestCase):

    def test_manager(self):
        request_command = RequestCommand(TestEnvironment(), SimulatorType.NONE, TaskPriority.MEDIUM)

        supervisor = SimulationSupervisor(request_command)

        self.assertEqual(supervisor.robot_queue.qsize(), 0)

        supervisor.work(Robot(), request_command)

        self.assertEqual(supervisor.robot_queue.qsize(), 1)

    def test_priority(self):
        request_command_high = RequestCommand(TestEnvironment(), SimulatorType.NONE, TaskPriority.HIGH)
        request_command_medium = RequestCommand(TestEnvironment(), SimulatorType.NONE, TaskPriority.MEDIUM)
        request_command_low = RequestCommand(TestEnvironment(), SimulatorType.NONE, TaskPriority.LOW)

        supervisor = SimulationSupervisor(request_command_medium)

        self.assertEqual(supervisor.robot_queue.qsize(), 0)

        low_robot = Robot()
        medium_robot = Robot()
        high_robot = Robot()
        supervisor.work(low_robot, request_command_low)
        supervisor.work(medium_robot, request_command_medium)
        supervisor.work(high_robot, request_command_high)

        self.assertEqual(supervisor.robot_queue.qsize(), 3)
        
        self.assertEqual(supervisor.robot_queue.get()[1], high_robot)
        self.assertEqual(supervisor.robot_queue.get()[1], medium_robot)
        self.assertEqual(supervisor.robot_queue.get()[1], low_robot)