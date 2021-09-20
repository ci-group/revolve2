import unittest

from test.evosphere.robot.test_birth_clinic import MockBirthClinic
from revolve2.nca.core.actor.actors import Actors
from revolve2.nca.core.actor.individual_factory import ActorFactory
from revolve2.revolve.robot.robot import Robot
from revolve2.simulation.simulator.simulator_command import SimulationRequest, TaskPriority

from src.simulation.simulation_supervisor import SimulationSupervisor

from test.evosphere.mock_ecosphere import MockEcosphere


class SimulationSupervisorTest(unittest.TestCase):

    def test_manager(self):

        request_command = SimulationRequest(Actors(), MockEcosphere(), MockBirthClinic(), TaskPriority.MEDIUM)

        supervisor = SimulationSupervisor(request_command)

        self.assertEqual(supervisor.robot_queue.size(), 0)

        supervisor.work(ActorFactory(actor_type=Robot).create(1)[0], request_command)

        self.assertEqual(supervisor.robot_queue.size(), 0)

    def test_priority(self):
        request_command_high = SimulationRequest(Actors(), MockEcosphere(), MockBirthClinic(), TaskPriority.HIGH)
        request_command_medium = SimulationRequest(Actors(), MockEcosphere(), MockBirthClinic(), TaskPriority.MEDIUM)
        request_command_low = SimulationRequest(Actors(), MockEcosphere(), MockBirthClinic(), TaskPriority.LOW)

        supervisor = SimulationSupervisor(request_command_medium)

        self.assertEqual(supervisor.robot_queue.size(), 0)
        robots = ActorFactory(actor_type=Robot).create(3)
        supervisor.work(robots[0], request_command_low)
        supervisor.work(robots[1], request_command_medium)
        supervisor.work(robots[2], request_command_high)

        self.assertEqual(supervisor.robot_queue.size(), 0)
