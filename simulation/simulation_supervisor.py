from enum import Enum, auto
from queue import Queue, PriorityQueue

from revolve.robot.robot import Robot
from simulation.simulator.simulation_connector import SimulatorConnector
from simulation.simulator.simulator_helper import SimulatorFactory, RequestCommand


class RequestPriority(Enum):
    LOW = auto()
    MEDIUM = auto()
    HIGH = auto()


class SimulationSupervisor:

    def __init__(self, request_command: RequestCommand, number_of_simulator_workers: int = 4):
        self.number_of_simulator_workers: int = number_of_simulator_workers
        self.robot_queue: Queue = PriorityQueue()
        self.simulator_connector: SimulatorConnector = SimulatorFactory(request_command).create()

    def work(self, robot: Robot, request_command: RequestCommand):
        self.robot_queue.put((request_command.task_priority.value, robot))
