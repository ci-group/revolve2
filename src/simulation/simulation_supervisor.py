
import bisect
from enum import Enum, auto

from nca.core.agent.individual import Individual
from simulation.simulator.simulator_command import SimulateCommand
from simulation.simulator.simulator_factory import SimulatorFactory
from src.simulation.simulator.simulation_connector import SimulatorConnector


class PriorityQueue:

    def __init__(self):
        self.queue = []

    def insert(self, data, priority):
        """ Insert a new element in the queue according to its priority. """
        bisect.insort(self.queue, (priority, data))

    def get(self):
        """ Pop the highest-priority element of the queue. """
        return self.queue.pop()[1]

    def size(self):
        return len(self.queue)


class RequestPriority(Enum):
    LOW = auto()
    MEDIUM = auto()
    HIGH = auto()


class SimulationSupervisor:

    def __init__(self, request_command: SimulateCommand, number_of_simulator_workers: int = 4):
        self.number_of_simulator_workers: int = number_of_simulator_workers
        self.robot_queue: PriorityQueue = PriorityQueue()
        self.simulator_connector: SimulatorConnector = SimulatorFactory(request_command).create()

    def work(self, individual: Individual, request_command: SimulateCommand):
        self.robot_queue.insert(individual, request_command.task_priority.value)
