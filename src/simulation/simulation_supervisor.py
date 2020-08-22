
import bisect
from enum import Enum, auto

from evosphere.mock_ecosphere import MockEcosphere
from nca.core.agent.individual import Individual
from revolve.evosphere.ecosphere import GeneticEcosphere
from simulation.simulator.simulator_command import SimulateCommand
from simulation.simulator.simulator_factory import SimulatorFactory
from simulation_test.simulator.mock_simulation_measures import MockSimulationMeasures
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

    def __init__(self, simulate_command: SimulateCommand, number_of_simulator_workers: int = 4):
        self.number_of_simulator_workers: int = number_of_simulator_workers
        self.robot_queue: PriorityQueue = PriorityQueue()
        self.simulator_connector: SimulatorConnector = SimulatorFactory(simulate_command).create()

    def work(self, individual: Individual, request_command: SimulateCommand):

        #self.robot_queue.insert(individual, request_command.task_priority.value)

        #if isinstance(request_command.ecosphere, SimulationEcosphere):
        pass