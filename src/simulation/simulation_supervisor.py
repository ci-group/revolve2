import bisect
from enum import Enum, auto

from nca.core.actor.individual import Individual
from simulation.simulator.simulator_command import SimulationRequest
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

    def __init__(self, simulation_request: SimulationRequest):
        self.robot_queue: PriorityQueue = PriorityQueue()
        self.simulation_request: SimulationRequest = simulation_request
        self.simulator_connector: SimulatorConnector = SimulatorFactory(simulation_request).create()
        self.simulator_connector.start_simulator()

    def work(self, individual: Individual):
        #self.robot_queue.insert(individual, self.simulation_request.task_priority.value)
        self.simulator_connector.simulate(individual)
        #if isinstance(request_command.ecosphere, SimulationEcosphere):
        pass
