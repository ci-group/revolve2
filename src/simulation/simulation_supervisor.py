import bisect
import threading
import time
from enum import Enum, auto
from typing import Dict

from nca.core.actor.individual import Individual
from simulation.simulation_measures import SimulationMeasures
from simulation.simulator.simulator_command import SimulationRequest
from simulation.simulator.simulator_factory import SimulatorFactory
from simulation.simulator.adapter.simulation_adapters import SimulatorAdapter


class PriorityQueue:

    def __init__(self):
        self.queue = []

    def insert(self, data, priority):
        """ Insert a new element in the queue according to its priority. """
        bisect.insort(self.queue, (priority, data))

    def get(self):
        """ Pop the highest-priority element of the queue. """
        return self.queue.pop()

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
        self.results: Dict[int, SimulationMeasures] = {}

    def prepare_adapter(self) -> SimulatorAdapter:
        return SimulatorFactory(self.simulation_request).create()

    def manager(self):
        return self.results

    def work(self, individual: Individual) -> SimulationMeasures:
        return self.prepare_adapter().execute(individual)


class ThreadedSimulationSupervisor(SimulationSupervisor):

    def __init__(self, simulation_request: SimulationRequest, number_of_threads: int = 4):
        super().__init__(simulation_request)

        self.threads = []
        self.number_of_threads = number_of_threads

    def work(self, individual: Individual):
        self.robot_queue.insert(individual, self.simulation_request.task_priority.value)

    def _run(self):
        priority, individual = self.robot_queue.get()
        self.results[individual.id] = self.prepare_adapter().execute(individual)

    def manager(self):
        while self.robot_queue.size() > 0:
            if len(self.threads) < self.number_of_threads:
                thread: threading.Thread = threading.Thread(target=self._run)
                self.threads.append(thread)
                thread.start()
            else:
                finished = []
                for thread in self.threads:
                    if not thread.is_alive():
                        finished.append(thread)

                for finish in finished:
                    self.threads.remove(finish)

            time.sleep(0.1)

        return self.results
