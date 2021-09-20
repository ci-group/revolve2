import bisect
from threading import Thread
import time
import matplotlib.pyplot as plt
import numpy as np

from enum import Enum, auto
from typing import Dict, List

from revolve2.nca.core.actor.individual import Individual


from revolve2.simulation.simulation_measures import SimulationMeasures
from revolve2.simulation.simulator.simulator_command import SimulationRequest, MockSimulationRequest
from revolve2.simulation.simulator.simulator_factory import SimulatorFactory
from revolve2.simulation.simulator.adapter.simulation_adapters import SimulatorAdapter


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

        self.adapter: SimulatorAdapter = self._prepare_adapter()

    def _prepare_adapter(self) -> SimulatorAdapter:
        return SimulatorFactory(self.simulation_request).create()

    def manager(self):
        return self.results

    def work(self, individual: Individual) -> SimulationMeasures:
        self.results[individual.id] = self.adapter.execute(individual)


class ThreadedSimulationSupervisor(SimulationSupervisor):

    def __init__(self, simulation_request: SimulationRequest):
        super().__init__(simulation_request)

        self.number_of_threads = simulation_request.number_of_workers
        self.threads: List[Thread] = [None for _ in range(self.number_of_threads)]
        self.active_threads_count = 0
        self.adapters = [self._prepare_adapter() for i in range(self.number_of_threads)]

        self.logger: ThreadedSimulationLogger = ThreadedSimulationLogger(self.number_of_threads)

    def work(self, individual: Individual):
        self.robot_queue.insert(individual, self.simulation_request.task_priority.value)

    def _run(self, adapter_index):
        priority, individual = self.robot_queue.get()
        self.results[individual.id] = self.adapters[adapter_index].execute(individual)
        self.threads[adapter_index] = None
        self.active_threads_count -= 1
        print("Finished ", individual.id)

    def manager(self):
        print("simulating robot: ", self.robot_queue.size())
        while self.robot_queue.size() > 0:
            self.logger.add(self.active_threads_count)
            if self.active_threads_count < self.number_of_threads:
                if None not in self.threads:
                    continue
                adapter_index = self.threads.index(None)
                self.active_threads_count += 1
                self.threads[adapter_index]: Thread = Thread(target=self._run, args=(adapter_index, ))
                self.threads[adapter_index].start()

            time.sleep(0.1)

        while self.active_threads_count > 0:
            time.sleep(0.1)

        return self.results


class ThreadedSimulationLogger:

    def __init__(self, max_threads):
        self.max_threads: int = max_threads
        self.threads_active: List[int] = [0]

    def add(self, active_threads: int):
        self.threads_active.append(active_threads)

    def plot_active(self):
        plt.plot(self.threads_active)
        plt.show()

    def plot_load(self):
        plt.plot(np.array(self.threads_active) / self.max_threads)
        plt.show()


if __name__ == "__main__":
    from revolve2.nca.core.genome.representations.representation import Representation
    from revolve2.nca.core.genome.operators.initialization import UniformInitialization

    individuals = [Individual(Representation(UniformInitialization())) for i in range(100)]

    supervisor: ThreadedSimulationSupervisor = ThreadedSimulationSupervisor(MockSimulationRequest())

    for individual in individuals:
        supervisor.work(individual)

    supervisor.manager()

    supervisor.logger.plot_load()

    print("finished")
