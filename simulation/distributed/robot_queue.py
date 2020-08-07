from queue import Queue
from typing import List

from revolve.robot.robot import Robot
from simulation import SimulationWorker


class RobotQueue(Queue):

    def __init__(self, workers: List[SimulationWorker]):
        self.queue = Queue()
        for worker in workers:
            self.queue.put(worker)

    def work(self, robot: Robot):
        worker = self.queue.get()

        if worker is not None:
            worker.work(robot)
            self.queue.put(worker)
            return True

        return False
