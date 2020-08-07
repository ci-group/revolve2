from revolve.robot.robot import Robot
from simulation import RobotQueue
from simulation import SimulationWorker


class SimulationSupervisor:

    def __init__(self, number_of_workers: int = 4):
        self.workers_queue: RobotQueue = RobotQueue([SimulationWorker() for _ in range(number_of_workers)])

    def work(self, robot: Robot):
        self.workers_queue.work(robot)
