from abc import abstractmethod

from revolve.robot.robot import Robot
from simulation.handler import SimulationHandler


class MeasurementHandler(SimulationHandler):

    def __init__(self):
        super().__init__(None)

    @abstractmethod
    def handle(self, robot: Robot):
        pass
