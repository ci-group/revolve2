from abc import abstractmethod

from revolve.robot.robot import Robot
from simulation.handler import MeasurementHandler
from simulation.handler import SimulationHandler


class ExperimentHandler(SimulationHandler):

    def __init__(self):
        super().__init__(MeasurementHandler())

    @abstractmethod
    def handle(self, robot: Robot):
        pass
