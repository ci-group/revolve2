from abc import abstractmethod

from revolve.robot.robot import Robot
from simulation.handler import ExperimentHandler
from simulation.handler import SimulationHandler


class AnalyzeHandler(SimulationHandler):

    def __init__(self):
        super().__init__(ExperimentHandler())

    @abstractmethod
    def handle(self, robot: Robot):
        pass
