from abc import abstractmethod

from nca.revolve.robot.robot import Robot
from nca.simulation.handler.experiment_handler import ExperimentHandler
from nca.simulation.handler.simulation_handler import SimulationHandler


class AnalyzeHandler(SimulationHandler):

    def __init__(self):
        super().__init__(ExperimentHandler())

    @abstractmethod
    def handle(self, robot: Robot):
        pass
