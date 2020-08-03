from abc import abstractmethod

from nca.revolve.robot.robot import Robot
from nca.simulation.handler.measurement_handler import MeasurementHandler
from nca.simulation.handler.simulation_handler import SimulationHandler


class ExperimentHandler(SimulationHandler):

    def __init__(self):
        super().__init__(MeasurementHandler())

    @abstractmethod
    def handle(self, robot: Robot):
        pass
