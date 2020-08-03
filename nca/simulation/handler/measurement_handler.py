from abc import abstractmethod

from nca.revolve.robot.robot import Robot
from nca.simulation.handler.simulation_handler import SimulationHandler


class MeasurementHandler(SimulationHandler):

    def __init__(self):
        super().__init__(None)

    @abstractmethod
    def handle(self, robot: Robot):
        pass
