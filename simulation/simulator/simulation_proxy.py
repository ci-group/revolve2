from abc import abstractmethod

from revolve.robot.robot import Robot
from simulation.simulator import SimulatorConnector


class SimulationProxy:

    def __init__(self, connector: SimulatorConnector):
        self.connector: SimulatorConnector = connector

    def evaluate(self):
        pass


    @abstractmethod
    def insert_robot(self, robot: Robot):
        pass

    @abstractmethod
    def remove_robot(self, robot: Robot):
        pass

