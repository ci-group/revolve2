from abc import abstractmethod

from nca.revolve.robot.robot import Robot
from nca.simulation.simulator.simulator_connector import SimulatorConnector


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

