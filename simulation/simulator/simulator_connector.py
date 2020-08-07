from abc import abstractmethod

from simulation.simulator import Connection
from simulation import Measures


class SimulatorConnector:

    def __init__(self, port: int, connection: Connection, measures: Measures):
        self.active: bool = False
        self.port = port
        self.connection = connection
        self.measures = measures

    @abstractmethod
    def start_simulator(self):
        self.active = True

    @abstractmethod
    def restart_simulator(self):
        if self.active:
            self.stop_simulator()
        self.start_simulator()

    @abstractmethod
    def stop_simulator(self):
        self.active = False

    @abstractmethod
    def add_object(self, element: object):
        pass

    @abstractmethod
    def remove_object(self, element: object):
        pass
