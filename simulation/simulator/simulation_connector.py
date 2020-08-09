from abc import abstractmethod, ABC
from enum import Enum, auto

from nca.core.abstract.sequential_identifier import SimulatorConnectionIdentifier
from revolve.evosphere.environment import Environment
from simulation.measures import Measures


class SimulatorState(Enum):
    STOPPED = auto()
    READY = auto()
    WORKING = auto()


class SimulatorConnector(ABC):

    port_identifier = SimulatorConnectionIdentifier()

    def __init__(self, environment: Environment):
        self.port = self.port_identifier.id()
        self.environment = environment
        self.state = SimulatorState.STOPPED
        self.initialize()

    def initialize(self):
        if not self.start_simulator():
            raise Exception("Problem starting Simulator")

        self.state = SimulatorState.READY

    def restart_simulator(self):
        if self.state != SimulatorState.STOPPED:
            self.stop_simulator()

        self.start_simulator()

    @abstractmethod
    def start_simulator(self) -> bool:
        pass

    @abstractmethod
    def stop_simulator(self) -> bool:
        pass

    def execute_robot(self, element: object) -> Measures:
        if self.state != SimulatorState.READY:
            raise Exception("Simulator is not ready")

        self.state = SimulatorState.WORKING
        self.add_robot(element)

        # wait for results
        measures = self.simulate()

        self.remove_robot(element)
        self.state = SimulatorState.READY

        return measures

    @abstractmethod
    def add_robot(self, element: object):
        pass

    @abstractmethod
    def remove_robot(self, element: object):
        pass

    @abstractmethod
    def simulate(self) -> Measures:
        pass

