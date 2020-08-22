from abc import abstractmethod, ABC
from enum import Enum, auto

from nca.core.abstract.sequential_identifier import SimulatorConnectionIdentifier
from revolve.evosphere.biosphere import Ecosphere
from revolve.robot.robot import Robot
from src.simulation.simulation_measures import SimulationMeasures


class SimulatorState(Enum):

    STOPPED = auto()
    READY = auto()
    WORKING = auto()

    def __eq__(self, other):
        return self.value == other.value


class SimulatorConnector(ABC):

    port_identifier = SimulatorConnectionIdentifier()

    def __init__(self, ecosphere: Ecosphere):
        self.port = self.port_identifier.id()
        self.ecosphere = ecosphere

        self.state = SimulatorState.STOPPED

        self.robot: Robot = None

        self.start()

    def restart(self):
        if self.state != SimulatorState.STOPPED:
            self.stop()

        self.start()

    def start(self):
        if not self.start_simulator():
            raise Exception("Problem starting Simulator")

        self.state = SimulatorState.READY

    def stop(self):
        if not self.stop_simulator():
            raise Exception("Problem starting Simulator")

        self.state = SimulatorState.STOPPED

    @abstractmethod
    def start_simulator(self) -> bool:
        pass

    @abstractmethod
    def stop_simulator(self) -> bool:
        pass

    def execute_robot(self, element: object) -> SimulationMeasures:
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
    def simulate(self) -> SimulationMeasures:
        pass

