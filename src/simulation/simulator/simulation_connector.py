from abc import abstractmethod, ABC
from enum import Enum, auto

from nca.core.abstract.sequential_identifier import SimulatorConnectionIdentifier
from nca.core.actor.agent import Agent
from nca.core.actor.individual import Individual
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

    @abstractmethod
    def _simulate(self) -> SimulationMeasures:
        pass

    @abstractmethod
    def execute(self, individual: Individual) -> SimulationMeasures:
        pass


class RobotSimulatorConnector(SimulatorConnector, ABC):

    def execute(self, robot: Robot) -> SimulationMeasures:
        if self.state != SimulatorState.READY:
            raise Exception("Simulator is not ready")

        self.state = SimulatorState.WORKING
        self._add_robot(robot)

        # wait for results
        measures = self._simulate()

        self._remove_robot(robot)
        self.state = SimulatorState.READY

        return measures

    @abstractmethod
    def _add_robot(self, robot: Robot):
        pass

    @abstractmethod
    def _remove_robot(self, robot: Robot):
        pass


class AgentSimulatorConnector(SimulatorConnector, ABC):

    def execute(self, agent: Agent) -> SimulationMeasures:
        if self.state != SimulatorState.READY:
            raise Exception("Simulator is not ready")

        self.state = SimulatorState.WORKING
        self._add_agent(agent)

        # wait for results
        measures = self._simulate()

        self._remove_agent(agent)
        self.state = SimulatorState.READY

        return measures

    @abstractmethod
    def _add_agent(self, agent: Agent):
        pass

    @abstractmethod
    def _remove_agent(self, agent: Agent):
        pass